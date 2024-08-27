from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
from pymavlink import mavutil

# Mendefinisikan variabel global
gps_data = {}
attitude_data = {}
velocity_data = {}
battery_data = {}
mode_data = {}
compass_data = {}
barometer_data = {}


class PixhawkData:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def get_gps_data(self):
        global gps_data
        gps_data = {
            'latitude': self.vehicle.location.global_frame.lat,
            'longitude': self.vehicle.location.global_frame.lon,
            'altitude': self.vehicle.location.global_frame.alt
        }
        return gps_data

    def get_attitude_data(self):
        global attitude_data
        attitude = self.vehicle.attitude
        attitude_data = {
            'roll': attitude.roll,
            'pitch': attitude.pitch,
            'yaw': attitude.yaw
        }
        return attitude_data

    def get_velocity_data(self):
        global velocity_data
        velocity = self.vehicle.velocity
        velocity_data = {
            'north': velocity[0],
            'east': velocity[1],
            'down': velocity[2]
        }
        return velocity_data

    def get_battery_data(self):
        global battery_data
        battery = self.vehicle.battery
        battery_data = {
            'voltage': battery.voltage,
            'current': battery.current,
            'level': battery.level
        }
        return battery_data

    def get_mode(self):
        global mode_data
        mode_data = {'mode': self.vehicle.mode.name}
        return mode_data

    def get_compass_data(self):
        global compass_data
        compass_data = {'heading': self.vehicle.heading}
        return compass_data

    def get_barometer_data(self):
        global barometer_data
        barometer_data = {
            'altitude': self.vehicle.location.global_relative_frame.alt
        }
        return barometer_data

    def can_arm_vehicle(self):
        mode = self.vehicle.mode.name
        return mode in ['GUIDED', 'LOITER', 'ALT_HOLD']

    def is_vehicle_armed(self):
        return self.vehicle.armed


def check_pre_arm_status(vehicle):
    """
    Cek apakah semua pre-arm checks telah lulus.
    """
    print("Checking pre-arm status...")
    if vehicle.is_armable:
        print("Vehicle is armable")
    else:
        print("Vehicle is not armable yet. Waiting...")
    print(f"GPS Fix Type: {vehicle.gps_0.fix_type}")
    print(f"Number of Satellites: {vehicle.gps_0.satellites_visible}")
    print(f"EKF Status: {vehicle.ekf_ok}")
    return vehicle.is_armable


class PixhawkAction:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def arm_vehicle(self):
        pixhawk_data = PixhawkData(self.vehicle)
        if pixhawk_data.can_arm_vehicle():
            if not pixhawk_data.is_vehicle_armed():
                print("Arming the vehicle...")
                self.vehicle.armed = True
                while not self.vehicle.armed:
                    print("Waiting for arming...")
                    time.sleep(1)
                print("Vehicle is armed.")
            else:
                print("Vehicle is already armed.")
        else:
            print(
                f"Cannot arm the vehicle. Current mode: {self.vehicle.mode.name}. Switch to GUIDED or LOITER mode.")

    def take_off(self, altitude):
        # Cek pre-arm status
        if not check_pre_arm_status(self.vehicle):
            print("Pre-arm checks failed or incomplete. Cannot take off.")
            return

        if not self.vehicle.armed:
            print("Vehicle is not armed. Arming the vehicle first...")
            self.arm_vehicle()

        if self.vehicle.mode.name != 'GUIDED':
            print(f"Switching to GUIDED mode...")
            self.vehicle.mode = VehicleMode("GUIDED")
            timeout = 15  # 15 detik timeout untuk perubahan mode
            start_time = time.time()

            while not self.vehicle.mode.name == 'GUIDED':
                if time.time() - start_time > timeout:
                    print("Failed to switch to GUIDED mode within timeout period.")
                    return
                print("Waiting for GUIDED mode...")
                time.sleep(1)

            print("Mode is now GUIDED.")

        print(f"Taking off to {altitude} meters...")
        self.vehicle.simple_takeoff(altitude)

        # Loop sampai mencapai ketinggian yang diinginkan atau mode berubah
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {current_altitude} meters")

            # Periksa apakah mode berubah menjadi "LAND"
            if self.vehicle.mode.name == 'LAND':
                print("Mode changed to LAND during takeoff, aborting...")
                break

            # Jika sudah mencapai 95% dari ketinggian target, keluar dari loop
            if current_altitude >= altitude * 0.95:
                print(f"Reached target altitude of {altitude} meters")
                break

            time.sleep(1)


    def send_local_ned_velocity(self,vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
		    0,
		    0, 0,
		    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		    0b0000111111000111,
		    0, 0, 0,
		    vx, vy, vz,
		    0, 0, 0, # ground speed
		    0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        

    def landing(self):
        if not self.vehicle.armed:
            print("Vehicle is not armed. No need to land.")
            return

        print("Landing the vehicle...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.mode.name != 'LAND':
            print("Waiting for LAND mode...")
            time.sleep(1)
        print("Vehicle is now landing.")

        while self.vehicle.location.global_relative_frame.alt > 0.1:
            print(
                f"Altitude: {self.vehicle.location.global_relative_frame.alt} meters")
            time.sleep(1)
        print("Vehicle has landed.")


def main():
    try:
        print("Connecting to Pixhawk")
        vehicle = connect('COM8', baud=115200, wait_ready=True)
    except Exception as e:
        print(f"Failed to connect to the vehicle: {e}")
        return

    pixhawk_data = PixhawkData(vehicle)
    pixhawk_action = PixhawkAction(vehicle)

    takeoff_executed = False
    landing_executed = False
    maju = False
    previous_mode = ""
    target_altitude = 1  # Target altitude for takeoff

    try:
        while True:
            # Update data
            pixhawk_data.get_gps_data()
            pixhawk_data.get_attitude_data()
            pixhawk_data.get_velocity_data()
            pixhawk_data.get_battery_data()
            pixhawk_data.get_mode()
            pixhawk_data.get_compass_data()
            pixhawk_data.get_barometer_data()

            # Print sensor data
            print(f"GPS Data: {gps_data}")
            print(f"Attitude Data: {attitude_data}")
            print(f"Velocity Data: {velocity_data}")
            print(f"Battery Data: {battery_data}")
            print(f"Mode: {mode_data}")
            print(f"Compass Data: {compass_data}")
            print(f"Barometer Data: {barometer_data}")
            print("------")

            # Check mode and handle landing
            current_mode = mode_data['mode']
            print(current_mode)
            if current_mode == 'LAND':
                print("LANDING")
                if not landing_executed:
                    pixhawk_action.landing()
                    landing_executed = True

            # Check if vehicle is armed and handle takeoff
            if pixhawk_data.is_vehicle_armed():
                print("ARMED")
                if not takeoff_executed:
                    print("START TAKEOFF")
                    time.sleep(5)  # Wait for 2 seconds
                    # Perform takeoff to target altitude
                    pixhawk_action.take_off(target_altitude)
                    takeoff_executed = True
                    landing_executed = False
                    print("TAKEOFF")
            else:
                print("DISARMED")
                takeoff_executed = False
                landing_executed = False

            # Check if takeoff has been executed and wait for 5 seconds before landing
            if takeoff_executed and not landing_executed:
                current_altitude = vehicle.location.global_relative_frame.alt
                if current_altitude < target_altitude * 0.95:
                    print(
                        f"Altitude ({current_altitude}) below target, initiating landing.")
                    # pixhawk_action.landing()
                    landing_executed = False
                    maju = True
                
                if maju:
                    pixhawk_action.send_global_ned_velocity(1,0,0)
                    print("Moving TRUE NORTH")
                    time.sleep(3)
                    pixhawk_action.landing()
                    landing_executed = True

            else:
                time.sleep(5)  # Wait for 5 seconds
                pixhawk_action.landing()
                landing_executed = True
                print("LANDING")

            # Update previous mode
            previous_mode = current_mode

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Interrupted by user.")
        pixhawk_action.landing()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        vehicle.close()
        print("Connection to Pixhawk closed")


if __name__ == "__main__":
    main()
