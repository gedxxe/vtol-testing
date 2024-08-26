from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

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

    # Other methods omitted for brevity

class PixhawkAction:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def arm_vehicle(self):
        pixhawk_data = PixhawkData(self.vehicle)
        # Arm vehicle logic

    def autonomous_flight(self):
        # Check pre-arm status
        if not check_pre_arm_status(self.vehicle):
            print("Pre-arm checks failed or incomplete. Cannot take off.")
            return

        if not self.vehicle.armed:
            print("Vehicle is not armed. Arming the vehicle first...")
            self.arm_vehicle()

        # Takeoff to 1 meter altitude
        print("Taking off to 1 meter...")
        self.vehicle.simple_takeoff(1)
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {current_altitude} meters")
            if current_altitude >= 1 * 0.95:
                print("Reached target altitude of 1 meter")
                break
            time.sleep(1)

        # Hold position for 10 seconds
        print("Holding position for 10 seconds...")
        start_time = time.time()
        while time.time() - start_time < 10:
            time.sleep(1)

        # Fly forward 3 meters
        print("Flying forward 3 meters...")
        self.vehicle.simple_goto(LocationGlobalRelative(
            gps_data['latitude'] + 3 * 0.00001, gps_data['longitude'], 1))
        while True:
            current_position = self.vehicle.location.global_relative_frame
            distance_to_target = self.get_distance_metres(
                current_position, self.vehicle.location.global_relative_frame)
            print(f"Distance to target: {distance_to_target} meters")
            if distance_to_target <= 0.5:
                print("Reached target position")
                break
            time.sleep(1)

        # Hold position for 10 seconds
        print("Holding position for 10 seconds...")
        start_time = time.time()
        while time.time() - start_time < 10:
            time.sleep(1)

        # Land the vehicle
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

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return pow(dlat * dlat + dlong * dlong, 0.5) * 1.113195e5

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

def main():
    try:
        print("Connecting to Pixhawk")
        vehicle = connect('COM8', baud=115200, wait_ready=True)
    except Exception as e:
        print(f"Failed to connect to the vehicle: {e}")
        return

    pixhawk_data = PixhawkData(vehicle)
    pixhawk_action = PixhawkAction(vehicle)

    try:
        # Perform autonomous flight
        pixhawk_action.autonomous_flight()
    except KeyboardInterrupt:
        print("Interrupted by user.")
    except:
        print("An error occurred during autonomous flight.")

    # Disconnect from the vehicle
    vehicle.close()
    print("Disconnected from the vehicle.")

if __name__ == "__main__":
    main()