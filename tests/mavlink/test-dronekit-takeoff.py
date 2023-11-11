from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

# Set the mode to GUIDED (required for takeoff)
vehicle.mode = VehicleMode("GUIDED")

# Arm the drone
vehicle.armed = True

# Wait for the vehicle to arm
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)

print("Vehicle armed!")



# Command the drone to take off at 2 meters altitude
print('taking off...')
vehicle.simple_takeoff(1)

time.sleep(1)

vehicle.mode = VehicleMode("LAND")

'''
# Wait for the drone to reach the desired altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    #if vehicle.location.global_relative_frame.alt >= 2 * 0.95:  # Allow some margin of error (5%)
        #print("Target altitude reached")
        #break
    time.sleep(1)
'''