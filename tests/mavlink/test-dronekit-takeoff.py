from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
target_system = 1  #vehicle._vehicle_id
target_component = 0  #1


# Set the drone forward velocity to 0.25 meters/second
def set_forward_velocity(velocity):
    global target_system, target_component
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # Timestamp (milliseconds since system boot)
        target_system,       # Target system
        target_component,       # Target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Frame of reference
        0b0000111111000011,  # Type mask - only speeds enabled
        0, 0, -3,  # x, y, z positions (not used)
        velocity, 0, 0,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)

    vehicle.send_mavlink(msg)
    vehicle.flush()


# Main script
try:

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
    vehicle.simple_takeoff(2)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=2*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

    print('Altitude reached')

    time.sleep(5)

    print('changing forward velocity')

    
    #vehicle.groundspeed = 0.25

    for _ in range(20):
        set_forward_velocity(0.5)
        #print("voltage: ",vehicle.battery.voltage)
        
        #print("current:",vehicle.battery.current)
        time.sleep(1)

    set_forward_velocity(0.00)
    #vehicle.groundspeed = 0.0

    time.sleep(10)

except KeyboardInterrupt:
    print("User interrupted the script.")

finally:
    # Disarm the vehicle and close the connection
    print("Disarming motors and closing the connection...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed = False
    vehicle.close()

