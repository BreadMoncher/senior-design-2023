'''from pymavlink import mavutil
import time

# Connect to Pixhawk
connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)

print("Waiting for Heartbeat")
connection.wait_heartbeat()
print("Heartbeat detected")

command = connection.mav.command_long_encode(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    0,  # set mode by custom mode (could also use MAV_MODE_FLAG)
    4,  # mode = stabilized
    0, 0, 0, 0, 0)

# continuously send and receive commands
while(True):
    # Send the command
    connection.mav.send(command)

    # Wait for a response
    response = None
    while response is None:
        response = connection.recv_msg()
        if response is not None:
            print(f"Received response: {response}")
    
    time.sleep(1.2)

connection.close()
'''
from pymavlink import mavutil
import time

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM1', baud=57600)

try:
    while True:
        # Request attitude information
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

        # Wait for a message
        msg = master.recv_match(type='ATTITUDE', blocking=True)

        # Print attitude information
        print(f"Yaw: {msg.yaw}, Pitch: {msg.pitch}, Roll: {msg.roll}")

        # Request position information
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        # Wait for a message
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        # Print altitude information
        print(f"Altitude: {msg.alt / 1000} meters")

        # Wait for a short duration before next iteration (adjust as needed)
        time.sleep(0.01)

except KeyboardInterrupt:
    pass
