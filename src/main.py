import interface
from enum import Enum
import threading
import time
from dronekit import connect, VehicleMode

class FSMState(Enum):
    INITIALIZE = 1
    ARM = 2
    WAIT_TAKEOFF = 3
    TAKEOFF = 4
    WAIT_FLIGHT_LOOP = 5
    FLIGHT_LOOP = 6
    LAND = 7
    END_FLIGHT = 8
    EXIT = 9

# Describe the state of the vehicle
system_state = {
    "altitude": 0,
    "obstacle_distance": [0,0,0,0,0,0,0,0],
    "forward_velocity": 0,
}

# Vehicle mavlink connection as an object
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
target_system = vehicle._vehicle_id
target_component = 1

# Set the drone forward velocity to 0.25 meters/second
def set_forward_velocity(velocity):
    global target_system, target_component
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # Timestamp (milliseconds since system boot)
        target_system,       # Target system
        target_component,       # Target component
        0b0000000100000000,  # Type mask - only speeds enabled
        0, 0, 0,  # x, y, z positions (not used)
        0, 0,  velocity,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)

    vehicle.send_mavlink(msg)
    vehicle.flush()

def refresh_data():
    # get teraranger data
    if (not interface.teraranger_data.empty()):
        while interface.teraranger_data.empty() != True:
            data = interface.teraranger_data.get()

        system_state["obstacle_distance"] = data

        print("teraranger:",data)
        
    else:
        print("no teraranger data, continuing")
        pass 

def run_command():
    global state

    # Get commands from the ground telemetry and jump between states
    while (not interface.telemetry_received_data.empty()):
        next_command = interface.telemetry_received_data.get()
        print(next_command)
        #print(next_command)
        if (next_command == b"ARM\n"):
            if (state == FSMState.INITIALIZE):
                return FSMState.ARM
            
        elif next_command == b"TF\n":
            if (state == FSMState.WAIT_TAKEOFF):
                return FSMState.TAKEOFF
            
        elif next_command == b"FLY\n":
            if (state == FSMState.WAIT_FLIGHT_LOOP):
                return FSMState.FLIGHT_LOOP
            
        elif (next_command == b"LND\n"):
            if (state == FSMState.FLIGHT_LOOP):
                return FSMState.END_FLIGHT
            
        elif (next_command == b"RST\n"):
            if (state == FSMState.INITIALIZE):
                return FSMState.END_FLIGHT
            

def fsm():
    global state, system_state

    if state == FSMState.INITIALIZE:
        # wait for arm command
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state", state)

    if state == FSMState.ARM:
        # Set the mode to GUIDED (required for takeoff)
        vehicle.mode = VehicleMode("GUIDED")

        # Arm the drone
        vehicle.armed = True

        # Go to next state
        print("going to WAIT_TAKEOFF")
        state = FSMState.WAIT_TAKEOFF

    if state == FSMState.WAIT_TAKEOFF:
        # Wait for takeoff command
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state", state)

    if state == FSMState.TAKEOFF:
        # run takeoff command with altitude of 1
        vehicle.simple_takeoff(1)
        time.sleep(1)

        # make drone move forward
        set_forward_velocity(0.25)

        state = FSMState.WAIT_FLIGHT_LOOP

    if state == FSMState.WAIT_FLIGHT_LOOP:

        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state",state)
        

    if state == FSMState.FLIGHT_LOOP:
        time.sleep(0.5)
        # get more system data
        refresh_data()

        # run flight control algorithm
        if system_state["obstacle_distance"][0] < 1.0 or system_state["obstacle_distance"][7] < 1.0:
            set_forward_velocity(0.0)
        
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state",state)
        

    if state == FSMState.END_FLIGHT:
        # Land the drone safely
        vehicle.mode = VehicleMode("LAND")
        print("Finished flight - enter reset command to run again")
        
        # Go to next state
        state = FSMState.EXIT

    if state == FSMState.EXIT:  
        # Wait for reset command
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state", state)

    

def setup():
    global state

    # setup interface to peripheral devices
    x1 = threading.Thread(target=interface.teraranger_setup, args=("/dev/ttyACM1",))
    x2 = threading.Thread(target=interface.telemetry_setup, args=("/dev/ttyUSB0",))
    #x3 = threading.Thread(target=interface.depth_sensing_setup)

    # start collecting data
    x1.start()
    x2.start()
    #x3.start()

    #put FSM in initial state
    print("Going to initial state...")
    state = FSMState.INITIALIZE

if __name__ == "__main__":
    setup()
    while(True):
        fsm()
