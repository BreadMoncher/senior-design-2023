import interface
from enum import Enum
import threading
import time

class FSMState(Enum):
    INITIALIZE = 1
    TAKEOFF = 2
    FLIGHT_LOOP = 4
    END_FLIGHT = 5
    EXIT = 6

system_state = {
    "altitude": 0,
    "obstacle_distance_front": 0,
    "obstacle_distance_left": 0,
    "obstacle_distance_right": 0,
    "forward_velocity": 0,
}

def refresh_data():
# get teraranger data
    if (not interface.teraranger_data.empty()):
        while interface.teraranger_data.empty() != True:
            data = interface.teraranger_data.get()

        system_state["obstacle_distance_left"] = data[0]
        system_state["obstacle_distance_front"] = data[1]
        system_state["obstacle_distance_right"] = data[2]

        print("teraranger:",data)
        
    else:
        print("no teraranger data, continuing")
    

def run_command():
    global state

    # Get commands from the ground telemetry
    while (not interface.telemetry_received_data.empty()):
        next_command = interface.telemetry_received_data.get()

        if (next_command == "takeoff"):
            if (state == FSMState.INITIALIZE):
                # Takeoff
                return FSMState.TAKEOFF
            
        elif (next_command == "land"):
            if (state == FSMState.FLIGHT_LOOP):
                return FSMState.LAND
            pass

def fsm():
    global state, system_state

    if state == FSMState.INITIALIZE:
        time.sleep(0.2)

        # wait for takeoff command
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state", state)

    if state == FSMState.TAKEOFF:
        # run takeoff commands
        state = FSMState.FLIGHT_LOOP

    if state == FSMState.FLIGHT_LOOP:

        # get more system data
        refresh_data()

        # run flight control algorithm


        
        next_state = run_command()
        if next_state != None:
            state = next_state
            print("going to state",state)
        

    if state == FSMState.END_FLIGHT:
        # Land the drone safely
        
        state = FSMState.EXIT

    if state == FSMState.EXIT:
        # Exit the program
        exit()
    

def setup():
    global state

    # setup interface to peripheral devices
    x1 = threading.Thread(target=interface.teraranger_setup, args=("/dev/ttyACM0",))
    x2 = threading.Thread(target=interface.telemetry_setup, args=("/dev/ttyUSB0",))
    #x3 = threading.Thread(target=interface.depth_sensing_setup)

    # start collecting data
    x1.start()
    #x2.start()
    #x3.start()

    #put FSM in initial state
    state = FSMState.INITIALIZE



if __name__ == "__main__":
    setup()
    while(True):
        fsm()
