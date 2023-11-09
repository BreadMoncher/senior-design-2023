import interface
from enum import Enum
import threading
import time

class FSMState(Enum):
    INITIALIZE = 1
    REFRESH_DATA = 2
    RUN_COMMANDS = 3
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

def fsm():
    global state, system_state

    if state == FSMState.INITIALIZE:
        time.sleep(0.2)
        state = FSMState.REFRESH_DATA

    if state == FSMState.REFRESH_DATA:
        
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
            
    if state == FSMState.RUN_COMMANDS:
        # Get commands from the ground telemetry
        while (not interface.telemetry_received_data.empty()):
            next_command = interface.telemetry_received_data.get()

            if (next_command == "takeoff"):
                #takeoff
                pass
            elif (next_command == "land"):
                #land
                pass
            

    if state == FSMState.FLIGHT_LOOP:
        
        
        
        state = FSMState.REFRESH_DATA

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
    #x2 = threading.Thread(target=interface.telemetry_setup, args=("/dev/ttyUSB0",))
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
