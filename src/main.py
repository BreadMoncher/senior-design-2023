import interface
from enum import Enum
import threading
import time

class FSMState(Enum):
    INITIALIZE = 1
    FLIGHT_LOOP = 2
    EXIT = 3

def fsm():
    global state

    if state == FSMState.INITIALIZE:
        time.sleep(0.2)
        state = FSMState.FLIGHT_LOOP

    if state == FSMState.FLIGHT_LOOP:
        
        if interface.teraranger_data.empty():
            print("no teraranger data, continuing")
        else:
            while interface.teraranger_data.empty() != True:
                data = interface.teraranger_data.get()
            print("teraranger:",data)
        
        state = FSMState.INITIALIZE
    

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
