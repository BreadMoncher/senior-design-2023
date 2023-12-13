import serial
import time

def read_from_serial(serial_port):

    ser = serial.Serial(serial_port, baudrate=115200, timeout=1)

    # More information on serial commands from the documentation:
    # https://terabee.b-cdn.net/wp-content/uploads/2021/03/TeraRanger-Hub-Evo-User-Manual.pdf

    # Set data-out type to 
    set_printout_text_command = bytes.fromhex("00110145")
    ser.write(set_printout_text_command)

    # Set data update rate to 50Hz
    set_data_refresh_rate_command = bytes.fromhex("00520302C3")
    ser.write(set_data_refresh_rate_command)

    # Enable data stream from the Teraranger
    activate_stream_command = bytes.fromhex("00520201DF")
    ser.write(activate_stream_command)

    # print data stream to console
    try:
        print("Waiting for data...")
        while True:
            a = ser.readline()
            try:
                line = a.decode()
                data = line.split('\t')
                print(data)
                print()
            except:
                print("fail")
                pass

    except KeyboardInterrupt:
        ser.close()
        print('\nSerial communication stopped.')
    except Exception as e:
        print(f'Error: {e}')
        if ser.is_open:
            ser.close()

serial_port = "/dev/ttyACM1"
read_from_serial(serial_port)