import serial
import time

# Configure serial port settings
#serial_port = "COM10"  # Replace with the appropriate COM port
serial_port = '/dev/ttyUSB0'

baud_rate = 57600  # Replace with the baud rate matching the receiver's configuration

# File path to the PNG file you want to send
image_path = "testimage1.jpeg"

# Define the packet size (in bytes)
packet_size = 128  # You can adjust this as needed

# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate)

try:

    # Open the jpeg file for reading
    with open(image_path, 'rb') as file:
        image_data = file.read()
        
        # Send the image size
        ser.write(len(image_data).to_bytes(4, byteorder='big'))
        time.sleep(1)  # Wait for the receiver to prepare for image data
        
        # Send the image data in chunks
        for i in range(0, len(image_data), packet_size):
            print("packet count:",i//packet_size)
            ser.write(image_data[i:i+packet_size])

    print(f"Image file '{image_path}' sent successfully in packets.")
except Exception as e:
    print(f"An error occurred while sending the file: {str(e)}")
finally:
    # Close the serial connection
    if ser.is_open:
        ser.close()
