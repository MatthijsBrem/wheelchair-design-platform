# Import required library
import os                       # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial                   # To connect via the serial port
import time                     # To sleep for a few seconds

# The thing ID and access token
load_dotenv()

# Start reading the serial port
ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    write_timeout = 0)

def serial_Reader():
    line_bytes = ser.readline()
    if(len(line_bytes))>0:
        line = line_bytes.decode('utf-8')
        values = line.split(',')
        property_id=values.pop(0)
        print(property_id)

while True:
    serial_Reader()

# ser.close()
