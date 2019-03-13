import os
from dotenv import load_dotenv
import serial
import time

load_dotenv()


ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    timeout = 2)

while True:
        ser.write("1".encode())
        time.sleep(5)
        ser.write("0".encode())
        time.sleep(2)
