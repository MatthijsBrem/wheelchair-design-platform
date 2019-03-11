import os
from dotenv import load_dotenv
import Serial
import time

load_dotenv()


ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    timeout = 2)

while True:
        ser.write('1')
        time.sleep(5)
        ser.write('0')
        time.sleep(2)
