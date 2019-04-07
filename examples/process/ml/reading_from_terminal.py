# Import required library
import os                       # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial                   # To connect via the serial port
import time                     # To sleep for a few seconds
import pickle
import numpy as np


# The thing ID and access token
load_dotenv()

PressureID = "discopressure-5988"

#load the ml model
MODEL_FILE_NAME = "model.pickle"

with open("model.pickle", 'rb') as file:
    neigh = pickle.load(file)

classes =["No Sitting", "Normal Sitting", "Forward",
           "Backward", "Left", "Right"]

# Start reading the serial port
ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    write_timeout = 0)

def predict(values):
    result = neigh.predict(values)
    print(classes[result[0]])

def serial_Reader():
    line_bytes = ser.readline()
    if(len(line_bytes))>0:
        line = line_bytes.decode('utf-8')
        values = line.split(',')
        property_id=values.pop(0)
        print(property_id)
        if(property_id == PressureID):
            print("this is the pressure")
            print([float(x) for x in values])
            values = [float(x) for x in values]
            values = [values]
            np.arry(values).reshape(1,-1)
            if len(values[0]) == 4 :
                predict(values)
            else :
                print("did not get 4 values")



while True:
    serial_Reader()

# ser.close()
