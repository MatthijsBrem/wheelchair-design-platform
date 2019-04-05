from dotenv import load_dotenv
import os
import pickle
import serial
import numpy as np

load_dotenv()

# Where to read the model from
MODEL_FILE_NAME = "model.pickle"

# load classifier
with open("model.pickle", 'rb') as file:
    neigh = pickle.load(file)

classes = ["No Sitting", "Normal Sitting", "Forward",
           "Backward", "Left", "Right"]

# Read data from serial port
ser = serial.Serial(
    port=os.environ['SERIAL'],
    baudrate=9600,
    timeout=2)

def predict(values):
    result = neigh.predict(values)
    print(classes[result[0]])

# Real time prediction
def serial_to_property_values():
    line_bytes = ser.readline()
    # If the line is not empty
    if len(line_bytes) > 0:
        # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        print("the line is as follows")
        print(line)
        str_values = line.split(',')
        if len(str_values) > 1:
            str_values.pop(0)
            values = [float(x) for x in str_values]
            values = [values]
            np.array(values).reshape(1, -1)
            print("checking what the values are")
            print(values[0])
            print("the lengt is")
            print(len(values[0]))
            if len(values[0]) == 4:
                print("i am in here")
                predict(values)


while(True):
    serial_to_property_values()
