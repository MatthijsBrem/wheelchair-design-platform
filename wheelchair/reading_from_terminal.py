# Import required library
import os                       # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial                   # To connect via the serial port
import time                     # To sleep for a few seconds
import pickle
import numpy as np
import socket

s = socket.socket()
host = socket.gethostname()
port = 3000

s.connect((host,port))

# The thing ID and access token
load_dotenv()

PressureID = "discopressure-5988"

#load the ml model
MODEL_FILE_NAME = "model.pickle"

#load classifier
with open("model.pickle", 'rb') as file:
    neigh = pickle.load(file)

classes =["No Sitting", "Normal Sitting", "Forward",
           "Backward", "Left", "Right"]

# Start reading the serial port
ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    write_timeout = 0)

lastpredictions = [-1]
currentPrediction = 0

def translatePredictionToPD(prediction):
    if len(lastpredictions) > 4:
        if lastpredictions[-1] == prediction:
            if lastpredictions[-2] == prediction:
                if lastpredictions[-3] == prediction:
                    if lastpredictions[-5] != prediction:
                        lastpredictions.clear()
                        currentPrediction = prediction
                        print("Time to clear the list")

    lastpredictions.append(prediction[0])
    print(lastpredictions)

    if currentPrediction == 0:
        music_off = "1 1 ;"
        s.send(music_off.encode('utf-8'))
        print("turning the music off")
    elif currentPrediction == 1:
        music_on = "0 1 ;"
        s.send(music_on.encode('utf-8'))
        print("turning the music on")
    elif currentPrediction == 2:
        pitch_increase= " ;"
        s.send(music_on.encode('utf-8'))


    elif prediction == 3:
        reverse_on = "2 1 ;"
        s.send(reverse_on.encode('utf-8'))


def predict(values):
    result = neigh.predict(values)
    print(classes[result[0]])
    print("the value of the prediction result is")
    print(result[0])
    translatePredictionToPD(result)

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
            np.array(values).reshape(1,-1)
            if len(values[0]) == 4 :
                predict(values)
            else :
                print("did not get 4 values")



while True:
    serial_Reader()

# ser.close()
