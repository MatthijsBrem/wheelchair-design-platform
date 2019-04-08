# Import required library
import os                       # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial                   # To connect via the serial port
import pickle
import numpy as np
import socket


from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType

s = socket.socket()
host = socket.gethostname()
port = 3000

s.connect((host,port))

# The thing ID and access token
load_dotenv()
THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

my_thing = Thing(thing_id = THING_ID, token = THING_TOKEN)

my_thing.read()

PressureID = "discopressure-5988"
GestureID = "discogesture-4e6b"
PostureID = "discopostures"

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
currentPitch = 0.0626
highpass = 20
lowpass = 20000


def translatePredictionToPD(prediction):
    global currentPitch
    global highpass
    global lowpass
    if len(lastpredictions) > 4:
        if lastpredictions[-1] == prediction:
            if lastpredictions[-2] == prediction:
                if lastpredictions[-3] == prediction:
                    if lastpredictions[0] != prediction:
                        lastpredictions.clear()
                        print("Time to clear the list")

    lastpredictions.append(prediction[0])
    print("the lengt of the list is")
    print(len(lastpredictions))
    print ("currently pushing the following to PD: ")
    print(classes[lastpredictions[0]])

    if lastpredictions[0] == 0:
        music_off = "1 1 ;"
        s.send(music_off.encode('utf-8'))
        print("turning the music off")
    elif lastpredictions[0] == 1:
        music_on = "0 1 ;"
        s.send(music_on.encode('utf-8'))
        print("turning the music on")
        if lowpass != 20000:
            lowpass = 20000
            lowpass_normal = "4 20000 ;"
            s.send(lowpass_normal.encode('utf-8'))
        if highpass != 20:
            highpass = 20
            highpass_normal = "3 20 ;"
            s.send(highpass_normal.encode('utf-8'))
    elif lastpredictions[0] == 2:
        if currentPitch < 0.08:
            currentPitch += 0.000428
        else:
            currentPitch = 0.08
        pitch_decrease = '6 ' + str(currentPitch) + " ;"
        s.send(pitch_decrease.encode('utf-8'))
        print("the pitch that is send is")
        print(currentPitch)
    elif lastpredictions[0] == 3:
        if currentPitch > 0.044:
            currentPitch -= 0.000428
        else:
            currentPitch = 0.044
        pitch_decrease = '6 ' + str(currentPitch) + " ;"
        s.send(pitch_decrease.encode('utf-8'))
        print("the pitch that is send is")
        print(currentPitch)
    elif lastpredictions[0] == 4:
        if lowpass > 200:
            lowpass -= 250
        else:
            lowpass = 200
        lowpass_str = "4 " + str(lowpass) + " ;"
        s.send(lowpass_str.encode('utf-8'))
    elif lastpredictions[0] == 5:
        if highpass < 5000:
            highpass += 25
        else:
            highpass = 5000
        highpass_str = "3 " + str(highpass) + " ;"
        s.send(highpass_str.encode('utf-8'))


def predict(values):
    result = neigh.predict(values)
    print(classes[result[0]])
    prop = my_thing.find_or_create_property(PostureID, PropertyType.CLASS)

    if prop is not None:
        posture = int(result[0])
        prop.update_values([posture])
    else:
        print('Warning: unknown property ' + PostureID)

    # print("the value of the prediction result is")
    # print(result[0])
    translatePredictionToPD(result)


def serial_Reader():
    line_bytes = ser.readline()
    if(len(line_bytes)) > 0:
        line = line_bytes.decode('utf-8')
        values = line.split(',')
        property_id = values.pop(0)
        print(property_id)

        if(property_id == PressureID):
            prop = my_thing.properties[property_id]

            if prop is not None:
                prop.update_values([float(x) for x in values])
            else:
                print('Warning: unknown property ' + property_id)

            # print("this is the pressure")
            # print([float(x) for x in values])
            values = [float(x) for x in values]
            values = [values]
            np.array(values).reshape(1, -1)
            if len(values[0]) == 4:
                predict(values)
            else:
                print("did not get 4 values")
        elif(property_id == GestureID):
            prop = my_thing.properties[property_id]

            if prop is not None:
                prop.update_values([float(x) for x in values])
            else:
                print('Warning: unknown property ' + property_id)

            print("the gesture is as follows: ")
            values = [float(x) for x in values]
            print (values[0])
            if values[0] == 4:
                reverse_str = "2 1 ;"
                s.send(reverse_str.encode('utf-8'))
                print("enabling reverse")
            elif values[0] == 6:
                reverse_str = "0 1 ;"
                s.send(reverse_str.encode('utf-8'))
                print("disableing reverse")
            elif values[0] == 2:
                delay_str = "5 0 ;"
                s.send(delay_str.encode('utf-8'))
            elif values[0] == 8:
                deylay_str = "5 0.6 ;"
                s.send(delay_str.encode('utf-8'))


while True:
    serial_Reader()

# ser.close()
