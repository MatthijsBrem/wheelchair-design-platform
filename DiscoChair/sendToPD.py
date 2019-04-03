import os
import time


def send2Pd(message=' '):
    os.system("echo '" + message + "' | pdsend 1000")


def audio_on():
    message = '0 1'
    send2Pd(message)

def audio_reverse():
    message = '1 1'
    send2Pd(message)

def audio_off():
    message = '2 1'
    send2Pd(message)

def hipassf():
    hi = 100
    message = '3' + str(hi) + ';'
    send2Pd(message)

def lowpassf():
    low = 10000
    message = '4' + str(low) + ';'
    send2Pd(message)

def delay():
    ddelay = 0.5
    message = '5' + str(ddelay) + ';'
    send2Pd(message)

def pitch():
    tempo = 0.05
    message = '6' + str(tempo) + ';'
    send2Pd(message)

while True:
    audio_on()
    time.sleep(3)
    audio_reverse()
    time.sleep(3)
    audio_on()
    time.sleep(3)
    pitch()
    delay()
    time.sleep(3)
    lowpassf()
    time.sleep(3)
    hipassf()
    time.sleep(3)
    audio_off()
