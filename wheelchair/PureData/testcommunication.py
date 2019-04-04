import socket
import time

s = socket.socket()
host = socket.gethostname()
port = 3000

s.connect((host, port))



#Need to add " ;" at the end so pd knows when you're finished writing.


play_normal = '0 ' + str(1) + " ;"
stop_playing = '1 ' + str(1) + " ;"
audio_reverse = '2 ' + str(1) + " ;"
hipassf = '3 ' + str(2000) + " ;"
hipassfoff = '3 ' + str(20) + " ;"
lowpassf = '4 ' + str(500) + " ;"
pitch = '6 ' + str(0.045) + " ;"

s.send(play_normal.encode('utf-8'))

time.sleep(6)

s.send(hipassf.encode('utf-8'))

s.send(audio_reverse.encode('utf-8'))

time.sleep(2)

s.send(play_normal.encode('utf-8'))
s.send(hipassfoff.encode('utf-8'))

time.sleep(4)

s.send(stop_playing.encode('utf-8'))

time.sleep(0.5)

s.send(play_normal.encode('utf-8'))

time.sleep(4)

s.send(lowpassf.encode('utf-8'))

#s.send(hello.encode('utf-8'))
