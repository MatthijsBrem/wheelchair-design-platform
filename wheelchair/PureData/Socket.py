import socket

s = socket.socket()
host = socket.gethostname()
port = 1000

s.connect((host, port))

mess = "Hello!"

#Need to add " ;" at the end so pd knows when you're finished writing.

message = mess + " ;"

s.send(message.encode('utf-8'))
