import socket

s = socket.socket()
host = '127.0.0.1'
port = 5670

s.connect((host, 3000))

mess = "Hello!"

#Need to add " ;" at the end so pd knows when you're finished writing.

message = mess + " ;"

s.send(message.encode('utf-8'))
