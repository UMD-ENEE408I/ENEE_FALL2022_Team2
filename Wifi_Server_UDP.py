
import socket
import struct
import time
 
localIP     = ""

localPort   = 3333

bufferSize  = 1024

# Create a packet to send to the mouse
my_str = '0' 
msgFromServer       = my_str.split(' ')
list_int			= [int(x) for x in msgFromServer]

my_str2 = '1' 
msgFromServer2       = my_str2.split(' ')
list_int2			= [int(x) for x in msgFromServer2]

val = struct.pack("b"*len(list_int),*list_int)
val2 = struct.pack("b"*len(list_int2),*list_int2)

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

print("UDP server up and ready to send a packet")

# Listen for incoming datagrams

while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    message = bytesAddressPair[0]

    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    print(clientMsg)
    print(clientIP)

    # Send a packet to the mouse

    UDPServerSocket.sendto(val, address)

    print(val)

    time.sleep(5)

    UDPServerSocket.sendto(val2, address)

    print(val2)

    time.sleep(5)

    exit()