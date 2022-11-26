import socket
import struct
import time
%matplotlib inline
import scipy, matplotlib.pyplot as plt, IPython.display as ipd
import librosa, librosa.display
#import stanford_mir; stanford_mir.init()
from ipywidgets import interact
import sys
import numpy as np
from IPython.display import Audio, display
import ruptures as rpt  # our package
import pyaudio
import pylab
import time
import sys
import wave
from scipy import signal

#######INFORMATION TO SEND#################

#Function/Declarations
RATE = 44100
CHUNK = int(RATE/20) # RATE / number of updates per second
frames = []
def soundplot(stream):

    t1=time.time()
    #use np.frombuffer if you face error at this line
    #data = np.fromstring(stream.read(CHUNK),dtype=np.int16)
    data = stream.read(CHUNK)
    frames.append(data) 

def find_offset(within_file, find_file, window):
    y_within, sr_within = librosa.load(within_file, sr=None)
    y_find, _ = librosa.load(find_file, sr=sr_within)

    c = signal.correlate(y_within, y_find[:sr_within*window], mode='valid', method='fft')
    peak = np.argmax(c)
    offset = round(peak / sr_within, 2)

    return offset

#BPM/FREQUENCY
duration = 20
x, sr = librosa.load('africa-toto.wav',duration=duration)
ipd.Audio(x, rate=sr) #Loading Audio File
tempo, beat_times = librosa.beat.beat_track(y=x, sr=sr, start_bpm=60, units='time')
print(tempo)
freq = tempo*.02
print(freq)

#DANCE ROUTINE
hop_length_tempo = 256
oenv = librosa.onset.onset_strength(y=x, sr=sr, hop_length=hop_length_tempo)
tempogram = librosa.feature.tempogram(onset_envelope=oenv,sr=sr,hop_length=hop_length_tempo)
algo = rpt.KernelCPD(kernel="linear").fit(tempogram.T)
n_bkps = 2
bkps = algo.predict(n_bkps=n_bkps)
bkps_times = librosa.frames_to_time(bkps, sr=sr, hop_length=hop_length_tempo)
print(bkps_times) #this corresponds to the changes
switch_times = bkps_times

############LAUNCHING THE SERVER############

localIP     = ""

localPort   = 3333

bufferSize  = 1024

# Creating initial packets to send to the mouse
routine_pack = struct.pack("ffff",switch_times[0],switch_times[1],switch_times[2],freq)

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

print("UDP server up and ready to send a packet")

#Initialize connection from the mouse
bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
message = bytesAddressPair[0]
address = bytesAddressPair[1]
clientMsg = "Message from Client:{}".format(message)
clientIP  = "Client IP Address:{}".format(address)
print(clientMsg)
print(clientIP)

#Send Dance Routine
time.sleep(1)
UDPServerSocket.sendto(routine_pack, address)

#Recieve confirmation times were sent
time.sleep(2)
bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
message = bytesAddressPair[0]
address = bytesAddressPair[1]
clientMsg = "Message from Client:{}".format(message)
print(clientMsg)

#Sending the live audio time in seconds every ~5 seconds
while True:
    p=pyaudio.PyAudio()
    stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
                  frames_per_buffer=CHUNK)
    for i in range(60):    #100 = 5 seconds
        soundplot(stream)
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    wf = wave.open("output.wav",'wb')
    wf.setnchannels(1)
    #wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setsampwidth(2)
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    frames = []
    
    #Finding Offset
    offset = find_offset('africa-toto.wav', 'output.wav', 10)
    print(f"Offset: {offset}s" )
    #Creating Packet to Send to Mouse
    offset_pack = struct.pack("f",offset)
    UDPServerSocket.sendto(offset_pack, address)
    
    #Ping to keep port open for sending packets
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    
exit
