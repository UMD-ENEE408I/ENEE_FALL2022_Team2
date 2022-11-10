mport pyaudio
import librosa, librosa.display
import numpy as np
import pylab
import time
import sys
import matplotlib.pyplot as plt
import wave
import numpy, scipy, matplotlib.pyplot as plt, IPython.display as ipd

RATE = 44100
CHUNK = int(RATE/20) # RATE / number of updates per second
frames = []
def soundplot(stream):
  
   t1=time.time()
   #use np.frombuffer if you face error at this line
   #data = np.fromstring(stream.read(CHUNK),dtype=np.int16)
   #print(data)
   data = stream.read(CHUNK)
   frames.append(data) 


print(data.shape);

if __name__=="__main__":
    p=pyaudio.PyAudio()
    stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
                  frames_per_buffer=CHUNK)
    #for i in range(sys.maxsize**10):
    for i in range(100):    #100 = 5 seconds
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
	
	
	import signal_envelope as se

W, sr = se.read_wav("output.wav")
X_pos_frontier, X_neg_frontier = se.get_frontiers(W, 0)
print(X_pos_frontier, W[X_pos_frontier])


X_envelope = se.get_frontiers(W, 1)
print(X_envelope, np.abs(W[X_envelope]))

F = np.abs(W[X_envelope])
print(F)
print(F.shape)

output_norm = F / np.linalg.norm(F)
print(output_norm)
P = np.transpose(output_norm)
d = P.shape
print(d)
    
	import numpy as np
#plt.figure(figsize=(14, 5))
#librosa.display.waveplot(W, alpha=0.6)
#plt.vlines(beat_times, -1, 1, color='r')
#plt.ylim(-1, 1)
t = np.arange(0,5,5/2020)
#t = np.transpose(t)
print (t.shape)
print (F.shape)
F = np.transpose(F)
#print (t)
plt.subplot(2,1,1)
plt.plot(t, F)
plt.subplot(2,1,2)
plt.plot(t,P)


#this is the end of the functioning part
#the next section needs work
import signal_envelope as se

W, sr = se.read_wav("audio/africa-toto.wav")
X_pos_frontier, X_neg_frontier = se.get_frontiers(W, 0)
print(X_pos_frontier, W[X_pos_frontier])


X_envelope = se.get_frontiers(W, 1)
print(X_envelope, np.abs(W[X_envelope]))

F = np.abs(W[X_envelope])

song_norm = F / np.linalg.norm(F)
print(song_norm)


#this part may be getting replaced so i'm commenting it out
#r = numpy.correlate(X_envelope, Y_envelope)
#print(X_envelope.shape, r.shape)

#plt.figure(figsize=(14, 5))
#plt.plot(r[:10000])
#plt.xlabel('Lag (samples)')
#plt.xlim(0, 10000)
    