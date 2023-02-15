import matplotlib.pyplot as plt
from matplotlib import animation
import serial
import time 

ser = serial.Serial("/dev/ttyUSB0", 115200)

plt.ylim([-0.25, 0.25])

x = []
y = [[],[],[],[],[],[],[]]
def upd(i):
    ln = [float(str(i)[2:-2]) for i in ser.readline().split()]
    x.append(time.time())

    plt.cla()

    y[0].append(0.25)
    plt.plot(x[-50:-1], y[0][-50:-1])
    
    for i in range(len(ln)):
        y[i+1].append(ln[i])
        plt.plot(x[-50:-1], y[i+1][-50:-1])
    
    y[-1].append(-0.25)
    plt.plot(x[-50:-1], y[-1][-50:-1])

anima = animation.FuncAnimation(plt.gcf(), upd, interval=0)

plt.show()

file = open("Log.txt", "w")

while True:
    ln = [str(i)[2:-2] for i in ser.readline().split()]
    print(ln[1])
    file.write(str(ln[1]) + "\n")
