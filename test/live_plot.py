from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ys = []
xs = []
global i
i=0
plt.ion()

def animate(t):
    #graph_data = open('/home/hongli/tello_ws/src/tello_mod/test/example.txt','r').read()
    global i
    y=np.cos(float(i)/10) + 0.3*np.cos(float(i)/15)
    xs.append(float(i))
    ys.append(float(y))
    print("len(x):"+str(len(xs)))
    ax1.plot(xs, ys)
    print(len(xs))
    print(len(ys))
    if len(xs) > 100:
        plt.xlim(xs[0],xs[100])
        xs.pop(0)
        ys.pop(0)
    i=i+1
    plt.draw()
    plt.pause(0.00000001)
    

    #plt.xlim(10,20)

while(True):
    animate(1)
    sleep(0.005)
  