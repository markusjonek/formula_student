import matplotlib.pyplot as plt
import numpy as np
from itertools import count
from matplotlib.animation import FuncAnimation
import time
from math import pi, e, sin
plt.style.use('fivethirtyeight')

x = []
y = []
index = count()

def f(t):
    return 3*pi*e**(-1*(5*sin(2*pi*t)))

def animate():
    x1 = next(index)
    x.append(x1)
    y.append(f(x1*pi/180))
    plt.cla()
    plt.plot(x, y)
    plt.xlim([0, 1000])
    plt.ylim([-2, 1400])

ani = FuncAnimation(plt.gcf(), animate, interval=100)

plt.tight_layout()
plt.show()