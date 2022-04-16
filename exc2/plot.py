import matplotlib.pyplot as plt
from itertools import count
from matplotlib.animation import FuncAnimation
from math import pi, e, sin


class Plot:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.x_values = []
        self.y_values = []
        self.index = count()

    def h(self, t):
        return 3 * pi * e ** (-1 * (5 * sin(2 * pi * t)))

    def animate_helper(self, i):
        t = next(self.index)
        self.x_values.append(t)
        self.y_values.append(self.h(t*pi/180))
        plt.cla()
        plt.plot(self.x_values, self.y_values)
        plt.xlim([self.x_min, self.x_max])
        plt.ylim([self.y_min, self.y_max])

    def live_animation(self):
        ani = FuncAnimation(plt.gcf(), self.animate_helper, interval=20)
        plt.tight_layout()
        plt.show()


hej = Plot(0, 1000, 0, 1600)
hej.live_animation()
