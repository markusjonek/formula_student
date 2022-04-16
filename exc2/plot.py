import matplotlib.pyplot as plt
from itertools import count
from matplotlib.animation import FuncAnimation
import numpy as np


class Plot:
    def __init__(self, x_min, x_max, y_min, y_max, unit="degrees"):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.x_values = []
        self.y_values = []
        self.unit = unit
        self.x_test = 0

    def h(self, t):
        if self.unit == "degrees":
            return 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * t * np.pi / 180)))
        elif self.unit == "radians":
            return 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * t)))

    def simple_plot(self):
        self.x_values = np.linspace(self.x_min, self.x_max, (self.x_max-self.x_min)*100)
        self.y_values = self.h(self.x_values)
        plt.plot(self.x_values, self.y_values)
        plt.xlim([self.x_min, self.x_max])
        plt.ylim([self.y_min, self.y_max])
        plt.tight_layout()
        plt.show()

    def animate_helper(self, i):
        if self.x_test > self.x_max:
            self.x_test = 0
            self.x_values = []
            self.y_values = []
        self.x_values.append(self.x_test)
        self.y_values.append(self.h(self.x_test))
        plt.cla()
        plt.plot(self.x_values, self.y_values)
        plt.xlim([self.x_min, self.x_max])
        plt.ylim([self.y_min, self.y_max])
        self.x_test += 1

    def live_animation(self):
        self.x_values = []
        self.y_values = []
        ani = FuncAnimation(plt.gcf(), self.animate_helper, interval=20)
        #plt.tight_layout()
        plt.show()




hej = Plot(0, 50, 0, 1600)
hej.live_animation()


