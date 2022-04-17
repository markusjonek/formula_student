import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import numpy as np

def bla():
    print("hej")
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
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(left=0.1, bottom=0.3)
        self.p, = plt.plot([], [])
        self.axButton = plt.axes([0.1, 0.1, 0.1, 0.1])

    def h(self, t):
        if self.unit == "degrees":
            return 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * t * np.pi / 180)))
        elif self.unit == "radians":
            return 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * t)))

    def simple_plot(self):
        self.x_values = np.linspace(self.x_min, self.x_max, (self.x_max-self.x_min)*100)
        self.y_values = self.h(self.x_values)
        self.p.set_data(self.x_values, self.y_values)
        self.ax.set_xlim([self.x_min, self.x_max])
        self.ax.set_ylim([self.y_min, self.y_max])
        plt.show()

    def animate_helper(self, i):
        if self.x_test > self.x_max:
            self.x_test = 0
            self.x_values = []
            self.y_values = []
        self.x_values.append(self.x_test)
        self.y_values.append(self.h(self.x_test))
        plt.cla()
        self.p.set_data(self.x_values, self.y_values)
        self.ax.set_xlim([self.x_min, self.x_max])
        self.ax.set_ylim([self.y_min, self.y_max])
        self.x_test += 1

    def live_animation(self, x):
        self.x_values = []
        self.y_values = []
        ani = FuncAnimation(plt.gcf(), self.animate_helper, interval=20)
        #ani.event_source.start()

    def activate(self):
        live_btn = Button(ax=self.axButton, label="Live", color="teal", hovercolor="tomato")
        live_btn.on_clicked(self.live_animation)



hej = Plot(0, 400, 0, 1600)
hej.activate()

plt.show()