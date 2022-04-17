import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import time

class Player(FuncAnimation):
    def __init__(self, fig, func, mini=0, maxi=100):
        self.i = mini
        self.min = mini
        self.max = maxi
        self.runs = True
        self.fig = fig
        self.func = func

        startax = plt.axes([0.13, 0.9, 0.08, 0.05])
        stopax = plt.axes([0.24, 0.9, 0.08, 0.05])
        slideax = plt.axes([0.37, 0.9, 0.5, 0.05])
        self.stop_button = Button(stopax, label='$\u25A0$')
        self.start_button = Button(startax, label='$\u25B6$')
        self.stop_button.on_clicked(self.stop)
        self.start_button.on_clicked(self.start)
        self.slider = Slider(slideax, '', self.min, self.max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

        FuncAnimation.__init__(self, self.fig, self.update_slider, frames=self.frame_updater(), interval=15)

    def frame_updater(self):
        while self.runs:
            if self.i >= self.max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self, event=None):
        if self.i < self.max:
            self.runs=True
            self.event_source.start()
        if self.i == self.max:
            self.i = self.min

    def stop(self, event=None):
        self.runs = False
        self.event_source.stop()

    def set_pos(self, i):
        self.i = int(self.slider.val)
        self.func(self.i)

    def update_slider(self, i):
        self.slider.set_val(i)


x_min = -1000
x_max = 1000
y_min = -200
y_max = 2000

fig, ax = plt.subplots()
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
point, = ax.plot([], [])


def plot_function(i):
    x_val = np.linspace(x_min, i, abs(i*100)+2000)
    y_val = 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * x_val * np.pi / 180)))
    point.set_data(x_val, y_val)


ani = Player(fig, plot_function, x_min, x_max)

plt.show()