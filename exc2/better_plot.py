import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider, TextBox
import os

class Animator(FuncAnimation):
    """
    Inherites the capabilities of FuncAnimation with extra features.
    Buttons for start, stop and save figure.
    Slider for quick scroll through x-values.
    Button to save the figure with custom name.
    """
    def __init__(self, fig, plot_function, mini=0, maxi=100):
        self.i = mini
        self.min = mini
        self.max = maxi
        self.runs = True
        self.fig = fig
        self.plot_function = plot_function

        widget_y_pos = 0.9
        button_width = 0.08
        widget_height = 0.05

        startax = plt.axes([0.13, widget_y_pos, button_width, widget_height])
        self.start_button = Button(startax, label='$\u25B6$')
        self.start_button.on_clicked(self.start)

        stopax = plt.axes([0.22, widget_y_pos, button_width, widget_height])
        self.stop_button = Button(stopax, label='$\u25A0$')
        self.stop_button.on_clicked(self.stop)

        slideax = plt.axes([0.33, widget_y_pos, 0.4, widget_height])
        self.slider = Slider(slideax, '', self.min, self.max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

        saveax = plt.axes([0.82, widget_y_pos, button_width, widget_height])
        self.save_button = Button(saveax, label='Save')
        self.save_button.on_clicked(self.save_figure)
        self.file_index = 1

        FuncAnimation.__init__(self, self.fig, self.update_slider, frames=self.frame_updater(), interval=15)

    def frame_updater(self):
        """
        Responsible for what happens between frames. Stops the slider if it reaches x_max.
        """
        while self.runs:
            if self.i >= self.max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self, event=None):
        """
        Starts the plot. Replays from start if the plot has reached the end.
        """
        if self.i < self.max:
            self.runs=True
            self.event_source.start()
        if self.i == self.max:
            self.i = self.min

    def stop(self, event=None):
        """
        Stops the plot
        """
        self.runs = False
        self.event_source.stop()

    def set_pos(self, i):
        """
        Gives the command to the given plot_funtion to plot up until the slider value.
        """
        self.i = int(self.slider.val)
        self.plot_function(self.i)

    def update_slider(self, i):
        """
        Updates the slider position as the animation is running.
        """
        self.slider.set_val(i)

    def save_figure(self, event=None):
        """
        Saves the current figure with a unique index, ex. "figure5".
        """
        file_names = os.listdir('./')
        indexes = []
        for file_name in file_names:
            if file_name[0:6] == "figure":
                indexes.append(int(file_name[6]))
        plt.savefig("figure" + str(max(indexes) + 1) + ".png")


fig, ax = plt.subplots()

x_min = -1000
x_max = 1000
y_min = -200
y_max = 2000

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
point, = ax.plot([], [])


def plot_function(i):
    x_val = np.linspace(x_min, i, abs(i*100)+2000)
    y_val = 3 * np.pi * np.exp(-5 * np.sin(2 * np.pi * x_val * np.pi / 180)) # *pi/180 for plot in degrees
    point.set_data(x_val, y_val)


animation = Animator(fig, plot_function, x_min, x_max)

plt.show()
