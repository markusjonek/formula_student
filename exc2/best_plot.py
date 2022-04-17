import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import os

class Animator(FuncAnimation):
    """
    Inherites the capabilities of FuncAnimation with extra features.
    Buttons for start, stop and save figure.
    Slider for quick scroll through x-values.
    Button to save the figure with custom name.
    """
    def __init__(self, fig, plot_function, x_min, x_max):
        self.fig = fig
        self.plot_function = plot_function
        self.x_min = x_min
        self.x_max = x_max
        self.i = x_min
        self.runs = True

        widget_y_pos = 0.9
        button_width = 0.08
        widget_height = 0.05

        # start button
        startax = plt.axes([0.13, widget_y_pos, button_width, widget_height])
        self.start_button = Button(startax, label='$\u25B6$')
        self.start_button.on_clicked(self.start)

        # stop button
        stopax = plt.axes([0.22, widget_y_pos, button_width, widget_height])
        self.stop_button = Button(stopax, label='$\u25A0$')
        self.stop_button.on_clicked(self.stop)

        # slider
        slideax = plt.axes([0.33, widget_y_pos, 0.3, widget_height])
        self.slider = Slider(slideax, '', self.x_min, self.x_max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

        FuncAnimation.__init__(self, self.fig, self.update_slider, frames=self.frame_updater(), interval=15)

    def frame_updater(self):
        """
        Responsible for what happens between frames. Stops the slider if it reaches x_max.
        """
        while self.runs:
            if self.i >= self.x_max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self, event=None):
        """
        Starts the plot. Replays from start if the plot has reached the end.
        """
        if self.i < self.x_max:
            self.runs=True
            self.event_source.start()
        if self.i == self.x_max:
            self.i = self.x_min

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


class Plot:
    """ Class for plotting a function """
    def __init__(self, x_min, x_max, y_min, y_max, grid_button=True, save_button=True):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.fig, self.ax = plt.subplots()
        self.point, = self.ax.plot([], [])

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)

        if grid_button:
            self.grid_button()

        if save_button:
            self.save_button()

    def grid_button(self):
        """ Adds grid on/off button. """
        gridax = plt.axes([0.72, 0.9, 0.08, 0.05])
        self.grid_button = Button(gridax, label='#')
        self.grid_button.on_clicked(self.add_grid)
        self.n = 1

    def add_grid(self, event=None):
            if self.n % 2:
                self.ax.grid(True)
            else:
                self.ax.grid(False)
            self.n += 1

    def save_button(self):
        """ Adds save button. """
        saveax = plt.axes([0.82, 0.9, 0.08, 0.05])
        self.save_button = Button(saveax, label='Save')
        self.save_button.on_clicked(self.save_figure)
        self.file_index = 1

    def save_figure(self, event=None):
        """ Saves the current figure (image) with a unique index, ex. "figure5". """
        file_names = os.listdir('./')
        indexes = []
        for file_name in file_names:
            if file_name[0:6] == "figure":
                indexes.append(int(file_name[6]))
        if len(indexes) == 0:
            index = "1"
        else:
            index = str(max(indexes) + 1)

        plt.savefig("figure" + index + ".png")

    def plot_function(self, i):
        """ The function to run at each update in the animation """
        x_values = np.linspace(self.x_min, i, abs(i * 100) + 2000)
        y_values = self.func(x_values)  # *pi/180 for plot in degrees
        self.point.set_data(x_values, y_values)

    def live_animate(self, math_func):
        """ Animates the plot_funtion """
        self.func = math_func
        animation = Animator(self.fig, self.plot_function, self.x_min, self.x_max)
        plt.show()

    def simple_plot(self, math_func):
        x_values = np.linspace(self.x_min, self.x_max, 2000+(self.x_max - self.x_min)*100)
        y_values = math_func(x_values)  # *pi/180 for plot in degrees
        self.point.set_data(x_values, y_values)
        plt.show()


def h(t):
    return 3 * np.pi * np.exp(-5 * np.sin(2 * np.pi * t * np.pi / 180))


plot = Plot(0, 1000, -100, 1500, grid_button=True, save_button=False)
plot.simple_plot(h)
