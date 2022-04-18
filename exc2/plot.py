import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider


class Animator(FuncAnimation):
    """ Inherites the capabilities of FuncAnimation with extra features.
    Buttons for start, stop and save figure.
    Slider for quick scroll through x-values.
    Button to save the figure with custom name. """
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
        """ Responsible for what happens between frames. Stops the slider if it reaches x_max. """
        while self.runs:
            if self.i >= self.x_max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self, event=None):
        """ Starts the plot. Replays from start if the plot has reached the end. """
        if self.i < self.x_max:
            self.runs=True
            self.event_source.start()
        if self.i == self.x_max:
            self.i = self.x_min

    def stop(self, event=None):
        """ Stops the plot """
        self.runs = False
        self.event_source.stop()

    def set_pos(self, i):
        """ Gives the command to the given plot_funtion to plot up until the slider value. """
        self.i = int(self.slider.val)
        self.plot_function(self.i)

    def update_slider(self, i):
        """ Updates the slider position as the animation is running. """
        self.slider.set_val(i)


class Plot:
    """ Class for plotting a function """
    def __init__(self, dim, grid_button=True, save_button=True, color_bar=True):
        self.x_min = dim[0]
        self.x_max = dim[1]
        self.y_min = dim[2]
        self.y_max = dim[3]

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)

        if grid_button:
            self.activate_grid_button()

        if save_button:
            self.activate_save_button()

        if color_bar:
            plt.subplots_adjust(bottom=0.2)
            self.activate_color_bar()

    def activate_grid_button(self):
        """ Adds grid on/off button. """
        gridax = plt.axes([0.72, 0.9, 0.08, 0.05])
        self.grid_button = Button(gridax, label='#')
        self.grid_button.on_clicked(self.grid)
        self.n = 1

    def grid(self, event=None):
        """ Adds/turns off grid"""
        self.fig.canvas.draw_idle()
        if self.n % 2:
            self.ax.grid(True)
        else:
            self.ax.grid(False)
        self.n += 1

    def activate_save_button(self):
        """ Adds save button. """
        saveax = plt.axes([0.82, 0.9, 0.08, 0.05])
        self.save_button = Button(saveax, label='Save')
        self.save_button.on_clicked(self.save_figure)
        self.file_index = 1

    def save_figure(self, event=None):
        """ Saves the current figure (current path) with a unique index, ex. "figure5". """
        file_names = os.listdir(os.getcwd())
        indexes = []
        for file_name in file_names:
            if file_name[0:6] == "figure":
                indexes.append(int(file_name[6]))
        if len(indexes) == 0:
            index = "1"
        else:
            index = str(max(indexes) + 1)
        plt.savefig("figure" + index + ".png")

    def activate_color_bar(self):
        """ Adds color-picker-bar """
        blueax = plt.axes([0.25, 0.06, 0.08, 0.05])
        self.blue_button = Button(blueax, label="", color="blue", hovercolor="blue")
        self.blue_button.on_clicked(lambda x: self.change_line_color("blue"))

        greenax = plt.axes([0.45, 0.06, 0.08, 0.05])
        self.green_button = Button(greenax, label='', color="green", hovercolor="green")
        self.green_button.on_clicked(lambda x: self.change_line_color("green"))

        redax = plt.axes([0.65, 0.06, 0.08, 0.05])
        self.red_button = Button(redax, label='', color="red", hovercolor="red")
        self.red_button.on_clicked(lambda x: self.change_line_color("red"))

    def change_line_color(self, color, event=None):
        """ changes line to blue"""
        self.point.set_data([], [])  # Clears current graph
        self.point, = self.ax.plot([], [], color)
        self.point.set_data(self.x_values, self.y_values)

    def plot_function(self, i):
        """ The function to run at each update in the animation """
        self.x_values = np.linspace(self.x_min, i, abs(i * 100) + 2000)
        self.y_values = self.math_func(self.x_values)
        self.point.set_data(self.x_values, self.y_values)

    def live_plot(self, math_func, line_color="blue"):
        """ Animates the plot_funtion with help from the Animator class"""
        self.math_func = math_func
        self.point, = self.ax.plot([], [], color=line_color)
        animation = Animator(self.fig, self.plot_function, self.x_min, self.x_max)
        plt.show()


def h(t):
    return 3 * np.pi * np.exp(-5 * np.sin(2 * np.pi * t * np.pi / 180))


def g(t):
    return 200 * np.sin(t / 20) + t


plot = Plot([-500, 1000, -500, 2000], grid_button=True, save_button=True, color_bar=True)
plot.live_plot(h, "black")
