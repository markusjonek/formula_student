import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider


class Player(FuncAnimation):
    def __init__(self, fig, func, frames=None, init_func=None, fargs=None,
                 save_count=None, mini=0, maxi=100, **kwargs):
        self.i = mini
        self.min = mini
        self.max = maxi
        self.runs = True
        self.forwards = True
        self.fig = fig
        self.func = func
        self.setup_buttons_slider()
        FuncAnimation.__init__(self,self.fig, self.update, frames=self.play(),
                                           init_func=init_func, fargs=fargs,
                                           save_count=save_count, **kwargs )

    def setup_buttons_slider(self):
        startax = plt.axes([0.13, 0.9, 0.08, 0.05])
        stopax = plt.axes([0.24, 0.9, 0.08, 0.05])
        slideax = plt.axes([0.37, 0.9, 0.5, 0.05])
        self.button_stop = Button(stopax, label='$\u25A0$')
        self.button_forward = Button(startax, label='$\u25B6$')
        self.button_stop.on_clicked(self.stop)
        self.button_forward.on_clicked(self.forward)
        self.slider = Slider(slideax, '', self.min, self.max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

    def play(self):
        while self.runs:
            if self.i >= self.max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self):
        if self.i < self.max:
            self.runs=True
            self.event_source.start()
        if self.i == self.max:
            self.i = self.min

    def stop(self, event=None):
        self.runs = False
        self.event_source.stop()

    def forward(self, event=None):
        self.forwards = True
        self.start()

    def set_pos(self, i):
        self.i = int(self.slider.val)
        self.func(self.i)

    def update(self,i):
        self.slider.set_val(i)


x_min = -20
x_max = 60

fig, ax = plt.subplots()
ax.set_xlim(x_min, x_max)
ax.set_ylim(-1.5, 1505)
point, = ax.plot([], [])

x_val = []
y_val = []


def update(i):
    global x_val, y_val
    x_val = np.linspace(x_min, i, abs(i*100)+20)
    y_val = 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * x_val * np.pi / 180)))
    point.set_data(x_val, y_val)


ani = Player(fig, update, mini = x_min, maxi=x_max)

plt.show()