import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import mpl_toolkits.axes_grid1

class Player(FuncAnimation):
    def __init__(self, fig, func, frames=None, init_func=None, fargs=None,
                 save_count=None, mini=0, maxi=100, pos=(0.125, 0.92), **kwargs):
        self.i = mini
        self.min = mini
        self.max = maxi
        self.runs = True
        self.forwards = True
        self.fig = fig
        self.func = func
        self.setup(pos)
        self.i_list = []
        FuncAnimation.__init__(self,self.fig, self.update, frames=self.play(),
                                           init_func=init_func, fargs=fargs,
                                           save_count=save_count, **kwargs )

    def play(self):
        while self.runs:
            if self.i >= self.max:
                self.stop()
            yield self.i
            self.i += 1

    def start(self):
        self.runs=True
        self.event_source.start()

    def stop(self, event=None):
        self.runs = False
        self.event_source.stop()

    def forward(self, event=None):
        self.forwards = True
        self.start()

    def setup(self, pos):
        playerax = self.fig.add_axes([pos[0],pos[1], 0.64, 0.04])
        divider = mpl_toolkits.axes_grid1.make_axes_locatable(playerax)
        sax = divider.append_axes("right", size="80%", pad=0.05)
        fax = divider.append_axes("right", size="80%", pad=0.05)
        sliderax = divider.append_axes("right", size="500%", pad=0.07)
        self.button_stop = Button(sax, label='$\u25A0$')
        self.button_forward = Button(fax, label='$\u25B6$')
        self.button_stop.on_clicked(self.stop)
        self.button_forward.on_clicked(self.forward)
        self.slider = Slider(sliderax, '',
                                                self.min, self.max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

    def set_pos(self, i):
        self.i = int(self.slider.val)
        self.i_list.append(self.i)
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

def steps(i):
    if i != 0:
        return 100
    else:
        return abs(i*100)

def update(i):
    global x_val, y_val
    x_val = np.linspace(x_min, i, abs(i*100)+20)
    y_val = 3 * np.pi * np.exp(-1 * (5 * np.sin(2 * np.pi * x_val * np.pi / 180)))
    point.set_data(x_val, y_val)


ani = Player(fig, update, mini = x_min, maxi=x_max)

plt.show()