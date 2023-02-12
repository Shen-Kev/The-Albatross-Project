# python_live_plot.py

#YOOOOOO THIS WORKS!!!!!!!!!!

import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')


x_values = []
y_values = []

index = count()


def animate(i):
    thelist = [1,2,5,2,3,4]
    nextint = thelist[i]
    x_values.append(next(index))
    y_values.append(nextint)

    if i > 10:
        x_values.pop(0)
        y_values.pop(0)

    plt.cla()
    plt.plot(x_values, y_values)


ani = FuncAnimation(plt.gcf(), animate, 100)


plt.tight_layout()
plt.show()






