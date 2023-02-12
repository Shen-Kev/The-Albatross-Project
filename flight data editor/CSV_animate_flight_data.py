#figure out how to import this stuff later

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Read CSV file
df = pd.read_csv('flight_data/flight_data_example.csv') #GOTTA CHANGE THIS TO THE NEEDED FILE.

# convert the time column to datetime
df['time'] = pd.to_datetime(df['time'],unit='ms')
# set time as index
df.set_index('time',inplace=True)

# Create line plot
fig, ax = plt.subplots()
lines = [ax.plot(df.index, df[col], label=col)[0] for col in df.columns]
ax.legend()
ax.set_xlabel('Time')

# Update plot in real time
def update(num):
    for line, col in zip(lines, df.columns):
        line.set_data(df.index[:num], df[col][:num])
    ax.relim()
    ax.autoscale_view()

ani = animation.FuncAnimation(fig, update, frames=range(1, len(df)), interval=50)
plt.show()
