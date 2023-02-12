#THIS PYTHON SCRIPT ALLOWS EASY VIEWING OF THE DATA. NO NEED FOR FILTER DATA ANYMORE, ADJUST WHICH INFORMATION/COLUNMS YOU WANT ON THIS ONE

# setting all the vals
timeInMillis = 0
flight_phase = 1
roll_IMU = 2
roll_des = 3
roll_PID = 4
pitch_IMU = 5
pitch_des = 6
pitch_PID = 7
GyroZ = 8
yaw_des = 9
yaw_PID = 10
airspeed_adjusted = 11
estimated_altitude = 12
altitudeTypeDataLog = 13

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.widgets import Cursor

# Read the csv file
df = pd.read_csv("C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filter_input.csv")

# Extract the time column and the next two columns to plot on the first chart
time = df.iloc[:,0]
data1 = df.iloc[:,1]
data2 = df.iloc[:,2]

# Extract the next three columns to plot on the second chart
data3 = df.iloc[:,3]
data4 = df.iloc[:,4]
data5 = df.iloc[:,5]

# Create subplots for the two charts
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

# Plot the first chart
ax1.plot(time, data1, label="Data 1")
ax1.plot(time, data2, label="Data 2")
ax1.set_ylabel("Value")
ax1.legend()

# Plot the second chart
ax2.plot(time, data3, label="Data 3")
ax2.plot(time, data4, label="Data 4")
ax2.plot(time, data5, label="Data 5")
ax2.set_xlabel("Time")
ax2.set_ylabel("Value")
ax2.legend()

# Adjust the space between the two charts
fig.tight_layout()

# Add the cursor widget
cursor = Cursor(ax1, useblit=True, color='red', linewidth=1)

# Display the plot
plt.show()