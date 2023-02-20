# THIS PYTHON SCRIPT ALLOWS EASY VIEWING OF THE DATA/
# EDIT IT SO IT TRIMS AWAY THE RIGHT ROWS AND COLUNMS.
# NOTE: TO GET HIGH QUALITY OUTPUTS, WHEN IN THE PLOT VIEW, SAVE THE PLOT AS AN SVG, NOT PNG


# TO ADD:
# matplotlib to calculate and draw the stats, like the airspeed difference, the histograms or correlation chats, and infer information like shape of flight path, make animation of the plane tilting up and down in time, multiple charts on the same screen,


# THE FILE FOR DOING DS ANALYSIS WILL BE ADDED LATER. RIGHT NOW ITS JUST SHOWING THE DATA

# setting all the vals UPDATED VERSION HERE.
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
from datetime import datetime
now = datetime.now() # current date and time
from scipy.stats import norm

from matplotlib.ticker import (MultipleLocator, FormatStrFormatter,
                               AutoMinorLocator)

timeInMillis = 0
flight_phase = 1
roll_IMU = 2
roll_des = 3
aileron_command_PWM = 4
pitch_IMU = 5
pitch_des = 6
DS_pitch_angle = 7
elevator_command_PWM = 8
yaw_IMU = 9
rudder_command_PWM = 10
airspeed_adjusted = 11
s1_command_scaled = 12
forwardsAcceleration = 13
estimated_altitude = 14


raw_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_raw_input.csv"
trimmed_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filtered_file.csv"

# FILTERS DATA AND WRITES TO TRIMMED FILE.
with open(raw_file, "r") as input_csv, open(trimmed_file, "w", newline="") as output_csv:
    data = input_csv.readlines()
    rows = len(data)
    input_reader = csv.reader(data)
    output_writer = csv.writer(output_csv)

    # FIRST when iterating through the rows, mark the start of the most recent run so that previous datalogs (if accidentally haven't been wiped, or just had to restart FC) get removed
    prevtime = 0
    mostRecentRunRowIndex = 0
    for i, row in enumerate(input_reader):
        if float(row[timeInMillis]) < prevtime:
            mostRecentRunRowIndex = i
        prevtime = float(row[timeInMillis])

    # reset pointers to run it again
    input_csv.seek(0)
    input_reader = csv.reader(input_csv)

    # Iterate through the rows in the input file, REMOVE THE ROWS
    for i, row in enumerate(input_reader):
        # change the time from milliseconds to seconds after microcontroller powered on
        row[timeInMillis] = float(row[timeInMillis]) / 1000.0

        # deciding which rows to keep and which to throw away
        if i < mostRecentRunRowIndex:  # if the row was generated in a previous run, don't include it by not doing anything
            continue
        # if the airspeed is high, include the data because its probably the flight
        elif float(row[airspeed_adjusted]) >= 5.0:
            # write row to the output file
            output_writer.writerow(row)

        # progress check:
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")


# READS THE TRIMMED FILE AND GRAPHS IT
# Read the csv file
df = pd.read_csv(trimmed_file)

# Extract the time column
time = df.iloc[:, timeInMillis]  # actually is time in seconds now

# extract the colunms
flight_phase_column = df.iloc[:, flight_phase]

roll_IMU_column = df.iloc[:, roll_IMU]
roll_des_column = df.iloc[:, roll_des]
aileron_command_column = df.iloc[:, aileron_command_PWM]

pitch_IMU_column = df.iloc[:, pitch_IMU]
pitch_des_column = df.iloc[:, pitch_des]
DS_pitch_angle_column = df.iloc[:, DS_pitch_angle]
elevator_command_column = df.iloc[:, elevator_command_PWM]

yaw_IMU_column = df.iloc[:, yaw_IMU]
rudder_command_column = df.iloc[:, rudder_command_PWM]

airspeed_adjusted_column = df.iloc[:, airspeed_adjusted]
s1_command_scaled_column = df.iloc[:, s1_command_scaled]
forwardsAcceleration_column = df.iloc[:, forwardsAcceleration]
estimated_altitude_column = df.iloc[:, estimated_altitude]


fig, ((roll, altitude), (pitch, airspeed), (yaw, state)
      ) = plt.subplots(3, 2, figsize=(10, 10), sharex=True)

# Set individual subplot titles
roll.set_title('Roll')
pitch.set_title('Pitch')
yaw.set_title('Yaw')
airspeed.set_title('Throttle and Airspeed')
# so i can compare the accerlation at the same altitude
altitude.set_title('Altitude and Forwards Acceleration')
state.set_title('Flight Phase')


#call the current date from the computer
date = now.strftime("%m/%d/%Y")
title = "All Flight Data. Analyzed on " + date

fig.suptitle(title, y=0.95)


# plot the charts
roll.plot(time, roll_IMU_column, label="Roll IMU", color='red')
roll.plot(time, roll_des_column, label="Roll setpoint", color='blue')
roll.plot(time, aileron_command_column, label="Aileron output", color='green')
roll.set_ylabel("deg")
roll.legend()

pitch.plot(time, pitch_IMU_column, label="Pitch IMU", color='red')
pitch.plot(time, pitch_des_column, label="Pitch setpoint", color='blue')
pitch.plot(time, DS_pitch_angle_column, label="DS pitch angle", color='orange')
pitch.plot(time, elevator_command_column, label="Elevator output", color='green')
pitch.set_ylabel("deg")
pitch.legend()

yaw.plot(time, yaw_IMU_column, label="Yaw IMU", color='red')
yaw.plot(time, rudder_command_column, label="Rudder output", color='green')
yaw.set_ylabel("deg")
yaw.legend()

ax1 = airspeed.twinx()  # create a second y-axis with the same x-axis
airspeed.plot(time, s1_command_scaled_column, label='Throttle', color='red')  # this is from 0-1
airspeed.set_ylabel("0%-100%", color='red')
airspeed.tick_params(axis='y', labelcolor='red')

airspeed.plot([], [], label="Airspeed", color='blue')  # to create the label
ax1.plot(time, airspeed_adjusted_column, label='Airspeed', color='blue')  # this is in m/s
ax1.set_ylabel("m/s", color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
airspeed.legend()

# altitude

ax2 = altitude.twinx()
altitude.plot(time, estimated_altitude_column, label='Altitude', color='blue')
altitude.set_ylabel("m", color='blue')
altitude.tick_params(axis='y', labelcolor='blue')

altitude.plot([], [], label="Forwards Acceleration", color='green')  # to create the label
ax2.plot(time, forwardsAcceleration_column, label='Forwards Acceleration', color='green')  # this is in m/s^2
ax2.set_ylabel("m/s^2", color='green')
ax2.tick_params(axis='y', labelcolor='green')

altitude.legend()

state.plot(time, flight_phase_column, label="Flight Phase", color='purple')
state.set_yticks([1.0, 2.0, 3.0])
state.set_yticklabels(["Manual", "Stabilized", "Dynamic Soaring"])
state.set_xlabel("time(s)")
state.legend()

# Adjust the space between the two charts
fig.tight_layout()

# Display the plot
plt.subplots_adjust(top=0.9)
plt.show()

# make an array of all the forwards acceleration values when throttle is 0, the roll_des, and _pitch_des are all 0, and the flight phase is 2
noThrottleAccelerationVals = []
for i in range(len(time)):
    if s1_command_scaled_column[i] == 0 and flight_phase_column[i] == 2 and roll_des_column[i] < 3 and roll_des_column > -3 and pitch_des_column[i] < 3 and pitch_des_column > -3: #within 3 degrees of 0 is close enough to 0
        noThrottleAccelerationVals.append(forwardsAcceleration_column[i])
#print the array
print(" ")
print("Array of acceleration when throttle is 0: ", noThrottleAccelerationVals)

#find the mean, standard deviation, and 90% confidence interval of the array
mean = np.mean(noThrottleAccelerationVals)
std = np.std(noThrottleAccelerationVals)
confInt = 1.645 * (std / np.sqrt(len(noThrottleAccelerationVals)))

binsSize = 0.01
range = max(noThrottleAccelerationVals) - min(noThrottleAccelerationVals)
binsNum = range / binsSize
binsNum = int(abs(binsNum))

#make the histogram
plt.hist(noThrottleAccelerationVals, bins = binsNum, density=True, alpha=0.6, color='g')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mean, std)
plt.plot(x, p, 'k', linewidth=2)
title = "Forwards Acceleration in Level Flight and when Throttle is 0. "
#put the number of samples, mean, and standard deviation in the title
title = title + "mean: " + str(round(mean,5)) + "m/s^2, s: " + str(round(std,5)) + "m/s^2" + ", n: " + str(len(noThrottleAccelerationVals))
plt.title(title)
plt.xlabel('Forwards Acceleration (m/s^2)')
plt.ylabel('Probability (%)')
plt.show()


'''
# print interesting statistics about the flight
print(" ")
print("Flight time: ", time.max() - time.min(), "s")
print("Max altitude: ", estimated_altitude_column.max(), "m")
print("Max airspeed: ", airspeed_adjusted_column.max(), "m/s")
print("Max forwards acceleration: ", forwardsAcceleration_column.max(), "m/s^2")
print("Max throttle: ", s1_command_scaled_column.max(), "%")
print("Average throttle: ", s1_command_scaled_column.mean(), "%")
print("Average altitude: ", estimated_altitude_column.mean(), "m")
print("Average airspeed: ", airspeed_adjusted_column.mean(), "m/s")
print("Average forwards acceleration: ",
      forwardsAcceleration_column.mean(), "m/s^2")
print("std of altitude: ", estimated_altitude_column.std(), "m")
print("std of airspeed: ", airspeed_adjusted_column.std(), "m/s")
print("std of forwards acceleration: ",
      forwardsAcceleration_column.std(), "m/s^2")
print("std of throttle: ", s1_command_scaled_column.std(), "%")


'''









'''
def getRowNumber(timeInput):
    for i in range(len(time)):
        if time[i] >= timeInput:
            return i

start_time = 250
end_time = 260

start_time_row = getRowNumber(start_time)
end_time_row = getRowNumber(end_time)


#just a test:
print(" ")
print("90% confidence interval for average forwards acceleration between t = ", start_time, " and t = ", end_time, "s: ")
print(" ")
print(forwardsAcceleration_column[time[start_time_row]:time[end_time_row]])

#print("mean: ", forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].mean())
#print("std: ", forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].std())
#print("n: ", len(forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]]))
#print("90% confidence interval: ", forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].mean() - 1.645 * forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].std() / np.sqrt(len(forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]])), forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].mean() + 1.645 * forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]].std() / np.sqrt(len(forwardsAcceleration_column[time[getRowNumber(start_time_row)]:time[getRowNumber(end_time_row)]])))
print(" ")

'''

