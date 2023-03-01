
# setting all the vals UPDATED VERSION HERE.
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter,
                               AutoMinorLocator)
from scipy.stats import norm
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
from datetime import datetime
now = datetime.now()  # current date and time


timeInMillis = 0
flight_phase = 1
roll_IMU = 2
roll_des = 3
aileron_command_PWM = 4
pitch_IMU = 5
pitch_des = 6
elevator_command_PWM = 7
angle_turned_DS = 8
rudder_command_PWM = 9
airspeed_adjusted = 10
s1_command_scaled = 11
forwardsAcceleration = 12


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
        elif float(row[airspeed_adjusted]) >= 5.0 and float(row[flight_phase]) == 3 and float(row[s1_command_scaled]) == 0.0:
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
elevator_command_column = df.iloc[:, elevator_command_PWM]

angle_turned_DS_column = df.iloc[:, angle_turned_DS]
rudder_command_column = df.iloc[:, rudder_command_PWM]

airspeed_adjusted_column = df.iloc[:, airspeed_adjusted]
s1_command_scaled_column = df.iloc[:, s1_command_scaled]
forwardsAcceleration_column = df.iloc[:, forwardsAcceleration]

#insert Nan values if there is a gap between time values in the data greater than 1 second
for i in range(len(time)):
    if i == 0:
        continue
    if time[i] - time[i-1] > 1:
        time[i] = np.nan

fig, ((roll), (pitch), (yaw), (accel)
      ) = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

# Set individual subplot titles to the left of the plot
roll.set_title('Roll')
pitch.set_title('Pitch')
yaw.set_title('Yaw')
# so i can compare the accerlation at the same altitude
accel.set_title('Forwards Acceleration')

title = "Dynamic Soaring Flight Data"

fig.suptitle(title, y=0.95)


# plot the charts
roll.plot(time, roll_IMU_column, label="Roll IMU", color='red')
roll.set_ylabel("deg")
roll.legend()

pitch.plot(time, pitch_IMU_column, label="Pitch IMU", color='green')

pitch.set_ylabel("deg")
pitch.legend()

yaw.plot(time, angle_turned_DS_column, label="Angle Turned", color='blue')
yaw.set_ylabel("deg")
yaw.legend()

accel.plot(time, forwardsAcceleration_column, label="Forwards Acceleration", color='purple')
accel.set_ylabel("m/s^2")
accel.legend()

# Adjust the space between the two charts
fig.tight_layout()

# Display the plot
plt.subplots_adjust(top=0.9)
plt.show()

