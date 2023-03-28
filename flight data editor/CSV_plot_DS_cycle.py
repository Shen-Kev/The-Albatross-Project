
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

# calculat roll and pitch error columns
roll_error_column = np.full(len(time), np.nan)
pitch_error_column = np.full(len(time), np.nan)
for i in range(len(time)):
    roll_error_column[i] = roll_IMU_column[i] - roll_des_column[i]
    pitch_error_column[i] = pitch_IMU_column[i] - pitch_des_column[i]

# insert Nan values if there is a gap between time values in the data greater than 1 second
for i in range(len(time)):
    if i == 0:
        continue
    if time[i] - time[i-1] > 1:
        time[i] = np.nan

fig, ((roll), (pitch), (yaw), (accel)
      ) = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

# Set individual subplot titles to the left of the plot
#roll.set_title('Roll')
#pitch.set_title('Pitch')
#yaw.set_title('Yaw (Angle Turned Since Cycle Start)')
# so i can compare the accerlation at the same altitude
#accel.set_title('Forward Acceleration')

#make subplot titles smaller
#roll.title.set_size(8)
#pitch.title.set_size(8)
#yaw.title.set_size(8)
#accel.title.set_size(8)

title = "Flight Data Over a Typical Dynamic Soaring Cycle"

fig.suptitle(title, y=0.95)


# plot the charts.

# for roll, if the roll error at this time is greater than 10 degrees, then plot the roll error in red, otherwise plot it in green
roll_column_high_error = np.full(len(time), np.nan)
roll_column_low_error = np.full(len(time), np.nan)

for i in range(len(time)):
    if abs(roll_error_column[i]) > 10:
        roll_column_high_error[i] = roll_IMU_column[i]
    else:
        roll_column_low_error[i] = roll_IMU_column[i]

roll.plot(time, roll_column_high_error, color='red')
roll.plot(time, roll_column_low_error, color='green')
#roll.plot(time, roll_IMU_column, label="Roll IMU", color='blue')
roll.plot(time, roll_des_column, color='orange')
#roll.plot(time, roll_error_column, label="Roll Error", color='purple')


roll.set_ylabel("Roll (deg)")
roll.yaxis.set_major_locator(plt.MultipleLocator(10))
roll.legend()

#same for pitch
pitch_column_high_error = np.full(len(time), np.nan)
pitch_column_low_error = np.full(len(time), np.nan)

for i in range(len(time)):
    if abs(pitch_error_column[i]) > 10:
        pitch_column_high_error[i] = pitch_IMU_column[i]
    else:
        pitch_column_low_error[i] = pitch_IMU_column[i]

pitch.plot(time, pitch_column_high_error, color='red')
pitch.plot(time, pitch_column_low_error, color='green')
#pitch.plot(time, pitch_IMU_column, label="Pitch IMU", color='blue')
pitch.plot(time, pitch_des_column, color='orange')
#pitch.plot(time, pitch_error_column, label="Pitch Error", color='purple')



pitch.set_ylabel("Pitch (deg)")
pitch.yaxis.set_major_locator(plt.MultipleLocator(10))

pitch.legend()

yaw.plot(time, angle_turned_DS_column, color='blue')
yaw.set_ylabel("Yaw (deg)")
yaw.yaxis.set_major_locator(plt.MultipleLocator(45))
yaw.legend()

accel.plot(time, forwardsAcceleration_column, color='purple')
accel.set_ylabel("Forward accel (m/s^2)")
accel.legend()



# Adjust the space between the two charts
fig.tight_layout()
#time
plt.xlabel("Time (s)")
# Display the plot
plt.subplots_adjust(top=0.9)
plt.show()

#print the proportion of the time the roll and pitch error less thatn 10 degrees
print(" ")
print("Proportion of time roll error less than 10 degrees: {}".format(len(roll_column_low_error[~np.isnan(roll_column_low_error)])/len(roll_column_low_error)))
print("Proportion of time pitch error less than 10 degrees: {}".format(len(pitch_column_low_error[~np.isnan(pitch_column_low_error)])/len(pitch_column_low_error)))


'''

#same as the commented out code above, but make each plot a separarte figure
# for roll, if the roll error at this time is greater than 10 degrees, then plot the roll error in red, otherwise plot it in green
roll_column_high_error = np.full(len(time), np.nan)
roll_column_low_error = np.full(len(time), np.nan)

for i in range(len(time)):
    if abs(roll_error_column[i]) > 10:
        roll_column_high_error[i] = roll_IMU_column[i]
    else:
        roll_column_low_error[i] = roll_IMU_column[i]

fig, (roll) = plt.subplots(1, 1, figsize=(10, 10), sharex=True)
roll.set_title('Roll Data Over a Typical Dynamic Soaring Cycle')
roll.plot(time, roll_column_high_error, label="Roll Measured (Abs. Error > 10 deg)", color='red')
roll.plot(time, roll_column_low_error, label="Roll Measured (Abs. Error < 10 deg)", color='green')
#roll.plot(time, roll_IMU_column, label="Roll IMU", color='blue')
roll.plot(time, roll_des_column, label="Roll Setpoint", color='orange')
#roll.plot(time, roll_error_column, label="Roll Error", color='purple')

roll.set_ylabel("deg")
roll.legend()
plt.xlabel("Time (s)")
plt.subplots_adjust(top=0.9)
plt.show()

#same for pitch
pitch_column_high_error = np.full(len(time), np.nan)
pitch_column_low_error = np.full(len(time), np.nan)

for i in range(len(time)):
    if abs(pitch_error_column[i]) > 10:
        pitch_column_high_error[i] = pitch_IMU_column[i]
    else:
        pitch_column_low_error[i] = pitch_IMU_column[i]

fig, (pitch) = plt.subplots(1, 1, figsize=(10, 10), sharex=True)
pitch.set_title('Pitch Data Over a Typical Dynamic Soaring Cycle')
pitch.plot(time, pitch_column_high_error, label="Pitch Measured (Abs. Error > 10 deg)", color='red')
pitch.plot(time, pitch_column_low_error, label="Pitch Measured (Abs. Error < 10 deg)", color='green')
#pitch.plot(time, pitch_IMU_column, label="Pitch IMU", color='blue')
pitch.plot(time, pitch_des_column, label="Pitch Setpoint", color='orange')
#pitch.plot(time, pitch_error_column, label="Pitch Error", color='purple')

pitch.set_ylabel("deg")
pitch.legend()
plt.xlabel("Time (s)")
plt.subplots_adjust(top=0.9)
plt.show()

fig, (yaw) = plt.subplots(1, 1, figsize=(10, 10), sharex=True)
yaw.set_title('Yaw Data Over a Typical Dynamic Soaring Cycle')
yaw.plot(time, angle_turned_DS_column, color='blue')
yaw.set_ylabel("deg")
yaw.legend()
plt.xlabel("Time (s)")
plt.subplots_adjust(top=0.9)
plt.show()

fig, (accel) = plt.subplots(1, 1, figsize=(10, 10), sharex=True)
accel.set_title('Forward Acceleration Over a Typical Dynamic Soaring Cycle')
accel.plot(time, forwardsAcceleration_column, color='purple')
accel.set_ylabel("m/s^2")
accel.legend()
plt.xlabel("Time (s)")
plt.subplots_adjust(top=0.9)
plt.show()

#print the proportion of the time the roll and pitch error less thatn 10 degrees
print(" ")
print("Proportion of time roll error less than 10 degrees: {}".format(len(roll_column_low_error[~np.isnan(roll_column_low_error)])/len(roll_column_low_error)))
print("Proportion of time pitch error less than 10 degrees: {}".format(len(pitch_column_low_error[~np.isnan(pitch_column_low_error)])/len(pitch_column_low_error)))

'''