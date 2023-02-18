# THIS PYTHON SCRIPT ALLOWS EASY VIEWING OF THE DATA/
# EDIT IT SO IT TRIMS AWAY THE RIGHT ROWS AND COLUNMS.
# NOTE: TO GET HIGH QUALITY OUTPUTS, WHEN IN THE PLOT VIEW, SAVE THE PLOT AS AN SVG, NOT PNG


#TO ADD:
#matplotlib to calculate and draw the stats, like the airspeed difference, the histograms or correlation chats, and infer information like shape of flight path, make animation of the plane tilting up and down in time, multiple charts on the same screen,


# THE FILE FOR DOING DS ANALYSIS WILL BE ADDED LATER. RIGHT NOW ITS JUST SHOWING THE DATA

# setting all the vals UPDATED VERSION HERE.
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import time

from matplotlib.ticker import (MultipleLocator, FormatStrFormatter,
                               AutoMinorLocator)



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
s1_command_scaled = 12
estimated_altitude = 13
altitudeTypeDataLog = 14
forwardsAcceleration = 15

#raw_file = "flight data editor/flight_data_filtered_file.csv"#"C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_raw_input.csv"
#trimmed_file = "flight data editor/flight_data_filtered_file.csv" #"C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filtered_file.csv"

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
roll_PID_column = df.iloc[:, roll_PID]
pitch_IMU_column = df.iloc[:, pitch_IMU]
pitch_des_column = df.iloc[:, pitch_des]
pitch_PID_column = df.iloc[:, pitch_PID]
GyroZ_column = df.iloc[:, GyroZ]
yaw_des_column = df.iloc[:, yaw_des]
yaw_PID_column = df.iloc[:, yaw_PID]
airspeed_adjusted_column = df.iloc[:, airspeed_adjusted]
s1_command_scaled_column = df.iloc[:, s1_command_scaled]
estimated_altitude_column = df.iloc[:, estimated_altitude]
altitudeTypeDataLog_column = df.iloc[:, altitudeTypeDataLog]
forwardsAcceleration_column = df.iloc[:, forwardsAcceleration]

# Create subplots for the two charts
# OOH for flight phase: instead of having it on a chart, what if the background for the orientation stuff changes color when the flght phase changes, and PID stuff only shows when DS or stabilized flight
# while in DS, maybe the background color could change at each DS cycle???
# and for altitude, have the background of the altitude change color based on the altitudetype datalog

# roll pitch yaw has the IMU measurement, the desired, the des, the PID. background of fight mode. forwards has accX and airspeed and throttle. background also flight mode. altitude has altitude, background of alititude type datalog

fig, ((roll, altitude), (pitch, airspeed), (yaw, state)
      ) = plt.subplots(3, 2, figsize=(10, 10), sharex=True)

# Set individual subplot titles
roll.set_title('Roll')
pitch.set_title('Pitch')
yaw.set_title('Yaw')
airspeed.set_title('Throttle and Airspeed')
altitude.set_title('Altitude and Forwards Acceleration')
state.set_title('State')

# Set a title for the entire figure
fig.suptitle('All Flight Data from Flight #1 on 2/16/23', y=0.95)

# plot the charts
ax5 = roll.twinx()
roll.plot(time, roll_IMU_column, label="Roll IMU", color='red')
roll.plot(time, roll_des_column, label="Roll setpoint", color='blue')
roll.plot([], [], label="Roll PID output",
          color='green')  # to create the label
ax5.plot(time, roll_PID_column, label="Roll PID output", color='green')
ax5.set_ylabel("0-1 roll PID output", color='green')
ax5.tick_params(axis='y', labelcolor='green')

roll.axhspan(0, 1, 20, 25, facecolor='b', alpha=0.2)

roll.set_ylabel("deg")
roll.legend()

ax6 = pitch.twinx()
pitch.plot(time, pitch_IMU_column, label="Pitch IMU", color='red')
pitch.plot(time, pitch_des_column, label="Pitch setpoint", color='blue')
pitch.plot([], [], label="Pitch PID output",
           color='green')  # to create the label
ax6.plot(time, pitch_PID_column, label="Pitch PID output", color='green')
ax6.set_ylabel("0-1 pitch PID output", color='green')
ax6.tick_params(axis='y', labelcolor='green')

pitch.set_ylabel("deg")

pitch.legend()

ax7 = yaw.twinx()
yaw.plot(time, GyroZ_column, label="Yaw IMU (GyroZ)", color='red')
yaw.plot(time, yaw_des_column, label="Yaw setpoint", color='blue')
yaw.plot([], [], label="Yaw PID output", color='green')  # to create the label
ax7.plot(time, yaw_PID_column, label="Yaw PID output", color='green')
ax7.set_ylabel("0-1 yaw PID output", color='green')
ax7.tick_params(axis='y', labelcolor='green')

yaw.set_ylabel("deg/s")
yaw.set_xlabel("time(s)")

yaw.legend()

# forwards stuff THE REFERENCE
ax1 = airspeed.twinx()  # create a second y-axis with the same x-axis
airspeed.plot(time, s1_command_scaled_column, label='Throttle',
              color='red')  # this is from 0-1
airspeed.set_ylabel("0%-100%", color='red')
airspeed.tick_params(axis='y', labelcolor='red')

airspeed.plot([], [], label="Airspeed", color='blue')  # to create the label

ax1.plot(time, airspeed_adjusted_column,
         label='Airspeed', color='blue')  # this is in m/s
ax1.set_ylabel("m/s", color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
airspeed.legend()

# altitude

ax3 = altitude.twinx()

altitude.plot(time, estimated_altitude_column, label='Altitude', color='blue')
altitude.set_ylabel("m", color='blue')
altitude.tick_params(axis='y', labelcolor='blue')

altitude.plot([], [], label="Forwards Acceleration",
              color='green')  # to create the label

ax3.plot(time, forwardsAcceleration_column, label='Forwards Acceleration',
         color='green')  # this is in m/s^2
ax3.set_ylabel("m/s^2", color='green')
ax3.tick_params(axis='y', labelcolor='green')

altitude.legend()

ax4 = state.twinx()
state.plot(time, flight_phase_column, label="Flight Phase", color='purple')
state.set_ylabel('flight phase code', color='purple')
state.tick_params(axis='y', labelcolor='purple')

state.plot([], [],
           label="Altitude Sensor Type", color='orange')

ax4.plot(time, altitudeTypeDataLog_column, color='orange')        
ax4.set_ylabel('altitude sensor code', color='orange')
ax4.tick_params(axis='y', labelcolor='orange')

state.set_xlabel("time(s)")

state.legend()

# Adjust the space between the two charts
fig.tight_layout()


#print interesting statistics about the flight
print(" ")
print("Flight time: ", time.max() - time.min(), "s")
print("Max altitude: ", estimated_altitude_column.max(), "m")
print("Max airspeed: ", airspeed_adjusted_column.max(), "m/s")
print("Max forwards acceleration: ", forwardsAcceleration_column.max(), "m/s^2")
print("Max throttle: ", s1_command_scaled_column.max(), "%")
print("Average throttle: ", s1_command_scaled_column.mean(), "%")
print("Average altitude: ", estimated_altitude_column.mean(), "m")
print("Average airspeed: ", airspeed_adjusted_column.mean(), "m/s")
print("Average forwards acceleration: ", forwardsAcceleration_column.mean(), "m/s^2")
print("std of altitude: ", estimated_altitude_column.std(), "m")
print("std of airspeed: ", airspeed_adjusted_column.std(), "m/s")
print("std of forwards acceleration: ", forwardsAcceleration_column.std(), "m/s^2")
print("std of throttle: ", s1_command_scaled_column.std(), "%")


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
# Display the plot
plt.subplots_adjust(top=0.9)
plt.show()
