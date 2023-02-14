# THIS PYTHON SCRIPT ALLOWS EASY VIEWING OF THE DATA/
# EDIT IT SO IT TRIMS AWAY THE RIGHT ROWS AND COLUNMS.


# THE FILE FOR DOING DS ANALYSIS WILL BE ADDED LATER. RIGHT NOW ITS JUST SHOWING THE DATA

# setting all the vals UPDATED VERSION HERE.
import pandas as pd
import matplotlib.pyplot as plt
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
AccX = 15



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
        elif float(row[airspeed_adjusted]) >= .0:
            # write row to the output file
            output_writer.writerow(row)

        # progress check:
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")


# READS THE TRIMMED FILE AND GRAPHS IT
# Read the csv file
df = pd.read_csv(trimmed_file)

# Extract the time column
time = df.iloc[:, timeInMillis] #actually is time in seconds now

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
s1_command_scaled_column = df.iloc[:,s1_command_scaled]
estimated_altitude_column = df.iloc[:, estimated_altitude]
altitudeTypeDataLog_column = df.iloc[:, altitudeTypeDataLog]
AccX_column = df.iloc[:,AccX]

# Create subplots for the two charts
# OOH for flight phase: instead of having it on a chart, what if the background for the orientation stuff changes color when the flght phase changes, and PID stuff only shows when DS or stabilized flight
# while in DS, maybe the background color could change at each DS cycle???
# and for altitude, have the background of the altitude change color based on the altitudetype datalog

# roll pitch yaw has the IMU measurement, the desired, the des, the PID. background of fight mode. forwards has accX and airspeed and throttle. background also flight mode. altitude has altitude, background of alititude type datalog

fig, (roll, pitch, yaw, forwards, altitude) = plt.subplots(5, 1, figsize=(10, 10), sharex=True)  # sharex = true sets the time all the s

# plot the charts
roll.plot(time, roll_IMU_column, label="roll IMU")
roll.plot(time, roll_des_column, label="roll setpoint")
roll.plot(time, roll_PID_column, label="roll PID output")
roll.set_ylabel("deg")
roll.legend()
roll.set_title("All Flight Data From Fight #1, 2/11/23")

pitch.plot(time, pitch_IMU_column, label="pitch IMU")
pitch.plot(time, pitch_des_column, label="pitch setpoint")
pitch.plot(time, pitch_PID_column, label="pitch PID output")
pitch.set_ylabel("deg")

pitch.legend()

yaw.plot(time, GyroZ_column, label="yaw IMU (GyroZ)")
yaw.plot(time, yaw_des_column, label="yaw setpoint")
yaw.plot(time, yaw_PID_column, label="yaw PID output")
yaw.set_ylabel("deg")

yaw.legend()

#forwards stuff THE REFERENCE 
ax1 = forwards.twinx()  # create a second y-axis with the same x-axis
forwards.plot(time, s1_command_scaled_column, color='red') #this is from 0-1
forwards.set_ylabel("throttle (0-1)", color='red')
forwards.tick_params(axis='y', labelcolor='red')

ax1.plot(time, airspeed_adjusted_column) #this is in m/s
ax1.set_ylabel("airspeed (m/s)", color='blue')
ax1.tick_params(axis='y', labelcolor='blue')
forwards.legend()

#altitude

ax3 = altitude.twinx()

altitude.plot(time, estimated_altitude_column, color = 'blue')
altitude.set_ylabel("altitude (m)", color = 'blue')
altitude.tick_params(axis='y', labelcolor='blue')

ax3.plot(time, AccX_column, color = 'green') #this is in m/s^2
ax3.set_ylabel("forward acceleration (m/s^2)", color='green')
ax3.tick_params(axis='y', labelcolor='green')

altitude.set_xlabel("time(s)")
altitude.legend()


# Adjust the space between the two charts
fig.tight_layout()

# Display the plot
plt.show()


'''
fig, (roll, pitch, yaw, forwards, altitude) = plt.subplots(
    5, 1, figsize=(10, 10), sharex=True)  # sharex = true sets the time all the s

# plot the charts
roll.plot(time, roll_IMU_column, label="roll IMU")
roll.plot(time, roll_des_column, label="roll setpoint")
roll.plot(time, roll_PID_column, label="roll PID output")
roll.set_ylabel("deg")
roll.set_xlabel("s")
roll.legend()

pitch.plot(time, pitch_IMU_column, label="pitch IMU")
pitch.plot(time, pitch_des_column, label="pitch setpoint")
pitch.plot(time, pitch_PID_column, label="pitch PID output")
pitch.set_ylabel("deg")
pitch.set_xlabel("time(s)")
pitch.legend()

yaw.plot(time, GyroZ_column, label="yaw IMU (GyroZ)")
yaw.plot(time, yaw_des_column, label="yaw setpoint")
yaw.plot(time, yaw_PID_column, label="yaw PID output")
yaw.set_ylabel("deg")
yaw.set_xlabel("time(s)")
yaw.legend()

#for forwards stuff, will need multiple y axis?
forwards.plot(time, s1_command_scaled_column, label="throttle (0-1)") #this is from 0-1
forwards.plot(time, airspeed_adjusted_column, label="airspeed (m/s)") #this is in m/s
forwards.plot(time, AccX_column, label="forward acceleration (m/s^2)") #this is in m/s^2
forwards.set_xlabel("time(s)")
forwards.legend()


#altitude, will need to add background
altitude.plot(time, estimated_altitude_column, label="altitude")
altitude.set_ylabel("m")
altitude.set_xlabel("time(s)")
altitude.legend()


# Adjust the space between the two charts
fig.tight_layout()

plt.title("All Flight Data From Fight #1, 2/11/23")


# Display the plot
plt.show()
'''