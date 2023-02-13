#THIS PYTHON SCRIPT ALLOWS EASY VIEWING OF THE DATA/
#EDIT IT SO IT TRIMS AWAY THE RIGHT ROWS AND COLUNMS.


#

# setting all the vals UPDATED VERSION HERE. 
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
#airspeed_adjusted = 11
#s1_command_scaled = 12
#estimated_altitude = 13
#altitudeTypeDataLog = 14
#AccX = 15

#OLD ONE HERE. REDEFINED
airspeed_adjusted = 11
estimated_altitude = 12
altitudeTypeDataLog = 13

import time
import csv
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.widgets import Cursor







raw_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_raw_input.csv"
trimmed_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filtered_file.csv"

#FILTERS DATA AND WRITES TO TRIMMED FILE.
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
        elif float(row[airspeed_adjusted]) >= 4.0:
            # write row to the output file
            output_writer.writerow(row)
    
        # progress check:
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")




#READS THE TRIMMED FILE AND GRAPHS IT
# Read the csv file
df = pd.read_csv(trimmed_file)
    
# Extract the time column
time = df.iloc[:,0]

#extract the colunms
flight_phase_column = df.iloc[:,flight_phase]
roll_IMU_column = df.iloc[:,roll_IMU]
roll_des_column = df.iloc[:,roll_des]
roll_PID_column = df.iloc[:,roll_PID]
pitch_IMU_column = df.iloc[:,pitch_IMU]
pitch_des_column = df.iloc[:,pitch_des]
pitch_PID_column = df.iloc[:,pitch_PID]
GyroZ_column = df.iloc[:,GyroZ]
yaw_des_column = df.iloc[:,yaw_des]
yaw_PID_column = df.iloc[:,yaw_PID]
airspeed_adjusted_column = df.iloc[:,airspeed_adjusted]
#s1_command_scaled_column = df.iloc[:,s1_command_scaled] REINTRODUCE WITH NEW DATA
estimated_altitude_column = df.iloc[:,estimated_altitude]
altitudeTypeDataLog_column = df.iloc[:,altitudeTypeDataLog]
#AccX_column = df.iloc[:,AccX]

# Create subplots for the two charts
# OOH for flight phase: instead of having it on a chart, what if the background for the orientation stuff changes color when the flght phase changes, and PID stuff only shows when DS or stabilized flight
#and for altitude, have the background of the altitude change color based on the altitudetype datalog
fig, (roll, pitch, yaw, speed, altitude) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

# Plot the first chart
roll.plot(time, data1, label="Data 1")
roll.plot(time, data2, label="Data 2")
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