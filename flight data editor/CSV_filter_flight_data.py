#THIS PYTHON SCRIPT PICKS COLUNMS FROM THE INPUT TO PUT TO THE OUTPUT
#IT ASSUMES THE DATA HAS ALREADY BEEN TRIMMED

import csv
import time

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


input_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filter_input.csv"
output_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filter_output.csv"

# Set the colunms to keep 
column_indices = [timeInMillis, airspeed_adjusted,
                  estimated_altitude, altitudeTypeDataLog]

with open(input_file, "r") as input_csv, open(output_file, "w", newline="") as output_csv:
    # Create CSV readers and writers
    data = input_csv.readlines()
    rows = len(data)
    input_reader = csv.reader(data)
    output_writer = csv.writer(output_csv)


    # Iterate through the rows in the input file, remove the coluns
    for i, row in enumerate(input_reader):
        # write the selected columns to the output file
        output_writer.writerow([row[i] for i in column_indices])

        # progress check:
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")
