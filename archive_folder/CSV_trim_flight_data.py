#THIS PYTHON SCRIPT OUTPUTS THE DATA WHEN THE UAV IS IN FLIGHT, OR WHATVER DATA IS RELEVANT.

#NOT NEEDED ANYMORE. INTEGRATED INTO MAIN CODE.

import csv

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
s1_command_scaled = 12
estimated_altitude = 13
altitudeTypeDataLog = 14


input_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filter_input.csv"
output_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filter_output.csv"


with open(input_file, "r") as input_csv, open(output_file, "w", newline="") as output_csv:
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
