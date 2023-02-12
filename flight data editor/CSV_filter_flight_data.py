import csv
import time

#        dataLogArray[currentRow][0] = timeInMillis;
#      dataLogArray[currentRow][2] = roll_IMU;
#     dataLogArray[currentRow][3] = roll_des;
#    dataLogArray[currentRow][4] = roll_PID;
#    dataLogArray[currentRow][5] = pitch_IMU;
#    dataLogArray[currentRow][6] = pitch_des;
#    dataLogArray[currentRow][7] = pitch_PID;
#    dataLogArray[currentRow][8] = GyroZ;
#    dataLogArray[currentRow][9] = yaw_des;
#    dataLogArray[currentRow][10] = yaw_PID;
#    dataLogArray[currentRow][11] = airspeed_adjusted;
#    dataLogArray[currentRow][12] = estimated_altitude;

# will need to extract setpoint data from flight phase and horizontal velocity and position from the flight data

# Set the name of the input and output files
# FIRST IMPORT THE FLIGHT DATA INTO THE "all flight data" FOLDER TO KEEP. EVERYTHING IN THE "flight data editor" FOLDER MUST BE WILLING TO BE SACRAFICED
# COPY THE FLIGHT DATA  INTO THE INPUT FILE, AND ONCE IT IS PUT INTO THE OUTPUT FILE, COPY IT AND STORE IT BACK IN THE "all flight data" FOLDER
#
input_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_example.csv"
output_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_example_filtered.csv"

# Set the indices of the columns to include in the output file
# For example, to include the first, second, and fifth columns, set column_indices = [0, 1, 4]
column_indices = [0, 8, 11]
rows = 0  # inital number of rows

with open(input_file, "r") as input_csv, open(output_file, "w", newline="") as output_csv:
    # Create CSV readers and writers
    print("creating readers")
    data = input_csv.readlines()
    rows = len(data)
    input_reader = csv.reader(data)
    output_writer = csv.writer(output_csv)
    print("rows: ", rows)
    time.sleep(1)

    # FIRST when iterating through the rows, mark the start of the most recent run
    prevtime = 0

    mostRecentRunRowIndex = 0
    for i, row in enumerate(input_reader):
        if float(row[0]) < prevtime:
            mostRecentRunRowIndex = i
        prevtime = float(row[0])

    #reset to run it again
    input_csv.seek(0)
    input_reader = csv.reader(input_csv)

     # Iterate through the rows in the input file, REMOVE THE ROWS
    for i, row in enumerate(input_reader):
      # If the current row is within the specified row range,
      # write the selected columns to the output file
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")
       # whatever condition to accept or reject the row, in this case the airspeed must be above 2. the row number for airsped is 11.
        if i < mostRecentRunRowIndex:  # if the row was generated in a previous run, don't include it by not doing anything
            continue
        elif float(row[11]) >= 2.0:
            output_writer.writerow([row[i] for i in column_indices])
        # also include if the yaw is chnaging a lot
        elif float(row[8]) > 10 or float(row[8]) < 10:
            output_writer.writerow([row[i] for i in column_indices])
