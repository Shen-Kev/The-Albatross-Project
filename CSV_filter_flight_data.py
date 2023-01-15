import csv

# this file has been mostly unmodified from template, will need to edit

# Set the name of the input and output files
input_file = "flight_data/flight_data_example.csv" #import the flight data into VS code, and specify the file path in this folder
output_file = "flight_data/flight_data_example_filtered.csv"

# Set the indices of the columns to include in the output file
# For example, to include the first, second, and fifth columns, set column_indices = [0, 1, 4]
column_indices = [0, 1, 4]

# Set the range of rows to include in the output file. maybe only read the rows that have high airspeed?
# For example, to include rows 2 through 10, set row_range = (1, 9)
row_range = (1, 9)

# Open the input and output files
with open(input_file, "r") as input_csv, open(output_file, "w", newline="") as output_csv:
    # Create CSV readers and writers
    input_reader = csv.reader(input_csv)
    output_writer = csv.writer(output_csv)

    # Iterate through the rows in the input file
    for i, row in enumerate(input_reader):
        # If the current row is within the specified row range,
        # write the selected columns to the output file
        if i >= row_range[0] and i <= row_range[1]:
            output_writer.writerow([row[i] for i in column_indices])
