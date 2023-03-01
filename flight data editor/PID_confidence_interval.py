#PROBABLY NOT GOING TO USE THIS PROGRAM, BUT KEEPING IT HERE JUST IN CASE

#this program extracts informaion from the CSV file just like CSV_plot_flight_data.py.
#then it calculates the error of the pitch and roll PID loops by finding the difference between the desired and actual pitch and roll.
#then it calculates the 95% confidence interval of the error
#then it plots the data in a histogram, draws a normal distribution over it, and draws the 95% confidence interval on the graph
#this is a PAIRED TEST, because it is the error between the desired and measured at each time step


import scipy.stats as stats
import statistics
import math
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter,
                               AutoMinorLocator)
from scipy.stats import norm
from scipy.stats import t
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
from datetime import datetime


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


raw_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_confidence_raw.csv"
trimmed_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_confidence_filtered.csv"

# FILTERS DATA AND WRITES TO TRIMMED FILE.
with open(raw_file, "r") as input_csv, open(trimmed_file, "w", newline="") as output_csv:
    data = input_csv.readlines()
    rows = len(data)
    input_reader = csv.reader(data)
    output_writer = csv.writer(output_csv)


    # Iterate through the rows in the input file, REMOVE THE ROWS
    for i, row in enumerate(input_reader):
        # change the time from milliseconds to seconds after microcontroller powered on
        row[timeInMillis] = float(row[timeInMillis]) / 1000.0

        # deciding which rows to keep and which to throw away
        if float(row[airspeed_adjusted]) >= 6.0 and float(row[flight_phase]) == 3 and float(row[s1_command_scaled]) == 0.0:
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

#set confidence interval 
confidence_int = 0.9

# calculate the error of the PID loops. Each value is paired with the value in the same row of the other column
roll_error = roll_IMU_column - roll_des_column
pitch_error = pitch_IMU_column - pitch_des_column


# calculate the confidence interval of the error
roll_error_mean = np.mean(roll_error)
roll_error_std = np.std(roll_error)
roll_error_confidence_interval = norm.interval(confidence_int, loc=roll_error_mean, scale=roll_error_std)

pitch_error_mean = np.mean(pitch_error)
pitch_error_std = np.std(pitch_error)
pitch_error_confidence_interval = norm.interval(confidence_int, loc=pitch_error_mean, scale=pitch_error_std)


#remove outliers using IQR method where an outlier is defined as a value that is more than 1.5 times the IQR away from the median
#find the median
median_roll_error = np.median(roll_error)
#find the first and third quartiles
q1_roll_error, q3_roll_error = np.percentile(roll_error, [25, 75])
#find the interquartile range
iqr_roll_error = q3_roll_error - q1_roll_error
#find the upper and lower bounds
lower_bound_roll_error = q1_roll_error - (1.5 * iqr_roll_error)
upper_bound_roll_error = q3_roll_error + (1.5 * iqr_roll_error)
#remove outliers
roll_error = roll_error[roll_error.between(lower_bound_roll_error, upper_bound_roll_error, inclusive=True)]

#remove outliers using IQR method where an outlier is defined as a value that is more than 1.5 times the IQR away from the median
#find the median
median_pitch_error = np.median(pitch_error)
#find the first and third quartiles
q1_pitch_error, q3_pitch_error = np.percentile(pitch_error, [25, 75])
#find the interquartile range
iqr_pitch_error = q3_pitch_error - q1_pitch_error
#find the upper and lower bounds
lower_bound_pitch_error = q1_pitch_error - (1.5 * iqr_pitch_error)
upper_bound_pitch_error = q3_pitch_error + (1.5 * iqr_pitch_error)
#remove outliers
pitch_error = pitch_error[pitch_error.between(lower_bound_pitch_error, upper_bound_pitch_error, inclusive=True)]

#plot the pitch data in a histogram, and draw the 95% confidence interval over it. label graph.
plt.hist(pitch_error, bins=100, density=True, alpha=0.6, color='g')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
#draw the 95% confidence interval as a horizontal bar representing the 95% confidence interval
plt.axvline(x=pitch_error_confidence_interval[0], color='r', linestyle='--')
plt.axvline(x=pitch_error_confidence_interval[1], color='r', linestyle='--')
plt.axvline(x=pitch_error_mean, color='b', linestyle='-')
plt.title("Pitch Error Histogram")
plt.xlabel("Error (degrees)")
plt.ylabel("Frequency")
plt.show()

#plot the roll data in a histogram, and draw the 95% confidence interval over it. label graph.
plt.hist(roll_error, bins=100, density=True, alpha=0.6, color='g')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
#draw the 95% confidence interval as a horizontal bar representing the 95% confidence interval
plt.axvline(x=roll_error_confidence_interval[0], color='r', linestyle='--')
plt.axvline(x=roll_error_confidence_interval[1], color='r', linestyle='--')
plt.axvline(x=roll_error_mean, color='b', linestyle='-')
plt.title("Roll Error Histogram")
plt.xlabel("Error (degrees)")
plt.ylabel("Frequency")
plt.show()


#plot when the pitch error is within 10 deg of the setpoints. make the plot so that each datapoint is represented by a bar that sticks up or down from 0 depending on the error. if it is within 10, make the bar green, and if its outside, make the bar red.
deg_tolerance = 10
plt.bar(pitch_error.index, pitch_error, color=pitch_error.apply(lambda x: 'g' if x < deg_tolerance and x > -deg_tolerance else 'r'))
plt.title("Pitch Error")
plt.xlabel("Index")
plt.ylabel("Error (degrees)")
plt.show()

#calculate what percent of the time the pitch error is within 5 deg of the setpoint
pitch_error_within_tolerance = pitch_error[(pitch_error < deg_tolerance) & (pitch_error > -deg_tolerance)]
pitch_error_within_tolerance = len(pitch_error_within_tolerance)/len(pitch_error)
print("Percent of time pitch error is within 10 deg of setpoint: " + str(pitch_error_within_tolerance))

#same for roll
deg_tolerance = 10
plt.bar(roll_error.index, roll_error, color=roll_error.apply(lambda x: 'g' if x < deg_tolerance and x > -deg_tolerance else 'r'))
plt.title("Roll Error")
plt.xlabel("Index")
plt.ylabel("Error (degrees)")
plt.show()

#calculate what percent of the time the roll error is within 5 deg of the setpoint
roll_error_within_tolerance = roll_error[(roll_error < deg_tolerance) & (roll_error > -deg_tolerance)]
roll_error_within_tolerance = len(roll_error_within_tolerance)/len(roll_error)
print("Percent of time roll error is within 10 deg of setpoint: " + str(roll_error_within_tolerance))




#print statistical information
print(" Statistical information: ")
print("Confidence proportion: " + str(confidence_int))
print ("Confidence interval for Roll: " + str(roll_error_confidence_interval))
print ("Confidence interval for Pitch: " + str(pitch_error_confidence_interval))
print(" Roll error paired mean: ", roll_error_mean)
print(" Roll error standard deviation: ", roll_error_std)
print(" Roll error confidence interval: ", roll_error_confidence_interval)
print("Roll n: " + str(len(roll_error)))
print(" Pitch error paired mean: ", pitch_error_mean)
print(" Pitch error standard deviation: ", pitch_error_std)
print(" Pitch error confidence interval: ", pitch_error_confidence_interval)
print("Pitch n: " + str(len(pitch_error)))
print("NEED TO ROUND TO THE HUNDRETH PLACE.")


