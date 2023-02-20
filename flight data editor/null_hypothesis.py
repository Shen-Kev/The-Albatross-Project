#this file takes the data form all the flight data tests and finds where the UAV 
#is flying straight and level and has 0 throttle to see how quickly it descelerates
#and will be used to compare to the DS data to see if the DS is working


#TO USE THIS CODE: put all the flight data into the input file, one after the other







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
DS_pitch_angle = 7
elevator_command_PWM = 8
yaw_IMU = 9
rudder_command_PWM = 10
airspeed_adjusted = 11
s1_command_scaled = 12
forwardsAcceleration = 13
estimated_altitude = 14


raw_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_raw_input.csv"
trimmed_file = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/flight_data_filtered_file.csv"

with open(raw_file, "r") as input_csv, open(trimmed_file, "w", newline="") as output_csv:
    data = input_csv.readlines()
    rows = len(data)
    input_reader = csv.reader(data)
    output_writer = csv.writer(output_csv)

    # Iterate through the rows in the input file, REMOVE THE ROWS
    for i, row in enumerate(input_reader):
        # change the time from milliseconds to seconds after microcontroller powered on
        row[timeInMillis] = float(row[timeInMillis]) / 1000.0
        # if the airspeed is high, include the data because its probably the flight
        if float(row[airspeed_adjusted]) >= 5.0:
            # write row to the output file
            output_writer.writerow(row)

        # progress check:
        print("\rProgress: {}%".format(round((i/rows)*100, 1)), end="")

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
DS_pitch_angle_column = df.iloc[:, DS_pitch_angle]
elevator_command_column = df.iloc[:, elevator_command_PWM]

yaw_IMU_column = df.iloc[:, yaw_IMU]
rudder_command_column = df.iloc[:, rudder_command_PWM]

airspeed_adjusted_column = df.iloc[:, airspeed_adjusted]
s1_command_scaled_column = df.iloc[:, s1_command_scaled]
forwardsAcceleration_column = df.iloc[:, forwardsAcceleration]
estimated_altitude_column = df.iloc[:, estimated_altitude]


# make an array of all the forwards acceleration values when throttle is 0, the roll_des, and _pitch_des are all 0, and the flight phase is 2
noThrottleAccelerationVals = []
for i in range(len(time)):
    if s1_command_scaled_column[i] == 0 and flight_phase_column[i] == 2 and roll_des_column[i] < 3 and roll_des_column[i] > -3 and pitch_des_column[i] < 3 and pitch_des_column[i] > -3:
        noThrottleAccelerationVals.append(forwardsAcceleration_column[i])
# print the array
print(" ")
print("Array of acceleration when throttle is 0: ", noThrottleAccelerationVals)

#trim the dataset to have 5% trimmed off the top and bottom
noThrottleAccelerationVals.sort()
#print the size of the array
print(" ")
print("Size of the array: ", len(noThrottleAccelerationVals))

#trim the array by 5% on each side)
noThrottleAccelerationVals = noThrottleAccelerationVals[int(len(noThrottleAccelerationVals)*0.05):int(len(noThrottleAccelerationVals)*0.95)]
#print the size of the array
print(" ")
print("Size of the array after trimming: ", len(noThrottleAccelerationVals))


# find the mean, standard deviation of the array
mean = np.mean(noThrottleAccelerationVals)
std = np.std(noThrottleAccelerationVals)

binsSize = 0.01
range = max(noThrottleAccelerationVals) - min(noThrottleAccelerationVals)
binsNum = range / binsSize
binsNum = int(abs(binsNum))

# make the histogram
plt.hist(noThrottleAccelerationVals, bins=binsNum,
         density=True, alpha=0.6, color='g')
xmin, xmax = plt.xlim()
xmin-=0.1
xmax+=0.1
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mean, std)
plt.plot(x, p, 'k', linewidth=2)


title = "Forwards Acceleration in Level Flight when Throttle is 0"
subtitle = "5% Trimmed Mean: " + str(round(mean, 5)) + "m/s^2, Std: " + str(
    round(std, 5)) + "m/s^2" + ", n: " + str(len(noThrottleAccelerationVals))
plt.title(subtitle)
plt.suptitle(title)
plt.xlabel('Forwards Acceleration (m/s^2)')
plt.ylabel('Probability (%)')
plt.show()