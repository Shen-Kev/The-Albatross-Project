# this file takes the data form all the flight data tests and finds where the UAV
# is flying straight and level and has 0 throttle to see how quickly it descelerates
# and will be used to compare to the DS data to see if the DS is working


# TO USE THIS CODE: put all the flight data into the input file, one after the other


import scipy.stats as stats
import statistics
import math
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

# all the flight data (multiple flights) for the forwards acceleration test without DS
raw_file_notDS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardsAccelDataNotDSraw.csv"

# all the flight data (multiple flights) for the forwards acceleration test with DS
raw_file_DS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardsAccelDataDSraw.csv"


# NOT DS DATA ANALYSIS STARTS HERE

# Read the csv file
df = pd.read_csv(raw_file_notDS)

# Extract the time column
time = df.iloc[:, timeInMillis]  # in millis for this one

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


notDS_accelValues = []
for i in range(len(time)):
    if s1_command_scaled_column[i] == 0 and flight_phase_column[i] == 2 and roll_des_column[i] < 3 and roll_des_column[i] > -3 and pitch_des_column[i] < 3 and pitch_des_column[i] > -3 and airspeed_adjusted_column[i] > 5:
        notDS_accelValues.append(forwardsAcceleration_column[i])

# trim the dataset to have 5% trimmed off the top and bottom
notDS_accelValues.sort()
notDS_accelValues = notDS_accelValues[int(
    len(notDS_accelValues)*0.05):int(len(notDS_accelValues)*0.95)]

# find the mean, standard deviation of the array
mean_notDS = np.mean(notDS_accelValues)
std_notDS = np.std(notDS_accelValues)

binsSize = 0.01
range_notDS = max(notDS_accelValues) - min(notDS_accelValues)
binsNum_notDS = range_notDS / binsSize
binsNum_notDS = int(abs(binsNum_notDS))


# make the histogram for not DS data
plt.subplot(2, 1, 1)

plt.hist(notDS_accelValues, bins=binsNum_notDS,
         density=True, alpha=0.6, color='g')
xmin_notDS, xmax_notDS = plt.xlim()
xmin_notDS -= 0.1
xmax_notDS += 0.1
x_notDS = np.linspace(xmin_notDS, xmax_notDS, 100)
p_notDS = norm.pdf(x_notDS, mean_notDS, std_notDS)
plt.plot(x_notDS, p_notDS, 'k', linewidth=2, color='g')


# NOT DS DATA ANALYSIS ENDS HERE

# DA DATA ANALYSIS STARTS HERE

# Read the csv file
df = pd.read_csv(raw_file_DS)

# Extract the time column
time = df.iloc[:, timeInMillis]  # in millis for this one

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


DS_accelValues = []
for i in range(len(time)):
    # and prob make it so that it only does it when accel in the wind, also like only at low alt
    if flight_phase_column[i] == 1 and airspeed_adjusted_column[i] > 5.0:
        DS_accelValues.append(forwardsAcceleration_column[i])

# trim the dataset to have 5% trimmed off the top and bottom
DS_accelValues.sort()
DS_accelValues = DS_accelValues[int(
    len(DS_accelValues)*0.05):int(len(DS_accelValues)*0.95)]

# find the mean, standard deviation of the array
mean_DS = np.mean(DS_accelValues)
std_DS = np.std(DS_accelValues)

binsSize = 0.01
range_DS = max(DS_accelValues) - min(DS_accelValues)
binsNum_DS = range_DS / binsSize
binsNum_DS = int(abs(binsNum_DS))

plt.subplot(2, 1, 1)
plt.xlabel('Forwards Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.hist(DS_accelValues, bins=binsNum_DS,
         density=True, alpha=0.6, color='b')
xmin_DS, xmax_DS = plt.xlim()
xmin_DS -= 0.1
xmax_DS += 0.1
x_DS = np.linspace(xmin_DS, xmax_DS, 100)
p_DS = norm.pdf(x_DS, mean_DS, std_DS)
plt.plot(x_DS, p_DS, 'k', linewidth=2, color='b')

# DS DATA ANALYSIS ENDS HERE

# two sample t test
t, p = stats.ttest_ind(DS_accelValues, notDS_accelValues, equal_var=False, nan_policy='omit', alternative='greater')
print("t = " + str(t))
print("p = " + str(p))

#determine if the difference is significant with alpha = 0.05
alpha = 0.05
if p < alpha:
    print("The difference is significant")
else: 
    print("The difference is not significant")

#also plot the difference between the two normal distributions DS and Not DS on the 2,1,2 subplot
plt.subplot(2, 1, 2)
#plot the difference between the two normal distributions
mean_difference = mean_DS - mean_notDS
std_difference = np.sqrt(((std_DS**2)/len(DS_accelValues)) + ((std_notDS**2) / len(notDS_accelValues)))
x_difference = np.linspace(-3*std_difference+mean_difference, 3*std_difference+mean_difference, 100)
p_difference = norm.pdf(x_difference, mean_difference, std_difference)
plt.plot(x_difference, p_difference, 'k', linewidth=2, color='b')
#fill in the area under the curve to the left of the cutoff
plt.fill_between(x_difference, p_difference, where=x_difference < -1.96*std_difference+mean_difference, color='r', alpha=1)
plt.plot(x_difference, p_difference, 'k', linewidth=2, color='r')

#plot the line on the differnce plot to show z score of -1.96
plt.axvline(x=-1.96*std_difference+mean_difference, color='k', linestyle='--')
#label the line
plt.text(-1.96*std_difference+mean_difference, 0.1, '  Left Tail Test Cutoff', rotation=90, color = 'k')
#draw a line at the null hypotehsis for the difference
plt.axvline(x=0, color='k', linestyle='--')
#label the line
plt.text(0, 0.1, '  Null Hypothesis', rotation=90, color = 'k')

#plot axies labels
plt.xlabel('Difference in Forwards Acceleration (m/s^2)')
plt.ylabel('Probability Density')


# plot the legend for both subplots
plt.subplot(2, 1, 1)
plt.legend(["Not DS" + " n = " + str(len(notDS_accelValues)), "DS" + " n = " + str(len(DS_accelValues))])


# plot the title
title = "Dynamic Soaring Two Sample T Test"
plt.suptitle(title)
plt.show()


#print all statistical info for all the charts (t value, p value, n, std, mean, df, etc)
print(" ")
print("Not DS n = " + str(len(notDS_accelValues)))
print("Not DS mean = " + str(mean_notDS))
print("Not DS std = " + str(std_notDS))
print("DS n = " + str(len(DS_accelValues)))
print("DS mean = " + str(mean_DS))
print("DS std = " + str(std_DS))
print("t = " + str(t))
print("p = " + str(p))
print("alpha = " + str(alpha))
print("df = " + str(len(notDS_accelValues) + len(DS_accelValues) - 2))
print("mean difference = " + str(mean_difference))
print("std difference = " + str(std_difference))
print("t critical = " + str(-1.96))
