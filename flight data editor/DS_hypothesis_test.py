# this file takes the data form all the flight data tests and finds where the UAV
# is flying straight and level and has 0 throttle to see how quickly it descelerates
# and will be used to compare to the DS data to see if the DS is working


# TO USE THIS CODE: put all the flight data into the input file, one after the other


# NOTE THERE IS AN ISSUE WITH SIGFIGS, IF THERE ARE TRAILING 0 IT WONT SHOW IT, SO BE CAREFUL.


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
now = datetime.now()  # current date and time

# all the accel data (multiple flights) for when doing DS at altitude with little wind
#raw_file_notDS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardAccelDataNotDSraw.csv"
raw_file_notDS = "flight data editor/forwardsAccelDataNotDSraw.csv"
# all the accel data (multiple flights) for when doing DS through the ground shear layer with wind
#raw_file_DS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardAccelDataDSraw.csv"
raw_file_DS = "flight data editor/forwardsAccelDataDSraw.csv"
# NOT DS DATA ANALYSIS STARTS HERE=========================================================================

# Read the csv file
df = pd.read_csv(raw_file_notDS)

# Extract the average accel column
notDS_accelValues = df.iloc[:, 2]

#find n
n_notDS = len(notDS_accelValues) + 1

#print the entire dataset one by one
for i in range(len(notDS_accelValues)):
    #print column number, then the value
    print(i, notDS_accelValues[i])
    
# make normal probability plot
stats.probplot(notDS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Control Data")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
#
plt.show()

#remove outliers using IQR method where an outlier is defined as a value that is more than 1.5 times the IQR away from the median
#find the median
median_notDS = np.median(notDS_accelValues)
#find the first and third quartiles
q1_notDS, q3_notDS = np.percentile(notDS_accelValues, [25, 75])
#find the interquartile range
iqr_notDS = q3_notDS - q1_notDS
#find the upper and lower bounds
lower_bound_notDS = q1_notDS - (1.5 * iqr_notDS)
upper_bound_notDS = q3_notDS + (1.5 * iqr_notDS)

#find number of outliers to remove
#num_outliers_notDS = notDS_accelValues[(notDS_accelValues < lower_bound_notDS) | (notDS_accelValues > upper_bound_notDS)]

#remove outliers
#notDS_accelValues = notDS_accelValues[notDS_accelValues.between(lower_bound_notDS, upper_bound_notDS, inclusive=True)]

# make normal probability plot
stats.probplot(notDS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Control Data")
#label x and y axes
plt.xlabel("Z-Score")
plt.ylabel("Forward Acceleration (m/s^2)")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
plt.show()


# find the mean, standard deviation of the array
mean_notDS = np.mean(notDS_accelValues)
std_notDS = np.std(notDS_accelValues)

binsSize = 0.01
range_notDS = max(notDS_accelValues) - min(notDS_accelValues)
binsNum_notDS = range_notDS / binsSize
binsNum_notDS = int(abs(binsNum_notDS))

# NOT DS DATA ANALYSIS ENDS HERE=============================================================================

# DA DATA ANALYSIS STARTS HERE===============================================================================

# Read the csv file
df = pd.read_csv(raw_file_DS)

# Extract the average accel column
DS_accelValues = df.iloc[:, 2]

#find n
n_DS = len(DS_accelValues) + 1

#print the entire dataset one by one
for i in range(len(DS_accelValues)):
    #print column number, then the value
    print(i, DS_accelValues[i]) 

# make normal probability plot
stats.probplot(DS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Dynamic Soaring Data")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
plt.show()

#remove outliers using IQR method where an outlier is defined as a value that is more than 1.5 times the IQR away from the median
#find the median
median_DS = np.median(DS_accelValues)
#find the first and third quartiles
q1_DS, q3_DS = np.percentile(DS_accelValues, [25, 75])
#find the interquartile range
iqr_DS = q3_DS - q1_DS
#find the upper and lower bounds
lower_bound_DS = q1_DS - (1.5 * iqr_DS)
upper_bound_DS = q3_DS + (1.5 * iqr_DS)

#find number of outliers to remove
#num_outliers_DS = DS_accelValues[(DS_accelValues < lower_bound_DS) | (DS_accelValues > upper_bound_DS)]

#remove outliers
#DS_accelValues = DS_accelValues[DS_accelValues.between(lower_bound_DS, upper_bound_DS, inclusive=True)]

# make normal probability plot
stats.probplot(DS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Dynamic Soaring Data")
#label x and y axes
plt.xlabel("Z-Score")
plt.ylabel("Forward Acceleration (m/s^2)")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
plt.show()

# find the mean, standard deviation of the array
mean_DS = np.mean(DS_accelValues)
std_DS = np.std(DS_accelValues)

range_DS = max(DS_accelValues) - min(DS_accelValues)
binsNum_DS = range_DS / binsSize
binsNum_DS = int(abs(binsNum_DS))

# DS DATA ANALYSIS ENDS HERE=================================================================================


# T TEST STARTS HERE======================================================================================
# two sample t test
t, p = stats.ttest_ind(notDS_accelValues, DS_accelValues,
                       equal_var=False, nan_policy='omit', alternative='less')
alpha = 0.01
t_critical = -2.39  # for alpha = 0.01, df  of 60

#degrees_of_freedom = n_DS + n_notDS - 2

#degrees of freedom equal to the lesser of the two sample sizes minus 1
#degrees_of_freedom = min(n_DS, n_notDS) - 1


#degrees of freedom
#df = (VDS + VC)2/(VDS/(nDS-1) + 
#VC/(nC-1)) 
#where VDS = sDS / nDS 
#and VC = sC / nC

degrees_of_freedom = (std_DS**2 / n_DS + std_notDS**2 / n_notDS)**2 / ((std_DS**2 / n_DS)**2 / (n_DS - 1) + (std_notDS**2 / n_notDS)**2 / (n_notDS - 1))

# T TEST ENDS HERE========================================================================================

# MAKE HISTOGRAMS AND GRAPH STARTS HERE=====================================================================

plt.title('Control (No Wind)')
plt.xlabel('Forward Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.hist(notDS_accelValues, bins=binsNum_notDS,
         density=True, alpha=0.6, color='g')
xmin_notDS, xmax_notDS = plt.xlim()
xmin_notDS -= 0.1
xmax_notDS += 0.1
x_notDS = np.linspace(xmin_notDS, xmax_notDS, 100)
p_notDS = norm.pdf(x_notDS, mean_notDS, std_notDS)
plt.plot(x_notDS, p_notDS, linewidth=2, color='g')
# plot n, std, mean underneath this subplot.
# mean with 2 significant figures
mean_notDS_2sf = round(
    mean_notDS, -int(math.floor(math.log10(abs(mean_notDS))) - 1))
# std with 2 significant figures
std_notDS_2sf = round(
    std_notDS, -int(math.floor(math.log10(abs(std_notDS))) - 1))
plt.text(0.5, 0.95, "n = " + str(n_notDS) + " mean = " + str(mean_notDS_2sf) + " std = " +
         str(std_notDS_2sf),  horizontalalignment='center', verticalalignment='baseline', transform=plt.gca().transAxes)
plt.show()


plt.title('Dynamic Soaring')
plt.xlabel('Forward Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.hist(DS_accelValues, bins=binsNum_DS,
         density=True, alpha=0.6, color='b')
xmin_DS, xmax_DS = plt.xlim()
xmin_DS -= 0.1
xmax_DS += 0.1
x_DS = np.linspace(xmin_DS, xmax_DS, 100)
p_DS = norm.pdf(x_DS, mean_DS, std_DS)
plt.plot(x_DS, p_DS, linewidth=2, color='b')
# plot n, std, mean underneath this subplot.
# mean with 2 significant figures
mean_DS_2sf = round(mean_DS, -int(math.floor(math.log10(abs(mean_DS))) - 1))
# std with 2 significant figures
std_DS_2sf = round(std_DS, -int(math.floor(math.log10(abs(std_DS))) - 1))
plt.text(0.5, 0.95, "n = " + str(n_DS) + " mean = " + str(mean_DS_2sf) + " std = " +
         str(std_DS_2sf),  horizontalalignment='center', verticalalignment='baseline', transform=plt.gca().transAxes)
plt.show()

#plot both the DS and not DS histograms on the same graph
plt.title('Histogram of Control and Dynamic Soaring Data')
plt.xlabel('Forward Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.hist(notDS_accelValues, bins=binsNum_notDS,
            density=True, alpha=0.6, color='g')
plt.hist(DS_accelValues, bins=binsNum_DS,
            density=True, alpha=0.6, color='b')
plt.plot(x_notDS, p_notDS, linewidth=2, color='g')
plt.plot(x_DS, p_DS, linewidth=2, color='b')
#create legend
plt.legend(['Control', 'Dynamic Soaring'])
plt.show()



# also plot the difference between the two normal distributions DS and Not DS on the 2,1,2 subplot
plt.title("Difference between Dynamic Soaring and Control")
# plot the difference between the two normal distributions
mean_difference = mean_DS - mean_notDS
std_difference = np.sqrt(
    ((std_DS**2)/n_DS) + ((std_notDS**2) / n_notDS))
x_difference = np.linspace(-20*std_difference+mean_difference,
                           3*std_difference+mean_difference, 500)
p_difference = norm.pdf(x_difference, mean_difference, std_difference)
plt.plot(x_difference, p_difference, linewidth=2, color='b')
# fill in the area under the curve to the left of the cutoff
plt.fill_between(x_difference, p_difference, where=x_difference <
                 t_critical*std_difference+mean_difference, color='r', alpha=1)
plt.plot(x_difference, p_difference, linewidth=2, color='r')
# plot n, std, mean underneath this subplot.\
# mean with 2 significant figures
# NOTE: ONLY WORKS FOR NON ZERO NUMBERS
mean_difference_2sf = round(
    mean_difference, -int(math.floor(math.log10(abs(mean_difference))) - 1))
# std with 2 significant figures
std_difference_2sf = round(
    std_difference, -int(math.floor(math.log10(abs(std_difference))) - 1))
plt.text(0.5, 0.95, "mean = " + str(mean_difference_2sf) + " std = " + str(std_difference_2sf),
         horizontalalignment='center', verticalalignment='baseline', transform=plt.gca().transAxes)
# plot the line on the differnce plot to show z score of t_critical
plt.axvline(x=t_critical*std_difference +
            mean_difference, color='k', linestyle='--')
# label the line
plt.text(t_critical*std_difference+mean_difference, 0.1,
         '  Left Tail Test Cutoff', rotation=90, color='k')
# draw a line at the null hypotehsis for the difference
plt.axvline(x=0, color='k', linestyle='--')
# label the line
plt.text(0, 0.1, '  Null Hypothesis', rotation=90, color='k')
# plot axies labels
plt.xlabel('Difference in Forward Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.show()

# MAKE HISTOGRAMS AND GRAPH ENDS HERE=======================================================================

# print the statistical information
# set the maximum number of rows to display
pd.set_option('display.max_rows', None)
print(" ")
print("Dynamic Soaring Dataset (each datapoint is one dynamic soaring cycle):")
print("Index, Value")
print(DS_accelValues.to_string(name=False))
print(" ")
# set the maximum number of rows to display
pd.set_option('display.max_rows', None)
print("Control Dataset (each datapoint is one circle with no wind):")
print("Index, Value (mean m/s^2 forward)")
print(notDS_accelValues.to_string(name=False))
print(" ")
print("Unfiltered Statistical Information:")
print(" ")
print("mean not DS (not rounded to sig fig) = " + str(mean_notDS))
print("std not DS (not rounded to sig fig) = " + str(std_notDS))
print("mean DS(not rounded to sig fig) = " + str(mean_DS))
print("std DS (not rounded to sig fig) = " + str(std_DS))
print("mean difference (not rounded to sig fig) = " + str(mean_difference))
print("std difference (not rounded to sig fig) = " + str(std_difference))
print("size of not DS array = " + str(n_notDS))
print("size of DS array = " + str(n_DS))
print("t = " + str(t))
print("t critical = " + str(t_critical))
print("df = " + str(degrees_of_freedom))
print("p val = " + str(p))
#print IQR, Q1, Q3, median, oulier bounds, number of datapoints removed for control and for DS. Use variables already generated in this script
print("IQR not DS = " + str(iqr_notDS))
print("Q1 not DS = " + str(q1_notDS))
print("Q3 not DS = " + str(q3_notDS))
print("median not DS = " + str(median_notDS))
print("lower outlier bound not DS = " + str(lower_bound_notDS))
print("upper outlier bound not DS = " + str(upper_bound_notDS))
print("IQR DS = " + str(iqr_DS))
print("Q1 DS = " + str(q1_DS))
print("Q3 DS = " + str(q3_DS))
print("median DS = " + str(median_DS))
print("lower outlier bound DS = " + str(lower_bound_DS))
print("upper outlier bound DS = " + str(upper_bound_DS))

#make boxplot of the two datasets
plt.title("Boxplot of Dynamic Soaring and Control")
plt.ylabel('Forward Acceleration (m/s^2)')
plt.boxplot([DS_accelValues, notDS_accelValues], labels=['Dynamic Soaring', 'Control'], widths=(0.7, 0.7))
plt.show()



print(" ")
print(" ")
print(" ")
