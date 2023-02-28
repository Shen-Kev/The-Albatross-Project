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
raw_file_notDS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardsAccelDataNotDSraw.csv"

# all the accel data (multiple flights) for when doing DS through the ground shear layer with wind
raw_file_DS = "C:/Users/kshen/OneDrive/Documents/PlatformIO/Projects/The Albatross Project PlatformIO/flight data editor/forwardsAccelDataDSraw.csv"

# NOT DS DATA ANALYSIS STARTS HERE=========================================================================

# Read the csv file
degrees_of_freedom = pd.read_csv(raw_file_notDS)

# Extract the average accel column
notDS_accelValues = degrees_of_freedom.iloc[:, 2]

# make normal probability plot
stats.probplot(notDS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Control Data Without Trimming")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
plt.show()

# remove outliers based on how far away it is from the mean
notDS_accelValues = notDS_accelValues[abs(
    notDS_accelValues - notDS_accelValues.mean()) <= (2*notDS_accelValues.std())]

# make normal probability plot
stats.probplot(notDS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of Control Data With 2 Standard Deviation Trimmed")
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
degrees_of_freedom = pd.read_csv(raw_file_DS)

# Extract the average accel column
DS_accelValues = degrees_of_freedom.iloc[:, 2]

# make normal probability plot
stats.probplot(DS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of DS data Without Trimming")
plt.legend(['Data Points', 'Normal Distribution With Mean and Std of Data'])
plt.show()

# remove outliers based on how far away it is from the mean
DS_accelValues = DS_accelValues[abs(
    DS_accelValues - DS_accelValues.mean()) <= (2*DS_accelValues.std())]

# make normal probability plot
stats.probplot(DS_accelValues, dist="norm", plot=plt)
plt.title("Normal Probability Plot of DS data With 2 Standard Deviation Trimmed")
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
t_critical = -2.38  # for alpha = 0.01, df  of 71, and 1 tail test

degrees_of_freedom = len(DS_accelValues) + len(notDS_accelValues) - 2

cohens_d = abs(np.mean(DS_accelValues) - np.mean(notDS_accelValues)) / np.sqrt(((len(DS_accelValues)-1)*np.var(
    DS_accelValues, ddof=1) + (len(notDS_accelValues)-1)*np.var(notDS_accelValues, ddof=1)) / degrees_of_freedom)
alpha = 0.01
nobs1 = len(DS_accelValues)
nobs2 = len(notDS_accelValues)
ratio = nobs2 / nobs1
noncen = cohens_d * np.sqrt(nobs1*nobs2 / (nobs1 + nobs2))
power = 1 - stats.nct.cdf(stats.t.ppf(1-alpha,
                          df=degrees_of_freedom), df=degrees_of_freedom, nc=noncen)
type1error = alpha
type2error = 1 - power
beta = 1 - power

# T TEST ENDS HERE========================================================================================

# MAKE HISTOGRAMS AND GRAPH STARTS HERE=====================================================================

plt.title('Control (No Wind)')
plt.xlabel('Forwards Acceleration (m/s^2)')
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
plt.text(0.5, 0.95, "n = " + str(len(notDS_accelValues)) + " mean = " + str(mean_notDS_2sf) + " std = " +
         str(std_notDS_2sf),  horizontalalignment='center', verticalalignment='baseline', transform=plt.gca().transAxes)
plt.show()


plt.title('Dynamic Soaring')
plt.xlabel('Forwards Acceleration (m/s^2)')
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
plt.text(0.5, 0.95, "n = " + str(len(DS_accelValues)) + " mean = " + str(mean_DS_2sf) + " std = " +
         str(std_DS_2sf),  horizontalalignment='center', verticalalignment='baseline', transform=plt.gca().transAxes)
plt.show()

#plot both the DS and not DS histograms on the same graph
plt.title('Control (No Wind) and Dynamic Soaring')
plt.xlabel('Forwards Acceleration (m/s^2)')
plt.ylabel('Probability Density')
plt.hist(notDS_accelValues, bins=binsNum_notDS,
            density=True, alpha=0.6, color='g')
plt.hist(DS_accelValues, bins=binsNum_DS,
            density=True, alpha=0.6, color='b')
plt.plot(x_notDS, p_notDS, linewidth=2, color='g')
plt.plot(x_DS, p_DS, linewidth=2, color='b')
plt.show()



# also plot the difference between the two normal distributions DS and Not DS on the 2,1,2 subplot
plt.title("Difference between Dynamic Soaring and Control")
# plot the difference between the two normal distributions
mean_difference = mean_DS - mean_notDS
std_difference = np.sqrt(
    ((std_DS**2)/len(DS_accelValues)) + ((std_notDS**2) / len(notDS_accelValues)))
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
plt.xlabel('Difference in Forwards Acceleration (m/s^2)')
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
print("Index, Value (mean m/s^2 forwards)")
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
print("t = " + str(t))
print("t critical = " + str(t_critical))
print("df = " + str(degrees_of_freedom))
print("P-value = " + str(p))
print("alpha = " + str(alpha))
print("type 1 error = " + str(type1error))
print("type 2 error = " + str(type2error))
print("beta = " + str(beta))
print("power = " + str(power))
print(" ")
print(" ")
print(" ")
