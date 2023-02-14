import matplotlib.pyplot as plt
import time

fig, altitude = plt.subplots()

altitudeTypeDataLog = [1, 2, 3, 4, 5]  # example array of values

for i in range(len(time)):
    altitude.plot(time[i], estimated_altitude_column[i], color='blue', marker='o', markersize=3, linestyle='--',
                  label='Altitude')

    # check the value of altitudeTypeDataLog at each index and set the background color accordingly
    if altitudeTypeDataLog[i] == 1:
        altitude.set_facecolor('red')
    elif altitudeTypeDataLog[i] == 2:
        altitude.set_facecolor('blue')
    elif altitudeTypeDataLog[i] == 3:
        altitude.set_facecolor('green')
    else:
        altitude.set_facecolor('gray')

altitude.set_ylabel("altitude (m)", color='blue')
altitude.tick_params(axis='y', labelcolor='blue')

ax3 = altitude.twinx()

ax3.plot(time, AccX_column, color='green')  # this is in m/s^2
ax3.set_ylabel("forward acceleration (m/s^2)", color='green')
ax3.tick_params(axis='y', labelcolor='green')

altitude.set_xlabel("time(s)")
altitude.legend()

plt.show()
