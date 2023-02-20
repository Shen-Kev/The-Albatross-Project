import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a 2D array representing the background color for each x value
colors = np.sin(x[np.newaxis, :])

fig, ax = plt.subplots()

# Plot the background colors using imshow
ax.imshow(colors, extent=(x[0], x[-1], y.min(), y.max()), aspect='auto', cmap='coolwarm')

# Plot the data
ax.plot(x, y)

# Turn off the axis labels and tick marks
ax.set_xticks([])
ax.set_yticks([])
ax.set_xlabel("")
ax.set_ylabel("")

plt.show()
