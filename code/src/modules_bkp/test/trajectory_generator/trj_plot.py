import matplotlib.pyplot as plt
import numpy as np

# Load data from CSV file (assuming space-delimited values)
data = np.loadtxt('/home/ostifede02/itacarobotics/Metis2/code/src/modules/test/trajectory_generator/trj_data.csv', delimiter=' ')

# Extract X, Y, and Z columns
x_vals = data[:, 0]
y_vals = data[:, 1]
z_vals = data[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points in 3D
ax.scatter(x_vals, y_vals, z_vals, label='Trajectory', color='b')

# Add labels and title
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('3D Trajectory Plot')

# Show the plot
plt.legend()
plt.show()
