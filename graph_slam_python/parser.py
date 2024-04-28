import matplotlib.pyplot as plt
import numpy as np

def plot_vector(x, y, theta, length=1):
    # Calculate the end point of the vector
    dx = length * np.cos(theta)
    dy = length * np.sin(theta)

    # Plot the vector
    plt.quiver(float(x), float(y), dx, dy, angles='xy', scale_units='xy', scale=1, color='r', width=0.005)

max_l = 20
r_idx = 0

# Set up the plot
plt.figure()
plt.axis('equal')

# with open("data/input_INTEL.g2o") as f:
with open("output.g2o") as f:
    line = f.readline()
    idx = 0
    while line and r_idx < max_l:
        if idx % 1 != 0:
            line = f.readline()
            idx += 1
            continue
        _, _, x, y, theta = line.split()
        print(x, y, theta)
        plot_vector(x, y, float(theta), 20)
        idx += 1
        r_idx += 1
        line = f.readline()

# Show the plot
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.show()
