# ============================================================================ #
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--file")
args = parser.parse_args()

# ============================================================================ #
# load csv
filename = './continuous.csv'

if args.file:
    filename = args.file

raw = np.loadtxt(filename, delimiter=',')
t = raw[:, 0]
v = raw[:, 1:5]

# ============================================================================ #
# plot
titles = ['jerk', 'accel', 'velocity', 'position']
ylabels = ['jerk [m/s/s/s]', 'accel [m/s/s]', 'velocity [m/s]', 'position [m]']
for i in range(4):
    plt.subplot(4, 1, 1 + i)
    plt.plot(t, v[:, i])
    plt.ylabel(ylabels[i])
    plt.grid()

plt.xlabel('Time [s]')

plt.tight_layout()
plt.show()
