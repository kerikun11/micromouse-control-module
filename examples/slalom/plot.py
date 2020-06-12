#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from matplotlib.ticker import ScalarFormatter
import numpy as np
import matplotlib.pyplot as plt

# ============================================================================ #
# global settings
# plt.rcParams["font.family"] = "IPAGothic"

# ============================================================================ #
# prepare figure
fig_xy, ax_xy = plt.subplots(figsize=(4, 3))
fig_t, ax_t = plt.subplots(4, 1, figsize=(4, 6))

# ============================================================================ #
# plot
filebase = f'./slalom'
for i in range(5):
    # slalom_i.csv
    # t,dddth,ddth,dth,th,dddx,ddx,dx,x,dddy,ddy,dy,y
    raw = np.loadtxt(f"{filebase}_{i}.csv", delimiter=',', ndmin=2)
    if raw.size == 0:
        raw = np.empty(shape=(0, 13))
    t = raw[:, 0]
    th = raw[:, 1:1+4]
    th = th / np.pi * 180  # rad -> degree
    x = raw[:, 5:5+4]
    y = raw[:, 9:9+4]
    # xy
    ax_xy.plot(x[:, -1], y[:, -1], lw=3)
    # theta
    for k in range(4):
        ax_t[k].plot(t, th[:, k], lw=3)

# ============================================================================ #
# xy style
ax = ax_xy
ax.set_title('Slalom Shape')
ax.set_xticks(np.arange(-360, 360, 15))
ax.set_xticks(np.arange(-360, 360, 5), minor=True)
ax.set_yticks(ax.get_xticks())
ax.set_yticks(ax.get_xticks(minor=True), minor=True)
ax.axis('equal')
ax.grid(which='major', linestyle='-')
ax.grid(which='minor', linestyle=':')
ax.legend(['straight before curve', 'transition curve',
           'pure arc', 'transition curve', 'straight after curve'])
ax.set_xlabel('x [mm]')
ax.set_ylabel('y [mm]')
# ax.set_title('')
# ax.axis('off')
# ax.legend().remove()

# ============================================================================ #
# t style
ylabels = ['jerk [deg/s/s/s]', 'accel. [deg/s/s]',
           'velocity [deg/s]', 'position [deg]']
titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
for i, ax in enumerate(ax_t):
    ax.grid(which='both')
    ax.set_ylabel(ylabels[i])
    ax.set_title('Angular ' + titles[i])
for ax in ax_t[0:-1]:
    ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
    ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))
ax_t[-1].set_xlabel('time [s]')
ax_t[-1].set_yticks(np.arange(-360, 360, 45))
ax_t[-1].autoscale()

# ============================================================================ #
# fit
fig_t.tight_layout()
fig_xy.tight_layout()

# ============================================================================ #
# save
for ext in ['.png', '.svg']:
    fig_xy.savefig(filebase + '_xy' + ext)
    fig_t.savefig(filebase + '_t' + ext)

# ============================================================================ #
# show
plt.show()
