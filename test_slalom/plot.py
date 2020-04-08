#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from matplotlib.ticker import ScalarFormatter
import numpy as np
import matplotlib.pyplot as plt

# ============================================================================ #
# global settings
plt.rcParams["font.family"] = "IPAGothic"

# ============================================================================ #
# prepare figure
fig_xy, ax_xy = plt.subplots(figsize=(4, 3))
fig_t, ax_t = plt.subplots(4, 1, figsize=(6, 8))

# ============================================================================ #
# plot
for i in range(5):
    # slalom_i.csv
    # t,dddth,ddth,dth,th,dddx,ddx,dx,x,dddy,ddy,dy,y
    raw = np.loadtxt(f"./build/slalom_{i}.csv", delimiter=',')
    t = raw[:, 0]
    th = raw[:, 1:1+4]
    th = th / np.pi * 180  # rad -> degree
    x = raw[:, 5:5+4]
    y = raw[:, 9:9+4]
    # xy
    ax_xy.plot(x[:, -1], y[:, -1], lw=4)
    # theta
    for k in range(4):
        ax_t[k].plot(t, th[:, k], lw=2)

# ============================================================================ #
# xy style
ax = ax_xy
ax.set_xticks(np.arange(-360, 360, 45))
ax.set_xticks(np.arange(-360, 360, 5), minor=True)
ax.set_yticks(ax.get_xticks())
ax.set_yticks(ax.get_xticks(minor=True), minor=True)
ax.axis('equal')
ax.grid(which='major', linestyle='-')
ax.grid(which='minor', linestyle=':')
ax.legend(['straight', 'transition curve',
           'pure arc', 'transition curve', 'straight'])

# ============================================================================ #
# t style
ylabels = ['jerk [deg/s/s/s]', 'accel [deg/s/s]', 'vel [deg/s]', 'pos [deg]']
titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
# ylabels = ['jerk [rad/s/s/s]', 'accel [rad/s/s]', 'vel [rad/s]', 'pos [rad]']
for i, ax in enumerate(ax_t):
    ax.grid(which='both')
    ax.set_ylabel(ylabels[i])
    ax.set_title('Angular ' + titles[i])
# ax_t[0].set_title('Angular Jerk, Acceleration, Velocity, Position')
for ax in ax_t[0:-1]:
    ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
    ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))
ax_t[-1].set_xlabel('time [s]')
ax_t[-1].set_yticks(np.arange(-360, 360, 45))
ax_t[-1].autoscale()

# ============================================================================ #
# save
fig_t.tight_layout()
fig_xy.tight_layout()
for ext in ['.png', '.pdf', '.svg']:
    fig_xy.savefig('build/xy' + ext)
    fig_t.savefig('build/t' + ext)

# ============================================================================ #
# show
plt.show()
