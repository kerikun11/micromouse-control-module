#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from matplotlib.ticker import ScalarFormatter
import numpy as np
import matplotlib.pyplot as plt

# ============================================================================ #
# prepare figure
fig_t, ax_t = plt.subplots(4, 1, figsize=(6, 8))

# ============================================================================ #
# plot
filebase = f'./accel'  # t,j,a,v,x
for i in range(8):
    raw = np.loadtxt(f"{filebase}_{i}.csv", delimiter=',', ndmin=2)
    if raw.size == 0:
        raw = np.empty(shape=(0, 5))
    t = raw[:, 0]
    value = raw[:, 1:1+4]
    # theta
    for k in range(4):
        ax_t[k].plot(t, value[:, k], lw=4)

# ============================================================================ #
# t style
ylabels = ['jerk [m/s/s/s]', 'accel. [m/s/s]',
           'velocity [m/s]', 'position [m]']
titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
for i, ax in enumerate(ax_t):
    ax.grid(which='both')
    ax.set_ylabel(ylabels[i])
    ax.set_title(titles[i])
for ax in ax_t[0:-1]:
    ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
    ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))
ax_t[-1].set_xlabel('time [s]')

# ============================================================================ #
# fit
fig_t.tight_layout()

# ============================================================================ #
# save
for ext in ['.png', '.svg']:
    fig_t.savefig(filebase + '_t' + ext)

# ============================================================================ #
# show
plt.show()
