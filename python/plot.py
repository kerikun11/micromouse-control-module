#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from matplotlib.ticker import ScalarFormatter
import numpy as np
import matplotlib.pyplot as plt

import ctrl  # calls build/ctrl.so

# prepare AccelDesigner
ad = ctrl.AccelDesigner()
ad.reset(60, 6, 2, 0, 1, 2, 0, 0)

# visualize j, a, v, x
fig, axes = plt.subplots(4, 1, figsize=(6, 8))
titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
ylabels = ['j [m/s/s/s]', 'a [m/s/s]', 'v [m/s]', 'p [m]']

time_stamps = ad.getTimeStamp()
for i in range(len(time_stamps)-1):
    t = np.arange(time_stamps[i], time_stamps[i+1], 1e-3)
    j = np.array([ad.j(tt) for tt in t])
    a = np.array([ad.a(tt) for tt in t])
    v = np.array([ad.v(tt) for tt in t])
    x = np.array([ad.x(tt) for tt in t])
    for i, d in enumerate([j, a, v, x]):
        ax = axes[i]
        ax.plot(t, d, lw=4)
        ax.set_title(titles[i])
        ax.set_ylabel(ylabels[i])
        ax.set_xlabel('Time [s]')
        ax.grid()
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))

plt.tight_layout()
plt.show()
