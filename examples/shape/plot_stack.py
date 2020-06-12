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
fig_xy, ax_xy = plt.subplots(figsize=(4, 4))

# ============================================================================ #
# plot
for s in [1, 2, 3, 4]:
    filebase = f'./shape/shape_{s}'
    raw = np.empty(shape=(0, 13))
    for i in range(5):
        # csv: t,dddth,ddth,dth,th,dddx,ddx,dx,x,dddy,ddy,dy,y
        raw_part = np.loadtxt(f"{filebase}_{i}.csv", delimiter=',')
        if raw_part.size == 0:
            continue
        raw = np.vstack([raw, raw_part])
    t = raw[:, 0]
    th = raw[:, 1:1+4]
    th = th / np.pi * 180  # rad -> degree
    x = raw[:, 5:5+4]
    y = raw[:, 9:9+4]
    ax_xy.plot(x[:, -1], y[:, -1], lw=3)

# ============================================================================ #
# xy style
ax = ax_xy
ax.set_title('Slalom Shape')
ax.set_xticks(np.arange(-360, 360, 45))
ax.set_xticks(np.arange(-360, 360, 5), minor=True)
ax.set_yticks(ax.get_xticks())
ax.set_yticks(ax.get_xticks(minor=True), minor=True)
ax.axis('equal')
ax.grid(which='major', linestyle='-')
ax.grid(which='minor', linestyle=':')
ax.legend(['45', '90', '135', '180'])

# ============================================================================ #
# fit
fig_xy.tight_layout()

# ============================================================================ #
# save
for ext in ['.png', '.svg']:
    fig_xy.savefig(filebase + '_stack' + ext)

# ============================================================================ #
# show
plt.show()
