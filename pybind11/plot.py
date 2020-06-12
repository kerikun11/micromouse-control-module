#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

import ctrl  # calls $PYTHONPATH/ctrl.so


def plot_accel_designer():
    # prepare AccelDesigner
    ad = ctrl.AccelDesigner(j_max=60, a_max=6, v_max=2,
                            v_start=0, v_target=1, dist=2)
    print(ad)

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


def plot_slalom():
    # prepare AccelDesigner
    shape = ctrl.Shape(total=ctrl.Pose(90, 90, math.pi/2), y_curve_end=75)
    print(shape)
    trajectory = ctrl.Trajectory(shape)
    v = shape.v_ref
    trajectory.reset(v, 0, shape.straight_prev / v)
    ad = trajectory.getAccelDesigner()
    Ts = trajectory.getTimeCurve() * 1e-5

    # visualize j, a, v, x
    fig_a, axes = plt.subplots(4, 1, figsize=(6, 8))
    titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
    ylabels = ['zeta [rad/s/s/s]', 'alpha [rad/s/s]',
               'omega [rad/s]', 'theta [rad]']

    time_stamps = list(ad.getTimeStamp())
    time_stamps.insert(0, 0)
    time_stamps.append(time_stamps[-1]+shape.straight_post / v)
    for i in range(len(time_stamps)-1):
        t = np.arange(time_stamps[i], time_stamps[i+1], Ts)
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
            ax.grid(True)
            ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
            ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))

    plt.tight_layout()

    # shape
    fig_xy, ax = plt.subplots(figsize=(6, 6))
    state = ctrl.State()
    for i in range(len(time_stamps)-1):
        t = np.arange(time_stamps[i], time_stamps[i+1], Ts)
        x = []
        y = []
        for tt in t:
            trajectory.update(state, tt, Ts)
            x.append(state.q.x)
            y.append(state.q.y)
        ax.plot(x, y, lw=4)

    ax.set_title('Slalom Shape')
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_xticks(np.arange(-360, 360, 45))
    ax.set_xticks(np.arange(-360, 360, 5), minor=True)
    ax.set_yticks(ax.get_xticks())
    ax.set_yticks(ax.get_xticks(minor=True), minor=True)
    ax.axis('equal')
    ax.grid(which='major', linestyle='-')
    ax.grid(which='minor', linestyle=':')

    plt.show()


plot_accel_designer()
plot_slalom()
