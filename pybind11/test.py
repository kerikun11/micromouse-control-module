#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

import ctrl  # calls $PYTHONPATH/ctrl.so


def plot_accel_designer(ad):
    # visualize j, a, v, x
    fig, axes = plt.subplots(4, 1, figsize=(6, 8))
    titles = ['Jerk', 'Acceleration', 'Velocity', 'Position']
    ylabels = ['j [m/s/s/s]', 'a [m/s/s]', 'v [m/s]', 'p [m]']

    time_stamps = ad.getTimeStamp()
    dt = (time_stamps[-1] - time_stamps[0]) * 1e-4
    for i in range(len(time_stamps)-1):
        t = np.arange(time_stamps[i]+dt, time_stamps[i+1], dt)
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


def test_accel_curve():
    params = [
        (100, 10, 0, 2),  # vm > 0
        (100, 10, 1, 2),  # vm < 0
        (100, 10, 2, 1),  # vm < 0
        (100, 10, 2, 0),  # vm > 0
    ]
    for p in params:
        ad = ctrl.AccelCurve(*p)
        plot_accel_designer(ad)


def test_accel_designer():
    params = [
        (100, 10, 4, 0, 2, 4),      # vs -> vm -> vt, tm1>0, tm2>0
        (100, 10, 4, 0, 3, 4),      # vs -> vm -> vt, tm1>0, tm2<0
        (100, 10, 4, 3, 0, 4),      # vs -> vm -> vt, tm1<0, tm2>0
        (100, 10, 8, 0, 2, 4),      # vs -> vr -> vt, vr<vm, tm1>0, tm2>0
        (100, 10, 8, 0, 6, 4),      # vs -> vr -> vt, vr<vm, tm1>0, tm2<0
        (100, 10, 8, 0, 0.5, 0.2),  # vs -> vr -> vt, vr<vm, tm1<0, tm2<0
        (100, 10, 6, 0, 3, 1),      # vs -> vr -> vt, vr<vm, tm1>0, tm2<0
        (100, 10, 6, 0, 4, 1),      # ve == vt, tm > 0 just
        (100, 10, 8, 0, 6, 1),      # ve != vt, tm > 0, accel
        (100, 10, 8, 4, 0, 1),      # ve != vt, tm > 0, decel
        (100, 10, 4, 0, 4, 0.1),    # ve != vt, tm < 0, accel
        (100, 10, 4, 4, 0, 0.1),    # ve != vt, tm < 0, decel
    ]
    for p in params:
        ad = ctrl.AccelDesigner(j_max=p[0], a_max=p[1], v_max=p[2],
                                v_start=p[3], v_target=p[4], dist=p[5])
        plot_accel_designer(ad)
        ad = ctrl.AccelDesigner(j_max=p[0], a_max=p[1], v_max=p[2],
                                v_start=-p[3], v_target=-p[4], dist=-p[5])
        plot_accel_designer(ad)


if __name__ == "__main__":
    # test_accel_curve()
    # test_accel_designer()
    plot_accel_designer(ctrl.AccelCurve(100, 6, 0, 1))
    # plt.savefig('accel_curve.pdf')
    plot_accel_designer(ctrl.AccelDesigner(100, 6, 2, 0, 1, 1))
    # plt.savefig('accel_designer.pdf')
