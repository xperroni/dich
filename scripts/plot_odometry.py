#!/usr/bin/env python

from math import cos, sin, pi
from re import search

from matplotlib import pyplot, cm

from numpy import array, arange
from numpy import amin, amax


def load_data(path):
    if path == '':
        return None

    (x, y) = ([], [])
    for line in open(path):
        record = eval(line)
        x.append(record[0])
        y.append(record[1])

    return (array(x), array(y))


def rotate(data, d):
    t = d * pi / 180.0
    r = array([
        [cos(t), -sin(t)],
        [sin(t), cos(t)]
    ])

    (xr, yr) = ([], [])
    for (x, y) in zip(*data):
        p = array([[x], [y]])
        pr = r.dot(p)
        xr.append(pr[0, 0])
        yr.append(pr[1, 0])

    return (array(xr), array(yr))


def plot_odometry(plotter, odometry, color, label):
    if odometry == None:
        return

    (x, y) = odometry
    plotter.plot(x, y, color, linewidth=2.0, label=label)


def setup_axes(axes, teach, replay):
    axes.grid()
    axes.set_aspect('equal', 'box')
    axes.set_xlabel('x (m)', labelpad=10)
    axes.set_ylabel('y (m)', labelpad=20)

    (X_teach, Y_teach) = teach
    (X_replay, Y_replay) = replay
    x0 = min(X_teach[0], X_replay[0])
    xn = max(X_teach[-1], X_replay[-1])
    y0 = min(amin(Y_teach), amin(Y_replay))
    yn = max(amax(Y_teach), amax(Y_replay))

    x0 = int(x0) - 1
    xn = int(xn) + 1
    y0 = int(y0) - 1
    yn = int(yn) #+ 1
    yn = max(abs(y0), abs(yn))

    axes.axis([x0, xn, -yn, yn])
    axes.set_yticks(arange(y0, yn + 1, 1.0))


def plot(path_teach, path_replay):
    teach = load_data(path_teach)
    replay = load_data(path_replay)

    #teach = rotate(load_data(path_teach), -1.5)
    #replay = rotate(load_data(path_replay), -1.5)

    (figure, axes) = pyplot.subplots()
    setup_axes(axes, teach, replay)

    plot_odometry(axes, teach, 'k--', 'Teach step')
    plot_odometry(axes, replay, 'k-', 'Replay step')

    pyplot.tight_layout()
    pyplot.show()


def main():
    from sys import argv
    plot(*argv[1:])


if __name__ == '__main__':
    main()
