#!/usr/bin/env python

from re import search

from matplotlib import pyplot, cm
from matplotlib.gridspec import GridSpec

from numpy import array, ones, zeros
from numpy import amin, amax, nonzero
from numpy import arange, argmax, exp
from numpy import abs as aabs
from numpy import sum as asum
from numpy.linalg import inv

from scipy.stats import gaussian_kde


Exposure = 5


def normalize(row):
    row = array(row)
    indexes = nonzero(row)
    row[indexes] -= amin(row[indexes])
    row[indexes] /= amax(row)
    return row


def fetch(expression, line):
    match = search(expression, line)
    if match == None:
        return None

    text = match.group(1).strip()
    return eval(text)


def load_data(path):
    rows = []
    correspondences = []
    for line in open(path):
        row = fetch(r'Similarities: ([^\"\r\n\x1b]+)', line)
        if row != None:
            rows.append(normalize(row[1]))
            continue

        point = fetch(r'Pairing: ([^\"\r\n\x1b]+)', line)
        if point != None:
            correspondences.append(point)

    correspondences.insert(0, (0, 0))

    return (array(rows).T, correspondences)


def load_ground_truth(path):
    if path == '':
        return None

    (x, y) = ([], [])
    for line in open(path):
        record = eval(line)
        x.append(record[0] / Exposure)
        y.append(record[1] / Exposure)

    return (x, y)


def load_errors(correspondences, ground_truth):
    (xs, ds) = ([], [])
    if ground_truth == None:
        return (xs, ds)

    ground_truth = dict((x, y) for (x, y) in zip(*ground_truth))

    for (y, x) in correspondences:
        g = ground_truth.get(x)
        if g == None:
            continue

        xs.append(x)
        ds.append(y - g)

    return (xs, ds)


def plot_similarities(plotter, plotter_c, similarities, colormap):
  c = plotter.matshow(similarities ** 2.0, cmap=getattr(cm, colormap), origin='lower') # cm.Greys
  pyplot.colorbar(c, cax=plotter_c)


def plot_points(plotter, points):
    (x, y) = zip(*points)
    plotter.plot(x, y, 'w,')


def plot_correspondences(plotter, correspondences):
    (y, x) = zip(*correspondences)
    plotter.plot(x, y, 'k-', linewidth=4.0)
    plotter.plot(x, y, 'w-', linewidth=2.0)


def plot_ground_truth(plotter, ground_truth):
    if ground_truth == None:
        return

    (x, y) = ground_truth
    plotter.plot(x, y, 'k--', linewidth=4.0)
    plotter.plot(x, y, 'w--', linewidth=2.0)


def plot_errors(plotter, errors):
    (x, y) = errors
    plotter.plot(x, y, 'k-', linewidth=2.0)


def setup_axes(axes1, axes2, similarities, errors, ground_truth):
    (m, n) = similarities.shape
    if ground_truth != None:
        (x, y) = ground_truth
        n = min(n, amax(x))

    axes1.axis([-0.5, n - 0.5, -0.5, m - 0.5])
    axes1.xaxis.set_ticks_position('bottom')
    axes1.set_aspect('equal', 'box')

    axes1.grid()
    axes1.set_xlabel('Repeat image (index)', labelpad=10)
    axes1.set_ylabel('Teach image (index)', labelpad=20)

    (x, e) = errors
    if len(x) == 0:
        return

    e0 = amin(e) - 10
    en = amax(e) + 10

    axes2.axis([0, n, e0, en])
    axes2.set_aspect('equal', 'box')

    axes2.grid()
    axes2.set_xlabel('Repeat image (index)', labelpad=10)
    axes2.set_ylabel('Pairing error (index)', labelpad=20)


def plot(path, path_ground_truth, colormap):
    (similarities, correspondences) = load_data(path)
    ground_truth = load_ground_truth(path_ground_truth)
    errors = load_errors(correspondences, ground_truth)

    gs = GridSpec(2, 2, width_ratios=[20, 1], height_ratios=[5, 1])
    gs.update(wspace=0.05, hspace=0.2)
    axes1 = pyplot.subplot(gs[0])
    axes1c = pyplot.subplot(gs[1])
    axes2 = pyplot.subplot(gs[2])

    plot_similarities(axes1, axes1c, similarities, colormap)
    plot_ground_truth(axes1, ground_truth)
    plot_correspondences(axes1, correspondences)
    plot_errors(axes2, errors)

    setup_axes(axes1, axes2, similarities, errors, ground_truth)

    pyplot.show()


def main():
    from sys import argv
    plot(*argv[1:])


if __name__ == '__main__':
    main()
