#!/usr/bin/env python

from re import search

from matplotlib import pyplot, cm

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


def plot_similarities(plotter, similarities):
  c = plotter.matshow(similarities, cmap=cm.jet, origin='lower') # cm.Greys
  pyplot.colorbar(c)


def plot_points(plotter, points):
    (x, y) = zip(*points)
    plotter.plot(x, y, 'w,')


def plot_correspondences(plotter, correspondences):
    (y, x) = zip(*correspondences)
    plotter.plot(x, y, 'r-', linewidth=2.0)


def plot_ground_truth(plotter, ground_truth):
    if ground_truth == None:
        return

    (x, y) = ground_truth
    plotter.plot(x, y, 'g-', linewidth=2.0, label='Ground Truth')


def plot(path, path_ground_truth=''):
    (similarities, correspondences) = load_data(path)
    ground_truth = load_ground_truth(path_ground_truth)
    (figure, axes) = pyplot.subplots()

    plot_similarities(axes, similarities)

    plot_correspondences(axes, correspondences)
    plot_ground_truth(axes, ground_truth)

    (m, n) = similarities.shape
    axes.axis([-0.5, n - 0.5, -0.5, m - 0.5])
    axes.xaxis.set_ticks_position('bottom')
    axes.set_aspect('auto', 'box')

    axes.grid()
    axes.set_xlabel('Replay image #', labelpad=10)
    axes.set_ylabel('Teach image #', labelpad=20)

    pyplot.tight_layout()
    pyplot.show()


def main():
    from sys import argv
    plot(*argv[1:])


if __name__ == '__main__':
    main()
