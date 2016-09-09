#!/usr/bin/env python

from re import search

from matplotlib import pyplot, cm
from matplotlib.gridspec import GridSpec

from numpy import array, amin, amax
from numpy import argmin, nonzero
from numpy import abs as aabs


Exposure = 5


def fetch(expression, line):
    match = search(expression, line)
    if match == None:
        return None

    text = match.group(1).strip()
    return eval(text)


def normalize(row):
    row = array(row)
    indexes = nonzero(row)
    row[indexes] -= amin(row[indexes])
    row[indexes] /= amax(row)
    return row


def load_data(path):
    indices = ([], [])
    (x, y) = ([], [])
    shift_vectors = []
    for line in open(path):
        #row = fetch(r'Shift correlations: ([^"\r\n\x1b]+)', line)
        #if row != None:
            #(j, correlations) = row
            #shift_vectors.append(normalize(correlations))

        row = fetch(r'Shift: ([^"\r\n\x1b]+)', line)
        if row != None:
            (j, shift, shift_vector) = row
            shift_vectors.append(normalize(shift_vector))
            n = len(shift_vector)
            c = n // 2

            x.append(j)
            y.append(shift - c)

    shifts = (array(x), array(y))
    shift_map = array(shift_vectors).T

    return (shifts, shift_map)


def load_ground_truth(path):
    if path == '':
        return None

    (x, y) = ([], [])
    for line in open(path):
        (x_replay, x_teach, shift) = eval(line)
        x.append(x_replay / Exposure)
        y.append(shift * 256.0 / 640.0)

    return (array(x), array(y))


def load_errors(shifts, ground_truth):
    (xs, ds) = ([], [])
    if ground_truth == None:
        return (xs, ds)

    ground_truth = dict((x, y) for (x, y) in zip(*ground_truth))

    for (x, y) in zip(*shifts):
        g = ground_truth.get(x)
        if g == None:
            continue

        xs.append(x)
        ds.append(y - g)

    return (xs, ds)


def plot_ground_truth(plotter, ground_truth):
    if ground_truth == None:
        return

    (x, y) = ground_truth

    plotter.plot(x, y, 'k--', linewidth=4.0)
    plotter.plot(x, y, 'w--', linewidth=2.0, label='Ground Truth')


def plot_shifts(plotter, shifts):
    (x, y) = shifts
    plotter.plot(x, y, 'k-', linewidth=4.0)
    plotter.plot(x, y, 'w-', linewidth=2.0, label='Shifts')


def plot_shift_map(plotter, plotter_c, x0, shift_map, colormap):
    (m, n) = shift_map.shape
    c = plotter.matshow(shift_map ** 2.0, cmap=getattr(cm, colormap), origin='lower', extent=(x0, x0 + n, -m // 2, m // 2))
    pyplot.colorbar(c, cax=plotter_c)


def plot_errors(plotter, errors):
    (x, y) = errors
    plotter.plot(x, y, 'k-', linewidth=2.0)


def setup_axes(axes, axes2, shifts, shift_map, ground_truth, errors):
    axes.grid()
    axes.set_xlabel('Repeat image (index)', labelpad=10)
    axes.set_ylabel('Shift (pixels)', labelpad=20)

    (x0, y0, xn, yn) = (0, 0, 0, 0)
    if ground_truth == None:
        (x, y) = shifts
        x0 = x[0]
        xn = 10 * (1 + x[-1] // 10)
    else:
        (X_s, Y_s) = shifts
        (X_g, Y_g) = ground_truth
        x0 = max(X_s[0], X_g[0])
        xn = max(X_s[-1], X_g[-1])

    (m, n) = shift_map.shape
    xn = min(xn, x0 + n)
    if ground_truth != None:
        (x, y) = ground_truth
        xn = min(xn, amax(x))

    yn = m // 2
    y0 = -yn

    axes.axis([x0, xn, y0, yn])

    (x, e) = errors
    if len(x) == 0:
        return x0

    e0 = amin(e[x0:xn]) - 10
    en = amax(e[x0:xn]) + 10

    axes2.axis([x0, xn, e0, en])
    axes2.set_aspect('auto', 'box')

    axes2.grid()
    axes2.set_xlabel('Repeat image (index)', labelpad=10)
    axes2.set_ylabel('Shift error (pixels)', labelpad=20)

    return x0


def plot(path, path_ground_truth, colormap):
    (shifts, shift_map) = load_data(path)
    ground_truth = load_ground_truth(path_ground_truth)
    errors = load_errors(shifts, ground_truth)

    gs = GridSpec(2, 2, width_ratios=[20, 1], height_ratios=[5, 1])
    gs.update(wspace=0.05, hspace=0.2)
    axes1 = pyplot.subplot(gs[0])
    axes1c = pyplot.subplot(gs[1])
    axes2 = pyplot.subplot(gs[2])

    x0 = setup_axes(axes1, axes2, shifts, shift_map, ground_truth, errors)

    plot_shift_map(axes1, axes1c, x0, shift_map, colormap)
    plot_ground_truth(axes1, ground_truth)
    plot_shifts(axes1, shifts)
    plot_errors(axes2, errors)

    axes1.set_aspect('equal', 'box')
    axes1.xaxis.set_ticks_position('bottom')
    pyplot.show()


def main():
    from sys import argv
    plot(*argv[1:])


if __name__ == '__main__':
    main()
