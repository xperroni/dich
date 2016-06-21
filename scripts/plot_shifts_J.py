#!/usr/bin/env python

from re import search

from matplotlib import pyplot, cm

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

    #return (array(x) / 3.0, array(y))
    return (array(x), array(y))


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


def plot_shift_map(plotter, x0, shift_map, colormap):
    (m, n) = shift_map.shape
    c = plotter.matshow(shift_map, cmap=getattr(cm, colormap), origin='lower', extent=(x0, x0 + n, -m // 2, m // 2))
    pyplot.colorbar(c)


def setup_axes(axes, shifts, shift_map, ground_truth):
    axes.grid()
    axes.set_xlabel('Replay image #', labelpad=10)
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
    #yn = m // 2
    #y0 = -yn
    yn = 100
    y0 = -100

    axes.axis([x0, xn, y0, yn])

    return x0


def plot(path, path_ground_truth, colormap):
    (shifts, shift_map) = load_data(path)
    ground_truth = load_ground_truth(path_ground_truth)

    (figure, axes) = pyplot.subplots()
    x0 = setup_axes(axes, shifts, shift_map, ground_truth)

    plot_shift_map(axes, x0, shift_map, colormap)
    plot_ground_truth(axes, ground_truth)
    plot_shifts(axes, shifts)

    axes.xaxis.set_ticks_position('bottom')
    axes.set_aspect('auto', 'box')
    pyplot.tight_layout()
    pyplot.show()


def main():
    from sys import argv
    plot(*argv[1:])


if __name__ == '__main__':
    main()
