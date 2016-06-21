#!/usr/bin/env python

from argparse import ArgumentParser


def plot(mode, path, path_ground_truth, colormap):
    if mode == 'similarity':
        import plot_similarities_J
        plot_similarities_J.plot(path, path_ground_truth, colormap)
    elif mode == 'shift':
        import plot_shifts_J
        plot_shifts_J.plot(path, path_ground_truth, colormap)
    else:
        raise Exception('Mode "%s" not known' % mode)


def main():
    parser = ArgumentParser(description='Plot similarity map from given log file.')
    parser.add_argument('mode', type=str, help='whether to plot similarity or shift map')
    parser.add_argument('path_log', type=str, help='path to the log file')
    parser.add_argument('path_ground_truth', nargs='?', default='', type=str, help='path to the ground truth file')
    parser.add_argument('--colormap', default='Greys', type=str, help='path to the ground truth file')

    args = parser.parse_args()

    plot(args.mode, args.path_log, args.path_ground_truth, args.colormap)


if __name__ == '__main__':
    main()
