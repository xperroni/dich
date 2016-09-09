#!/usr/bin/env python

from re import search


Exposure = 5


def extract(path):
    loaded = []

    for line in open(path):
        match = search(r'\(([^,]+), ([^\)]+)\)', line)
        if match == None:
            break

        loaded.append('[%s, %s]' % (match.group(1), match.group(2)))

        if len(loaded) < Exposure:
            continue

        for steering in loaded:
            print steering

        del loaded[:]


def main():
    from sys import argv
    extract(argv[1])


if __name__ == '__main__':
    main()
