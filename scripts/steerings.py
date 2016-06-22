#!/usr/bin/env python

from re import search


def extract(path):
    for line in open(path):
        match = search(r'\(([^,]+), ([^\)]+)\)', line)
        if match == None:
            break

        print '[%s, %s]' % (match.group(1), match.group(2))


def main():
    from sys import argv
    extract(argv[1])


if __name__ == '__main__':
    main()
