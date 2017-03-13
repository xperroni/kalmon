#!/usr/bin/env python

from math import cos, sin

from matplotlib import pyplot as pp


def plot_readings(readings, color):
    (x, y) = zip(*readings)


def plot(path):
    error = []

    for line in open(path):
        values = [float(value) for value in line.split('\t')]
        (x1, y1) = values[:2]
        (x2, y2) = values[6:8]
        e = ((x1 - x2) ** 2.0 + (y1 - y2) ** 2.0) ** 0.5
        error.append(e)

    pp.plot(error, 'b-')

    pp.show()


def main():
    from sys import argv
    plot(argv[1])


if __name__ == '__main__':
    main()
