#!/usr/bin/env python

from math import cos, sin

from matplotlib import pyplot as pp


class Sensor(object):
    def plot(self):
        (x, y) = zip(*self.readings)
        pp.plot(x, y, self.color + '.')


class Laser(Sensor):
    def __init__(self, color):
        self.readings = []
        self.color = color

    def append(self, values, ground_truth):
        self.readings.append(values[:2])
        ground_truth.append(values[3:5])



class Radar(Sensor):
    def __init__(self, color):
        self.readings = []
        self.color = color

    def append(self, values, ground_truth):
        (d, r, v) = values[:3]
        x = d * cos(r)
        y = d * sin(r)

        self.readings.append([x, y])
        ground_truth.append(values[4:6])


def plot(path):
    sensors = {
        'L': Laser('b'),
        'R': Radar('g')
    }

    ground_truth = []

    for line in open(path):
        row = line.split('\t')
        (s, row) = (row[0], row[1:])
        values = [float(value) for value in row]
        sensors[s].append(values, ground_truth)

    for sensor in sensors.values():
        sensor.plot()

    #sensors['R'].plot()

    (x, y) = zip(*ground_truth)
    pp.plot(x, y, 'r,')

    pp.show()


def main():
    from sys import argv
    plot(argv[1])


if __name__ == '__main__':
    main()
