#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Take a set of expected auv states and convert them to way points that could
be pasted into a xml paparazzi flight plan.

Generate source file with:

 - `rostopic echo -p /expected_uav_state > ros_expected_traj.csv`

This would listen to /expected_uav_state topic and redirect messages to a csv
file.
"""

from __future__ import absolute_import, print_function, division

import csv

points = []  # list of tuples (seq_number, x, y)

with open('ros_expected_traj.csv') as csvfile:
    csvfile.readline()  # Discard first line
    reader = csv.reader(csvfile, )
    for row in reader:
        points.append((row[1], row[4], row[5]))

with open('pprz_wp.xml', 'w+') as wp_xml:
    wp_xml.writelines(['<waypoint name="S{}" x="{}" y="{}"/>\n'.format(
        point[0], point[1], point[2]) for point in points])
