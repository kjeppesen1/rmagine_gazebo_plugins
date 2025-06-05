#!/usr/bin/env python3

"""
Example script that generates a CSV file containing O1DN (one origin, multiple directions) LiDAR
scan pointing angles, for ingestion by the rmagine_embree_o1dn_gzplugin. The pointing angles
must be expressed in terms of horizontal and vertical pointing angles from the origin, in radians.
These follow spherical coordinates. The rmagine_embree_o1dn_gzplugin will convert these to cartesian
coordinates for generating the point cloud.

E.g. first few rows of the CSV should have the follwing format:
-0.6981317007977318,0.0
-0.6980799854230292,0.004061624375325018
-0.6980282700483266,0.008122698830992594

With the first entry being the horizontal pointing angle for the scan point, and the second
entry being the vertical pointing angle for the scan point.

This script is purely for example purposes. Users wishing the generate their own config files to
model their own LiDAR sensor scan patterns may use this script with adjusted parameters to produce
a sinusoidal pattern, but will probably be better of creating a new script tailored to fit the
actual needs for modeling that LiDAR's scan pattern.
"""

import csv
import math

# Parameters
scan_points = 27000
cycles = 50
h_fov = 80.0 * (math.pi / 180.0) # horizontal field of view, radians
v_fov = 40.0 * (math.pi / 180.0) # vertical field of view, radians

# Generate spherical coordinate scan pointing angles,
# write to CSV
with open('output.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(scan_points):
        # t goes from 0 to 1 across the scan
        t = float(i) / float(scan_points - 1)

        # Horizontal angle increments across the HFOV
        h_angle =  h_fov * t - (h_fov / 2.0)

        # Phase for sine wave
        phase = 2.0 * math.pi * cycles * t

        # Vertical angle calculated at this h_angle
        amp = v_fov / 2.0
        v_angle = amp * math.sin(phase)

        writer.writerow((h_angle, v_angle))
        