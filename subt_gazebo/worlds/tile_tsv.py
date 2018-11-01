#!/usr/bin/env python
from __future__ import print_function
import csv
import math
import os
import sys

model_name_counter = 0

def model_include_string(namePrefix, modelType,
                         pose_x, pose_y, pose_z, pose_yaw):
    global model_name_counter
    modelName = namePrefix + "_" + str(model_name_counter)
    model_name_counter += 1
    return """
    <include>
      <name>%s</name>
      <uri>model://%s</uri>
      <pose>%f %f %f 0 0 %f</pose>
    </include>""" % (modelName, modelType,
                     float(pose_x), float(pose_y), float(pose_z),
                     float(pose_yaw))

def print_tsv_model_includes(fileName):
    with open(fileName, 'rb') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        print(model_include_string("tile", modelType,
                                                   ix*20, -iy*20, z_level*5,
                                                   yawDegrees*math.pi/180))

def usage():
    print("""Usage:
\ttile_tsv.py test-file.tsv
""", file=sys.stderr)
    print(sys.argv)
    sys.exit(getattr(os, 'EX_USAGE', 1))

def check_main():
    if len(sys.argv) < 2:
        usage()
    print("""
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">""")
    print_tsv_model_includes(sys.argv[1])
    print("""
  </world>
</sdf>""")
        
if __name__ == '__main__':
    check_main()


