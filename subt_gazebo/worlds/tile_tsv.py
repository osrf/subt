#!/usr/bin/env python
from __future__ import print_function
import argparse
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

def print_tsv_model_includes(args):
    with open(args.file_name, 'rb') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        print(model_include_string("tile", modelType,
                                         args.x0 + ix*args.x_scale,
                                         args.y0 - iy*args.y_scale,
                                         args.z0 + z_level*args.z_scale,
                                         yawDegrees * math.pi / 180))

def parse_args(argv):
    parser = argparse.ArgumentParser('Generate tiled world file from tsv.')
    parser.add_argument('file_name', help='name of tsv file to read')
    parser.add_argument('--x0', dest='x0', type=float, default=0, help='origin X coordinate')
    parser.add_argument('--y0', dest='y0', type=float, default=0, help='origin Y coordinate')
    parser.add_argument('--z0', dest='z0', type=float, default=0, help='origin Z coordinate')
    parser.add_argument('--x_scale', dest='x_scale', type=float, default=20, help='tile scale in X')
    parser.add_argument('--y_scale', dest='y_scale', type=float, default=20, help='tile scale in Y')
    parser.add_argument('--z_scale', dest='z_scale', type=float, default=5,  help='tile scale in Z')
    args = parser.parse_args()
    return args

def check_main():
    args = parse_args(sys.argv)
    print("""
<?xml version="1.0" ?>
<!--
  Generated with the tile_tsv.py script:
    %s
-->
<sdf version="1.6">
  <world name="default">""" %
  (' '.join(sys.argv)))
    print_tsv_model_includes(args)
    print("""
  </world>
</sdf>""")
        
if __name__ == '__main__':
    check_main()


