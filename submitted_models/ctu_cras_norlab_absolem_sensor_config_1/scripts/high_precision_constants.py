#!/usr/bin/env python
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
from lxml import etree
import math
import argparse
import sys

# Values of PI to check. Only put positive values.
CONSTANTS_TO_CHECK = [math.pi, math.pi/2]

def convert_to_high_precision_pi(urdf_file, tol, precision):
    """Search all poses for values that are near PI and PI/2 and replace them
    with their high precision counterparts.

    Parameters
    ----------
    urdf_file : Input urdf file
    tol: Tolerance for equality check

    Returns
    -------
    etree.Element

    """
    inp = etree.parse(urdf_file)
    sdf = inp.getroot()
    poses = sdf.xpath('//pose')

    for pose in poses:
        new_pose_list = []
        for val in pose.text.split():
            val_fl = float(val)
            for check_val in CONSTANTS_TO_CHECK:
                if abs(abs(val_fl) - check_val) < tol:
                    new_pose_list.append('{:.{precision}f}'.format(
                        math.copysign(check_val, val_fl), precision=precision))
                    break
            # The else block executes if none of the values in
            # CONSTANTS_TO_CHECK match val_fl
            else:
                new_pose_list.append(val)

        new_pose = ' '.join(new_pose_list)
        pose.text = new_pose
    return inp


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Search all <pose> elements for values that are near PI '
        'and PI/2 and replace them with their high precision counterparts.')
    parser.add_argument('infile', type=argparse.FileType('r'),
                        help='Input URDF file')
    parser.add_argument('-t', '--tolerance', type=float, default=1e-3,
                        help='Tolerance for equality checks')
    parser.add_argument('-p', '--precision', type=float, default=16,
                        help='Tolerance for equality checks')

    args = parser.parse_args()
    out = convert_to_high_precision_pi(args.infile, args.tolerance,
                                       args.precision)
    print(etree.tostring(out, pretty_print=True, encoding='utf-8',
                         xml_declaration=True))
