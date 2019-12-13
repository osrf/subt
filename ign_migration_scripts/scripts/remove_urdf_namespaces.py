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
import argparse
import sys
from sets import Set


def remove_namespace(inp):
    """Remove namespace from input string.

    A namespace in this context is the first substring before the '/' character
    Parameters
    ----------
    inp : Input string

    Returns
    -------
    string with namespace removed.

    """
    ind = inp.find('/')
    if ind != -1:
        return inp[ind+1:]

    return inp


def get_namespace(links):
    """Get all the namespaces in the list of links

    A namespace in this context is the first substring before the '/' character
    Parameters
    ----------
    links : A list of link elements

    Returns
    -------
    list of namespaces

    """
    namespaces = Set()
    for link in links:
        name = link.get('name')
        ind = name.find('/')
        if ind != -1:
            namespaces.add(name[:ind])
        else:
            namespaces.add('')

    return namespaces


def remove_urdf_namespaces(urdf_file):
    """Remove namespaces from link names in urdf files.

    Parameters
    ----------
    urdf_file : Input urdf file

    Returns
    -------
    etree.Element

    """
    inp = etree.parse(urdf_file)
    robot = inp.getroot()
    link_renames = {}
    links = robot.xpath('//robot/link')

    namespaces = get_namespace(links)
    if len(namespaces) > 1:
        print('WARNING: More than one namespace was found', namespaces,
              '. This script is not designed to handle multipe namespaces.',
              file=sys.stderr)

    for link in links:
        name = link.get('name')
        new_name = remove_namespace(name)
        link.set('name', new_name)
        link_renames[name] = new_name

    joints = robot.xpath('//robot/joint')

    for joint in joints:
        parent = joint.find('parent')
        parent_name = parent.get('link')
        try:
            parent_new_name = link_renames[parent_name]
            parent.set('link', parent_new_name)
        except KeyError:
            print('ERROR: Parent link', parent_name, 'of joint: ',
                  joint.get('name'), 'is not a valid link.', file=sys.stderr)

        child = joint.find('child')
        child_name = child.get('link')
        try:
            child_new_name = link_renames[child_name]
            child.set('link', child_new_name)
        except KeyError:
            print('ERROR: Child link', child_name, 'of joint: ',
                  joint.get('name'), 'is not a valid link.', file=sys.stderr)

    # Handle joints in nested gazebo models
    gazebo_joints = robot.xpath('//gazebo/joint')

    for joint in gazebo_joints:
        parent = joint.find('parent')
        parent_name = parent.text
        if '::' not in parent_name:
            try:
                parent_new_name = link_renames[parent_name]
                parent.text = parent_new_name
            except KeyError:
                print('ERROR: Parent link', parent_name, 'of joint: ',
                      joint.get('name'), 'is not a valid link.',
                      file=sys.stderr)

        child = joint.find('child')
        child_name = child.text
        if '::' not in child_name:
            try:
                child_new_name = link_renames[child_name]
                child.text = child_new_name
            except KeyError:
                print('ERROR: Child link', child_name, 'of joint: ',
                      joint.get('name'), 'is not a valid link.',
                      file=sys.stderr)
    return inp


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', type=argparse.FileType('r'),
                        help='Input URDF file')

    args = parser.parse_args()
    out = remove_urdf_namespaces(args.infile)
    print(etree.tostring(out, pretty_print=True, encoding='utf-8',
                         xml_declaration=True))
