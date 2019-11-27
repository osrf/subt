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
import tf.transformations as tft
import numpy as np
from numpy.linalg import inv
import argparse
import sys


def pose_sdf2tf(pose_el):
    pose = np.eye(4)
    if pose_el is not None and pose_el.text is not None:
        pose_array = np.array([float(d)
                            for d in pose_el.text.split()])
        pose = tft.translation_matrix(pose_array[:3]).dot(
            tft.euler_matrix(*pose_array[3:]))
    return pose

def pose_tf2sdf_text(pose):
    pose_array = np.r_[tft.translation_from_matrix(pose),
            tft.euler_from_matrix(pose)]
    # print(pose_array, file=sys.stderr)
    pose_text = np.array2string(
        pose_array, suppress_small=True, precision=16)[1:-1]
    # Remove extraneous spaces
    return ' '.join(pose_text.split())

def set_element_pose(elem, pose_txt):
    pose_el = elem.find('pose')
    if pose_el is None:
        pose_el = etree.Element('pose')
        # Use the tail of the first child of elem to keep the correct
        # indentation
        if elem[0] is not None:
            pose_el.tail = elem[0].tail
        elem.insert(0, pose_el)

    pose_el.text = pose_txt

def find_link_to_join(model, link_name):
    """Finds a link in the model to move the contents of the provided link.
    This is done by first finding the joint that attaches this link to another
    link in the model."""

    for el in model.findall('joint'):
        # Joining is only possible if the joint is a fixed joint. Sometimes a
        # a revolute joint with 0 upper and lower limits is used for fixed
        # joint. Handle that case as well.
        if el.get('type') != 'fixed':
            if el.get('type') == 'revolute':
                limit = el.find('limit')
                if limit is not None:
                    lower = limit.find('lower')
                    upper = limit.find('upper')
                    if float(lower.text) != 0 or float(upper.text) != 0:
                        continue
            else:
                continue

        parent = el.find('parent')
        child = el.find('child')
        if parent is not None and child is not None:
            if parent.text == link_name:
                attach_links = model.xpath(
                    './link[@name="{}"]'.format(child.text))
                if len(attach_links) > 0:
                    return el, attach_links[0]

            if child is not None:
                if child.text == link_name:
                    attach_links = model.xpath(
                        './link[@name="{}"]'.format(parent.text))
                    if len(attach_links) > 0:
                        return el, attach_links[0]

    return None, None

def change_parent_link(elem, current_link, new_link):
    """
    We'll use the
    [notation](http://sdformat.org/tutorials?tut=specify_pose&cat=specification&)
    `X_BA` to mean the pose of frame A expressed with respect to frame B.

       `model` = 'M'
       `new parent link` = `P`
       `current parent link` = `C`
       `element` = `E`

    We are moving element `E`, currently a child of `C` to be a child of `P`.
    With this notation, the values we are computing is X_PE. We are given from
    the SDF document, `X_MP`, `X_MC`, `X_CE`. To compute `X_PE` we first
    express the pose of `E` w.r.t `M`, i.e, `X_ME`:

        X_ME = X_MC * X_CE

    Then

        X_PE = inv(X_MP) * X_ME

    """
    elem_pose_el = elem.find('pose')

    X_MP = pose_sdf2tf(new_link.find('pose'))
    X_MC = pose_sdf2tf(current_link.find('pose'))
    X_CE = pose_sdf2tf(elem_pose_el)

    X_ME = X_MC.dot(X_CE)
    X_PE = inv(X_MP).dot(X_ME)

    elem.set('name', '{}_{}_{}'.format(
        current_link.get('name'), elem.get('name'), elem.tag))
    # print("Moving {}: {}".format(elem.tag, elem.get('name')), file=sys.stderr)
    set_element_pose(elem, pose_tf2sdf_text(X_PE))
    new_link.append(elem)

def flatten_model(model_sdf, args):
    """Flatten a model by moving the contents of nested models out to the
    parent model

    Parameters
    ----------
    model_sdf : TODO

    Returns
    -------
    TODO

    """
    inp = etree.parse(model_sdf)
    sdf = inp.getroot()
    # Assume there's just one <model>
    model = sdf.find('model')
    nested_models = model.findall('model')

    # First pass, move links and joints from nested models to the parent model.
    # May need to run multiple times to flatten recursively nested models.
    while len(nested_models) > 0:
        for nm in nested_models:

            nm_pose = pose_sdf2tf(nm.find('pose'))
            nm_name = nm.get('name')
            for el in nm:
                if el.tag in ['link', 'joint', 'model']:
                    # Rename each element to ensure unique names
                    el.set('name', '{}_{}'.format(nm_name, el.get('name')))

                    if el.tag == 'joint':
                        parent = el.find('parent')
                        if parent is not None:
                            parent.text = '{}_{}'.format(nm_name, parent.text)

                        child = el.find('child')
                        if child is not None:
                            child.text = '{}_{}'.format(nm_name, child.text)

                    # Update poses to be w.r.t parent model
                    pose_el = el.find('pose')
                    pose = pose_sdf2tf(pose_el)
                    new_pose = nm_pose.dot(pose)
                    set_element_pose(el, pose_tf2sdf_text(new_pose))

                    model.append(el)

            model.remove(nm)

        # Look for nested models again
        nested_models = model.findall('model')

    # Convert all references to links inside nested models to flattened model
    # links
    for el in model.findall('joint'):
        parent = el.find('parent')
        if parent is not None:
            parent.text = parent.text.replace('::', '_')

        child = el.find('child')
        if child is not None:
            child.text = child.text.replace('::', '_')

    if not args.no_reduce_links:
        # Second pass: Reduce massless links by moving their contents to the
        # link they are connected to. This may need to be done multiple times
        # to ensure all massless links have been removed.
        found_massless_link = True
        while found_massless_link:
            found_massless_link = False
            elements_to_remove = []
            for link_el in model.findall('link'):

                link_name = link_el.get('name')
                inertial = link_el.find('inertial')

                # print("Examining link {}".format(link_name), file=sys.stderr)
                if inertial is not None:
                    mass = float(inertial.find('mass').text)
                    if mass > 1e-3:
                        continue

                # Found a massless link
                found_massless_link = True

                # If inertial is not specified, we'll assume a massless link.
                # This is not in line the default behavior of Gazeob, which
                # assumes a mass of 1 kg if inertial is not specified.

                # Find the this link is connected in order to find the parent/child
                # link to which all the contents of this link are to be moved.
                attaching_joint, parent_link = find_link_to_join(
                    model, link_name)

                if parent_link is not None:
                    # Hoist all elements in this link to parent link

                    elements_to_hoist = []
                    for child_el in link_el:
                        if child_el.tag in ['visual', 'collision', 'sensor',
                                            'battery', 'light']:
                            # Append it to a temporary list. Appending to
                            # parent_link here causes an infinite loop
                            elements_to_hoist.append(child_el)

                    for child_el in elements_to_hoist:
                        change_parent_link(child_el, link_el, parent_link)

                    elements_to_remove.append(link_el)
                    elements_to_remove.append(attaching_joint)
                else:
                    print("Massless link {} found, but can't find a parent link"
                          " for it".format(link_name), file=sys.stderr)
                    elements_to_remove.append(link_el)


            for el in elements_to_remove:
                model.remove(el)

    print(etree.tostring(sdf, pretty_print=True, encoding="unicode"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', type=argparse.FileType('r'),
                        help='Input SDF file')
    parser.add_argument('--no_reduce_links', action='store_true',
                        help='Whether to reduce massless links by hoisting '
                        'their contents to a non-massless link they are '
                        'connected to. Defaults to true')

    args = parser.parse_args()
    flatten_model(args.infile, args)
