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
import subprocess
import argparse
import sys
import os
import re
from convert_uris import convert_uris

def lanch_ros_nodes(args):
    """Launches launch/preview.launch."""
    import roslaunch
    import rospy
    rospy.init_node('model_spawner', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = ['ign_migration_scripts', 'preview.launch',
                'model_name:={}'.format(args.model_name),
                'world_name:={}'.format(args.world_name)]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                       roslaunch_args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    parent.start()
    rospy.spin()

def spawn_preview_model(args):
    """Spawn model into a running ign gazebo instance.

    This script will automatically convert uris of type `model://` to absolute
    URIs assuming that all such URIs refer to resources within the model
    directory of this model

    Parameters
    ----------
    args: Command line arguments

    Returns
    -------
    TODO

    """
    sdf = convert_uris(args.infile)

    # Add pose publisher system to help with visualization in rviz

    # Assume there's just one <model>
    model = sdf.find('model')
    model.set('name', args.model_name)
    plugin = etree.SubElement(model, 'plugin',
                    filename='libignition-gazebo-pose-publisher-system.so',
                    name='ignition::gazebo::systems::PosePublisher')

    etree.SubElement(plugin, 'publish_link_pose').text = 'true'
    etree.SubElement(plugin, 'publish_sensor_pose').text = 'true'
    etree.SubElement(plugin, 'publish_collision_pose').text = 'false'
    etree.SubElement(plugin, 'publish_visual_pose').text = 'false'
    etree.SubElement(plugin, 'publish_nested_model_pose').text = 'true'
    etree.SubElement(plugin, 'use_pose_vector_msg').text = 'true'
    etree.SubElement(plugin, 'static_publisher').text = 'true'
    etree.SubElement(plugin, 'static_update_frequency').text = '1'

    sdf_esc = etree.tostring(sdf, encoding="unicode",
                             with_tail=False).replace('"', r'\"').replace('\n', '')

    # xyz xyzw
    pose = [0, 0, 0.3, 0, 0, 0, 1.0]
    if args.pose:
        for i, val in enumerate(args.pose):
            if i < len(pose):
                pose[i] = val
    # pose: {position: {x:0 y:0 z:0} orientation {x:0 y:0 z:0 w:1}}
    pose_str = '{{position: {{x: {0} y: {1} z: {2}}} \
                    orientation {{x: {3} y: {4} z: {5} w: {6}}}}}'.format(*pose)
    print("pose str", pose_str)

    cmd = ['ign', 'service', '-s', '/world/{}/create'.format(args.world_name),
            '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
            'ignition.msgs.Boolean', '--timeout', '300', '--req',
            'sdf: "{}" pose: {}'.format(sdf_esc, pose_str)]
    return subprocess.call(cmd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-m', '--model_name', dest="model_name",
            default='preview_model', help='Model name')
    parser.add_argument('-p', '--pose', dest="pose",
                        type=float, nargs='*', help='Model name')
    parser.add_argument('--ros', action='store_true', default=False,
                        help='Launch ros nodes for visualization in Rviz')
    parser.add_argument('infile', type=argparse.FileType('r'),
                        help='Input SDF file')
    parser.add_argument('world_name',  nargs="?",
                        default="preview", help='Name of world')

    args = parser.parse_args()
    spawn_preview_model(args)
    if args.ros:
        lanch_ros_nodes(args)
