#!/usr/bin/env python
from __future__ import print_function
import argparse
import csv
import math
import os
import sys

tunnel_tile_name_counter = 0
artifact_name_counter = {}
plugin_artifacts = ''

def model_include_string(tileNamePrefix, modelType,
                         pose_x, pose_y, pose_z, pose_yaw):
    if 'tunnel_tile_' in modelType:
        global tunnel_tile_name_counter
        modelName = tileNamePrefix + "_" + str(tunnel_tile_name_counter)
        tunnel_tile_name_counter += 1
    else:
        global artifact_name_counter
        if not modelType in artifact_name_counter:
            artifact_name_counter[modelType] = 0
        artifact_name_counter[modelType] += 1
        model_type = modelType.lower().replace(' ', '_')
        modelName = model_type + '_' + str(artifact_name_counter[modelType])
        global plugin_artifacts
        plugin_artifacts += """
      <artifact>
        <name>%s</name>
        <type>TYPE_%s</type>
      </artifact>""" % (modelName, model_type.upper())
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
                                         args.x0 + ix*args.scale_x,
                                         args.y0 - iy*args.scale_y,
                                         args.z0 + z_level*args.scale_z,
                                         yawDegrees * math.pi / 180))

def parse_args(argv):
    parser = argparse.ArgumentParser('Generate tiled world file from tsv.')
    parser.add_argument('file_name', help='name of tsv file to read')
    parser.add_argument('--world-name', dest='world_name', type=str, default='default', help='world name')
    parser.add_argument('--x0', dest='x0', type=float, default=0, help='origin X coordinate')
    parser.add_argument('--y0', dest='y0', type=float, default=0, help='origin Y coordinate')
    parser.add_argument('--z0', dest='z0', type=float, default=0, help='origin Z coordinate')
    parser.add_argument('--scale_x', dest='scale_x', type=float, default=20, help='tile scale in X')
    parser.add_argument('--scale_y', dest='scale_y', type=float, default=20, help='tile scale in Y')
    parser.add_argument('--scale_z', dest='scale_z', type=float, default=5,  help='tile scale in Z')
    parser.add_argument('--wind_x', dest='wind_x', type=float, default=0, help='global wind velocity in X')
    parser.add_argument('--wind_y', dest='wind_y', type=float, default=0, help='global wind velocity in Y')
    parser.add_argument('--wind_z', dest='wind_z', type=float, default=0, help='global wind velocity in Z')
    args = parser.parse_args()
    return args

def check_main():
    args = parse_args(sys.argv)
    print("""<?xml version="1.0" ?>
<!--
  Generated with the tile_tsv.py script:
    %s
-->
<sdf version="1.6">
  <world name="%s">

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-6.3 -4.2 3.6 0 0.268 0.304</pose>
      </camera>
    </gui>

    <scene>
      <ambient>0.2 0.2 0.2 1.0</ambient>
      <background>0.34 0.39 0.43 1.0</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- TODO(chapulina): Entrance placeholder -->
    <model name="staging">
      <pose>-1.181716 0 -0.002 0 0 0</pose>
     <static>true</static>
     <link name="link">
       <pose>0 0 -0.5 0 0 0</pose>
       <visual name="visual">
         <geometry>
           <box>
             <size>25 25 1</size>
           </box>
         </geometry>
         <material>
           <script>
             <uri>file://media/materials/scripts/gazebo.material</uri>
             <name>Gazebo/Residential</name>
           </script>
         </material>
       </visual>
       <collision name="collision">
         <geometry>
           <box>
             <size>25 20 1</size>
           </box>
         </geometry>
       </collision>
     </link>
    </model>

    <model name="tunnel_entrance">
      <pose>
        10
        0
        -0.001
        0
        0
        -1.5707963267948966
      </pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <scale>
                0.008
                0.008 0.008
              </scale>
              <uri>model://tunnel_entrance/meshes/Entrance.obj</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <scale>
                0.008
                0.008 0.008
              </scale>
              <uri>model://tunnel_entrance/meshes/Entrance.obj</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
    <!-- Start Area where an object enters to initiate the game. -->
    <model name="start_area">
      <static>true</static>
      <pose>
        10
        0
        1.5
        0
        0
        0
      </pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>
                1.0
                5.0
                3.0
              </size>
            </box>
          </geometry>
          <surface>
            <contact>
              <collide_without_contact>1</collide_without_contact>
            </contact>
          </surface>
        </collision>
        <sensor name="start_area_sensor" type="contact">
          <contact>
            <collision>collision</collision>
          </contact>
        </sensor>
      </link>
      <plugin name="touch_start_area" filename="libTouchPlugin.so">
        <sensor>start_area_sensor</sensor>
        <!-- empty target name lets the plugin detects everything -->
        <target></target>
        <time>0.001</time>
        <namespace>subt/start</namespace>
        <enabled>true</enabled>
      </plugin>
    </model>

    <light name='user_spot_light_0' type='spot'>
      <pose>12 0 15 0 0.65 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0 0 -1</direction>
      <attenuation>
        <range>30</range>
        <constant>0.001</constant>
        <linear>0.01</linear>
        <quadratic>0.000</quadratic>
      </attenuation>
      <cast_shadows>1</cast_shadows>
      <spot>
        <inner_angle>0.0</inner_angle>
        <outer_angle>1.5</outer_angle>
        <falloff>0.01</falloff>
      </spot>
    </light>

    <include>
      <name>barrier_0_-1</name>
      <pose>
        8.0
        -11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_0_1</name>
      <pose>
        8.0
        11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_1_-1</name>
      <pose>
        3.7
        -11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_1_1</name>
      <pose>
        3.7
        11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_2_-1</name>
      <pose>
        -0.5999999999999996
        -11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_2_1</name>
      <pose>
        -0.5999999999999996
        11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_3_-1</name>
      <pose>
        -4.899999999999999
        -11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_3_1</name>
      <pose>
        -4.899999999999999
        11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_4_-1</name>
      <pose>
        -9.2
        -11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_4_1</name>
      <pose>
        -9.2
        11
        0
        0
        0
        0
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  

  
    <include>
      <name>barrier_-9.5_-2</name>
      <pose>
        10.5
        -8.6
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-9.5_2</name>
      <pose>
        10.5
        8.6
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-32_-2</name>
      <pose>
        -12
        -8.6
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-32_-1</name>
      <pose>
        -12
        -4.3
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-32_0</name>
      <pose>
        -12
        0.0
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-32_1</name>
      <pose>
        -12
        4.3
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
  
    <include>
      <name>barrier_-32_2</name>
      <pose>
        -12
        8.6
        0
        0
        0
        1.5707963267948966
      </pose>
      <uri>model://jersey_barrier</uri>
    </include>
""" %
  (' '.join(sys.argv).replace('--', '-\-'), args.world_name))
    print_tsv_model_includes(args)
    global plugin_artifacts
    print("""
    <!-- Base station -->
    <include>
      <uri>model://base_station</uri>
      <pose>-8 0 0 0 0 -1.5708</pose>
      <plugin name="base_station_plugin" filename="libBaseStationPlugin.so">
      </plugin>
    </include>

    <!-- The SubT challenge logic plugin -->
    <plugin name="game_logic_plugin" filename="libGameLogicPlugin.so">
      <!-- The collection of artifacts to locate -->
%s
    </plugin>

    <!-- The SubT comms broker plugin -->
    <plugin name="comms_broker_plugin" filename="libCommsBrokerPlugin.so">
      <comms_model>
        <neighbor_distance_min>0.0</neighbor_distance_min>
        <neighbor_distance_max>100.0</neighbor_distance_max>
        <comms_distance_min>0.0</comms_distance_min>
        <comms_distance_max>100.0</comms_distance_max>
        <comms_drop_probability_min>0.0</comms_drop_probability_min>
        <comms_drop_probability_max>0.0</comms_drop_probability_max>
        <comms_outage_probability>0.0</comms_outage_probability>
        <comms_outage_duration_min>0.0</comms_outage_duration_min>
        <comms_outage_duration_max>10.0</comms_outage_duration_max>
      </comms_model>
    </plugin>

    <!-- rotors_gazebo support -->
    <plugin name="ros_interface_plugin"
            filename="librotors_gazebo_ros_interface_plugin.so"/>

    <wind>
      <linear_velocity>%f %f %f</linear_velocity>
    </wind>

    <!-- Load the plugin for the wind -->
    <plugin name="wind" filename="libWindPlugin.so">
      <horizontal>
        <magnitude>
          <time_for_rise>10</time_for_rise>
          <sin>
            <amplitude_percent>0.05</amplitude_percent>
            <period>60</period>
          </sin>
          <noise type="gaussian">
           <mean>0</mean>
           <stddev>0.0002</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin>
            <amplitude>5</amplitude>
            <period>20</period>
          </sin>
          <noise type="gaussian">
           <mean>0</mean>
           <stddev>0.03</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
         <mean>0</mean>
         <stddev>0.03</stddev>
        </noise>
      </vertical>
    </plugin>

  </world>
</sdf>""" %
(plugin_artifacts, args.wind_x, args.wind_y, args.wind_z))
        
if __name__ == '__main__':
    check_main()


