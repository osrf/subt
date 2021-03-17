#!/usr/bin/env python3

import argparse
import csv
import math
import numpy as np
import os
import sys
import yaml

tunnel_tile_name_counter = 1
artifact_name_counter = {}
plugin_artifacts = ''
# Maps from tuple representing the center of level bounding box, to Level
plugin_levels_x = {}
plugin_levels_y = {}
levels_min_x = sys.float_info.max
levels_min_y = sys.float_info.max
levels_max_x = sys.float_info.min
levels_max_y = sys.float_info.min
levels_counter = 0

class Level:

    def __init__(self, name, x, y, z, sx, sy, sz, buf=10):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.sx = sx
        self.sy = sy
        self.sz = sz
        self.buffer = buf

        # List of strings of model names
        self.refs = []

    def add_ref(self, ref_name):
        self.refs.append(ref_name)

    def str(self):
        s = """
      <level name="%s">""" % self.name

        for ref in self.refs:
            s += """
        <ref>%s</ref>""" % ref

        s += """
        <pose>%f %f %f 0 0 0</pose>
        <geometry><box><size>%f %f %f</size></box></geometry>
        <buffer>%f</buffer>
      </level>""" % (self.x, self.y, self.z, self.sx, self.sy, self.sz, self.buffer)

        return s


# Calculate the center of the level to put a model, based on the model center
# and bounding box size of levels. Assumes all levels have the same size.
def calculate_level_center(model_x, model_y, model_z, levels_sx, levels_sy, levels_sz):
    lx = math.trunc(model_x / levels_sx) * levels_sx + 0.5 * levels_sx
    ly = math.trunc(model_y / levels_sy) * levels_sy + 0.5 * levels_sy
    lz = math.trunc(model_z / levels_sz) * levels_sz + 0.5 * levels_sz

    return (lx, ly, lz)


def expand_levels(levels_dictionary):
    s = ''
    for level in levels_dictionary.values():
        s += level.str()
    return s


def generate_model_name(tileNamePrefix, modelType):
    # Tunnels
    if modelType in GraphRules.STRAIGHTNESS:
        global tunnel_tile_name_counter
        modelName = tileNamePrefix + "_" + str(tunnel_tile_name_counter)
        counter = tunnel_tile_name_counter
        tunnel_tile_name_counter += 1
    # Artifacts
    else:
        global artifact_name_counter
        if not modelType in artifact_name_counter:
            artifact_name_counter[modelType] = 0
        artifact_name_counter[modelType] += 1
        model_type = modelType.lower().replace(' ', '_')
        modelName = model_type + '_' + str(artifact_name_counter[modelType])
        counter = artifact_name_counter[modelType]
        global plugin_artifacts
        plugin_artifacts += """
      <artifact>
        <name>%s</name>
        <type>TYPE_%s</type>
      </artifact>""" % (modelName, model_type.upper())
    return (modelName, counter)


def generate_levels(modelName, level_type,
                    pose_x, pose_y, pose_z,
                    scale_x, scale_y, scale_z,
                    levels_sx, levels_sy, levels_sz, levels_buf):
    # Levels plugin for this model
    global plugin_levels_x
    global plugin_levels_y
    global levels_counter
    global levels_min_x
    global levels_min_y
    global levels_max_x
    global levels_max_y

    # Method 01: generate one level per tile
    # A new level is created for every unique (x, y z) position of a model
    # e.g. used to generate levels for tunnel_circuit_practice_02
    if level_type == "single_tile":
        lx = pose_x
        ly = pose_y
        lz = pose_z
        if (lx, ly, lz) not in plugin_levels_x.keys():
            plugin_levels_x[(lx, ly, lz)] = Level("level" + str(levels_counter),
                                                  lx, ly, lz, levels_sx, levels_sy, levels_sz, levels_buf)
            levels_counter += 1
        plugin_levels_x[(lx, ly, lz)].add_ref(modelName)

    # Method 02: generate level for models in the same row and column
    # A new 'row' level is created for every unique x position of a model
    # Similarly, a new 'col' level is created fro every unique y position
    # e.g. used to generate levels for tunel_circuit_practice_01 and
    # tunnel_circuit_practice_03
    elif level_type == "row_col":
       # get min/max pos values in order to work out level bounding box later
       # in update_levels_size()
       if pose_x < levels_min_x:
         levels_min_x = pose_x
       if pose_x > levels_max_x:
         levels_max_x = pose_x
       if pose_y < levels_min_y:
         levels_min_y = pose_y
       if pose_y > levels_max_y:
         levels_max_y = pose_y

       level_size_factor = 1.875

       # column (x)
       col_levels_sx = levels_sx; # updated later
       col_levels_sy = scale_y * level_size_factor;
       col_levels_sz = levels_sz;
       col_lx = 0; # updated later
       col_ly = pose_y;
       col_lz = 0;

       if col_ly not in plugin_levels_x.keys():
           plugin_levels_x[col_ly] = Level("level" + str(levels_counter),
                                           col_lx, col_ly, col_lz, col_levels_sx, col_levels_sy, col_levels_sz,
                                           levels_buf)
           levels_counter += 1
       plugin_levels_x[(col_ly)].add_ref(modelName)

       # row (y)
       row_levels_sx = scale_x * level_size_factor;
       row_levels_sy = levels_sy; # updated later
       row_levels_sz = levels_sz;

       row_lx = pose_x;
       row_ly = 0; # updated later
       row_lz = 0;

       if row_lx not in plugin_levels_y.keys():
           plugin_levels_y[row_lx] = Level("level" + str(levels_counter),
                                           row_lx, row_ly, row_lz, row_levels_sx, row_levels_sy,
                                           row_levels_sz, levels_buf)
           levels_counter += 1
       plugin_levels_y[(row_lx)].add_ref(modelName)


def update_levels_size(scale_x, scale_y, scale_z):
    global plugin_levels_x
    global plugin_levels_y
    global levels_min_x
    global levels_min_y
    global levels_max_x
    global levels_max_y

    # compute level size and mid pos
    level_sx = levels_max_x - levels_min_x
    level_x = levels_min_x + level_sx * 0.5
    level_sy = levels_max_y - levels_min_y
    level_y = levels_min_y + level_sy * 0.5

    # add buffer to size
    level_sx += scale_x * 2.0
    level_sy += scale_y * 2.0

    for key, level in plugin_levels_x.items():
        level.x = level_x
        level.sx = level_sx
    for key, level in plugin_levels_y.items():
        level.y = level_y
        level.sy = level_sy


def model_include_string(modelName, modelType,
                         pose_x, pose_y, pose_z, pose_yaw,
                         pose_roll='0', pose_pitch='0',
                         types_to_paths=None, gz=False):
    # For Gazebo 9, use model:// prefix
    if gz:
        if types_to_paths is None:
            types_to_paths_f = lambda t: 'model://' + t
        else:
            # Retain just the base name after the last slash. That is the downloaded model folder name
            types_to_paths_f = lambda t: 'model://' + types_to_paths[t][types_to_paths[t].rfind('/') + 1:]
    else:
        if types_to_paths is None:
            types_to_paths_f = lambda t: t
        else:
            types_to_paths_f = lambda t: types_to_paths[t]

    return """    <include>
      <name>%s</name>
      <uri>%s</uri>
      <pose>%f %f %f %s %s %f</pose>
    </include>
""" % (modelName, types_to_paths_f(modelType),
       float(pose_x), float(pose_y), float(pose_z),
       pose_roll, pose_pitch, float(pose_yaw))


class GraphRules:
    # For computing edge cost
    STRAIGHT = 0
    TURN = 1
    # Key: tile mesh name
    STRAIGHTNESS = {
        'base_station': STRAIGHT,
        'tunnel_tile_1': TURN,
        'tunnel_tile_2': TURN,
        'tunnel_tile_3': TURN,
        'tunnel_tile_4': TURN,
        'tunnel_tile_5': STRAIGHT,
        'tunnel_tile_6': STRAIGHT,
        'tunnel_tile_7': TURN,
        'tunnel_tile_blocker': STRAIGHT,
        'constrained_tunnel_tile_tall': STRAIGHT,
        'constrained_tunnel_tile_short': STRAIGHT,
        'Rough Tunnel Tile 4-way Intersection': TURN,
        'Rough Tunnel Tile 90-degree Turn': TURN,
        'Rough Tunnel Tile Straight': STRAIGHT,
        'Rough Tunnel Tile Ramp': STRAIGHT,
        'Rough Tunnel Tile Vertical Shaft': TURN,
        'niosh_overpass': STRAIGHT,
        'niosh_blocker': STRAIGHT,
        'niosh_blocker1': STRAIGHT,
        'niosh_blocker2': STRAIGHT,
        'niosh_blocker3': STRAIGHT,
        'niosh_corner_1': TURN,
        'niosh_corner_2': TURN,
        'niosh_corner_3': TURN,
        'niosh_straight_1': STRAIGHT,
        'niosh_straight_2': STRAIGHT,
        'niosh_straight_3': STRAIGHT,
        'niosh_straightlong_1': STRAIGHT,
        'niosh_straightlong_2': STRAIGHT,
        'niosh_straightlong_3': STRAIGHT,
        'niosh_3way_1': TURN,
        'niosh_3way_2': TURN,
        'niosh_3way_3': TURN,
        'niosh_3wayslant_1': TURN,
        'niosh_3wayslant_2': TURN,
        'niosh_3wayslant_3': TURN,
        'niosh_4way_1': TURN,
        'niosh_4way_2': TURN,
        'niosh_4way_3': TURN,
        'niosh_4wayslant_1': TURN,
        'niosh_4wayslant_2': TURN,
        'niosh_4wayslant_3': TURN,
        'niosh_4wayslantreverse_1': TURN,
        'niosh_4wayslantreverse_2': TURN,
        'niosh_4wayslantreverse_3': TURN,
        'Urban Starting Area': STRAIGHT,
        'Urban Straight': STRAIGHT,
        'Urban Straight Lights': STRAIGHT,
        'Urban Elevation Up': STRAIGHT,
        'Urban Elevation Up Lights': STRAIGHT,
        'Urban Elevation Down': STRAIGHT,
        'Urban Straight Door Left': STRAIGHT,
        'Urban Straight Door Left Flipped': STRAIGHT,
        'Urban Straight Door Right': STRAIGHT,
        'Urban Straight Door Right Flipped': STRAIGHT,
        'Urban Straight Door Right Flipped Lights': STRAIGHT,
        'Urban Straight Door Right Extension': TURN,
        'Urban Straight Door Right Extension Lights': TURN,
        # 'Urban Stairwell Straight': TURN,
        'Urban Stairwell Platform': STRAIGHT,
        'Urban Stairwell Platform Centered': STRAIGHT,
        'Urban Stairwell Platform Centered Lights': STRAIGHT,
        'Urban Platform': TURN,
        'Urban Platform Open': TURN,
        # 'Urban Station': STRAIGHT,
        'Urban Service Room': STRAIGHT,
        'Urban Service Room Lights': STRAIGHT,
        'Urban Service Room Centered': STRAIGHT,
        'Urban Service Room Centered Lights': STRAIGHT,
        'Urban Bend Left': TURN,
        'Urban Bend Left Lights': TURN,
        'Urban Bend Right': TURN,
        'Urban 4way Intersection': TURN,
        'Urban 3-Way Right Intersection': TURN,
        'Urban Large Room Split': TURN,
        'Urban Large Room Split Lights': TURN,
        'Urban Superpose': TURN,
        'Urban Service Room Straight': STRAIGHT,
        'Urban Service Room Straight Lights': STRAIGHT,
        'Urban 2 Story': TURN,
        'Urban 2 Story Lights': TURN,
        'Urban 2 Story Large Side 1': STRAIGHT,
        'Urban 2 Story Large Side 1 Lights': STRAIGHT,
        'Urban 2 Story Large Side 2': STRAIGHT,
        'Urban 2 Story Large Side 2 Lights': STRAIGHT,
    }

    # Constraint rule 5: disconnect flat tiles at non-matching levels
    # {'tile_name': {(world_x, world_y): relative_level}, ...}
    #      where (world_x, world_y) specifies an endpoint of the tile in tile's
    #          0 orientation wrt z,
    #      relative_level is specified in -1, 0, +1. Adding or subtracting the
    #          relative level between two endpoints will give the difference
    #          that is expected between connecting tiles at those endpoints,
    #          if the three tiles are connected.
    STEP_TILES = {
        'tunnel_tile_6',
        'tunnel_tile_7',
        'Rough Tunnel Tile Ramp',
        'Rough Tunnel Tile Vertical Shaft',
        'Urban Elevation Up',
        'Urban Elevation Up Lights',
        'Urban Elevation Down',
    }
    MULTI_STEP_TILES = {
        'Urban 2 Story',
        'Urban 2 Story Lights',
        'Urban 2 Story Large Side 1',
        'Urban 2 Story Large Side 1 Lights',
        'Urban 2 Story Large Side 2',
        'Urban 2 Story Large Side 2 Lights',
        'Urban Stairwell Platform',
        'Urban Stairwell Platform Centered',
        'Urban Stairwell Platform Centered Lights',
        'Urban Superpose',
    }
    # Non-zero values are hardcoded based on mesh height.
    # TODO(mabelmzhang): This is not fully tested to work yet
    # TILE_STEPS = {
    #     'tunnel_tile_1': {(0, 1): 0, (0, -1): 0, (1, 0): 0, (-1, 0): 0},
    #     'tunnel_tile_2': {(1, 0): 0, (0, -1): 0},
    #     'tunnel_tile_3': {(0, 1): 0, (0, -1): 0, (1, 0): 0, (-1, 0): 0},
    #     'tunnel_tile_4': {(0, 1): 0, (0, -1): 0, (1, 0): 0, (-1, 0): 0},
    #     'tunnel_tile_5': {(0, 1): 0, (0, -1): 0},
    #     'tunnel_tile_6': {(0, 1): 1, (0, -1): -1},
    #     'tunnel_tile_7': {(0, 1): 1, (0, -1): -1},
    #     'constrained_tunnel_tile_tall': {(0, 1): 0, (0, -1): 0},
    #     'constrained_tunnel_tile_short': {(0, 1): 0, (0, -1): 0},
    #     'Rough Tunnel Tile 4-way Intersection': {(0, 1): 0, (0, -1): 0, (1, 0): 0, (-1, 0): 0},
    #     'Rough Tunnel Tile 90-degree Turn': {(1, 0): 0, (0, -1): 0},
    #     'Rough Tunnel Tile Straight': {(0, 1): 0, (0, -1): 0},
    #     'Rough Tunnel Tile Ramp': {(0, 1): 1, (0, -1): -1},
    #     'Rough Tunnel Tile Vertical Shaft': {(0, 1): 1, (0, -1): -1},
    # }

    # For comments in .dot
    INTERSECTIONS = ['tunnel_tile_1',
                     'tunnel_tile_4',
                     'Rough Tunnel Tile 4-way Intersection',
                     'niosh_3way_1',
                     'niosh_3way_2',
                     'niosh_3way_3',
                     'niosh_3wayslant_1',
                     'niosh_3wayslant_2',
                     'niosh_3wayslant_3',
                     'niosh_4way_1',
                     'niosh_4way_2',
                     'niosh_4way_3',
                     'niosh_4wayslant_1',
                     'niosh_4wayslant_2',
                     'niosh_4wayslant_3',
                     'niosh_4wayslantreverse_1',
                     'niosh_4wayslantreverse_2',
                     'niosh_4wayslantreverse_3',
                     'Urban 4way Intersection',
                     'Urban 3-Way Right Intersection',
                     'Urban Large Room Split',
                     'Urban Large Room Split Lights',
                     'Urban Superpose',
                     'Urban 2 Story',
                     'Urban 2 Story Lights',
                     ]

    # Assumption to constraints: tsv file is a valid tunnel system.
    #     Currently only checking ambiguous cases in a valid tsv specification.
    # Constraint rule 1:
    # Yaw of 90-degree corner tile resolves ambiguous edges e.g. when
    #     all cells in a 2 x 2 block in tsv are occupied
    # Now merged to ENDPOINTS check. Remove after testing that.
    # CORNER_TILES = ['tunnel_tile_2',
    #                'Rough Tunnel Tile 90-degree Turn',
    # ]
    # NIOSH_CORNER_TILES = ['niosh_corner_1',
    #                      'niosh_corner_2',
    #                      'niosh_corner_3',
    # ]
    # Constraint rule 2: Yaw of tiles resolves ambiguous edges
    # Endpoints in tiles' original orientation, i.e. yaw == 0.
    # A more simplistic and systematic way for determining connectivity.
    #     Not all tiles use this yet.
    #     TODO: Eventually should update all tiles to use this.
    # 'tile_prefix': ((endpoint1_x, endpoint1_y), ...)
    #     Points must be (x, y) in world frame, to multiply by rotation matrix correctly.
    #     To get y in world frame, multiply by -1 (flipped).
    #     To rotate to yaw angle, multiply 2D rotation matrix wrt z by the endpoint.
    ENDPOINTS = {
        ### Tunnel tiles
        # TODO: populated from check_corner_tile_connection(). Test these
        'tunnel_tile_1': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'tunnel_tile_2': ((1, 0), (0, -1)),
        'tunnel_tile_3': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'tunnel_tile_4': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'tunnel_tile_5': ((0, 1), (0, -1)),
        'tunnel_tile_6': ((0, 1), (0, -1)),
        'tunnel_tile_7': ((0, 1), (0, -1)),
        'constrained_tunnel_tile_tall': ((0, 1), (0, -1)),
        'constrained_tunnel_tile_short': ((0, 1), (0, -1)),
        'Rough Tunnel Tile 4-way Intersection': ((0, 1), (0, -1), (1, 0), (-1, 0)),
        'Rough Tunnel Tile 90-degree Turn': ((1, 0), (0, -1)),
        'Rough Tunnel Tile Straight': ((0, 1), (0, -1)),
        'Rough Tunnel Tile Ramp': ((0, 1), (0, -1)),
        'Rough Tunnel Tile Vertical Shaft': ((0, 1), (0, -1)),
        ### NIOSH intersections
        'niosh_corner_1': ((0, 1), (1, 0)),
        'niosh_corner_2': ((0, 1), (1, 0)),
        'niosh_corner_3': ((0, 1), (1, 0)),
        # Orientation of niosh linear (overpass and straight) tiles are consistent
        'niosh_straight_1': ((0, 1), (0, -1)),
        'niosh_straight_2': ((0, 1), (0, -1)),
        'niosh_straight_3': ((0, 1), (0, -1)),
        'niosh_straightlong_1': ((0, 1), (0, -1)),
        'niosh_straightlong_2': ((0, 1), (0, -1)),
        'niosh_straightlong_3': ((0, 1), (0, -1)),
        'niosh_overpass': ((0, 1), (0, -1)),
        # Orientation of niosh_3way tiles are consistent
        'niosh_3way_1': ((1, 0), (-1, 0), (0, 1)),
        'niosh_3way_2': ((1, 0), (-1, 0), (0, 1)),
        'niosh_3way_3': ((1, 0), (-1, 0), (0, 1)),
        # Orientation of niosh_3wayslant tiles are opposite of niosh_3way_
        'niosh_3wayslant_1': ((-1, 0), (1, 0), (0, -1)),
        'niosh_3wayslant_2': ((-1, 0), (1, 0), (0, -1)),
        'niosh_3wayslant_3': ((-1, 0), (1, 0), (0, -1)),
        'niosh_4way_1': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4way_2': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4way_3': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslant_1': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslant_2': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslant_3': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslantreverse_1': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslantreverse_2': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'niosh_4wayslantreverse_3': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        ### Urban
        'Urban Straight': ((0, 1), (0, -1)),
        'Urban Straight Lights': ((0, 1), (0, -1)),
        'Urban Elevation Up': ((0, 1), (0, -1)),
        'Urban Elevation Up Lights': ((0, 1), (0, -1)),
        'Urban Elevation Down': ((0, 1), (0, -1)),
        'Urban Straight Door Left': ((1, 0), (-1, 0), (0, -1)),
        'Urban Straight Door Left Flipped': ((1, 0), (-1, 0), (0, -1)),
        'Urban Straight Door Right': ((1, 0), (-1, 0), (0, 1)),
        'Urban Straight Door Right Flipped': ((1, 0), (-1, 0), (0, 1)),
        'Urban Straight Door Right Flipped Lights': ((1, 0), (-1, 0), (0, 1)),
        'Urban Straight Door Right Extension': ((1, 0), (-1, 0), (0, 1)),
        'Urban Straight Door Right Extension Lights': ((1, 0), (-1, 0), (0, 1)),
        # 'Urban Stairwell Straight': ((0, 1), (0, -1)),
        # 'Urban Station': ((0, 1), (0, -1)),
        'Urban Stairwell Platform': ((0, 1), (0, -1)),
        'Urban Stairwell Platform Centered': ((0, 1), (0, -1)),
        'Urban Stairwell Platform Centered Lights': ((0, 1), (0, -1)),
        'Urban Platform': ((1, 0), (-1, 0), (0, 1)),
        'Urban Platform Open': ((1, 0), (-1, 0), (0, 1)),
        # Make sure this is a tuple of tuples, i.e. ((),) with comma. Else will
        #     get shrunk to a single-layer tuple and mess up matrix sizes.
        'Urban Service Room': ((0, -1),),
        'Urban Service Room Lights': ((0, -1),),
        'Urban Service Room Centered': ((0, -1),),
        'Urban Service Room Centered Lights': ((0, -1),),
        'Urban Bend Left': ((-1, 0), (0, -1)),
        'Urban Bend Left Lights': ((-1, 0), (0, -1)),
        'Urban Bend Right': ((1, 0), (0, -1)),
        'Urban 4way Intersection': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'Urban 3-Way Right Intersection': ((1, 0), (0, 1), (0, -1)),
        'Urban Large Room Split': ((-1, 0), (0, 1), (0, -1)),
        'Urban Large Room Split Lights': ((-1, 0), (0, 1), (0, -1)),
        'Urban Superpose': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'Urban Service Room Straight': ((0, 1), (0, -1)),
        'Urban Service Room Straight Lights': ((0, 1), (0, -1)),
        'Urban 2 Story': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'Urban 2 Story Lights': ((1, 0), (-1, 0), (0, 1), (0, -1)),
        'Urban 2 Story Large Side 1': ((0, 1), (0, -1)),
        'Urban 2 Story Large Side 1 Lights': ((0, 1), (0, -1)),
        'Urban 2 Story Large Side 2': ((0, 1), (0, -1)),
        'Urban 2 Story Large Side 2 Lights': ((0, 1), (0, -1)),
    }

    # Tiles that are defined as subtiles in the .tsv, in the same format as
    #     artifacts, but may have a graph connection to the main tile in the
    #     same tsv cell. To check the connection between the main tile and the
    #     subtile require a self-edge for the (y, x) cell and therefore cannot
    #     be obtained by simple cell adjacency assumption.
    SUB_TILES = {'Urban Service Room', 'Urban Service Room Centered',
                 'Urban Service Room Lights', 'Urban Service Room Centered Lights'
                }

    # Constraint rule 3: parallel non-intersecting (not on same line) linear
    #     tiles cannot be connected. Check yaw to determine connection.
    # 'tile_name': orientation yaw in degrees at which the linear tile is
    #     parallel to world x-axis.
    LINEAR_TILES = {'tunnel_tile_5': 90,
                    'tunnel_tile_6': 90,
                    'tunnel_tile_7': 90,
                    'constrained_tunnel_tile_tall': 90,
                    'constrained_tunnel_tile_short': 90,
                    'Rough Tunnel Tile Straight': 90,
                    'Rough Tunnel Tile Ramp': 90,
                    'Rough Tunnel Tile Vertical Shaft': 90,
                    'niosh_overpass': 90,
                    'niosh_straight_1': 90,
                    'niosh_straight_2': 90,
                    'niosh_straight_3': 90,
                    'niosh_straightlong_1': 90,
                    'niosh_straightlong_2': 90,
                    'niosh_straightlong_3': 90,
                    'Urban Straight': 90,
                    'Urban Elevation Up': 90,
                    'Urban Elevation Down': 90,
                    # Urban Straight Door is not strictly linear, but because there is no way to
                    #     distinguish between Straight Door next to base station (connected), service
                    #     room (connected), or another Straight Door (not connected), will use
                    #     linearity check to catch the not-connected case.
                    'Urban Straight Door Left': 0,
                    'Urban Straight Door Left Flipped': 0,
                    'Urban Straight Door Right': 0,
                    'Urban Straight Door Right Flipped': 0,
                    'Urban Straight Door Right Flipped Lights': 0,
                    # 'Urban Station': 90,
                    'Urban Service Room': 90,
                    'Urban Service Room Lights': 90,
                    'Urban Service Room Centered': 90,
                    'Urban Service Room Centered Lights': 90,
                    'Urban Stairwell Platform': 90,
                    'Urban Stairwell Platform Centered': 90,
                    'Urban Stairwell Platform Centered Lights': 90,
                    # 'Urban Stairwell Straight': 90,
                    'Urban Service Room Straight': 90,
                    'Urban Service Room Straight Lights': 90,
                    'Urban 2 Story Large Side 1': 90,
                    'Urban 2 Story Large Side 1 Lights': 90,
                    'Urban 2 Story Large Side 2': 90,
                    'Urban 2 Story Large Side 2 Lights': 90,
                    }
    # Constraint rule 4: disconnect tiles with a blocker between them
    BLOCKER_TILE = 'tunnel_tile_blocker'

    # Padding
    # Extra-long cells
    PAD_TILE_PREFIXES = ['niosh_straightlong', 'niosh_overpass']
    # Cell with this positive +_ offset is expected to be empty, pad it.
    # For straightlong, it is 2 cells long, and the next cell is expected to be empty
    # For overpass, it is 3 cells long, the next cell may be occupied, and the 2nd next cell is expected to be empty
    PAD_TILE_OFFSETS = [1, 2]
    # Define within [0, 360)
    # For 270, pad 1 cell to +x (tsv column to the right, cell x + 1)
    PAD_X_YAWS = [90, 270]
    PAD_X_OFFSET_SIGNS = [-1, 1]
    # For 180, pad 1 cell to -y in world frame (tsv row above, cell y + 1)
    PAD_Y_YAWS = [0, 180]
    # Offests are specified in terms of tsv cells. To get world frame, multiply y by -1
    PAD_Y_OFFSET_SIGNS = [-1, 1]

    # Ignored in scene graph
    # Only needed if these are specified in individual cells, as opposed to after @ in a tile cell.
    ARTIFACTS = ['Backpack', 'Electrical Box', 'Extinguisher', 'Phone',
                 'Radio', 'Survivor Female', 'Survivor Male', 'Toolbox', 'Valve',
                 'Drill', 'Rescue Randy', 'Vent', 'Gas', 'Cube',
                 BLOCKER_TILE, 'niosh_blocker', 'niosh_blocker1', 'niosh_blocker2', 'niosh_blocker3']

    @classmethod
    def calc_edge_cost(self, mesh1, mesh2):

        if not mesh1 in self.STRAIGHTNESS:
            print("Key '%s' not found in GraphRules.STRAIGHTNESS. Is it defined?" % (
                mesh1), file=sys.stderr)
        if not mesh2 in self.STRAIGHTNESS:
            print("Key '%s' not found in GraphRules.STRAIGHTNESS. Is it defined?" % (
                mesh2), file=sys.stderr)
        try:
            # Heuristic: if both tiles are straight, cost 1;
            #   if both are turns, cost 6;
            #   otherwise (one is straight, one is a turn), cost 3.
            if self.STRAIGHTNESS[mesh1] == self.STRAIGHT and \
                    self.STRAIGHTNESS[mesh2] == self.STRAIGHT:
                return 1
            elif self.STRAIGHTNESS[mesh1] == self.TURN and \
                    self.STRAIGHTNESS[mesh2] == self.TURN:
                return 6
            else:
                return 3
        except KeyError:
            if mesh1 in self.ARTIFACTS or mesh2 in self.ARTIFACTS:
                return 0
            else:
                raise

    '''
    # Constraint rule 1: corner tile yaw degrees
    #     cdy, cdx: current dy and dx, of cell indices in tsv, with respect to
    #         corner tile.
    @classmethod
    def check_corner_tile_connection(self, cdy, cdx, yaw):

        is_connected = True

        # Keep angles positive in range [0, 360)
        yaw = yaw % 360

        # Hardcoded based on corner tile mesh
        # Yaw degrees are with reference to the corner tile
        # yaw 0, neighbors are necessarily in cells (y=0, x=+1) or (+1, 0)
        #     (right, below)
        if abs(yaw - 0) < 1e-6:
            if not ((cdy == 0 and cdx == 1) or (cdy == 1 and cdx == 0)):
                is_connected = False
        # yaw 90, neighbors are necessarily in cells (0, +1) or (-1, 0)
        #     (right, above)
        elif abs(yaw - 90) < 1e-6:
            if not ((cdy == 0 and cdx == 1) or (cdy == -1 and cdx == 0)):
                is_connected = False
        # yaw 180, neighbors are necessarily in cells (0, -1) or (-1, 0)
        #     (left, above)
        elif abs(yaw - 180) < 1e-6:
            if not ((cdy == 0 and cdx == -1) or (cdy == -1 and cdx == 0)):
                is_connected = False
        # yaw 270 (or -90), neighbors are necessarily in cells (0, -1) or
        #   (+1, 0) (left, below)
        elif abs(yaw - 270) < 1e-6 or abs (yaw + 90) < 1e-6:
            if not ((cdy == 0 and cdx == -1) or (cdy == 1 and cdx == 0)):
                is_connected = False

        return is_connected


    @classmethod
    def check_niosh_corner_tile_connection(self, cdy, cdx, yaw):
        is_connected = True
        # Keep angles positive in range [0, 360)
        yaw = yaw % 360

        # Hardcoded based on corner tile mesh
        # Neighbors are necessarily in cells (cell y=-1, x=0) or (y=0, x=1) (above, right)
        if abs(yaw - 0) < 1e-6:
            if not ((cdy == -1 and cdx == 0) or (cdy == 0 and cdx == 1)):
                is_connected = False
        # Neighbors are necessarily in cells (cell y=-1, x=0) or (y=0, x=-1) (below, left)
        elif abs(yaw - 90) < 1e-6:
            if not ((cdy == -1 and cdx == 0) or (cdy == 0 and cdx == -1)):
                is_connected = False
        # Neighbors are necessarily in cells (cell y=1, x=0) or (y=0, x=-1) (below, left)
        elif abs(yaw - 180) < 1e-6:
            if not ((cdy == 1 and cdx == 0) or (cdy == 0 and cdx == -1)):
                is_connected = False
        # Neighbors are necessarily in cells (cell y=1, x=0) or (y=0, x=1) (below, left)
        elif abs(yaw - 270) < 1e-6 or abs (yaw + 90) < 1e-6:
            if not ((cdy == 1 and cdx == 0) or (cdy == 0 and cdx == 1)):
                is_connected = False

        return is_connected
    '''

    # Constraint rule 2: specific to NIOSH 3-way intersection tiles.
    #     Connectivity depends on rotation of 3-way intersection tile.
    #     cdy, cdx: current dy and dx, of cell indices in tsv, with respect to
    #         corner tile.
    @classmethod
    def check_endpoints_tile_connection(self, cdy, cdx, yaw, this_mesh, that_yaw, that_mesh):

        # Find endpoints of the other tile
        if that_mesh in GraphRules.ENDPOINTS.keys():
            that_endpoints = GraphRules.ENDPOINTS[that_mesh]
        else:
            print('ERROR: %s undefined in ENDPOINTS. Behavior undefined from here on' % (
                that_mesh), file=sys.stderr)

        if this_mesh in GraphRules.ENDPOINTS.keys():
            this_endpoints = GraphRules.ENDPOINTS[this_mesh]
        else:
            print('ERROR: %s undefined in ENDPOINTS. Behavior undefined from here on' % (
                this_mesh), file=sys.stderr)

        # Keep angles positive in range [0, 360)
        yaw = yaw % 360
        that_yaw = that_yaw % 360

        # 2D rotation matrix
        yaw_r = yaw * np.pi / 180.0
        yaw_mat = np.array(((np.cos(yaw_r), -np.sin(yaw_r)),
                            (np.sin(yaw_r), np.cos(yaw_r))))
        that_yaw_r = that_yaw * np.pi / 180.0
        that_yaw_mat = np.array(((np.cos(that_yaw_r), -np.sin(that_yaw_r)),
                                 (np.sin(that_yaw_r), np.cos(that_yaw_r))))

        # Check orientation of the other tile actually has an endpoint
        #     connecting to an endpoint of this tile.
        # If ANY endpoint of the two tiles connect, then they are connected
        for this_endpt in this_endpoints:
            # Rotate endpoint to the yaw of this tile
            this_endpt_yawed = np.dot(yaw_mat, this_endpt)
            for that_endpt in that_endpoints:
                # Transform by rotation and translation from this cell
                that_endpt_yawed = np.dot(that_yaw_mat, that_endpt)

                # Self-loops in the same tile
                if cdx == 0 and cdy == 0:
                    if np.all(np.abs(this_endpt_yawed + that_endpt_yawed) < 1e-6):
                        return True
                # Regular connections for adjacent cells need to satisfy constraints on cdx and cdy
                else:
                    # Endpoints must be opposite to be connected,
                    #    e.g. +x of this tile connects to -x of that tile.
                    # Endpoint of that tile must also align with position of that tile wrt this tile.
                    if np.all(np.abs(this_endpt_yawed + that_endpt_yawed) < 1e-6) and \
                            np.all(np.abs(that_endpt_yawed + np.array((cdx, -cdy))) < 1e-6):
                        return True

        return False

    # Constraint rule 3: parallel non-intersecting linear tiles aren't neighbors
    @classmethod
    def check_linear_tile_connection(self, mesh1, y1, x1, yaw1, mesh2, y2, x2, yaw2):

        # Keep angles positive in range [0, 360)
        yaw1 = yaw1 % 360
        yaw2 = yaw2 % 360

        # Yaws at which the meshes are parallel to world x- and y-axes.
        # At this point can assume yaw2 || yaw1. So only need to check yaw1.
        # Truncate to range [0, 180), so that when added to 180, angle stays in
        #     [0, 360).
        XPARALLEL_YAW1 = self.LINEAR_TILES[mesh1] % 180
        YPARALLEL_YAW1 = (XPARALLEL_YAW1 + 90) % 180

        # If consecutive parallel linear tiles are not on the same line, they
        #     are not connected.
        #     yaw parallel to x: y1 == y2 (same tsv row) required.
        #     yaw parallel to y: x1 == x2 (same tsv column) required.
        if abs(yaw1 - YPARALLEL_YAW1) < 1e-6 or abs(yaw1 - (YPARALLEL_YAW1 + 180)) < 1e-6:
            if x1 != x2:
                return False
        elif abs(yaw1 - XPARALLEL_YAW1) < 1e-6 or abs(yaw1 - (XPARALLEL_YAW1 + 180)) < 1e-6:
            if y1 != y2:
                return False

        return True

    # Constraint rule 4: disconect tiles with a blocker between them
    #     (y1, x1), (y2, x2): Coordinates of two adjacent cells in the tsv.
    #     blocker_xyz#: [(x, y, z), ...] local position(s) of blocker(s) wrt
    #         tile.
    @classmethod
    def check_blocker_tile_connection(self, y1, x1, blocker_xyz1, y2, x2, blocker_xyz2):

        # Assumptions: exactly one of x or y is non-zero. That is the direction
        #     the blocker will be placed. No connections to diagonal cells.

        # If tile 1 has blockers
        if blocker_xyz1 != None:
            # Use tile 1 (y, x) coords as reference
            # Flip y. y in tsv increases downwards. y in world decreases downwards.
            dy = -(y2 - y1)
            dx = x2 - x1

            # Check each blocker in the list
            for blk_xyz in blocker_xyz1:
                # Break connection if blocker is in between the two tiles
                # dx > 0 places blocker toward the cell to the right
                if np.sign(blk_xyz[0]) == np.sign(dx) and np.sign(blk_xyz[1]) == np.sign(dy):
                    return False

        if blocker_xyz2 != None:
            # Use tile 2 (y, x) coords as reference
            # Flip y. y in tsv increases downwards. y in world decreases downwards.
            dy = -(y1 - y2)
            dx = x1 - x2

            # Check each blocker in the list
            for blk_xyz in blocker_xyz2:
                if np.sign(blk_xyz[0]) == np.sign(dx) and np.sign(blk_xyz[1]) == np.sign(dy):
                    return False

        return True

    # Constraint rule 5: disconnect tiles at non-matching levels
    # Parameters:
    # pos_2_wrt_1: (dx, dy) relative tsv cell position of cell 2 wrt cell 1
    @classmethod
    def check_z_level(self, mesh1, yaw1, lvl1, mesh2, yaw2, lvl2, pos_2_wrt_1, scale_z):

        # print('checking z level', file=sys.stderr)

        # If one of the tiles contains a multi-level step
        if mesh1 in GraphRules.MULTI_STEP_TILES or \
                mesh2 in GraphRules.MULTI_STEP_TILES:
            # Arbitrarily assume anything within 2 levels apart are connected.
            #   This is mostly for Urban 2 Story tiles, without having to
            #   define specific levels in each orientation for each individual
            #   tile.
            if abs(lvl1 - lvl2) <= 2:
                return True
            else:
                print('DEBUG: Level %g and %g are not within the range (<= 2) required for a multi-step tile (%s OR %s)' % (
                        lvl1, lvl2, mesh1, mesh2))
                return False

        # If neither tile contains a step, i.e. both flat, their levels must
        #     match to be connected
        if not mesh1 in GraphRules.STEP_TILES and \
                not mesh2 in GraphRules.STEP_TILES:
            if lvl1 == lvl2:
                return True
            else:
                print('DEBUG: Level %g does not match %g in two non-step tiles %s and %s' % (lvl1, lvl2, mesh1, mesh2))
                return False

        # If both tiles are step tiles
        elif mesh1 in GraphRules.STEP_TILES and \
                mesh2 in GraphRules.STEP_TILES:

            # Simple check that might work. Slanted tiles are only 1 level deep.
            # 0 for when 2 tiles go up and down each, 1 for when both go down/up
            if abs(lvl1 - lvl2) <= 1:
                return True
            else:
                print('DEBUG: Level %g and %g are not within the range (<= 1) required for two single-step tiles %s and %s' % (
                        lvl1, lvl2, mesh1, mesh2))
                return False

        # If only one tile contains a step, check tile orientation to
        #     determine level on the connecting side
        else:
            # Simple check that might work. Slanted tiles are only 1 level deep.
            #     Just check for 1 absolute difference no matter which way - note
            #     this might connect tiles even if the slanted tile is oriented
            #     the wrong way and not connected.
            # 1 for going from a higher tile to a slanted tile
            # 0 for going from a slanted tile to a lower tile
            if abs(lvl1 - lvl2) <= 1:
                return True
            else:
                print('DEBUG: Level %g and %g are not within the range (<= 1) required for a single-step tile (%s OR %s)' % (
                        lvl1, lvl2, mesh1, mesh2))
                return False

            # TODO Slanted tiles inherently have different levels and need more constants defined
            #   Will not take care of tiles containing a step rigorously right now.
            '''
            if mesh1 in GraphRules.STEP_TILES:
                stepmesh = mesh1
                stepyaw = yaw1
                steplvl = lvl1
                flatmesh = mesh2
                flatyaw = yaw2
                flatlvl = lvl2
                # Cell position of flat tile wrt step tile, in tsv cell position
                flat_cell_wrt_step = np.array(pos_2_wrt_1)
            elif mesh2 in GraphRules.STEP_TILES:
                stepmesh = mesh2
                stepyaw = yaw2
                steplvl = lvl2
                flatmesh = mesh1
                flatyaw = yaw1
                flatlvl = lvl1
                flat_cell_wrt_step = -np.array(pos_2_wrt_1)
            # Flip tsv y to get world y position
            flat_tile_wrt_step = np.array((flat_cell_wrt_step[0], -flat_cell_wrt_step[1]))

            stepyaw_r = stepyaw * np.pi / 180.0
            stepyaw_mat = np.array((
                (np.cos(stepyaw_r), -np.sin(stepyaw_r)),
                (np.sin(stepyaw_r), np.cos(stepyaw_r))))

            flatyaw_r = flatyaw * np.pi / 180.0
            flatyaw_mat = np.array((
                (np.cos(flatyaw_r), -np.sin(flatyaw_r)),
                (np.sin(flatyaw_r), np.cos(flatyaw_r))))

            # For each endpoint of this step tile
            for step_endpt in GraphRules.TILE_STEPS[stepmesh].keys():

                # Rotate endpoint
                step_endpt_yawed = np.dot(stepyaw_mat, step_endpt)

                # Find the relative level at the endpoint
                step_rel_lvl = GraphRules.TILE_STEPS[stepmesh][step_endpt]

                for flat_endpt in GraphRules.TILE_STEPS[flatmesh].keys():

                    # Rotate endpoint
                    flat_endpt_yawed = np.dot(flatyaw_mat, flat_endpt)

                    # Find the relative level at the endpoint
                    flat_rel_lvl = GraphRules.TILE_STEPS[flatmesh][flat_endpt]

                    print('%s endpoint (%g, %g), %s endpoint (%g, %g)' % (
                          stepmesh, step_endpt_yawed[0], step_endpt_yawed[1], flatmesh, flat_endpt_yawed[0],
                          flat_endpt_yawed[1]), file=sys.stderr)
                    print('%s endpoint z %g, %s endpoint z %g' % (
                          stepmesh, steplvl + step_rel_lvl * scale_z, flatmesh, flatlvl + flat_rel_lvl * scale_z),
                          file=sys.stderr)

                    print(flat_endpt_yawed - step_endpt_yawed)
                    print(flat_tile_wrt_step)

                    # If current pair of endpoints matches the actual pair in tsv
                    # TODO: This subtraction never produces the flat_tile_wrt_step truth! This is not right!
                    if np.all(np.abs((flat_endpt_yawed - step_endpt_yawed) - flat_tile_wrt_step) < 1e-6):
                        # If z match, then two tiles are connected
                        if steplvl + step_rel_lvl * scale_z == flatlvl + flat_rel_lvl * scale_z:
                            print('Returning true', file=sys.stderr)
                            return True

            # No match in any endpoints found. Tiles not connected
            return False
            '''


def parse_args():
    parser = argparse.ArgumentParser(
        'Generate tiled world and connectivity graph files from tsv. '
        'The graph file is not written if the --graph-file argument is not specified.')
    parser.add_argument('tsv_name', help='name of tsv file to read')
    parser.add_argument('--urban', action='store_true', help='Use Urban world tiles (default is Tunnel world tiles)')
    parser.add_argument('--world-name', dest='world_name', type=str, default='default', help='world name')
    parser.add_argument('--world-file', dest='world_file', type=str, default='', help='world output file')
    parser.add_argument('--gz9', action='store_true', default=False,
                        help='Generate Gazebo 9 world files. By default, Ignition world files are generated')
    parser.add_argument('--graph-file', dest='graph_file', type=str, default='', help='dot graph output file')
    parser.add_argument('--x0', dest='x0', type=float, default=0, help='origin X coordinate')
    parser.add_argument('--y0', dest='y0', type=float, default=0, help='origin Y coordinate')
    parser.add_argument('--z0', dest='z0', type=float, default=0, help='origin Z coordinate')
    parser.add_argument('--scale_x', dest='scale_x', type=float, default=20, help='tile scale in X')
    parser.add_argument('--scale_y', dest='scale_y', type=float, default=20, help='tile scale in Y')
    parser.add_argument('--scale_z', dest='scale_z', type=float, default=5, help='tile scale in Z')
    parser.add_argument('--wind_x', dest='wind_x', type=float, default=0, help='global wind velocity in X')
    parser.add_argument('--wind_y', dest='wind_y', type=float, default=0, help='global wind velocity in Y')
    parser.add_argument('--wind_z', dest='wind_z', type=float, default=0, help='global wind velocity in Z')
    parser.add_argument('--levels_sx', dest='levels_sx', type=float, default=620, help='Levels box size in X')
    parser.add_argument('--levels_sy', dest='levels_sy', type=float, default=620, help='Levels box size in Y')
    parser.add_argument('--levels_sz', dest='levels_sz', type=float, default=130, help='Levels box size in Z')
    parser.add_argument('--levels_buf', dest='levels_buf', type=float, default=10, help='Levels box buffer size')
    parser.add_argument('--level_type', dest='level_type', type=str, default='single_tile',
                        help='Type of tile generation method: single_tile or row_col')
    parser.add_argument('--map-file', type=str,
                        default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "type_to_path.yaml"),
                        help='YAML file mapping model types in tsv to file paths')
    args = parser.parse_args()
    return args

# Simplify all paths to be put in header comment to basenames
# argv: List of string arguments
def simplify_header_paths(argv):

    simplified = []

    for arg in argv:
        # If this arg is a path, take only basename
        if os.path.exists(arg):
            simplified.append(os.path.basename(arg))
        else:
            simplified.append(arg)

    return simplified

def print_world_top(args, world_file, tile_type='tunnel'):
    if args.gz9:
        if tile_type == 'urban':
            stage_uri = 'model://Urban Starting Area'
        else:
            stage_uri = 'model://tunnel_staging_area'

    else:
        if tile_type == 'urban':
            artifact_origin_pos = '17.5 27 7.5'
            tent_pos = '1 27 7.5'
            stage_mesh = """
    <!-- The staging area -->
    <include>
      <static>true</static>
      <name>staging_area</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Urban Starting Area</uri>
    </include>"""

        # Default
        else:
            artifact_origin_pos = '10.0 0.0 0.0'
            tent_pos = '-8 0 0'
            stage_mesh = """
    <!-- The staging area -->
    <include>
      <static>true</static>
      <name>staging_area</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/subt_tunnel_staging_area</uri>
    </include>

    <!-- Barriers -->
    <include>
      <name>barrier_0_-1</name>
      <pose>8.0 -11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_0_1</name>
      <pose>8.0 11 0 0 0 0 </pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_1_-1</name>
      <pose>3.7 -11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_1_1</name>
      <pose>3.7 11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_2_-1</name>
      <pose>-0.5999999999999996 -11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_2_1</name>
      <pose>-0.5999999999999996 11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_3_-1</name>
      <pose>-4.899999999999999 -11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_3_1</name>
      <pose>-4.899999999999999 11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_4_-1</name>
      <pose>-9.2 -11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_4_1</name>
      <pose>-9.2 11 0 0 0 0</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-9.5_-2</name>
      <pose>10.5 -8.6 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-9.5_2</name>
      <pose>10.5 8.6 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-32_-2</name>
      <pose>-12 -8.6 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-32_-1</name>
      <pose>-12 -4.3 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-32_0</name>
      <pose>-12 0.0 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-32_1</name>
      <pose>-12 4.3 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>

    <include>
      <name>barrier_-32_2</name>
      <pose>-12 8.6 0 0 0 1.5707963267948966</pose>
      <uri> https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier </uri>
    </include>"""

    print("""<?xml version="1.0" ?>
<!--
  Generated with the {file_name} script:
    {command}
-->
<sdf version="1.6">
  <world name="{world_name}">""".format(
        file_name=os.path.basename(__file__), command=' '.join(simplify_header_paths(sys.argv)).replace('--', '-\-'),
        world_name=args.world_name), file=world_file)

    # Gazebo 9
    if args.gz9:
        print("""
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-6.3 -4.2 3.6 0 0.268 0.304</pose>
      </camera>
    </gui>

    <scene>
      <ambient>0.1 0.1 0.1 1.0</ambient>
      <background>0 0 0 1.0</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- The base station / staging area -->
    <!-- Important: Do not rename this model! -->
    <include>
      <static>true</static>
      <name>BaseStation</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>{stage_uri}</uri>
    </include>

    <!-- Fiducial marking the origin for artifacts reports -->
    <include>
      <name>artifact_origin</name>
      <pose>10.0 0.0 0.0 0 0 0</pose>
      <uri>model://fiducial</uri>
    </include>


    <!-- Tunnel tiles and artifacts -->""".format(stage_uri=stage_uri), file=world_file)

    # Ignition
    else:
        print("""
    <physics name="1ms" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.1 0.1 0.1 1.0</ambient>
      <background>0 0 0 1.0</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

{stage_mesh}

    <!-- The base station -->
    <include>
      <static>true</static>
      <name>base_station</name>
      <pose>{tent_pos} 0 0 -1.5708</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Base Station</uri>
    </include>

    <!-- Fiducial marking the origin for artifacts reports -->
    <include>
      <name>artifact_origin</name>
      <pose>{artifact_origin_pos} 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Fiducial</uri>
    </include>

    <!-- Tunnel tiles and artifacts -->""".format(stage_mesh=stage_mesh, tent_pos=tent_pos,
                                                  artifact_origin_pos=artifact_origin_pos), file=world_file)


def print_graph_top(args, graph_file):
    print('''/* Visibility graph for %s */
/* Generated with the %s script: */
/*   %s */

graph {
  /* ==== Vertices ==== */

  /* Base station / Staging area */''' % (
        os.path.basename(args.tsv_name), os.path.basename(__file__), ' '.join(simplify_header_paths(sys.argv)).replace('--', '-\-')),
          file=graph_file)


def check_main():
    args = parse_args()

    if len(args.world_file) > 0:
        world_file = open(args.world_file, 'w')
    else:
        world_file = sys.stdout

    if len(args.graph_file) > 0:
        graph_file = open(args.graph_file, 'w')
    else:
        graph_file = open(os.devnull, 'w')

    if args.urban:
        tile_type = 'urban'
    else:
        tile_type = 'tunnel'

    print_graph_top(args, graph_file=graph_file)
    print_world_top(args, world_file=world_file, tile_type=tile_type)

    types_to_paths = None
    if (os.path.exists(args.map_file)):
        with open(args.map_file, 'rb') as mapf:
            types_to_paths = yaml.load(mapf)

    # Topology graph format
    # vert_id, vert_id, tile_type, vert_id
    vert_fmt_base = '  %d   [label="%d::%s::BaseStation"];'
    vert_fmt = '  %-3d [label="%d::%s::%s"];'
    # vert1_id, vert2_id, edge_cost
    edge_fmt = '  %-2s -- %-3d [label=%d];%s'

    # Data to store while reading tsv file, to infer graph node connections
    # (iy, ix): [iv1, iv2, ...]
    cell_to_iv = dict()
    # (iy, ix): ['', ...]
    cell_to_mesh = dict()
    # (iy, ix): [yaw1, yaw2, ...]
    cell_to_yaw = dict()
    # (iy, ix): [z1, z2, ...]
    cell_to_level = dict()
    # (iy, ix): [(dx, dy, dz), ...]
    cell_to_blocker = dict()

    self_loops = []

    # Keep a sorted list of vertex indices. This makes output prettier.
    # [(iy, ix), ...]
    iyx = []

    start_tile_ix = None
    start_tile_iy = None
    start_tile_iv = None
    start_type = None

    # Padded cells to merge with original cell. Used for NIOSH extra-long tiles
    # (iy_original, ix_original): (iy_padded, ix_padded)
    merge_padded_cells = dict()

    BASE_MESH = 'base_station'
    base_tile_iv = 0

    # First vertex is base station
    print(vert_fmt_base % (base_tile_iv, base_tile_iv, BASE_MESH), file=graph_file)
    print('', file=graph_file)

    # read from tsv spreadsheet file
    with open(args.tsv_name, 'rt') as tsvfile:
        spamreader = csv.reader(tsvfile, delimiter='\t')
        for iy, row in enumerate(spamreader):
            for ix, cell in enumerate(row):
                if (len(cell) > 0):
                    for parts in csv.reader([cell]):
                        # each cell of spreadsheet contains comma-separated
                        # value strings with at least 3 fields
                        # modelType,yawDegrees,z_level eg. 'tunnel_tile_1,180,-1'
                        # it can have more fields, which are described below
                        modelType = parts[0]
                        yawDegrees = float(parts[1])
                        z_level = float(parts[2])
                        pose_x = args.x0 + ix * args.scale_x
                        pose_y = args.y0 - iy * args.scale_y
                        pose_z = args.z0 + z_level * args.scale_z

                        # base_station cell is only for topology graph. Ignore it for SDF generation.
                        if modelType == BASE_MESH:
                            start_type = modelType
                            # Record the future tile to look for in a subsequent iteration
                            if args.urban:
                                # Urban start tile in yaw = 0 faces -y, so take the cell below
                                #    as starting node in graph
                                start_tile_ix = ix
                                start_tile_iy = iy + 1
                            else:
                                # Assumption: Base station always faces +x, so always take the cell
                                #     to the right of it as starting node in graph.
                                start_tile_ix = ix + 1
                                start_tile_iy = iy
                            continue

                        (modelName, iv) = generate_model_name("tile", modelType)
                        # Record vertex index
                        if iy == start_tile_iy and ix == start_tile_ix:
                            start_tile_iv = iv

                        # Generate SDF
                        print(model_include_string(modelName, modelType,
                                                   pose_x, pose_y, pose_z,
                                                   yawDegrees * math.pi / 180,
                                                   types_to_paths=types_to_paths, gz=args.gz9),
                              file=world_file)
                        generate_levels(modelName, args.level_type,
                                        pose_x, pose_y, pose_z,
                                        args.scale_x, args.scale_y, args.scale_z,
                                        args.levels_sx, args.levels_sy, args.levels_sz,
                                        args.levels_buf)

                        # Generate DOT. Print this tile as a vertex in topology graph.
                        # Ignore artifacts.
                        if modelType not in GraphRules.ARTIFACTS:
                            print(vert_fmt % (iv, iv, modelType, modelName),
                                  file=graph_file)

                            iyx.append((iy, ix))
                            # Yaw resolves ambiguous connected vertices
                            cell_to_yaw[(iy, ix)] = [yawDegrees]
                            cell_to_iv[(iy, ix)] = [iv]
                            cell_to_mesh[(iy, ix)] = [modelType]
                            cell_to_level[(iy, ix)] = [z_level]  # *args.scale_z]

                        # the 4th field is another string that contains a list
                        # of submodels and a relative pose where they are to be
                        # placed within the tile
                        # submodels are separated by `;` eg. 'Phone;tunnel_tile_blocker'
                        # a relative pose can be specified with @
                        # pose is specified like in sdformat but with angles in degrees
                        # eg. 'Phone@0 0 0.004 90 0 0;tunnel_tile_blocker@11 0 0 0 0 0'
                        if len(parts) > 3:
                            submodels = parts[3]
                            for submodel in submodels.split(';'):
                                subpose_xi = pose_x
                                subpose_yi = pose_y
                                subpose_zi = pose_z
                                # pose_roll and pose_pitch are printed as strings for now
                                # to minimize the diff for tunnel_qual.world
                                # which has a bunch of poses with no trailing zeros for roll and pitch
                                # like '100.0000 200.0000 0.0000 0 0 -1.57079'
                                # so let pose_roll and pose_pitch default to '0'
                                subpose_roll = '0'
                                subpose_pitch = '0'
                                subpose_yawDegrees = 0
                                subpose_yaw = 0.0

                                # separate name from pose string by splitting at `@`
                                submodelType = ''
                                subPoseStr = ''
                                submodelType_poseStr = submodel.split('@')
                                if len(submodelType_poseStr) == 0:
                                    print("ERROR: invalid submodel specification %s" % submodel)
                                    continue
                                submodelType = submodelType_poseStr[0]

                                # pose is optional
                                if len(submodelType_poseStr) >= 2:
                                    subPoseStr = submodelType_poseStr[1]
                                subpose = subPoseStr.split(' ')
                                # set position if only 3 pose values are given
                                if len(subpose) >= 3:
                                    subpose_xi += float(subpose[0])
                                    subpose_yi += float(subpose[1])
                                    subpose_zi += float(subpose[2])
                                # additionally set roll, pitch, yaw if 6 values are given
                                if len(subpose) == 6:
                                    # print pose_roll and pose_pitch as %f if
                                    # they aren't exactly '0'
                                    if subpose_roll != subpose[3]:
                                        subpose_roll = '%f' % (float(subpose[3]) * math.pi / 180)
                                    if subpose_pitch != subpose[4]:
                                        subpose_pitch = '%f' % (float(subpose[4]) * math.pi / 180)
                                    subpose_yawDegrees = float(subpose[5])
                                    subpose_yaw = subpose_yawDegrees * math.pi / 180

                                # Generate SDF
                                (submodelName, subiv) = generate_model_name("tile", submodelType)
                                print(model_include_string(submodelName, submodelType,
                                                           subpose_xi, subpose_yi, subpose_zi,
                                                           subpose_yaw,
                                                           pose_roll=subpose_roll,
                                                           pose_pitch=subpose_pitch,
                                                           types_to_paths=types_to_paths, gz=args.gz9),
                                      file=world_file)
                                generate_levels(submodelName, args.level_type,
                                                subpose_xi, subpose_yi, subpose_zi,
                                                args.scale_x, args.scale_y, args.scale_z,
                                                args.levels_sx, args.levels_sy, args.levels_sz,
                                                args.levels_buf)

                                # Record blocker position for inferring graph connectivity
                                if submodelType == GraphRules.BLOCKER_TILE:
                                    # Currently ignoring orientation of blocker. Just take position wrt main tile
                                    if (iy, ix) not in cell_to_blocker.keys():
                                        cell_to_blocker[(iy, ix)] = [
                                            (float(subpose[0]), float(subpose[1]), float(subpose[2]))]
                                    else:
                                        cell_to_blocker[(iy, ix)].append(
                                            (float(subpose[0]), float(subpose[1]), float(subpose[2])))

                                # Generate DOT for this subtile
                                # If submodel is a regular tunnel tile, which would be overlapping the main tile in this cell
                                if submodelType not in GraphRules.ARTIFACTS:
                                    # Generate DOT. Create a separate vertex for the regular tile
                                    print(vert_fmt % (subiv, subiv, submodelType, submodelName),
                                          file=graph_file)

                                    # Assumption: z is a whole multiple of scale of tile. Level = z / scale_z.
                                    #     Subtile's z is relative to main tile's z.
                                    # Level 0 is at z == 0.
                                    # TODO(mabelmzhang): Use level number, or use actual z value, to determine connectivity?
                                    # Use absolute subtile z to compute level of subtile
                                    #subz_level = subpose_zi / float(args.scale_z)
                                    # Use offset of subtile to compute level of subtile
                                    subz_level = z_level + float(subpose[2]) / args.scale_z

                                    # Should never happen if main tile is not artifact, which should always be the case
                                    if (iy, ix) not in iyx:
                                        iyx.append((iy, ix))
                                    if (iy, ix) not in cell_to_yaw.keys():
                                        cell_to_yaw[(iy, ix)] = [subpose_yawDegrees]
                                        cell_to_iv[(iy, ix)] = [subiv]
                                        cell_to_mesh[(iy, ix)] = [submodelType]
                                        cell_to_level[(iy, ix)] = [
                                            subz_level]  # [subpose_zi] #[subz_level*args.scale_z]
                                    else:
                                        # Yaw resolves ambiguous connected vertices
                                        # These keys should already exist from main model above.
                                        cell_to_yaw[(iy, ix)].append(subpose_yawDegrees)
                                        cell_to_iv[(iy, ix)].append(subiv)
                                        cell_to_mesh[(iy, ix)].append(submodelType)
                                        cell_to_level[(iy, ix)].append(
                                            subz_level)  # (subpose_zi) #(subz_level*args.scale_z)

                                    # Further consider "self-loop" connection for this subtile with the main tile
                                    if submodelType in GraphRules.SUB_TILES:
                                        self_loops.append((iy, ix))

                        # If this is a 2-cell long mesh, e.g. NIOSH extra-long
                        #     tiles, and the next cell is expected to be empty,
                        #     pad the next tile for topology adjacency.
                        #     Treat the padded tile as an alias of this tile.
                        for pad_i in range(len(GraphRules.PAD_TILE_PREFIXES)):
                            if modelType.startswith(GraphRules.PAD_TILE_PREFIXES[pad_i]):
                                # Unsigned magnitude of number of cells to offset
                                offset_mag = GraphRules.PAD_TILE_OFFSETS[pad_i]

                                # If tile oriented along x-axis
                                if yawDegrees % 360 in GraphRules.PAD_X_YAWS:
                                    # Calculate offset, in number of cells
                                    offset_sign = GraphRules.PAD_X_OFFSET_SIGNS[
                                        GraphRules.PAD_X_YAWS.index(yawDegrees % 360)]
                                    offset = offset_sign * offset_mag
                                    merge_padded_cells[(iy, ix)] = (iy, ix + offset)
                                    # Populate records the padded cell with the same records as the main tile
                                    iyx.append((iy, ix + offset))
                                    cell_to_yaw[(iy, ix + offset)] = cell_to_yaw[(iy, ix)]
                                    cell_to_iv[(iy, ix + offset)] = cell_to_iv[(iy, ix)]
                                    cell_to_mesh[(iy, ix + offset)] = cell_to_mesh[(iy, ix)]
                                    cell_to_level[(iy, ix + offset)] = cell_to_level[(iy, ix)]
                                    if (iy, ix) in cell_to_blocker.keys():
                                        cell_to_blocker[(iy, ix + offset)] = cell_to_blocker[(iy, ix)]

                                # Similarly for tile oriented along y-axis
                                elif yawDegrees % 360 in GraphRules.PAD_Y_YAWS:
                                    # Calculate offset, in number of cells
                                    offset_sign = GraphRules.PAD_Y_OFFSET_SIGNS[
                                        GraphRules.PAD_Y_YAWS.index(yawDegrees % 360)]
                                    offset = offset_sign * offset_mag
                                    merge_padded_cells[(iy, ix)] = (iy + offset, ix)
                                    # Populate records the padded cell with the same records as the main tile
                                    iyx.append((iy + offset, ix))
                                    cell_to_yaw[(iy + offset, ix)] = cell_to_yaw[(iy, ix)]
                                    cell_to_iv[(iy + offset, ix)] = cell_to_iv[(iy, ix)]
                                    cell_to_mesh[(iy + offset, ix)] = cell_to_mesh[(iy, ix)]
                                    cell_to_level[(iy + offset, ix)] = cell_to_level[(iy, ix)]
                                    if (iy, ix) in cell_to_blocker.keys():
                                        cell_to_blocker[(iy + offset, ix)] = cell_to_blocker[(iy, ix)]
                                # Should only match with one
                                break

    if start_tile_iv == None or start_type == None:
        print('ERROR: Could not find where to place %s in the graph. Did you specify it in the .tsv file, with the child cell in the expected (iy, ix)?' % BASE_MESH,
              file=sys.stderr)
        print(start_tile_iv, file=sys.stderr)
        print(start_type, file=sys.stderr)
        return

    if args.level_type == "row_col":
        update_levels_size(args.scale_x, args.scale_y, args.scale_z)

    # Print edges of topology graph
    print('''
  /* ==== Edges ==== */

  /* Base station */''', file=graph_file)
    # Base station (vertex base_tile_iv) to start tunnel tile
    print(edge_fmt % (base_tile_iv, start_tile_iv,
          GraphRules.calc_edge_cost(BASE_MESH, start_type), ''), file=graph_file)

    y, x = zip(*iyx)

    # Tile the array to n x n, then use vectorized subtraction
    yt = np.tile(y, (len(y), 1))
    xt = np.tile(x, (len(x), 1))
    dy = np.abs(yt - yt.T)
    dx = np.abs(xt - xt.T)

    dy = np.triu(dy)
    dx = np.triu(dx)

    # Indices of adjacent tiles r and c
    # Take upper triangle, to eliminate symmetric duplicates
    # Horizontal and vertical neighbors are adjacent, i.e. exactly one of dx
    #   and dy is 1.
    adjy1 = np.array(dy == 1)
    adjy0 = np.array(dy == 0)
    adjx1 = np.array(dx == 1)
    adjx0 = np.array(dx == 0)

    # Test (dy == 1 and dx == 0) or (dy == 0 and dx == 1)
    adj_r, adj_c = np.where(np.logical_or(np.logical_and(adjy1, adjx0),
                                          np.logical_and(adjy0, adjx1)))

    # Append self-loops to adjacency list
    if len(self_loops) > 0:
        self_loop_offset = len(y)

        # Append cell position of self-loops to coordinate lists
        self_loops_r, self_loops_c = zip(*self_loops)
        yl = list(y)
        xl = list(x)
        yl.extend(self_loops_r)
        xl.extend(self_loops_c)
        y = yl
        x = xl

        # Append indices of self-loops to adjacency lists
        adj_r = np.append(adj_r, range(self_loop_offset, len(y)))
        adj_c = np.append(adj_c, range(self_loop_offset, len(x)))

    edges = set()

    # For each pair of adjacent cells
    for t1, t2 in zip(adj_r, adj_c):
        # Cell positions
        y1 = y[t1]
        x1 = x[t1]
        y2 = y[t2]
        x2 = x[t2]

        # To record self-loops that have already been checked once. Don't need
        #     to check both (t1, t2) and (t2, t1). Happens between a main tile
        #     and its subtiles.
        self_loop_checked = []

        # TODO(mabelmzhang): Test with NIOSH extra-long cells, this needs disambiguation if a NIOSH
        #   extra-long tile is part of a cell with multiple tiles at different levels - there was
        #   already a case with this in NIOSH 10 or 11. Just test with that and diff the SDF and DOT files.
        # If one of the cells is the padded alias of the other cell, skip.
        #     They are the same cell, not really adjacent. This gets rid of self loops.
        if ((y1, x1) in merge_padded_cells.keys() and merge_padded_cells[(y1, x1)] == (y2, x2)) or \
                ((y2, x2) in merge_padded_cells.keys() and merge_padded_cells[(y2, x2)] == (y1, x1)):
            continue

        # For each tile in a cell (even though this is m x n loop, most cells
        #   have only 1 tile. But multiple tiles at different levels in the
        #   same cell is allowed).
        for tile1 in range(len(cell_to_iv[y1, x1])):
            for tile2 in range(len(cell_to_iv[y2, x2])):

                # Same cell
                if t1 == t2:
                    # Same vertex (tile)
                    # Eliminate self-loops. This happens between a main tile and its subtiles.
                    if tile1 == tile2:
                        continue
                    # If (tile1, tile2) has been checked, don't need to check (tile2, tile1).
                    #     Else it results in a loop between two vertices.
                    elif set((tile1, tile2)) in self_loop_checked:
                        continue
                    else:
                        self_loop_checked.append(set((tile1, tile2)))

                # Unique vertex (tile) IDs
                iv1 = cell_to_iv[y1, x1][tile1]
                iv2 = cell_to_iv[y2, x2][tile2]
                mesh1 = cell_to_mesh[y1, x1][tile1]
                mesh2 = cell_to_mesh[y2, x2][tile2]
                yaw1 = cell_to_yaw[y1, x1][tile1]
                yaw2 = cell_to_yaw[y2, x2][tile2]
                lvl1 = cell_to_level[y1, x1][tile1]
                lvl2 = cell_to_level[y2, x2][tile2]

                blocker1 = (y1, x1) in cell_to_blocker.keys()
                if blocker1:
                    blocker1 = cell_to_blocker[y1, x1]
                else:
                    blocker1 = None
                blocker2 = (y2, x2) in cell_to_blocker.keys()
                if blocker2:
                    blocker2 = cell_to_blocker[y2, x2]
                else:
                    blocker2 = None

                # Resolve ambiguities in connectivity, e.g. 2 x 2 blocks in tsv

                is_connected = True

                # Now merged to ENDPOINTS check. Remove after testing that.
                '''
                # Constraint rule 1
                check_corner = False
                check_niosh_corner = False
                # Set corner tile as reference tile, subtract reference tile
                if mesh1 in GraphRules.CORNER_TILES + GraphRules.NIOSH_CORNER_TILES:
                    cdy = y2 - y1
                    cdx = x2 - x1
                    cy = y1
                    cx = x1
                    ctile = tile1
                    if mesh1 in GraphRules.CORNER_TILES:
                        check_corner = True
                    elif mesh1 in GraphRules.NIOSH_CORNER_TILES:
                        check_niosh_corner = True
                elif mesh2 in GraphRules.CORNER_TILES + GraphRules.NIOSH_CORNER_TILES:
                    cdy = y1 - y2
                    cdx = x1 - x2
                    cy = y2
                    cx = x2
                    ctile = tile2
                    if mesh2 in GraphRules.CORNER_TILES:
                        check_corner = True
                    elif mesh2 in GraphRules.NIOSH_CORNER_TILES:
                        check_niosh_corner = True
                if check_corner:
                    is_connected = GraphRules.check_corner_tile_connection(
                        cdy, cdx, cell_to_yaw[cy, cx][ctile])
                if check_niosh_corner:
                    is_connected = GraphRules.check_niosh_corner_tile_connection(
                        cdy, cdx, cell_to_yaw[cy, cx][ctile])
                '''

                # Constraint rule 2: specific to NIOSH 3-way intersection tiles.
                #     Similar to corner tiles
                # This does not check the parallel non-intersecting case for
                #     linear tiles, which requires extra check below.
                check_endpoints = False
                if mesh1 in GraphRules.ENDPOINTS.keys():
                    cdy = y2 - y1
                    cdx = x2 - x1
                    cy = y1
                    cx = x1
                    ctile = tile1
                    that_yaw = yaw2
                    this_mesh = mesh1
                    that_mesh = mesh2
                    check_endpoints = True
                elif mesh2 in GraphRules.ENDPOINTS.keys():
                    cdy = y1 - y2
                    cdx = x1 - x2
                    cy = y2
                    cx = x2
                    ctile = tile2
                    that_yaw = yaw1
                    this_mesh = mesh2
                    that_mesh = mesh1
                    check_endpoints = True
                if check_endpoints:
                    is_connected = GraphRules.check_endpoints_tile_connection(
                        cdy, cdx, cell_to_yaw[cy, cx][ctile], this_mesh, that_yaw, that_mesh)

                # Constraint rule 3: parallel non-intersecting linear tiles aren't nbrs
                if is_connected:
                    if mesh1 in GraphRules.LINEAR_TILES and \
                            mesh2 in GraphRules.LINEAR_TILES:
                        is_connected = GraphRules.check_linear_tile_connection(
                            mesh1, y1, x1, cell_to_yaw[y1, x1][tile1],
                            mesh2, y2, x2, cell_to_yaw[y2, x2][tile2])

                # If connected so far, check if need to break connections
                if is_connected:
                    # Constraint rule 4: disconect tiles with a blocker between them
                    if blocker1 != None or blocker2 != None:
                        is_connected = GraphRules.check_blocker_tile_connection(
                            y1, x1, blocker1, y2, x2, blocker2)

                    # Constraint rule for subtiles
                    # break connection between two subtiles that are in the same cell, and
                    # break connection between subtile and any other adjacent tiles
                    # that are not in the same cell as the subtile
                    if t1 != t2:
                        if mesh1 in GraphRules.SUB_TILES or mesh2 in GraphRules.SUB_TILES:
                            is_connected = False
                    elif mesh1 in GraphRules.SUB_TILES and mesh2 in GraphRules.SUB_TILES:
                        is_connected = False

                # This is under development for rare caess and does not work yet.
                # If still connected, check z levels (rare case, for superposed
                #     main tile and subtiles in same cell)
                if is_connected:
                    # Constraint rule 5: disconnect tiles at different levels
                    is_connected = GraphRules.check_z_level(
                        mesh1, yaw1, lvl1, mesh2, yaw2, lvl2, (x2-x1, y2-y1), args.scale_z)
                    # DEBUG
                    #if not is_connected:
                    #    print('z level returned %s for %s (lvl %g) and %s (lvl %g)' % (is_connected, mesh1, lvl1, mesh2, lvl2), file=sys.stderr)

                # Output connecting edge
                if is_connected:
                    # filter our duplicate edges
                    if (iv1, iv2) not in edges:
                      cmt = ''
                      if mesh1 in GraphRules.INTERSECTIONS or mesh2 in GraphRules.INTERSECTIONS:
                          cmt = '  /* Intersection */'
                      print(edge_fmt % (iv1, iv2, GraphRules.calc_edge_cost(mesh1, mesh2),
                                        cmt), file=graph_file)
                      edges.add((iv1, iv2))
                else:
                    print('DEBUG: Ambiguity resolved: tile %s (%d) and %s (%d) not connected' % (
                        mesh1, iv1, mesh2, iv2), file=sys.stderr)

    print_graph_bottom(args, graph_file=graph_file)
    print_world_bottom(args, world_file=world_file)

    if len(args.world_file) > 0:
        world_file.close()

    if len(args.graph_file) > 0:
        graph_file.close()


def print_graph_bottom(args, graph_file):
    print('}', file=graph_file)


def print_world_bottom(args, world_file=sys.stdout):
    global plugin_artifacts
    global plugin_levels_x
    global plugin_levels_y

    # Gazebo 9
    if args.gz9:
        print("""
    <!-- The SubT challenge logic plugin -->
    <plugin name="game_logic_plugin" filename="libGameLogicPlugin.so">
      <logging>
        <filename_prefix>subt_%s</filename_prefix>
      </logging>
      <!-- The collection of artifacts to locate -->
%s
    </plugin>

    <!-- The SubT comms broker plugin -->
    <plugin name="comms_broker_plugin" filename="libCommsBrokerPlugin.so">
      <comms_model>
        <comms_model_type>visibility_range</comms_model_type>

        <range_config>
          <max_range>500.0</max_range>
          <fading_exponent>2.5</fading_exponent>
          <L0>40</L0>
          <sigma>10.0</sigma>
        </range_config>

        <visibility_config>
          <visibility_cost_to_fading_exponent>0.2</visibility_cost_to_fading_exponent>
          <comms_cost_max>10</comms_cost_max>
        </visibility_config>

        <radio_config>
          <capacity>1000000</capacity>
          <tx_power>20</tx_power>
          <noise_floor>-90</noise_floor>
          <modulation>QPSK</modulation>
        </radio_config>
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
              (args.world_name, plugin_artifacts, args.wind_x, args.wind_y, args.wind_z), file=world_file)

    # Ignition
    else:
        print("""
    <wind>
      <linear_velocity>%f %f %f</linear_velocity>
    </wind>

    <!-- Load the plugin for the wind -->
    <plugin name="ignition::gazebo::systems::WindEffects" filename="libignition-gazebo-wind-effects-system.so">
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

    <!-- Levels plugin -->
    <plugin name="ignition::gazebo" filename="dummy">
%s

%s
    </plugin>

  </world>
</sdf>""" %
              (args.wind_x, args.wind_y, args.wind_z, expand_levels(plugin_levels_x), expand_levels(plugin_levels_y)),
              file=world_file)


if __name__ == '__main__':
    check_main()
