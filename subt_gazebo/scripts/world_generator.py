#!/usr/bin/env python3

import argparse
import os
import sys

import numpy as np
import em

script_dir = os.path.dirname(os.path.realpath(__file__))
template_files = [
        os.path.join(script_dir, 'benchmark.world.template'),
        os.path.join(script_dir, 'benchmark.launch.template')
]

DIRECTIONS = ('north', 'east', 'south', 'west')

class CavePiece(object):
    """
    Stores the properties of a single piece of cave system.

    In the current iteration, this is a 20m x 20m tile
    """

    def __init__(self, name, connections, model_name, tf, weight):
        self.name = name
        self.connections = {
            'north': connections[0],
            'east': connections[1],
            'south': connections[2],
            'west': connections[3],
        }
        self.model_name = model_name
        self.tf = tf
        self.weight = weight

    def __repr__(self):
        return '<CavePiece: {name}>'.format(name=self.name)


class Node(object):
    """
    Represents a single connected node in a cave system.
    """

    def __init__(self, piece, loc):
        self.piece = piece
        self.open_connections = piece.connections
        self.loc = loc
        self.connections = {
            'north': None,
            'east': None,
            'south': None,
            'west': None
        }

    def __repr__(self):
        return '<Node: {t} @ {loc} >'.format(t=self.piece.name, loc=self.loc)


class Grid(object):
    """
    Represents the grid of nodes in a cave system.
    """
    def __init__(self, shape):
        self.shape = shape
        self.grid = np.zeros(shape)
        self.types = np.zeros(shape, dtype='a2')
        self.directions = {
            'north': (1, 0),
            'east': (0, -1),
            'south': (-1, 0),
            'west': (0, 1)
        }
        self.opposites = {
            'north': 'south',
            'south': 'north',
            'east': 'west',
            'west': 'east'
        }

    def is_valid(self, loc):
        # Check if a given location is a valid place to put a new node.
        return (
            loc[0] > 0 and loc[0] < self.shape[0] and
            loc[1] > 0 and loc[1] < self.shape[1] and
            self.grid[loc[0], loc[1]] == 0
        )


# List of available pieces
# Columns:
#  * Name
#  * Connectivity (N, E, S, W)
#  * Collada model name
#  * Additional transform to apply to collada model (R, P, Y)
#  * Weight in the random draw process
PIECES = (
    ('L1', (1, 1, 0, 0), 'cave_L_20m', (0, 0, np.deg2rad(-90)), 5),
    ('L2', (0, 1, 1, 0), 'cave_L_20m', (0, 0, np.deg2rad(180)), 5),
    ('L3', (0, 0, 1, 1), 'cave_L_20m', (0, 0, np.deg2rad(90)), 5),
    ('L4', (1, 0, 0, 1), 'cave_L_20m', (0, 0, np.deg2rad(0)), 5),
    ('S1', (1, 0, 1, 0), 'cave_straight_20m', (0, 0, np.deg2rad(90)), 20),
    ('S2', (0, 1, 0, 1), 'cave_straight_20m', (0, 0, np.deg2rad(0)), 20),
    ('X1', (1, 1, 1, 1), 'cave_4way_20m', (0, 0, np.deg2rad(-90)), 0.1),
    ('T1', (1, 1, 1, 0), 'cave_3way_20m', (0, 0, np.deg2rad(180)), 1),
    ('T2', (0, 1, 1, 1), 'cave_3way_20m', (0, 0, np.deg2rad(90)), 1),
    ('T3', (1, 0, 1, 1), 'cave_3way_20m', (0, 0, np.deg2rad(0)), 1),
    ('T4', (1, 1, 0, 1), 'cave_3way_20m', (0, 0, np.deg2rad(-90)), 1),
)

PIECES = {p[0]: CavePiece(*p) for p in PIECES}


def random_element():
    """Draw a random (weighted) piece from the set of pieces."""
    keys = list(PIECES.keys())
    weights = np.array([PIECES[k].weight for k in PIECES.keys()])
    weights = weights/np.sum(weights)
    return np.random.choice(keys, p=weights)


def insert_element(node, grid):
    """Insert a new element."""
    for check_dir in DIRECTIONS:
        # Make sure that the current piece has an exit and that
        # it isn't already occupied by a connection
        has_connection = node.open_connections[check_dir] != 0
        has_open_connection = node.connections[check_dir] is None

        if has_connection and has_open_connection:
            # Check that connection direction is within the grid
            # and not already occupied by a node.
            step = grid.directions[check_dir]
            check_loc = [node.loc[0] + step[0], node.loc[1] + step[1]]

            if not grid.is_valid(check_loc):
                continue

            t = None
            for ii in range(0, 20):
                # If last piece was straight, increase the odds
                # that the next piece will be straight.
                if node.piece.name is 'S1':
                    if np.random.randint(0, 10) > 3:
                        new_t = 'S1'
                    else:
                        new_t = random_element()
                elif node.piece.name is 'S2':
                    if np.random.randint(0, 10) > 3:
                        new_t = 'S2'
                    else:
                        new_t = random_element()
                else:
                    # Otherwise draw a random piece.
                    new_t = random_element()

                # Check that the random piece has connectivity in the right dir
                if PIECES[new_t].connections[grid.opposites[check_dir]] != 1:
                    continue
                t = new_t
                break

            if t:
                node.connections[check_dir] = Node(PIECES[t], check_loc)
                grid.grid[check_loc[0], check_loc[1]] = 1
                grid.types[check_loc[0], check_loc[1]] = t
        elif has_connection:
            # If the connection exists, but is already occupied, recurse
            insert_element(node.connections[check_dir], grid)


def draw_element(dg, node, depth=0):
    """
    Draws elements in a 2d grid for quick verification.

    Expands each element to a 3x3 grid, with center and appropriate
    connectivity marked.

    """
    elem = np.zeros((3, 3))
    elem[1, 1] = 1

    if node.open_connections['north']:
        elem[1, 2] = 1
    if node.open_connections['east']:
        elem[0, 1] = 1
    if node.open_connections['south']:
        elem[1, 0] = 1
    if node.open_connections['west']:
        elem[2, 1] = 1

    locx = node.loc[1]*3
    locy = node.loc[0]*3
    dg[locx:locx + 3, locy:locy + 3] = elem

    if depth != 0:
        for d in DIRECTIONS:
            if node.connections[d]:
                draw_element(dg, node.connections[d], depth-1)


def node_to_xml(node, origin, depth=0):
    """
    Converts element to SDF XML.
    """

    class NodeInfo(object):
        def __init__(self, model_name, loc_x, loc_y, tf):
            self.loc_x = loc_x
            self.loc_y = loc_y
            self.model_name = model_name
            self.x = loc_x * 20
            self.y = loc_y * 20
            self.z = 0.0
            self.roll = node.piece.tf[0]
            self.pitch = node.piece.tf[1]
            self.yaw = node.piece.tf[2]

    loc_x = node.loc[0] - origin[0]
    loc_y = node.loc[1] - origin[1]

    nodes = []
    ni = NodeInfo(node.piece.model_name, loc_x, loc_y, node.piece.tf)
    nodes.append(ni)

    if depth != 0:
        for d in DIRECTIONS:
            if node.connections[d]:
                nodes.extend(node_to_xml(node.connections[d], origin, depth-1))
    return nodes


def generate_files(template_data):
    files = {}
    for template_file in template_files:
        with open(template_file, 'r') as f:
            data = f.read()
        files[template_file] = em.expand(data, template_data)
    return files


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid-size', type=int, default=200,
            help='Length and width of the grid that will be populated')
    parser.add_argument('--origin-x', type=int, default=100,
            help='X origin in the grid that will be populated.')
    parser.add_argument('--origin-y', type=int, default=100,
            help='Y origin in the grid that will be populated.')
    parser.add_argument('--target-length', type=int, default=1000,
            help='Target total length of the cave system to be generated')
    parser.add_argument('--seed', type=int,
            help='Random number generator seed')
    parser.add_argument('-n', '--dry-run', action='store_true', default=False,
            help='print generated files to stdout, but do not write them to disk')
    parser.add_argument('-o', '--output-dir', default='/tmp/subt/',
        help='directory in which to output the generated files')
    parser.add_argument('output_name')
    args = parser.parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    length_met = False

    # Brute force attempt to find cave system with given length.
    # This could certainly be more intelligent.
    while not length_met:
        grid = Grid((args.grid_size, args.grid_size))
        root = Node(PIECES['X1'], (args.origin_x, args.origin_y))
        grid.grid[args.origin_x, args.origin_y] = 1
        grid.types[args.origin_x, args.origin_y] = 'X1'

        for ii in range(0, 200):
            insert_element(root, grid)
            current_length = np.sum(grid.grid) * 20
            if current_length > args.target_length:
                length_met = True
                break

    template_data = {
            'output_dir': args.output_dir,
            'world_name': args.output_name,
            'nodes': node_to_xml(root, (args.origin_x, args.origin_y), 200)
            }

    files = generate_files(template_data)
    if not args.dry_run and not os.path.isdir(args.output_dir):
        if os.path.exists(args.output_dir) and not os.path.isdir(args.output_dir):
            print('Error, given output directory exists but is not a directory.', file=sys.stderr)
            sys.exit(1)
        print('creating directory: ' + args.output_dir)
        os.makedirs(args.output_dir)

    for name, content in files.items():
        if name.endswith('.template'):
            name = name[:-len('.template')]
        name = os.path.basename(name)
        if args.dry_run:
            print('# file: ' + name)
            print(content)
        else:
            if name.endswith('.world'):
                ext = '.world'
            elif name.endswith('.launch'):
                ext = '.launch'
            file_path = os.path.join(args.output_dir, args.output_name + ext)
            print('writing file ' + file_path)
            with open(file_path, 'w+') as f:
                f.write(content)
