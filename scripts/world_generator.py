#!/usr/bin/env python3

import argparse

import numpy as np
import matplotlib.pyplot as plt


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
    loc_x = node.loc[0] - origin[0]
    loc_y = node.loc[1] - origin[1]

    model_name = node.piece.model_name
    rr, pp, yy = node.piece.tf
    x, y, z = loc_x * 20, loc_y * 20, 0

    s = []
    s.append('<include>')
    s.append('  <static>true</static>')
    s.append('  <name>cave_{loc_x}_{loc_y}</name>'.format(**locals()))
    s.append('  <pose>{x} {y} {z} {rr} {pp} {yy}</pose>'.format(**locals()))
    s.append('  <uri>model://{model_name}</uri>'.format(**locals()))
    s.append('</include>')

    if depth != 0:
        for d in DIRECTIONS:
            if node.connections[d]:
                s.extend(node_to_xml(node.connections[d], origin, depth-1))
    return s


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid_size', type=int, default=200)
    parser.add_argument('--origin_x', type=int, default=100)
    parser.add_argument('--origin_y', type=int, default=100)
    parser.add_argument('--target-length', type=int, default=1000)
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--num-robots', type=int, default=0)
    parser.add_argument('output_name')
    args = parser.parse_args()

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

    print('Target Length: ', args.target_length)
    print('Current Length: ', np.sum(grid.grid) * 20)

    robots = []
    robots_idx = []
    used = set()

    if args.num_robots > 0:
        (r, c) = np.where(grid.grid > 0)
        tries = 0
        while len(robots) != args.num_robots:
            idx = np.random.randint(0, len(r))
            if tries > 100:
                print("Couldn't locate robots")
                break
            if idx in used:
                tries = tries + 1
                continue
            if grid.types[r[idx], c[idx]] in (b'L1', b'L2', b'L3', b'L4', b'T1', b'T2', b'T3', b'T4'):
                tries = tries + 1
                continue
            print(grid.types[r[idx],c[idx]])
            name = 'robot_' + str(len(robots))
            robots.append((name,
                           20 * (r[idx] - args.origin_x),
                           20 * (c[idx] - args.origin_y),
                           np.pi * np.random.randn(1)[0]))
            robots_idx.append((c[idx], r[idx]))
            used.add(idx)

    if args.show:
        dg = np.zeros((args.grid_size*3, args.grid_size*3))
        draw_element(dg, root, 500)
        plt.figure()
        dg = dg / np.max(dg)
        plt.imshow(dg)
        robots_idx = np.array(robots_idx)
        plt.scatter(robots_idx[:, 1]*3, robots_idx[:, 0]*3)

        plt.figure()
        plt.imshow(grid.grid)
        plt.scatter(robots_idx[:, 0], robots_idx[:, 1])
        plt.show()

    with open(args.output_name + '.world', 'w') as f:
        f.write("""
<sdf version="1.6">
<world name="{name}">
""".format(name=args.output_name))

        f.write('\n  '.join(node_to_xml(root,
                                        (args.origin_x, args.origin_y),
                                        200)))
        for robot in robots:
            f.write("""
<include>
  <static>false</static>
  <name>{name}</name>
  <pose>{x} {y} 0.1 0 0 {yaw}</pose>
  <uri>model://subt_pioneer</uri>
</include>
""".format(name=robot[0], x=robot[1], y=robot[2], yaw=robot[3]))
        f.write('\n</world></sdf>')
