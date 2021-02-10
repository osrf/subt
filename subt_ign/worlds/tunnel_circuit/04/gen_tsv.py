#!/usr/bin/env python

# Generates .tsv file of a world that roughly conforms to conditional parameters
#   suggested in command line arguments.

import argparse
import numpy as np
import csv


class TunnelTiles:

    # Terminology:
    # Absolute facing direction: +x is 0, +y 90, -x 180, -y 270
    # Rotation: tile orientation, dependent on the actual mesh

    # Straight tile
    #       0 degrees is along y, define this as -y to avoid ambiguity
    #      90 degrees is along x, define this as +x to avoid ambiguity
    #     Then
    #     180 is toward +y
    #     270 is toward -x
    STRT = 'tunnel_tile_5'
    STRT_FACE_TO_ROT = {0: 90, 90: 180, 180: 270, 270: 0}
    # Straight constrained tile
    CONS_TALL = 'constrained_tunnel_tile_short'
    CONS_SHRT = 'constrained_tunnel_tile_tall'
    # Ramp tile
    #       0 degrees is going downward in +y
    #      90 is downward in +x
    #     180 is downward in -y
    #     270 is downward in -x
    RAMP = 'tunnel_tile_6'
    RAMP_DN_FACE_TO_ROT = {0: 90, 90: 180, 180: 270, 270: 0}
    RAMP_UP_FACE_TO_ROT = {0: 270, 90: 0, 180: 90, 270: 180}
    # Shaft tile
    #       0 degrees is going downward in -y
    #      90 is downward in +x
    #     180 is downward in +y
    #     270 is downward in -x
    SHFT = 'tunnel_tile_7'
    SHFT_DN_FACE_TO_ROT = {0: 90, 90: 180, 180: 270, 270: 0}
    SHFT_UP_FACE_TO_ROT = {0: 270, 90: 0, 180: 90, 270: 180}
    # Turn tile
    #       0 is turn between +x and -y
    #      90 is turn between +x and +y
    #     180 is turn between -x and +y
    #     270 is turn between -x and -y
    TURN = 'tunnel_tile_2'
    # Key (a, b) for turning from facing direction a to facing direction b,
    #   which is symmetric, i.e. the same tile for turning from facing b to facing a.
    TURN_FACE_TO_ROT = {(0, 90): 90, (90, 0): 90,
                        (90, 180): 180, (180, 90): 180,
                        (180, 270): 270, (270, 180): 270,
                        (270, 0): 0, (0, 270): 0}
    # 4-way intersection tunnel tile is invariant to rotation
    INTR = 'tunnel_tile_1'
    # Tunnel tile 4-way intersection is invariant.
    #   Other 4-way intersection tiles may not be, e.g. urban world.
    INTR_FACE_TO_ROT = {0: 0, 90: 0, 180: 0, 270: 0}

    # Tiles with debris
    STRT_DEB = 'Rough Tunnel Tile Straight'
    RAMP_DEB = 'Rough Tunnel Tile Ramp'
    SHFT_DEB = 'Rough Tunnel Tile Vertical Shaft'
    TURN_DEB = 'Rough Tunnel Tile 90-degree Turn'
    INTR_DEB = 'Rough Tunnel Tile 4-way Intersection'

    BLCK = 'tunnel_tile_blocker'

    # Scale of tunnel tiles. World-specific. Fixed parameters that will be
    #   used to generate world files from scaled meshes (passed to tile_tsv.py).
    TUNNEL_TILE_SCALE_XY = 20
    TUNNEL_TILE_SCALE_Z = 5
    # Blockers are placed at scale * 0.5 + 1
    BLCK_SHIFT_XY = 0.5 * TUNNEL_TILE_SCALE_XY + 1
    BLCK_SHIFT_Z = TUNNEL_TILE_SCALE_Z


# cell: (row, col)
def valid_cell_range(cell, nrows, ncols):
    if cell[0] >= 0 and cell[0] < nrows and cell[1] >= 0 and cell[1] < ncols:
        return True
    else:
        return False


# abs_face_dir: Absolute facing direction in coordinate frame, expressed in
#     degrees from x-axis
# lvl: Integer, level offset from the tile. If flat tile, pass in 0. Only
#     need non-zero value if vertical tile e.g. ramp, shaft.
def blocker_string(abs_face_dir, lvl_offset):

    # Facing +x
    if abs_face_dir == 0:
        blocker_xy = (TunnelTiles.BLCK_SHIFT_XY, 0)
    # Facing -x
    elif abs_face_dir == 180:
        blocker_xy = (-TunnelTiles.BLCK_SHIFT_XY, 0)
    # Facing +y
    elif abs_face_dir == 90:
        blocker_xy = (0, TunnelTiles.BLCK_SHIFT_XY)
    # Facing -y
    elif abs_face_dir == 270:
        blocker_xy = (0, -TunnelTiles.BLCK_SHIFT_XY)
    # Invalid!!
    else:
        print('Invalid facing direction. Cannot place blocker.')
        return ''

    blocker = '%s@%g %g %g 0 0 0' % (TunnelTiles.BLCK,
      blocker_xy[0], blocker_xy[1], TunnelTiles.BLCK_SHIFT_Z * lvl_offset)

    return blocker


# tsv: Dictionary mapping (row, column) to string in cell
def output_tsv(tsv, path, nrows, ncols):

    # Sort (row, column)
    # Note that not necessarily every cell has an item, in fact most won't
    keys_sorted = sorted(tsv.keys(), key=lambda element: (element[0], element[1]))

    if len(keys_sorted) == 0:
        print('Nothing to write')
        return

    row = 0

    with open(path, 'w') as tsvfile:
        tsvwriter = csv.writer(tsvfile, delimiter='\t')
        # Each row of tsv file
        for row in range(nrows):
            rowlist = []

            for col in range(ncols):
                if len(keys_sorted) > 0:
                    key = keys_sorted[0]
                    # If the current cell is the next cell with content
                    # (Empty cells are not in the list)
                    if key[0] == row and key[1] == col:
                        # Fill the cell, and move onto to the next key
                        rowlist.append(tsv[key])
                        keys_sorted.pop(0)
                    else:
                        # Empty cell, fill it with empty string
                        rowlist.append('')
                else:
                    rowlist.append('')

            tsvwriter.writerow(rowlist)
            print('wrote row %d' % row)
            row += 1
    print('Written to [%s]' % path)
    print('Outputted %d non-empty tiles of %d m x %d m, approximately %d meters' % (
        len(tsv), TunnelTiles.TUNNEL_TILE_SCALE_XY, TunnelTiles.TUNNEL_TILE_SCALE_XY,
        len(tsv) * TunnelTiles.TUNNEL_TILE_SCALE_XY))


def main():

    parser = argparse.ArgumentParser('Generate tiled world file from tsv.')

    parser.add_argument('--tsv-file', type=str, default='test.tsv', help='path to tsv file to output')
    parser.add_argument('--dim_x', type=int, default=10, help='Rough suggestion of world dimension in x in number of tiles. (Final dimension depends on scaling of tiles)')
    parser.add_argument('--dim_y', type=int, default=10, help='Rough suggestion of world dimension in y in number of tiles. (Final dimension depends on scaling of tiles)')
    parser.add_argument('--intersection-density', type=float, default=0.2, help='Density of intersection or branching, in range [0, 1]')
    parser.add_argument('--verticality-level', type=float, default=0.0, help='Amount of verticality, in range [0, 1]')
    parser.add_argument('--debris-level', type=float, default=0.0, help='Amount of debris, in range [0, 1]')
    parser.add_argument('--turn-level', type=float, default=0.4, help='Likelihood of turning, as opposed to straight, in range [0, 1]. Tune this to get desired shape.')
    parser.add_argument('--random-seed', type=int, default=None, help='Integer random seed, which can be set for reproducibility. Defaults to a random seed.')

    args = parser.parse_args()

    np.random.seed(args.random_seed)

    if args.intersection_density < 0.0 or args.intersection_density > 1.0:
        print('Error: Invalid value (%g) for intersection density. Must be in range [0, 1]' % (
            args.intersection_density))
        return

    # x determins number of columns in output file
    # y determines number of rows in output file
    nrows = args.dim_y
    ncols = args.dim_x

    prob_turn = args.turn_level

    # These are sub-probabilities of debris level. They could be added to
    #     command line, but need clear help messages.
    # Constrained and debris are in the same category.
    # This is probability of constrained, 1-this is that of debris.
    prob_constrain = 0.5
    # Short constrain tunnel, as opposed to tall constrain tunnel
    prob_high_constrain = 0.2


    # Straight or branch
    I_STRT = 0
    I_TURN = 1
    I_BRCH = 2
    prob_brch = args.intersection_density

    # Clean, debris, or constrained
    I_CLN = 0
    I_DEB = 1
    I_CON = 2

    # Tall or short constrained
    I_TALL = 0
    I_SHRT = 1

    # Flat, ramp, or vertical
    I_FLAT = 0
    I_RAMP = 1
    I_SHFT = 2
    prob_flat = 1 - args.verticality_level


    # Maps (row, col) of a matrix to the string to go in the cell
    tsv = {}
    # Current (row, column)
    # Start in middle row
    crow = nrows / 2
    # Leave column 0 empty for base station
    tsv[(crow, 0)] = ''
    # Hardcode starting with straight tile
    #tsv[(crow, 1)] = '%s,%d,%d' % (TunnelTiles.STRT, 90, 0)

    # Maps (row, col) to the level in the cell before that cell
    level_start = {}
    # Maps (row, col) to the facing direction the cell must start with.
    #   90 for +x, 180 for +y, 270 for -x, 0 for -y
    face_start = {}

    # Start at level 0
    level_start[(crow, 1)] = 0
    next_level = 0

    # Absolute facing direction.
    # This is not the same as tile orientation, as each tile is defined differently
    face_start[(crow, 1)] = 0

    # Queue for graph traversal
    ccol = 1
    queue = [(crow, 1)]

    # Map (row, col) to cell info which will be useful for adding blockers
    cells_info = {}
    cells_info[(crow, 0)] = None

    # Grow the graph
    while len(queue) > 0:
        
        crow, ccol = queue.pop(0)

        clvl = level_start[(crow, ccol)]
        cface = face_start[(crow, ccol)]

        # Choose branch or not
        # ========================================================================
        # hack for tunnel 4
        pa=[(1 - prob_turn) * (1 - prob_brch), prob_turn * (1 - prob_brch), prob_brch]
        if crow == 0 or crow == nrows-1:
            if cface == 0:
              print(" tunnel 4 hack =============")
              pa=[0.9, 0.05, 0.05]
            #else:
            #  print(" tunnel 4 hack2 =============")
            #  pa=[0.4, 0.3, 0.3]

        brch = np.random.choice([I_STRT, I_TURN, I_BRCH],
             p=pa)
        # ========================================================================
        #    p=[(1 - prob_turn) * (1 - prob_brch), prob_turn * (1 - prob_brch), prob_brch])

        # Choose verticality of straight tile: flat, ramp, or vertical
        # Split verticality level into randomly choosing ramp or shaft
        prob_shft = np.random.random()
        # ========================================================================
        # hack for tunnel 4
        #vert = np.random.choice([I_FLAT, I_RAMP, I_SHFT],
        vert = np.random.choice([I_FLAT, I_RAMP, I_RAMP],
            p=[prob_flat, (1 - prob_flat) * (1 - prob_shft), (1 - prob_flat) * prob_shft])
        # Choose clean or debris
        deb = np.random.choice([I_CLN, I_DEB],
            p=[1 - args.debris_level, args.debris_level])

        ## Info for adding blockers
        # Book-keeping whether the cell is climbing up/down
        climb_up = None
        # relative level of next level to current level
        blocker_relative_level = 0
        # faces of the openings
        end_faces = []

        # Straight tile
        if brch == I_STRT:
            # Flat
            if vert == I_FLAT:
                # Clean, debris, or constrained
                if deb == I_CLN:
                    tile = TunnelTiles.STRT
                # Constrained and debris are in the same category
                else:
                    # Choose debris or constrained
                    cons = np.random.choice([I_DEB, I_CON],
                        p=[1 - prob_constrain, prob_constrain])

                    if cons == I_DEB:
                        tile = TunnelTiles.STRT_DEB
                    elif cons == I_CON:
                        # Pick tall or short
                        height = np.random.choice([I_SHRT, I_TALL],
                            p=[prob_high_constrain, 1 - prob_high_constrain])
                        if height == I_TALL:
                            tile = TunnelTiles.CONS_TALL
                        else:
                            tile = TunnelTiles.CONS_SHRT

                # Determine rotation of tile based on absolute facing direction
                crot = TunnelTiles.STRT_FACE_TO_ROT[cface]
                next_level = clvl
            # Ramp
            elif vert == I_RAMP:
                if deb == I_CLN:
                    tile = TunnelTiles.RAMP
                else:
                    tile = TunnelTiles.RAMP_DEB

                vote = np.random.random()
                # Ramp tile going down needs to decrement, going up stays at current level
                if vote < 0.5:
                    crot = TunnelTiles.RAMP_DN_FACE_TO_ROT[cface]
                    next_level = clvl - 1
                    clvl -= 1
                    climb_up = False
                else:
                    crot = TunnelTiles.RAMP_UP_FACE_TO_ROT[cface]
                    next_level = clvl + 1
                    climb_up = True
            # Shaft
            elif vert == I_SHFT:
                if deb == I_CLN:
                    tile = TunnelTiles.SHFT
                else:
                    tile = TunnelTiles.SHFT_DEB

                vote = np.random.random()
                # Shafts tile going down needs to decrement, going up stays at current level
                if vote < 0.5:
                    crot = TunnelTiles.SHFT_DN_FACE_TO_ROT[cface]
                    next_level = clvl - 1
                    clvl -= 1
                    climb_up = False
                else:
                    crot = TunnelTiles.SHFT_UP_FACE_TO_ROT[cface]
                    next_level = clvl + 1
                    climb_up = True

            # Facing +x, go to column on right
            if cface == 0:
                cell = (crow, ccol+1)
            # Facing -x, go to column on left
            elif cface == 180:
                cell = (crow, ccol-1)
            # Facing +y, go to row above
            elif cface == 90:
                cell = (crow-1, ccol)
            # Facing -y, go to row below
            elif cface == 270:
                cell = (crow+1, ccol)

            # Queue cell
            # If cell already filled in tsv, do not overwrite
            valid_cell = valid_cell_range(cell, nrows, ncols)
            if valid_cell and cell not in queue and cell not in cells_info.keys():
                queue.append(cell)
                level_start[cell] = next_level
                face_start[cell] = cface
            else:
                print('1: skipped queueing %s' % str(cell))

            blocker_relative_level = next_level - clvl
            end_faces = [cface]

        # Turning tile
        elif brch == I_TURN:
            if deb == I_CLN:
                tile = TunnelTiles.TURN
            else:
                tile = TunnelTiles.TURN_DEB

            # Turns are always 90 degrees, either +90, or -90
            vote = np.random.random()
            if vote < 0.5:
                delta_face = 90
            else:
                delta_face = -90
            next_face = (cface + delta_face) % 360

            # Determine the cell, which depends on facing direction of current cell
            # Facing +x, i.e. column to the right
            if cface == 0:
                if delta_face == 90:
                    cell = (crow-1, ccol)
                elif delta_face == -90:
                    cell = (crow+1, ccol)
            # Facing +y, i.e. row above
            elif cface == 90:
                if delta_face == 90:
                    cell = (crow, ccol-1)
                elif delta_face == -90:
                    cell = (crow, ccol+1)
            # Facing -x, i.e. column to the left
            elif cface == 180:
                if delta_face == 90:
                    cell = (crow+1, ccol)
                elif delta_face == -90:
                    cell = (crow-1, ccol)
            # Facing -y, i.e. row below
            elif cface == 270:
                if delta_face == 90:
                    cell = (crow, ccol+1)
                elif delta_face == -90:
                    cell = (crow, ccol-1)

            # Determine rotation of tile based on absolute facing direction
            # If facing direction theta, want a tile that goes toward -theta.
            crot = TunnelTiles.TURN_FACE_TO_ROT[((cface + 180) % 360, next_face)]

            valid_cell = valid_cell_range(cell, nrows, ncols)
            if valid_cell and cell not in queue and cell not in cells_info.keys():
                queue.append(cell)
                level_start[cell] = clvl
                face_start[cell] = next_face
            else:
                print('2: skipped queueing %s' % str(cell))

            blocker_relative_level = 0
            end_faces = [next_face]

        # Branching tile
        elif brch == I_BRCH:
            if deb == I_CLN:
                tile = TunnelTiles.INTR
            else:
                tile = TunnelTiles.INTR_DEB

            next_faces = [(cface + 90) % 360, (cface - 90) % 360, cface]

            # Determine the cell, which depends on facing direction of current cell
            # Facing +x, i.e. column to the right
            if cface == 0:
                # NOTE: Items must be ordered corresponding to next_faces
                # Queue row above (+90), row below (-90), column to the right (0)
                cells_to_queue = [(crow-1, ccol), (crow+1, ccol), (crow, ccol+1)]

            # Facing +y, i.e. row above
            elif cface == 90:
                # Queue column to the left (+90), column to the right (-90), row above (0)
                cells_to_queue = [(crow, ccol-1), (crow, ccol+1), (crow-1, ccol)]
            # Facing -x, i.e. column to the left
            elif cface == 180:
                # Queue row below (+90), row above (-90), column to the left (0)
                cells_to_queue = [(crow+1, ccol), (crow-1, ccol), (crow, ccol-1)]
            # Facing -y, i.e. row below
            elif cface == 270:
                # Queue column to the right (+90), column to the left (-90), row below (0)
                cells_to_queue = [(crow, ccol+1), (crow, ccol-1), (crow+1, ccol)]

            for i in range(len(next_faces)):
                cell = cells_to_queue[i]
                valid_cell = valid_cell_range(cell, nrows, ncols)
                if valid_cell and cell not in queue and cell not in cells_info.keys():
                    queue.append(cell)
                    level_start[cell] = clvl
                    face_start[cell] = next_faces[i]
                else:
                    print('3: skipped queueing %s' % str(cell))

            # Determine rotation of tile based on absolute facing direction
            crot = TunnelTiles.INTR_FACE_TO_ROT[cface]

            blocker_relative_level = 0
            end_faces = next_faces

        else:
            print('Invalid brch value')

        cells_info[(crow, ccol)] = CellInfo(tile, crot, clvl, start_face=cface, end_faces=end_faces,
                                            blocker_relative_level=blocker_relative_level, climb_up=climb_up)
    
    tsv = add_blockers(cells_info)

    output_tsv(tsv, args.tsv_file, nrows, ncols)


class CellInfo:
    """ Class for containing information for generating tsv file
    """
    def __init__(self, tile, rotation, level, start_face, end_faces, blocker_relative_level, climb_up):
        self.tile = tile
        self.rotation = rotation
        self.level = level
        self.start_face = start_face
        self.end_faces = end_faces
        self.relative_level = blocker_relative_level
        self.climb_up = climb_up
        self.blockers = []

    def add_blocker(self, face):
        """ Add a blocker to current cell at face
        :param face: the direction at which to add blocker
        """
        self.blockers.append(blocker_string(face, self.relative_level))

    def generate_string(self):
        """
        :return: the string for current cell to be put into tsv file
        """
        blocker = ";".join(self.blockers)
        if blocker == "":
            return '%s,%g,%d' % (self.tile, self.rotation, self.level)
        else:
            return '%s,%g,%d,%s' % (self.tile, self.rotation, self.level, blocker)


def match_end_faces(test_end_face, end_faces):
    """ Test if test_end_face match with any end face in end_faces
        Each face takes value in (0, 90, 180, 270)
    :param test_end_face: one face
    :param end_faces: end_faces_to_match_against
    :return: True if there is a match
    """
    for ef in end_faces:
        if (test_end_face+180) % 360 == ef:
            return True
    return False


def add_blockers(cells_info):
    """ Add blockers to cells where appropriate
    :param cells_info: dictionary of cell information, key being (row, col)
    :return: tsv dictionary for generating tsv file
    """
    # intersection tiles
    inter_tiles = (TunnelTiles.INTR, TunnelTiles.INTR_DEB)

    tsv = {}
    for key, cell_info in cells_info.items():
        # If is the staring cell
        if cell_info is None:
            tsv[key] = ""
            continue

        row, col = key
        # Check alignment for every opening
        for end_face in cell_info.end_faces:
            if end_face == 0:
                next_cell = (row, col+1)
            elif end_face == 90:
                next_cell = (row-1, col)
            elif end_face == 180:
                next_cell = (row, col-1)
            elif end_face == 270:
                next_cell = (row+1, col)

            # If no cell at that location or if is the starting cell
            if next_cell not in cells_info or cells_info[next_cell] is None:
                cell_info.add_blocker(end_face)
            else:
                next_cell_info = cells_info[next_cell]
                # If not aligned
                if next_cell_info.start_face != end_face and not match_end_faces(end_face, next_cell_info.end_faces) and \
                        next_cell_info.tile not in inter_tiles:
                    cell_info.add_blocker(end_face)

                # If not at correct level
                if next_cell_info.level != cell_info.level:
                    diff_level = next_cell_info.level - cell_info.level
                    if diff_level == 1 and cell_info.climb_up is not True:
                        cell_info.add_blocker(end_face)
                    elif diff_level == -1 and next_cell_info.climb_up is not False:
                        cell_info.add_blocker(end_face)
                    elif abs(diff_level) > 1:
                        cell_info.add_blocker(end_face)

        tsv[key] = cell_info.generate_string()
    return tsv


if __name__ == '__main__':
    main()
