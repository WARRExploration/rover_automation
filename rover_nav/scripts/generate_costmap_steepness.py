#!/usr/bin/env python
"""
generate a costmap used for autonomous navigation based on the terrain's steepness
"""

import numpy as np
import os
from rospkg import RosPack
import cv2

# use files relative to rover_sim and rover_nav
rospack = RosPack()
rover_sim_dir = rospack.get_path('rover_sim')
rover_nav_dir = rospack.get_path('rover_nav')


def get_coordinates_from_csv(csv_file_path):
    """This function extracts the coordinates from a csv file based on the provided files of the ERC

    Arguments:
        csv_file_path {str} -- path to the ERC csv file (ver2)

    Returns:
        [[[]]] -- 2d array of 3d coordinates
    """

    # read context informations from *.csv file
    with open(csv_file_path) as fp:
        for i, line in enumerate(fp):
            if i != 1:
                continue

            # we are only interested in the spacing and coords_0
            _, _, spacing_y, spacing_x, x_0, y_0 = np.fromstring(
                line, dtype=float, sep=' ')
            break

    # load heights from *.csv file
    data = np.loadtxt(open(csv_file_path), delimiter=',', skiprows=2)

    # transform the matrix so heights are accessible by intuitive indices
    data = np.swapaxes(np.flip(data, 0), 0, 1)

    # apply threshold (set invalid values to 0)
    data[data >= 2.8] = 0

    # get dimensions of the matrix
    number_of_cols, number_of_rows = data.shape

    # convert y_0 to actual coordinate at ind_y=0
    y_0 -= (number_of_rows-1)*spacing_y

    # x-Axis
    # generate vector
    xs = np.mgrid[:number_of_cols] * spacing_x
    # add possible offset
    xs += x_0
    # generate grid from vector
    xs = np.stack((xs,) * number_of_rows, axis=-1)

    # y-Axis
    # generate vector
    ys = np.mgrid[:number_of_rows] * spacing_y
    # add possible offset
    ys += y_0
    # generate grid from vector
    ys = np.stack((ys,) * number_of_cols, axis=0)

    # coordinates
    coords = np.stack((xs, ys, data), axis=2)

    return coords


def generate_normal_array(coords):
    """generate the normal array out of the coordinates

    Arguments:
        coords {[[[]]]} -- 2d array of 3d coordinates

    Returns:
        [[[]]] -- 2d array of normals
    """

    number_of_cols, number_of_rows, _ = coords.shape

    normal_floats = np.empty((number_of_cols, number_of_rows, 3))
    for x in range(number_of_cols):
        for y in range(number_of_rows):

            # terrain's edge is not traversable...
            if x == 0 or x == number_of_cols - 1 or y == 0 or y == number_of_rows - 1:
                normal_floats[x, y, :] = [1, 0, 0]
                continue

            # ... as well as all invalid values
            if coords[x, y, 2] == 0:
                normal_floats[x, y, :] = [1, 0, 0]
                continue

            # calculate normal to gradient
            current_normal = np.cross(coords[x+1, y]-coords[x-1, y],
                                      coords[x, y+1]-coords[x, y-1])

            # normalize normal
            current_normal /= np.linalg.norm(current_normal)
            normal_floats[x, y] = current_normal

    return normal_floats

def classify(normals, threshold):
    """map each normal to a greyscale color value

    Arguments:
        normals {[[[]]]} -- 2d array of normals
        threshold {int} -- max angle the rover can handle

    Returns:
        [[]] -- 2d array of greyscale color values
    """

    # calculate each normal's deviation from the vertical
    costs = np.arccos(normals.dot([0,0,1]))

    # set threshold for the max steepness the rover can handle
    var = np.radians(threshold)

    # classify normals' deviation by given threshold
    mask = costs > var

    # region is not traversable
    costs[mask] = 0

    # region is plane enough
    costs[~mask] = 255

    return costs

def write_costmap(costs, output):
    """write the costmap to a *.pgm file

    Arguments:
        costs {[[]]} -- 2d array of greyscale color values
        output {str} -- path to the finished costmap
    """
    
    # reverse initial transformation
    costs = np.swapaxes(np.flip(costs, 1), 0, 1)

    # write *.pgm file
    cv2.imwrite(output, costs)


def generate_costmap(heightmap, output, threshold):
    """generate a costmap used for autonomous navigation based on the terrain's steepness

    Arguments:
        heightmap {str} -- path to the ERC csv file (ver2)
        output {str} -- path to the finished costmap
        threshold {int} -- max angle the rover can handle
    """

    # read coords
    coords = get_coordinates_from_csv(heightmap)

    # calculate normals
    normals = generate_normal_array(coords)

    # classify normals
    costs = classify(normals, threshold)

    # write costmap
    write_costmap(costs, output)


if __name__ == '__main__':

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    
    # default values
    heightmap_csv_path = os.path.join(rover_sim_dir, 'worlds/erc2018final/Heightmap.csv')
    costmap_pgm_path = os.path.join(rover_nav_dir, 'envs/steepness.pgm')
    threshold_radians = 15

    # parse command line arguments
    parser = ArgumentParser(
        description="generate a costmap used for autonomous navigation based on the terrain's steepness",
        formatter_class=ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-m", "--heightmap", type=str, help="path to an ERC csv file (ver2)", default=heightmap_csv_path)
    parser.add_argument("-o", "--output", type=str, help="path to the finished costmap (*.pgm)", default=costmap_pgm_path)
    parser.add_argument("-t", "--threshold", type=int, help="max steepness (radians)", default=threshold_radians)
    args = parser.parse_args()

    # generate costmap
    generate_costmap(heightmap=args.heightmap, output=args.output, threshold=args.threshold)
