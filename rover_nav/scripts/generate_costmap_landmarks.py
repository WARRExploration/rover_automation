#!/usr/bin/env python
"""
generate a costmap used for autonomous navigation based on the landmarks' position and size
"""

import numpy as np
import os
from rospkg import RosPack
import cv2

# use files relative to rover_sim and rover_automation
rospack = RosPack()
rover_sim_dir = rospack.get_path('rover_sim')
rover_nav_dir = rospack.get_path('rover_nav')

def get_context_info_from_csv(csv_file_path):
    """This function extracts the context info from a csv file based on the provided files of the ERC

    Arguments:
        csv_file_path {str} -- path to the ERC csv file (ver2)

    Returns:
        () -- touple containing the dimensions, the spacing and the offset of the terrain
    """

    # read context informations from *.csv file
    with open(csv_file_path) as fp:
        for i, line in enumerate(fp):
            if i != 1:
                continue

            # we are interested in all of the context information
            num_of_rows, num_of_columns, spacing_y, spacing_x, x_0, y_0 = np.fromstring(line, dtype=float, sep=' ')

            break

    # convert y_0 to actual coordinate at ind_y=0
    y_0 -= (num_of_rows-1)*spacing_y

    return (int(num_of_rows), int(num_of_columns), spacing_y, spacing_x, x_0, y_0)

def get_landmark_coords_from_csv(csv_file_path):
    """This function extracts the landmarks' coordinates from a csv file based on the provided 
        files of the ERC
    
    Arguments:
        csv_file_path {str} -- path to the ERC csv file (Landmarks)

    Returns:
        [[]] -- array of coordinates
    """

    # load coords from *.csv file
    coords = np.loadtxt(open(csv_file_path), delimiter=',', skiprows=1, usecols=(1,2))

    return coords

def create_costmap(context_info, coords, radius):
    """This function creates a costmap of correct size and marks the landmarks' position
    
    Arguments:
        context_info {()} -- dimensions, spacing and offset of the terrain
        coords {[[]]} -- array of landmarks' coordinates
        radius {int} -- radius of the area a landmark covers

    Returns:
        [[]] -- 2d array of greyscale color values
    """

    # extract the context information
    num_of_rows, num_of_columns, spacing_y, spacing_x, x_0, y_0 = context_info

    # landmarks' position relative to coords_0
    coords -= (x_0, y_0)

    # create blank costmap
    costs = np.full((num_of_columns, num_of_rows), 255)

    # get coordinates of all elements in the costmap
    coords_x = np.indices(costs.shape)[0]*spacing_x
    coords_y = np.indices(costs.shape)[1]*spacing_y

    # create empty mask
    mask = np.full((num_of_columns, num_of_rows), False)

    # for each landmark, add a circle in appropriate size
    for (x,y) in coords:
        new_mask = (coords_x - x)**2 + (coords_y - y)**2 <= radius **2
        mask = np.ma.mask_or(new_mask, mask)

    # color the circles in black
    costs[mask] = 0

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


def generate_costmap(heightmap, landmarks, output, radius):
    """generate a costmap used for autonomous navigation based on the landmarks' position and size

    Arguments:
        heightmap {str} -- path to the ERC csv file (ver2)
        landmarks {str} -- path to a landmarks csv file
        output {str} -- path to the finished costmap
        radius {int} -- radius of the area a landmark covers
    """

    # read context info
    context_info = get_context_info_from_csv(heightmap)

    # read landmark coords
    coords = get_landmark_coords_from_csv(landmarks)

    # mark landmarks' position in a costmap
    costs = create_costmap(context_info, coords, radius)

    # write costmap
    write_costmap(costs, output)


if __name__ == '__main__':

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    
    # default values
    heightmap_csv_path = os.path.join(rover_sim_dir, 'worlds/erc2018final/Heightmap.csv')
    landmarks_csv_path = os.path.join(rover_sim_dir, 'worlds/erc2018final/Landmarks.csv')
    costmap_pgm_path = os.path.join(rover_nav_dir, 'envs/landmarks.pgm')
    landmarks_radius = 0.3

    # parse command line arguments
    parser = ArgumentParser(
        description="generate a costmap used for autonomous navigation based on the landmarks' position and size",
        formatter_class=ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-m", "--heightmap", type=str, help="path to an ERC csv file (ver2)", default=heightmap_csv_path)
    parser.add_argument("-l", "--landmarks", type=str, help="path to a landmarks csv file", default=landmarks_csv_path)
    parser.add_argument("-o", "--output", type=str, help="path to the finished costmap (*.pgm)", default=costmap_pgm_path)
    parser.add_argument("-r", "--radius", type=float, help="radius of the area a landmark covers", default=landmarks_radius)
    args = parser.parse_args()

    # generate costmap
    generate_costmap(heightmap=args.heightmap, landmarks=args.landmarks, output=args.output, radius=args.radius)
