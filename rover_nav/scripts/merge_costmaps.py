#!/usr/bin/env python
"""
merge multiple costmaps provided as *.pgm files"
"""

import numpy as np
import os, sys
from rospkg import RosPack
import cv2

# use files relative to rover_nav
rospack = RosPack()
rover_nav_dir = rospack.get_path('rover_nav')

# import relative to rover_nav
sys.path.append(os.path.dirname(rover_nav_dir))

import rover_nav.scripts.generate_costmap_landmarks as lm
import rover_nav.scripts.generate_costmap_steepness as sn

def merge_costs(costs_1, costs_2):
    """merge two costmaps by applying logical or

    Arguments:
        costs_1 {[[]]} -- 2d array of greyscale color values
        costs_2 {[[]]} -- 2d array of greyscale color values

    Returns:
        [[]] -- 2d array of greyscale color values
    """

    # mark all elements which are colored black at least once
    mask = np.ma.mask_or(costs_1==0, costs_2==0)

    # create empty costmap
    costs = np.full(costs_1.shape, 255)

    # color marked elements black
    costs[mask] = 0

    return costs

def write_costmap(costs, output):
    """write the costmap to a *.pgm file

    Arguments:
        costs {[[]]} -- 2d array of greyscale color values
        output {str} -- path to the finished costmap
    """

    # write *.pgm file
    cv2.imwrite(output, costs)

def merge_costmaps(paths, output):
    """merge multiple costmaps provided as *.pgm files

    Arguments:
        paths {[]} -- array of paths to costmaps that should be merged
        output {str} -- path to the finished costmap
    """

    # read costmaps from images
    costmaps = []
    for path in paths:
        costmap = cv2.imread(path, 0)

        if costmap is None:
            raise Exception('"{}" is not a valid path.'.format(path))
        
        costmaps.append(costmap)

    # merge costs by applying logical or
    costs = np.full(costmaps[0].shape, 255)
    for costmap in costmaps:
        costs = merge_costs(costs, costmap)

    # write new costmap
    write_costmap(costs, output)


if __name__ == '__main__':

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    
    # default values
    steepness_costmap_path = os.path.join(rover_nav_dir, 'envs/steepness.pgm')
    landmarks_costmap_path = os.path.join(rover_nav_dir, 'envs/landmarks.pgm')

    paths = (steepness_costmap_path, landmarks_costmap_path)
    costmap_pgm_path = os.path.join(rover_nav_dir, 'envs/costmap.pgm')

    # parse command line arguments
    parser = ArgumentParser(
        description="merge multiple costmaps provided as *.pgm files",
        formatter_class=ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-p", "--paths", type=str, nargs='+', help="paths to several costmaps (*.pgm)", default=paths)
    parser.add_argument("-o", "--output", type=str, help="path to the finished costmap (*.pgm)", default=costmap_pgm_path)
    args = parser.parse_args()

    # merge costmaps
    merge_costmaps(paths=args.paths, output=args.output)