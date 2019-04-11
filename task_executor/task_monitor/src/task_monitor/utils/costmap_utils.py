#!/usr/bin/env python
# Helper functions for translating costmaps

from __future__ import print_function, division

import numpy as np

from geometry_msgs.msg import Point


# Constants defined in move_base
CONST_NO_INFORMATION = 255
CONST_LETHAL_OBSTACLE = 254
CONST_INSCRIBED_INFLATED_OBSTACLE = 253
CONST_FREE_SPACE = 0


# Functions. Also defined in move_base
def make_footprint(radius, num_points):
    """Given a radius param and the number of points in the footprint polygon,
    make a footprint polygon. The footprint is relative to the center"""
    footprint = []
    for i in xrange(num_points):
        angle = i * 2 * np.pi / num_points
        footprint.append(Point(
            x=(radius * np.cos(angle)),
            y=(radius * np.sin(angle))
        ))
    return footprint


def make_cost_translation_func():
    """Values published in the costmap don't correspond to the parameter
    settings for obstacles; we need a translation function. This creates that
    function."""
    internal_table = {
        0: CONST_FREE_SPACE,
        -1: CONST_NO_INFORMATION,
        99: CONST_INSCRIBED_INFLATED_OBSTACLE,
        100: CONST_LETHAL_OBSTACLE,
    }
    func = lambda x: internal_table.get(x, int(1 + (251 * (x - 1)) / 97))
    return func


def get_map_coords(wx, wy, costmap):
    """Given coordinates in the world, figure out the indices in the costmap
    that correspond to them. Make sure that the frame_id of the coordinates are
    the same as that of the costmap!"""
    ox, oy, res = (costmap.info.origin.position.x,
                   costmap.info.origin.position.y,
                   costmap.info.resolution)
    size_x, size_y = costmap.info.width, costmap.info.height

    # Sanity check the inputs
    assert not (wx < ox or wy < oy), ("World coordinates {} not in map with origin {}"
                                      .format((wx, wy,), (ox, oy,)))

    # Calculate the map coordinate
    mx = (wx - ox) / res
    my = (wy - oy) / res

    # Sanity check the outputs
    assert (mx < size_x and my < size_y), ("Calculated coords {} larger than map size {}"
                                           .format((mx, my,), (size_x, size_y,)))

    # Return
    return (int(mx), int(my),)


def check_collision(x, y, footprint, costmap, cost_translation_func):
    """
    Assuming the robot is at coordinates (x, y) check to see if it is in
    collision based on its footprint polygon, the costmap, and a cost
    translation function to interpret the costmap values.
    """
    in_collision = False
    for point in footprint:
        px, py = x + point.x, y + point.y
        mx, my = get_map_coords(px, py, costmap)
        cost = cost_translation_func(costmap.data[my * costmap.info.width + mx])
        in_collision = in_collision or (
            cost in [CONST_NO_INFORMATION,
                     CONST_LETHAL_OBSTACLE,
                     CONST_INSCRIBED_INFLATED_OBSTACLE]
        )

    return in_collision
