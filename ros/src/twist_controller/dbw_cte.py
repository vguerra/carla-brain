import math
import numpy as np

from tf.transformations import euler_from_quaternion

WAYPOINTS_LOOKAHEAD = 8

def compute_cte(waypoints, pose):
    xs, ys = get_points_wrt_pose(waypoints[:WAYPOINTS_LOOKAHEAD], pose)
    fit = np.poly1d(np.polyfit(xs, ys, 3))
    cte = fit(4)

    return cte

def get_points_wrt_pose(waypoints, pose):
    yaw = yaw_from_orientation(orientation=pose.orientation)

    x, y = pose.position.x, pose.position.y

    shifted_rotated_xs = []
    shifted_rotated_ys = []

    for waypoint in waypoints:
        shift_x = waypoint.pose.pose.position.x - x
        shift_y = waypoint.pose.pose.position.y - y

        shifted_rotated_xs.append(shift_x * math.cos(0 - yaw) - shift_y * math.sin(0 - yaw))
        shifted_rotated_ys.append(shift_x * math.sin(0 - yaw) + shift_y * math.cos(0 - yaw))

    return shifted_rotated_xs, shifted_rotated_ys

def yaw_from_orientation(orientation):
    quaternion = [orientation.x,
                  orientation.y,
                  orientation.z,
                  orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    
    return yaw
