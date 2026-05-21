#!/usr/bin/env python3
"""Convert a ROS map pose to the Gazebo spawn pose used by this simulation.

This script only prints values; it does not edit any project file.

Normal usage:
    1. Edit MAP_X, MAP_Y, MAP_YAW and GOAL_NAME in the config block below.
    2. Run:
           python3 tools/map_to_gazebo_pose.py

Coordinates to edit after running:

1. Gazebo spawn pose:
   nav_sim_ws/src/car_sim/launch/car_urdf.launch
   Change the spawn_model args:
       -x <gazebo_x> -y <gazebo_y> -z 0.1 -Y <gazebo_yaw>

2. AMCL initial pose:
   nav_sim_ws/src/car_sim/launch/amcl.launch
   Change or add:
       <param name="initial_pose_x" value="<map_x>"/>
       <param name="initial_pose_y" value="<map_y>"/>
       <param name="initial_pose_a" value="<map_yaw>"/>

3. Pharmacy control navigation point, if this pose is also a task point:
   control_ws/src/move_nav/src/control_node_yaofang_msg_template.cpp
   Change the GOAL_LIST item, for example:
       {<map_x>, <map_y>, <map_yaw>, "home"},

Default calibration used here:
    Gazebo/world pose (1.74, 1.74, 3.14)
    corresponds to map pose (0.0, 0.0, 0.0)

This matches the current nav_sim_ws setup observed from:
    rostopic echo -n1 /gazebo/model_states
    rosrun tf tf_echo odom base_footprint
    rosrun tf tf_echo map odom
"""

import argparse
import math


# =========================
# User config: edit here
# =========================
#
# Fill in the pose you picked in RViz/map.
# These values are the same kind of coordinates used in:
#   control_ws/src/move_nav/src/control_node_yaofang_msg_template.cpp
#   GOAL_LIST item:
#       {MAP_X, MAP_Y, MAP_YAW, GOAL_NAME}
#
# Example home point:
#   {1.210, 3.725, -3.062, "home"}
MAP_X = 1.210
MAP_Y = 3.725
MAP_YAW = -3.062
GOAL_NAME = "home"


# Calibration between map coordinates and Gazebo/world coordinates.
#
# Current observed relationship in this simulation:
#   map    (0.0, 0.0, 0.0)
#   equals
#   Gazebo (1.74, 1.74, 3.14)
#
# If you recalibrate the map/world relationship later, edit these values.
MAP_REF_X = 0.0
MAP_REF_Y = 0.0
MAP_REF_YAW = 0.0
GAZEBO_REF_X = 1.74
GAZEBO_REF_Y = 1.74
GAZEBO_REF_YAW = 3.14


def normalize_angle(angle):
    """Normalize angle to [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def map_to_gazebo(map_x, map_y, map_yaw,
                  map_ref_x, map_ref_y, map_ref_yaw,
                  gazebo_ref_x, gazebo_ref_y, gazebo_ref_yaw):
    """Convert a map-frame pose to a Gazebo/world-frame pose.

    The relationship is treated as a 2D rigid transform:
        map_ref pose <-> gazebo_ref pose

    For the current project default:
        map (0, 0, 0) <-> gazebo (1.74, 1.74, 3.14)
    """
    dx = map_x - map_ref_x
    dy = map_y - map_ref_y

    theta = gazebo_ref_yaw - map_ref_yaw
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    gazebo_x = gazebo_ref_x + cos_t * dx - sin_t * dy
    gazebo_y = gazebo_ref_y + sin_t * dx + cos_t * dy
    gazebo_yaw = normalize_angle(gazebo_ref_yaw + (map_yaw - map_ref_yaw))

    return gazebo_x, gazebo_y, gazebo_yaw


def main():
    parser = argparse.ArgumentParser(
        description="Convert a map pose to the Gazebo spawn pose for nav_sim_ws."
    )
    parser.add_argument("map_x", type=float, nargs="?", default=MAP_X,
                        help="Target x in map frame. Defaults to MAP_X in this file.")
    parser.add_argument("map_y", type=float, nargs="?", default=MAP_Y,
                        help="Target y in map frame. Defaults to MAP_Y in this file.")
    parser.add_argument("map_yaw", type=float, nargs="?", default=MAP_YAW,
                        help="Target yaw in map frame, radians. Defaults to MAP_YAW in this file.")
    parser.add_argument("--name", default=GOAL_NAME, help="Goal name shown in output snippets.")

    parser.add_argument("--map-ref-x", type=float, default=MAP_REF_X)
    parser.add_argument("--map-ref-y", type=float, default=MAP_REF_Y)
    parser.add_argument("--map-ref-yaw", type=float, default=MAP_REF_YAW)
    parser.add_argument("--gazebo-ref-x", type=float, default=GAZEBO_REF_X)
    parser.add_argument("--gazebo-ref-y", type=float, default=GAZEBO_REF_Y)
    parser.add_argument("--gazebo-ref-yaw", type=float, default=GAZEBO_REF_YAW)

    args = parser.parse_args()

    gazebo_x, gazebo_y, gazebo_yaw = map_to_gazebo(
        args.map_x,
        args.map_y,
        args.map_yaw,
        args.map_ref_x,
        args.map_ref_y,
        args.map_ref_yaw,
        args.gazebo_ref_x,
        args.gazebo_ref_y,
        args.gazebo_ref_yaw,
    )

    print("Input map pose:")
    print(f"  x   = {args.map_x:.6f}")
    print(f"  y   = {args.map_y:.6f}")
    print(f"  yaw = {args.map_yaw:.6f} rad")
    print()

    print("Gazebo spawn pose to use in nav_sim_ws/src/car_sim/launch/car_urdf.launch:")
    print(f"  -x {gazebo_x:.6f} -y {gazebo_y:.6f} -z 0.1 -Y {gazebo_yaw:.6f}")
    print()

    print("Replace the spawn_model args with:")
    print('  args="-urdf -model car_simple -param robot_description '
          f'-x {gazebo_x:.6f} -y {gazebo_y:.6f} -z 0.1 -Y {gazebo_yaw:.6f}"')
    print()

    print("AMCL initial pose to use in nav_sim_ws/src/car_sim/launch/amcl.launch:")
    print(f'  <param name="initial_pose_x" value="{args.map_x:.6f}"/>')
    print(f'  <param name="initial_pose_y" value="{args.map_y:.6f}"/>')
    print(f'  <param name="initial_pose_a" value="{args.map_yaw:.6f}"/>')
    print()

    print("GOAL_LIST item to use in control_ws/src/move_nav/src/control_node_yaofang_msg_template.cpp:")
    print(f'  {{{args.map_x:.6f}, {args.map_y:.6f}, {args.map_yaw:.6f}, "{args.name}"}},')
    print()

    print("Verification commands after relaunch:")
    print("  rostopic echo -n1 /gazebo/model_states")
    print("  rosrun tf tf_echo odom base_footprint")
    print("  rosrun tf tf_echo map base_footprint")


if __name__ == "__main__":
    main()
