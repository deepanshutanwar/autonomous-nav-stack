#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from rrt import rrt_pathfinder

# ── Global state ────────────────────────────────────────────────────────────
map_img    = None
resolution = None
origin     = None
width      = None
height     = None

# ── Publisher (set up after node init) ──────────────────────────────────────
trajectory_pub = None


# ── Callback: /map ───────────────────────────────────────────────────────────
def occupancygrid_to_image(msg):
    """Convert OccupancyGrid → flipped grayscale image (image coordinates)."""
    global map_img, width, height, resolution, origin

    resolution = msg.info.resolution
    x_origin   = msg.info.origin.position.x
    y_origin   = msg.info.origin.position.y
    origin     = [x_origin, y_origin]

    width  = msg.info.width
    height = msg.info.height

    data = np.array(msg.data).reshape((height, width))

    # Convert occupancy values → grayscale
    #   free (0)     → 255  (white)
    #   unknown (−1) → 127  (grey)
    #   occupied(100)→ 0    (black)
    img = np.full_like(data, 255, dtype=np.uint8)
    img[data == -1]  = 127
    img[data == 100] = 0

    # Flip vertically so origin is top-left (image convention)
    map_img = np.flipud(img)

    rospy.loginfo("Map received: %dx%d pixels, resolution=%.4f m/px",
                  width, height, resolution)


def map_callback(msg):
    occupancygrid_to_image(msg)


# ── Callback: /start_goal ────────────────────────────────────────────────────
def start_goal_callback(msg):
    global map_img, resolution, origin

    if map_img is None:
        rospy.logwarn("Map not yet received – ignoring start_goal message.")
        return

    if len(msg.data) < 4:
        rospy.logerr("Expected 4 values in /start_goal [x_start, y_start, x_goal, y_goal]")
        return

    x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data[:4]

    start = [x_start_real, y_start_real]
    goal  = [x_goal_real,  y_goal_real]

    rospy.loginfo("Planning RRT from %s  →  %s", start, goal)

    try:
        start_px, goal_px, path_px, path_world = rrt_pathfinder(
            start, goal, map_img, resolution, origin
        )
    except Exception as e:
        rospy.logerr("RRT failed: %s", str(e))
        return

    rospy.loginfo("Path found: %d waypoints", len(path_px))

    # ── Visualise trajectory on the map and save to file ────────────────────
    vis = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

    # Draw path
    for i in range(1, len(path_px)):
        cv2.line(vis, path_px[i-1], path_px[i], (0, 0, 255), 2)   # red line

    # Draw start (green) and goal (blue)
    if path_px:
        cv2.circle(vis, start_px, 6, (0, 255, 0), -1)
        cv2.circle(vis, goal_px,  6, (255, 0, 0), -1)

    out_file = "/tmp/rrt_trajectory.png"
    cv2.imwrite(out_file, vis)
    rospy.loginfo("Trajectory image saved to %s", out_file)

    # ── Publish trajectory ───────────────────────────────────────────────────
    traj_msg = Float64MultiArray()
    trajectory = []
    for (x_world, y_world) in path_world:  # ← fixed: world coords
        trajectory.extend([float(x_world), float(y_world)])
    traj_msg.data = trajectory

    trajectory_pub.publish(traj_msg)
    rospy.loginfo("Trajectory published on /trajectory (%d points)", len(path_px))


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    global trajectory_pub

    rospy.init_node('RRT_node', anonymous=False)

    # Publisher
    trajectory_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)

    # Subscribers
    rospy.Subscriber('/map',        OccupancyGrid,     map_callback)
    rospy.Subscriber('/start_goal', Float64MultiArray, start_goal_callback)

    rospy.loginfo("RRT_node ready.")
    rospy.spin()


if __name__ == '__main__':
    main()