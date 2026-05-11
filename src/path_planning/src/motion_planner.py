#!/usr/bin/env python3

"""
motion_planner.py  –  Lab 5
-----------------------------
Subscribes to:
    /odom            (nav_msgs/Odometry)
    /trajectory      (std_msgs/Float64MultiArray)  [x0,y0, x1,y1, ...]  world coords

Publishes to:
    /start_goal      (std_msgs/Float64MultiArray)  [x_start, y_start, x_goal, y_goal]
    /reference_pose  (std_msgs/Float64MultiArray)  [xr, yr, theta_r, mode]
"""

import rospy
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ── Tolerances ───────────────────────────────────────────────────────────────
TOL_POS = 0.30      # metres — how close = "waypoint reached"

class MotionPlanner:

    def __init__(self):
        rospy.init_node('motion_planner', anonymous=False)

        # Robot pose (updated by /odom)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Trajectory state
        self.waypoints   = []    # list of (x, y) in world coords
        self.wp_idx      = 0
        self.goal_sent   = False
        self.goal_reached = False

        # Mode for PID (0 = sequential, 1 = simultaneous) — keep same as Lab 4
        self.mode = 0

        # Publishers
        self.start_goal_pub  = rospy.Publisher('/start_goal',      Float64MultiArray, queue_size=10)
        self.ref_pose_pub    = rospy.Publisher('/reference_pose',  Float64MultiArray, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber('/odom',       Odometry,          self.odom_cb)
        rospy.Subscriber('/trajectory', Float64MultiArray, self.trajectory_cb)

        rospy.sleep(1.0)   # let connections establish

        rospy.loginfo("Motion Planner ready.")
        self.run()

    # ── Callbacks ────────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def trajectory_cb(self, msg):
        data = msg.data
        if not data or len(data) < 2:
            rospy.logwarn("Empty trajectory received.")
            return
        # Parse flat [x0,y0, x1,y1, ...] into list of (x,y) tuples
        self.waypoints = [(data[i], data[i+1]) for i in range(0, len(data)-1, 2)]
        self.wp_idx    = 0
        self.goal_reached = False
        rospy.loginfo("Trajectory received: %d waypoints", len(self.waypoints))
        # Immediately send the first waypoint to PID
        self.send_next_waypoint()

    # ── Helpers ──────────────────────────────────────────────────────────────

    def dist_to_waypoint(self, wx, wy):
        return math.hypot(wx - self.x, wy - self.y)

    def send_next_waypoint(self):
        """Publish current waypoint as /reference_pose to PID controller."""
        if self.wp_idx >= len(self.waypoints):
            return
        wx, wy = self.waypoints[self.wp_idx]

        # Compute heading angle toward this waypoint
        theta_r = math.atan2(wy - self.y, wx - self.x)

        # If it's the LAST waypoint, use angle toward goal (or 0.0)
        if self.wp_idx == len(self.waypoints) - 1:
            theta_r = self.theta   # keep current heading at final goal

        msg = Float64MultiArray()
        msg.data = [wx, wy, theta_r, float(self.mode)]
        self.ref_pose_pub.publish(msg)
        rospy.loginfo("Waypoint %d/%d → (%.2f, %.2f, theta=%.2f)",
                      self.wp_idx + 1, len(self.waypoints), wx, wy, theta_r)

    def ask_goal(self):
        """Ask user for goal coordinate (x, y) — same style as Lab 3/4."""
        print("\n" + "-"*50)
        print("  Enter the GOAL coordinate (global map frame, metres)")
        while not rospy.is_shutdown():
            raw = input("  x_goal  y_goal  (or 'q' to quit): ").strip()
            if raw.lower() == 'q':
                return None, None
            parts = raw.split()
            if len(parts) == 2:
                try:
                    return float(parts[0]), float(parts[1])
                except ValueError:
                    pass
            print("  Please enter exactly 2 numbers, e.g.:  2.0  0.0")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)   # 10 Hz check loop

        while not rospy.is_shutdown():

            # ── Step 1: get goal from user and send /start_goal to RRT ───────
            if not self.goal_sent:
                gx, gy = self.ask_goal()
                if gx is None:
                    break

                msg = Float64MultiArray()
                msg.data = [self.x, self.y, gx, gy]
                self.start_goal_pub.publish(msg)
                rospy.loginfo("Sent /start_goal: start=(%.2f,%.2f) goal=(%.2f,%.2f)",
                              self.x, self.y, gx, gy)
                self.goal_sent   = True
                self.goal_reached = False
                # Wait for /trajectory callback to arrive (trajectory_cb handles the rest)
                rate.sleep()
                continue

            # ── Step 2: check if current waypoint is reached ─────────────────
            if self.waypoints and self.wp_idx < len(self.waypoints):
                wx, wy = self.waypoints[self.wp_idx]

                if self.dist_to_waypoint(wx, wy) < TOL_POS:
                    rospy.loginfo("Reached waypoint %d", self.wp_idx)
                    self.wp_idx += 1

                    if self.wp_idx >= len(self.waypoints):
                        rospy.loginfo("=== FINAL GOAL REACHED ===")
                        self.goal_reached = True
                        self.goal_sent    = False   # ready for next goal
                        self.waypoints    = []
                        print("\n  Goal reached! Press Enter to set a new goal...")
                        input()
                    else:
                        # Send next waypoint to PID
                        self.send_next_waypoint()

            rate.sleep()


if __name__ == '__main__':
    try:
        MotionPlanner()
    except rospy.ROSInterruptException:
        pass