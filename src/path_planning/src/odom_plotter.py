#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter', anonymous=True)
        self.path_pub = rospy.Publisher('/trajectory_path', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "odom"
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tp = TrajectoryPlotter()
        tp.run()
    except rospy.ROSInterruptException:
        pass