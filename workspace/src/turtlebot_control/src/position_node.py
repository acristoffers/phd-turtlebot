#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy


class Node:
    def __init__(self):
        rospy.init_node("position_printer_node")
        rospy.Subscriber("/timescale/odom", PoseWithCovarianceStamped, self.odom_callback)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _t = np.array([o.x, o.y, o.z, o.w])
        _t = R.from_quat(_t)
        _z = _t.as_euler("xyz")[-1]
        rospy.loginfo(f"[x,y,a]: [{p.x}, {p.y}, {_z}]")

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    try:
        node = Node()
        node.loop()
    except rospy.ROSInterruptException:
        pass
