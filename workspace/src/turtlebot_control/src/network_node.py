#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
from numpy import random
import rospy


class NetworkNode:
    def __init__(self):
        rospy.init_node("turtlebot_network")
        rospy.Subscriber("/optitrack", PoseWithCovarianceStamped, self.__update)
        self.__publisher = rospy.Publisher("/timescale/odom", PoseWithCovarianceStamped, queue_size=1)
        self.t = rospy.Time.now()
        self.skip = 0
        rospy.loginfo("Network node started")

    def __update(self, msg):
        t = rospy.Time.now()
        if self.__attack():
            if (t - self.t).to_sec() >= self.skip:
                self.skip = 0.5 # random.randint(0, 1500) / 1000
                self.t = t
            else:
                return
        self.__publisher.publish(msg)

    def __attack(self):
        return True

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()


def main(args=None):
    node = NetworkNode()
    node.loop()


if __name__ == "__main__":
    main()
