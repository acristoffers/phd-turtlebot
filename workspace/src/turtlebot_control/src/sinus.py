#!/usr/bin/env python3

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import json
import numpy as np
import os
import rospy
import scipy.io as sio


class Node:
    def __init__(self):
        rospy.init_node("controller_node")
        rospy.Subscriber(
            "/timescale/odom", PoseWithCovarianceStamped, self.odom_callback
        )
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose = None
        json_path = os.path.join("/workspace", "gains.json")
        with open(json_path) as json_file:
            self.gains = json.load(json_file)
        self.m1 = self.gains["m1"]
        self.m2 = self.gains["m2"]
        self.__t = rospy.Time.now()
        self.__keys = [int(k) for k in self.gains.keys() if not k.startswith("m")]
        self.__keys = np.asarray(self.__keys)
        self.data = []
        self.cur_waypoint = 0
        self.waypoints = [
            np.array([[x], [y]])
            for (x, y) in [
                [4 + 0.5 * np.sin(t), 2 + t / np.pi]
                for t in np.arange(0, 2 * np.pi, 0.5)
            ]
        ]
        rospy.loginfo("Controller started")
        self.u = np.array([[0, 0]]).T
        self.continuous = False

    def odom_callback(self, msg):
        t = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        if (rospy.Time.now() - t).to_sec() > 0.1:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _t = R.from_quat([o.x, o.y, o.z, o.w])
        _z = _t.as_euler("xyz")[-1]
        self.pose = [p.x, p.y, _z]
        self.update(self.pose, t)

    def update(self, pose, t):
        x = np.array([pose]).T
        t_delta = (rospy.Time.now() - t).to_sec()
        m = t_delta + (t - self.__t).to_sec()
        if m < self.m1:
            return
        if m > self.m2:
            m = float(self.m2)
        theta = self.rad_range(x[-1].item())
        closest_theta = self.__keys[np.abs(self.__keys - np.rad2deg(theta)).argmin()]
        obs = self.gains[str(closest_theta.item())]
        g2 = (1 / m - 1 / self.m1) / (1 / self.m2 - 1 / self.m1)
        g1 = 1 - g2
        K = g1 * np.asarray(obs["K"][0]) + g2 * np.asarray(obs["K"][1])
        if self.continuous:
            K = np.asarray(obs["K"][0])
        r = self.waypoints[self.cur_waypoint].copy()
        angle = self.signed_angle(x[0:2], r[0:2], x[-1].item()).item()
        e = (
            np.array([[x[0].item(), x[1].item(), angle]]).T
            - np.array([[r[0].item(), r[1].item(), 0]]).T
        )
        e[-1] = self.rad_range(e[-1])
        u = -K @ e
        if np.abs(angle) > 0.15 and np.linalg.norm(e[0:2]) > 0.1:
            rospy.loginfo("rotating..................")
            u[0] = 0
        self.u = u
        dir = np.sign(self.is_facing(x[0:2], r[0:2], x[-1].item()))
        u[0] = dir * np.abs(u[0])
        u[0] = np.sign(u[0]) * min(abs(u[0].item()), 0.2)
        u[1] = np.sign(u[1]) * min(abs(u[1].item()), 0.8)
        rospy.loginfo(
            f"---\n"
            f"Time: {t.to_time()}\n"
            f"Time Delta: {t_delta}\n"
            f"µ: {m:.2}\n"
            f"Closest theta: {closest_theta}\n"
            f"Waypoint: {self.cur_waypoint}\n"
            f"Error: {self.fmt(e.T)}\n"
            f"r: {self.fmt(r.T)}\n"
            f"x: {self.fmt(x.T)}\n"
            f"u: {self.fmt(u.T)}\n"
            f"K: {self.fmt(K)}\n"
        )
        # self.data.append(
        #     {
        #         "t": t.to_time(),
        #         "closest_theta": closest_theta,
        #         "mu": m,
        #         "waypoint": self.cur_waypoint,
        #         "error": e,
        #         "r": r,
        #         "x": x,
        #         "u": u,
        #         "K": K,
        #     }
        # )
        command_message = Twist()
        self.__t = t
        command_message.linear.x = u[0].item()
        command_message.angular.z = u[1].item()
        if np.linalg.norm(e[0:2]) < 3e-2:
            self.cur_waypoint = (self.cur_waypoint + 1) % len(self.waypoints)
        self.pub.publish(command_message)
        # if t.secs > 10:
        #     sio.savemat("/tmp/signals.mat", {"signals": self.data})

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            command_message = Twist()
            command_message.linear.x = self.u[0].item()
            command_message.angular.z = self.u[1].item()
            self.pub.publish(command_message)
            rate.sleep()
        command_message = Twist()
        command_message.linear.x = 0.0
        command_message.angular.z = 0.0
        self.pub.publish(command_message)

    def signed_angle(self, p1, p2, a):
        Va = np.array([[np.cos(a), np.sin(a)]]).T
        Vb = p2 - p1
        return np.arctan2(Vb[0] * Va[1] - Vb[1] * Va[0], Vb[0] * Va[0] + Vb[1] * Va[1])

    def rad_range(self, xs):
        return (xs + np.pi) % (2 * np.pi) - np.pi

    def deg_range(self, xs):
        return (xs + 180) % 360 - 180

    def is_facing(self, v, u, a):
        """
        Verifies if the vector at v with angle a is facing the point u

        Returns:
        closest to
        +1 -> facing perfectly
        -1 -> facing the oposite direction

        Parameters:
        v -> point of start of vector
        a -> angle the vector is facing
        u -> point to verify if the vector is facing

        See
        https://gamedev.stackexchange.com/questions/109513/how-to-find-if-an-object-is-facing-another-object-given-position-and-direction-a
        """
        v = v.flatten()
        u = u.flatten()
        directionFacingOfV = np.array([np.cos(a), np.sin(a)])
        directionFacingOfV = directionFacingOfV / np.linalg.norm(directionFacingOfV)
        return np.dot((u - v) / np.linalg.norm(u - v), directionFacingOfV)

    def fmt(self, x):
        return np.array2string(x, precision=2, floatmode="fixed")


if __name__ == "__main__":
    try:
        node = Node()
        node.loop()
    except rospy.ROSInterruptException:
        if node is not None:
            command_message = Twist()
            command_message.linear.x = 0.0
            command_message.angular.z = 0.0
            node.pub.publish(command_message)
