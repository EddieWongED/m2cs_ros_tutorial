#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import *

cur_pos = Pose()


def cb_pose(data):
    global cur_pos
    cur_pos = data


def cb_walk(req):
    if (req.distance < 0):
        return False

    # hint: calculate the projected (x, y) after walking the distance,
    # and return false if it is outside the boundary

    rate = rospy.Rate(100)  # 100Hz control loop

    while ('''goal not reached'''):  # control loop

        # in each iteration of the control loop, publish a velocity

        # hint: you need to use the formula for distance between two points

        rate.sleep()

    vel = Twist()  # publish a velocity 0 at the end, to ensure the turtle really stops
    pub.publish(vel)

    return True


def cb_orientation(req):
    rate = rospy.Rate(100)

    rospy.loginfo(cur_pos.theta)
    rospy.loginfo(req.theta)

    theta_diff = req.theta - cur_pos.theta
    if (theta_diff > pi):
        theta_diff -= 2 * pi
    elif (theta_diff < -pi):
        theta_diff += 2 * pi

    if (theta_diff > 2 * pi or theta_diff < -2 * pi):
        rospy.loginfo("Input theta is too large (range: -2PI to 2PI)")
        resp = SetOrientationResponse()
        resp.success = False
        return resp
    else:
        while (theta_diff != 0):  # control loop

            twist = Twist()
            twist.angular.z = theta_diff * 10
            pub.publish(twist)

            theta_diff = req.theta - cur_pos.theta
            if (theta_diff > pi):
                theta_diff -= 2 * pi
            elif (theta_diff < -pi):
                theta_diff += 2 * pi

            rate.sleep()

    rospy.loginfo("Successfully set orientation")
    twist = Twist()
    pub.publish(twist)

    resp = SetOrientationResponse()
    resp.success = True
    return resp


if __name__ == '__main__':
    rospy.init_node('path_manager')

    # Subscribe to /turtle1/pose
    rospy.Subscriber("/turtle1/pose", Pose, cb_pose)

    # Hosting service /set_orientation
    rospy.Service('set_orientation', SetOrientation, cb_orientation)
    rospy.loginfo("Service /set_orientation is ready")

    # Publishing velocity to /turtle1/cmd_vel
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    # init each service server here:
    # rospy.Service( ... )		# callback to cb_orientation
    # rospy.Service( ... )		# callback to cb_walk

    rospy.spin()
