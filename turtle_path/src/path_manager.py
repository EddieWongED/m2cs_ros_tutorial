#!/usr/bin/env python
import rospy
from math import pi, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import *

# roscore
# rosrun turtlesim turtlesim_node
# rosrun turtle_path path_manager.py

# Subscribe:
# Topic: /turtle1/pose
# Type: Post

# Publish:
# Topic: /turtle1/cmd_vel
# Type: Twist

# Server:
# Service: /set_orientation
# Type: SetOrientation

# Server:
# Service: /walk_distance
# Type: WalkDistance

cur_pos = Pose()


def cb_pose(data):
    global cur_pos
    cur_pos = data


def cb_walk(req):
    if (req.distance < 0):
        resp = WalkDistanceResponse()
        resp.success = False
        rospy.loginfo("Negative distance is not accepted")
        return resp

    target_x = cur_pos.x + req.distance * cos(cur_pos.theta)
    target_y = cur_pos.y + req.distance * sin(cur_pos.theta)

    if (target_x < 0 or target_x > 11 or target_y < 0 or target_y > 11):
        resp = WalkDistanceResponse()
        resp.success = False
        rospy.loginfo("Target is out of the boundary")
        return resp

    rate = rospy.Rate(100)

    distance_diff = req.distance

    while (abs(distance_diff) > 0.005):
        twist = Twist()
        twist.linear.x = distance_diff * 10
        pub.publish(twist)

        distance_diff = sqrt((target_x - cur_pos.x) ** 2 +
                             (target_y - cur_pos.y) ** 2)

        rate.sleep()

    twist = Twist()
    pub.publish(twist)

    resp = WalkDistanceResponse()
    resp.success = True
    rospy.loginfo("Target reached")
    return resp


def cb_orientation(req):
    rate = rospy.Rate(100)

    theta_diff = req.orientation - cur_pos.theta
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
        while (abs(theta_diff) > 0.005):  # control loop

            twist = Twist()
            twist.angular.z = theta_diff * 10
            pub.publish(twist)

            theta_diff = req.orientation - cur_pos.theta
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

    # Hosting service /set_orientation
    rospy.Service('walk_distance', WalkDistance, cb_walk)
    rospy.loginfo("Service /walk_distance is ready")

    # Publishing velocity to /turtle1/cmd_vel
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    rospy.spin()

    rospy.loginfo("Node path_manager terminated")
