#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import *
from m2_ps4.msg import *
from std_srvs.srv import *

# roscore
# rosrun turtlesim turtlesim_node
# roslaunch m2_ps4 ps4_msg.launch joy:=/dev/ds4camo
# rosrun turtle_ps4 ps4_controller.py

# Subscribe:
# Topic: /input/ps4_data
# Type: Ps4Data

# Publish:
# Topic: /turtle1/cmd_vel
# Type: Twist

# Client:
# Service: /turtle1/set_pen
# Type: SetPen

# Client:
# Service: /clear
# Type: Empty

pen_color_r = 179
pen_color_g = 184
pen_color_b = 255
DEFAULT_PEN_WIDTH = 3

pen_client = rospy.ServiceProxy("turtle1/set_pen", SetPen)
clear_client = rospy.ServiceProxy("clear", Empty)
old_data = Ps4Data()
twist = Twist()
speed_level = 1
pen_on = True
change_pen_color = False


def send_pen_req(r, g, b, width, off):
    pen_req = SetPenRequest()
    pen_req.r = r
    pen_req.g = g
    pen_req.b = b
    pen_req.width = width
    pen_req.off = off
    rospy.wait_for_service("turtle1/set_pen")
    pen_resp = pen_client(pen_req)


def callback(data):
    global old_data, twist, speed_level, pen_on, change_pen_color, pen_color_r, pen_color_g, pen_color_b

    # Set Speed Level
    if (data.dpad_y != 0 and old_data.dpad_y == 0):
        speed_level += int(data.dpad_y)
        if (speed_level == 0):
            speed_level = 1
            rospy.loginfo("You cannot set speed level lower than 1!")
        elif (speed_level == 6):
            speed_level = 5
            rospy.loginfo("You cannot set speed level higher than 5!")
        else:
            rospy.loginfo("Speed level set to " + str(speed_level))

    # Set Pen Color
    if (data.triangle and not(old_data.triangle)):
        pen_color_r = 0
        pen_color_g = 255
        pen_color_b = 0
        send_pen_req(pen_color_r, pen_color_g, pen_color_b,
                     DEFAULT_PEN_WIDTH, not(pen_on))
    elif (data.circle and not(old_data.circle)):
        pen_color_r = 255
        pen_color_g = 0
        pen_color_b = 0
        send_pen_req(pen_color_r, pen_color_g, pen_color_b,
                     DEFAULT_PEN_WIDTH, not(pen_on))
    elif (data.cross and not(old_data.cross)):
        pen_color_r = 0
        pen_color_g = 0
        pen_color_b = 255
        send_pen_req(pen_color_r, pen_color_g, pen_color_b,
                     DEFAULT_PEN_WIDTH, not(pen_on))
    elif (data.square and not(old_data.square)):
        pen_color_r = 128
        pen_color_g = 0
        pen_color_b = 128
        send_pen_req(pen_color_r, pen_color_g, pen_color_b,
                     DEFAULT_PEN_WIDTH, not(pen_on))

    # Set Pen On/Off
    if (data.options and not(old_data.options)):
        pen_on = not(pen_on)
        send_pen_req(pen_color_r, pen_color_g, pen_color_b,
                     DEFAULT_PEN_WIDTH, not(pen_on))
        if (pen_on):
            rospy.loginfo("Pen is on")
        else:
            rospy.loginfo("Pen is off")

    # Clear Background
    if (data.ps and not(old_data.ps)):
        clear_req = EmptyRequest()
        rospy.wait_for_service("turtle1/set_pen")
        clear_resp = clear_client(clear_req)
        rospy.loginfo("Background is clear")

    # Toward / Backward
    twist.linear.x = data.hat_ly * speed_level

    # Clockwise / Anti-Clockwise
    twist.angular.z = data.hat_rx * speed_level

    pub.publish(twist)

    old_data = data


if __name__ == '__main__':
    rospy.init_node('ps4_controller')

    rospy.Subscriber(rospy.get_param(
        "~ps4_topic", "/input/ps4_data"), Ps4Data, callback)

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(1)

    rospy.spin()

    print("node ps4_controller terminated")
