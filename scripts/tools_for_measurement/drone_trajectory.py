#!/usr/bin/env python
# -*- coding: utf-8 -*-
#author : bingo
#email  : bingobin.lw@gmail.com
#description  :  px4 controlled by keyboard 
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from nav_msgs.msg import Path
import time
import math




def __init__():
	print("Initialized")
def poseCallback(msg):
	# tragectory = Path()
	# tragectory = msg
    # rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f, z:%0.6f", msg.poses[0].pose.position.x, msg.poses[0].pose.position.y, msg.poses[0].pose.position.z)
	px = msg.poses[0].pose.position.x
	py = msg.poses[0].pose.position.y
	pz = msg.poses[0].pose.position.z
	point = (px, py ,pz)
	print(point) 
def pose_subscriber():
	rospy.init_node('drone_trajectory_observer' ,anonymous=True)
	rospy.Subscriber('/prometheus/drone_trajectory', Path, poseCallback)
	rospy.spin()



if __name__=="__main__":
	pose_subscriber()
