#!/usr/bin/env python
# -*- coding: utf-8 -*-
#author : bingo
#email  : bingobin.lw@gmail.com
#description  :  px4 controlled by keyboard 
from glob import glob
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
import time
import math

ORIENTATION = 0
SPEED = 0



msg = """
---------------------
motor_orientation:
z : left |  x : right
---------------------
blade_speed:
c : decrease |  v : add

CTRL-C to quit

"""



motor_orientation = rospy.Publisher('/wind_motor/joint1_p_controller/command', Float64, queue_size=1)
blade_speed = rospy.Publisher('/wind_motor/joint2_v_controller/command', Float64, queue_size=1)

def __init__():
	rospy.init_node('wind_motor_controller' ,anonymous=True)
	print("Initialized")

def orientation_msg(value):
	target_orientation = Float64
	global ORIENTATION
	ORIENTATION = ORIENTATION + value
	if (value != 0):
		print("Update: orientation = %d ,speed = %d"  % (ORIENTATION,SPEED))
	target_orientation = ORIENTATION
	return target_orientation

def speed_msg(value):
	global SPEED
	SPEED = SPEED + value
	if (value != 0):
		print("Update: orientation = %d ,speed = %d"  % (ORIENTATION,SPEED))
	target_speed = Float64
	target_speed = SPEED
	return target_speed


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def control():
	#orientation
	if key == 'z'or key == 'Z':
		value_ori = 1.0
	elif key == 'x'or key == 'X':
		value_ori = -1.0
	else :
		value_ori = 0.0

	#speed
	if key == 'c' or key == 'C':
		value_sp = 1.0
	elif key == 'v' or key == 'V':
		value_sp = -1.0
	else:
		value_sp = 0.0

	global ori_msg	
	global spe_msg
	ori_msg = orientation_msg(value_ori)
	spe_msg = speed_msg(value_sp)
	


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	print (msg)
	__init__()
	ori_msg = orientation_msg(0)
	spe_msg = speed_msg(0)
	while(1):
		key= getKey()
		control()
		motor_orientation.publish(ori_msg)
		blade_speed.publish(spe_msg)
		if (key == '\x03'):
			break
#		rospy.spin()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
