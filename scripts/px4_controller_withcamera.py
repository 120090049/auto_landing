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

import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msg = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
If you use this script, please read auto_landing/scripts/README.md first.
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%
action_control:
%%%%%%%%%%%%%%%%%%%%%%%
        e    
   s    d    f
e and d : pitch control
s and f : roll control
---------------------------
        i    
   j    k    l
i and k : throttle control
j and l : yaw control
---------------------------
g : Speed reduction
h : Speed increase
---------------------------
%%%%%%%%%%%%%%%%%%%%%%%
command_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
0 : ARM
1 : TAKEOFF
2 : OFFBOARDS
3 : LAND
4 : POSCTL
5 : ATTITCTL
---------------------------

CTRL-C to quit

"""

# COM_FLTMODE1 = Position
# RC_CHAN_CNT = 8
# RC_MAP_FLTMODE = Channel 5g
# RC_MAP_PITCH = Channel 3
# RC_MAP_ROLL= Channel 1
# RC_MAP_THROTTLE = Channel 2
# RC_MAP_YAW = Channel 4
# channel_1: roll; channel_2: throttle; channel_3: pitch; channel_4: yaw; channel_5: flight mode;
# com_flightmode1 = position #个人推测： chane



def RCInOverride(channel0,channel1,channel2,channel3):
	target_RC_yaw = OverrideRCIn()
	target_RC_yaw.channels[0] = channel0
	target_RC_yaw.channels[1] = channel1
	target_RC_yaw.channels[2] = channel2
	target_RC_yaw.channels[3] = channel3
	target_RC_yaw.channels[4] = 1100
	return target_RC_yaw

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

class Controller: 
	def __init__(self):
		rospy.init_node('px4_controller' ,anonymous=True)
		rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
		
		rospy.Subscriber("/prometheus/sensor/monocular_down/image_raw", Image, self.downlooking_camera_callback)
		rospy.Subscriber("/prometheus/sensor/monocular_front/image_raw", Image, self.frontlooking_camera_callback)
		self.bridge = CvBridge()

		self.throttle_middle = 1750
		self.speed_control = 1750
		self.cur_target_rc_yaw = RCInOverride(1500,2000,1000,1500)
		self.mavros_state = State()
		self.armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.local_target_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=100)

		self.swithch_front = 0
		self.swithch_down = 0
	
		str = os.getcwd()
		self.vw_down = cv2.VideoWriter(str+'/videos/down.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280,720))
		self.vw_front = cv2.VideoWriter(str+'/videos/front.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280,720))
		print("Initialized")

	def downlooking_camera_callback(self, data): #e
		# 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
		if (self.swithch_down):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)
			if (self.swithch_down % 2 == 0):
				self.vw_down.write(cv_image)
				cv2.putText(cv_image,"RECORDING",(1050, 40),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
			# 显示Opencv格式的图像
			cv2.imshow("Image down", cv_image)
			cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
       
		return

	def frontlooking_camera_callback(self, data):
		if (self.swithch_front):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)
			if (self.swithch_front % 2 == 0):
				self.vw_front.write(cv_image)
				cv2.putText(cv_image,"RECORDING",(1050, 40),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
				
			cv2.imshow("Image front", cv_image)
			cv2.waitKey(3)

		return

	def mavros_state_callback(self, msg):
		self.mavros_state = msg

	def command_control(self):
		if self.key == 'e'or self.key == 'E':
			self.swithch_down += 1
		if self.key == 'q'or self.key == 'Q':
			self.swithch_front += 1
		if self.key == '0':
			if self.armServer(True) :
				print("Vehicle arming succeed!")
			else:
				print("Vehicle arming failed!")
		elif self.key == '1':
			if self.setModeServer(custom_mode='AUTO.TAKEOFF'):
				print("Vehicle takeoff succeed!")
			else:
				print("Vehicle takeoff failed!")
		elif self.key == '2':
			if self.setModeServer(custom_mode='OFFBOARD'):
				print("Vehicle offboard succeed!")
			else:
				print("Vehicle offboard failed!")
		elif self.key == '3':
			if self.setModeServer(custom_mode='AUTO.LAND'):
				print("Vehicle land succeed!")
			else:
				print("Vehicle land failed!")
		elif self.key == '4':
			if self.setModeServer(custom_mode='POSCTL'):
				print("Vehicle posControl succeed!")
			else:
				print("Vehicle posControl failed!")
		elif self.key == '5':
			if self.setModeServer(custom_mode='STABILIZED'):
				print("Vehicle stabilized succeed!")
			else:
				print("Vehicle stabilized failed!")

	def action_control(self):
		#throttle
		if self.mavros_state.mode == 'POSCTL':
			if self.key == 'i'or self.key == 'I':
				channel1 = self.throttle_middle + 200
			elif self.key == 'k'or self.key == 'K':
				channel1 = self.throttle_middle - 200
			else :
				channel1 = self.throttle_middle
		elif self.mavros_state.mode == 'STABILIZED':
			if self.key == 'i'or self.key == 'I':
				channel1 = 1400
			elif self.key == 'k'or self.key == 'K':
				channel1 = 1200
			else :
				channel1 = 1200
		else:
			channel1 = 1000
		#pitch
		if self.key == 'w' or self.key == 'W':
			channel2 = self.speed_control
		elif self.key == 's' or self.key == 'S':
			channel2 = 3000-self.speed_control
		else:
			channel2 = 1500
		#roll
		if self.key == 'a' or self.key == 'A':
			channel0 = 3000-self.speed_control
		elif self.key == 'd' or self.key == 'D':
			channel0 = self.speed_control
		else:
			channel0 = 1500
		#yaw
		if self.key == 'j' or self.key == 'J':
			channel3 = 1300
		elif self.key == 'l' or self.key == 'L':
			channel3 = 1700
		else:
			channel3 = 1500

		self.cur_target_rc_yaw = RCInOverride(channel0,channel1,channel2,channel3)
		if self.key == 'h' or self.key == 'H':
			self.speed_control = self.speed_control + 10
			print('Current control speed :',self.speed_control)
		elif self.key == 'g' or self.key == 'G':
			self.speed_control = self.speed_control - 10
			print('Current control speed :',self.speed_control)

	def main_loop(self):
		while(1):
			self.key= getKey()
			self.command_control()
			self.action_control()
			self.local_target_pub.publish(self.cur_target_rc_yaw)
			if (controller.key == '\x03'):
				break


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	print (msg)
	controller = Controller()
	controller.main_loop()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
