#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author : bingo
# email  : bingobin.lw@gmail.com
# description  :  px4 controlled by keyboard
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import rospy

from gazebo_msgs.msg import LinkStates

import sys
import rospy
import matplotlib.pyplot as plt  

X = []
Y = []
Z = []

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
        rospy.init_node('pos_inspector', anonymous=True)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pos_callback)
        self.gazebo_link_states = LinkStates()
        self.x = 0
        self.y = 0
        self.z = 0

    def pos_callback(self, msg):
        self.gazebo_link_states = msg
        index = self.gazebo_link_states.name.index(
            'wamv::wamv/base_link')
        self.x = self.gazebo_link_states.pose[index].position.x
        self.y = self.gazebo_link_states.pose[index].position.y
        self.z = self.gazebo_link_states.pose[index].position.z

    def main_loop(self):
        i = 0
        r = rospy.Rate(15)
        while (1):
            self.key = getKey()
            # print(self.x, self.y, self.z)
            X.append(self.x)
            Y.append(self.y)
            Z.append(self.z)
            i += 1
            if (self.key == '\x03' or i > 500):
                break
            r.sleep()

        
        
            


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    # input = input("Do you want to record video or capture image? (0 for video and 1 for picture)")

    controller = Controller()
    controller.main_loop()

    print(X)
    print(Y)
    print(Z)
    ax = plt.axes(projection ="3d")  
    # Add x, and y gridlines for the figure  
    ax.grid(b = True, color ='blue',linestyle ='-.', linewidth = 0.5,alpha = 0.3)  
    # Creating the color map for the plot  
# Creating the 3D plot  
    sctt = ax.scatter3D(X, Y, Z) 


    plt.title("3D scatter plot")  
        # display the  plot  
    plt.show()  
    