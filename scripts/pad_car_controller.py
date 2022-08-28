#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from turtle import st
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from gazebo_msgs.msg import ModelState
from turtlesim.msg import Pose
from gazebo_msgs.srv import *

speed = 1.2
turn = 1
Velocity = 1

msg = """
Control pad car!
---------------------------
Moving around:
        w    
   a    s    d

q/e : increase/decrease linear speeds by 10%
space key: force stop

press 0 to quit
"""

moveBindings = {
        'w':(1,0),
        'd':(1,-1),
        'a':(1,1),
        's':(-1,0),
           }

speedBindings={
        'q':(1.1,1),
        'e':(.9,1),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def move_a_square(side_length, velocity):
    square_linear = [1 for i in range(side_length)]
    square_angular = [0 for i in range(side_length)]
    square_linear +=  [0, 0, 0]
    square_angular += [0, 1, 0]
    freq = velocity
    print(freq)
    rate = rospy.Rate(freq)
    time = 0
    print("Keep pressing q for multiple times to quit")
    try:
        while(1):
            key = getKey()
            if (key != ''):
                break
            # 创建并发布twist消息
            else:
                twist = Twist()
                time = time + 1
                index = time % (side_length+3)
                twist.linear.x = velocity*square_linear[index]; 
                twist.linear.y = 0; 
                twist.linear.z = 0
                twist.angular.x = 0; 
                twist.angular.y = 0; 
                twist.angular.z = 2*square_angular[index]*velocity
                ori = (velocity*square_linear[index], 2*turn*square_angular[index])
                # print(ori)
                pub.publish(twist)
                rate.sleep()

    except:
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def move_a_circle(radius, velocity):
    linear = velocity
    angular = linear / float(radius)
    print(angular)
    rate = rospy.Rate(100)
    time = 0
    print("Keep pressing q for multiple times to quit")
    try:
        while(1):
            key = getKey()
            if (key == 'q'):
                break
            # 创建并发布twist消息
            else:
                twist = Twist()
                
                twist.linear.x = linear
                twist.linear.y = 0; 
                twist.linear.z = 0
                twist.angular.x = 0; 
                twist.angular.y = 0; 
                twist.angular.z = angular
                pub.publish(twist)
                rate.sleep()
    except:
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def vels(speed):
    return "currently:\tspeed %s" % (speed)

def set_position(x, y):
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'pad_car'
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = 0.01
    pub.publish(pose_msg)

def pose_subscriber():
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'pad_car'
    objstate = get_state_service(model)
    state = (objstate.pose.position.x, objstate.pose.position.y)
    print("position: (%.2f, %.2f)" % (state[0], state[1]))

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('pad_car_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    # status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    while(True):
        
        move_type =input("---------------------\nSelect a control mode, \n\
 1 for keyboard control,\n 2 for moving in a square,\n 3 for moving a circle,\n\
 4 for get position,\n 5 for set position,\n enter 0 for quit\n:") 
        move_type = int(move_type)
        try:
            if move_type == 0:
                print("quit")
                break

            elif move_type == 2:
                print("Moving in a square")
                side_length = 3
                move_a_square(side_length, Velocity)
            
            elif move_type == 3:
                print("Moving a circle")
                radius = 2
                move_a_circle(radius, Velocity)

            elif move_type == 4:
                pose_subscriber()

            elif move_type == 5:
                print("Kindly notice, if you use this command for the first time, you need enter the coordinates twice to move the car. This is a bug, but somehow I don't know how to solve it. :( \n ")

                x = input("Please input x: ")
                y = input("Input y: ")
                set_position(x, y)
                set_position(x, y)


            elif move_type == 1:
                print("start keyboard control")
                try:
                    print (msg)
                    print( vels(speed))
                    while(1):
                        key = getKey()
                        # 运动控制方向键（1：正方向，-1负方向）
                        if key in moveBindings.keys():
                            x = moveBindings[key][0]
                            th = moveBindings[key][1]
                            count = 0
                        # 速度修改键
                        elif key in speedBindings.keys():
                            speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                            # turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                            count = 0

                            print (vels(speed))
                            # if (status == 14):
                            #     print (msg)
                            # status = (status + 1) % 15
                        # 停止键
                        elif key == ' ' :
                            x = 0
                            th = 0
                            control_speed = 0
                            control_turn = 0
                        elif key == '0' :
                            break
                        else:
                            count = count + 1
                            if count > 4:
                                x = 0
                                th = 0
                            if (key == '\x03'):
                                break

                        # 目标速度=速度值*方向值
                        target_speed = speed * x
                        target_turn = turn * th

                        # 速度限位，防止速度增减过快
                        if target_speed > control_speed:
                            control_speed = min( target_speed, control_speed + 0.02 )
                        elif target_speed < control_speed:
                            control_speed = max( target_speed, control_speed - 0.02 )
                        else:
                            control_speed = target_speed

                        if target_turn > control_turn:
                            control_turn = min( target_turn, control_turn + 0.1 )
                        elif target_turn < control_turn:
                            control_turn = max( target_turn, control_turn - 0.1 )
                        else:
                            control_turn = target_turn

                        # 创建并发布twist消息
                        twist = Twist()
                        twist.linear.x = control_speed; 
                        twist.linear.y = 0; 
                        twist.linear.z = 0
                        twist.angular.x = 0; 
                        twist.angular.y = 0; 
                        twist.angular.z = control_turn
                        pub.publish(twist)

                except:
                    pass

                finally:
                    twist = Twist()
                    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                    pub.publish(twist)

                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            else:
                pass

        except rospy.ROSInterruptException:
            pass

    
