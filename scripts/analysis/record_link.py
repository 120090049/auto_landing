#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
import rosbag
import sys

pwd = sys.path[0]
dir = pwd + '/base_link.bag'
bag = rosbag.Bag(dir, 'w')
model_states = ModelStates()

def callback(data):
    global model_states
    model_states = data

print("start recording at" + dir)
rospy.init_node('record_base_link')
sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

while not rospy.is_shutdown():
    for i in range(len(model_states.name)):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = model_states.name[i] + "/base_link"
        pose_stamped.pose = model_states.pose[i]
        bag.write('/' + model_states.name[i] + '/base_link', pose_stamped, pose_stamped.header.stamp)

bag.close()
