#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
START = 0
start_plot_time = 75
if __name__ == '__main__':
    bag_file = "base_link.bag"

    bag = rosbag.Bag(bag_file)

    topic_names = ['/boat/base_link']
    z_boat = []
    time_data_boat = []
    first_time = True
    # Iterate over all messages in the rosbag file
    for topic, msg, t in bag.read_messages(topics=topic_names):
        if (first_time): 
            START = msg.header.stamp.to_sec()
            first_time = False
        # Parse the link name and position information from the message
        if (msg.header.stamp.to_sec()-START > start_plot_time):
            position = msg.pose.position
            time_data_boat.append(msg.header.stamp.to_sec())
            z_boat.append(position.z)

    time_data_boat = np.array(time_data_boat) - time_data_boat[0]
        # Save the position information to the corresponding list
    z_boat = z_boat
    plt.plot(time_data_boat, z_boat, label='heave motion of the USV')



    topic_names = ['/p450_monocular/base_link']
    z_drone = []
    time_data_usv = []
     # Iterate over all messages in the rosbag file
    for topic, msg, t in bag.read_messages(topics=topic_names):
        # Parse the link name and position information from the message
        if (msg.header.stamp.to_sec()-START > start_plot_time):
            position = msg.pose.position
            z_drone.append(position.z-1.35)
            time_data_usv.append(msg.header.stamp.to_sec())
        # Save the position information to the corresponding list
   
        
    time_data_usv = np.array(time_data_usv) - time_data_usv[0]
    plt.plot(time_data_usv, z_drone, label='height of the UAV')


    plt.xlabel('Time (s)')
    plt.ylabel('Z-axis coordinate')
    # plt.title('Link Z-axis coordinate over time')
    
    plt.legend()
    # Show the plot
    plt.show()

    # Close the rosbag file
    bag.close()