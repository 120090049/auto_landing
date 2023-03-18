#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    bag_file = "base_link.bag"
    
    # Define the topic names for the link positions
    # topic_names = ['/boat/base_link', '/p450_monocular/base_link']
    # Read the rosbag file
    bag = rosbag.Bag(bag_file)

    # Create a 3D coordinate system
    fig = plt.figure()
    ax = fig.gca(projection='3d')


    topic_names = ['/boat/base_link']
    x_boat = []
    y_boat = []
    z_boat = []

    # Iterate over all messages in the rosbag file
    for topic, msg, t in bag.read_messages(topics=topic_names):
        # Parse the link name and position information from the message
        link_name = topic.split("/")[-1]
        position = msg.pose.position
        x_boat.append(position.x)
        y_boat.append(position.y)
        z_boat.append(position.z)

        # Save the position information to the corresponding list
   
        
    # Plot the 3D trajectories of the links
    for topic_name in topic_names:
        link_name = topic_name.split("/")[0]
        ax.plot(x_boat , y_boat, z_boat, label="USV trajectory")


    x_drone = []
    y_drone = []
    z_drone = []
    topic_names = ['/p450_monocular/base_link']

     # Iterate over all messages in the rosbag file
    for topic, msg, t in bag.read_messages(topics=topic_names):
        # Parse the link name and position information from the message
        link_name = topic.split("/")[-1]
        position = msg.pose.position
        x_drone.append(position.x-0.1)
        y_drone.append(position.y-0.1)
        z_drone.append(position.z-1.35)

        # Save the position information to the corresponding list
   
        

    # Plot the 3D trajectories of the links
    for topic_name in topic_names:
        link_name = topic_name.split("/")[0]
        ax.plot(x_drone , y_drone, z_drone, label="UAV trajectory")


    # Set the legend and axis labels
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.legend()
    # Show the plot
    plt.show()

    # Close the rosbag file
    bag.close()