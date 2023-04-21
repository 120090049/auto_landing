# Project Overview


**auto_landing** is a ros package for UAV tracking and landing job in a simulated ocean environment. I hope this project can provide simulation assistance for the research of collaborative mission of unmanned aerial vehicle (**UAV**)  and unmanned surface vehicle (**USV**) in our college's lab (CUHKSZ AIRS).

Currently, I have realized the landing of UAV on a shaking USV platform. The results can be seen from this [video](https://drive.google.com/file/d/1Du2hd4LyCqpviYpElHeEIIWHoj1bF2gY/view?usp=sharing). The whole work has been summarized in to a paper.

![0system.jpg](https://github.com/120090049/auto_landing/blob/master/pic/0system.jpg)


# Installation

## Environment prerequisite

Our UAV is based on Pixhawk V4 flight control architecture and utilizes **PX4 Drone Autopilot** (**PX4)** flight control firmware. High-level UAV control is developed under the Robot Operating System (ROS). **MAVROS** provides a communication interface between the ROS nodes and PX4, through which ROS nodes can send commands such as waypoint navigation and speed instructions. 

The simulation environment is in Gazebo. It is developed based on **Virtual RobotX (VRX)**, an open-source simulation environment designed for marine robot experiments.

Please refer to [environment_setup](https://github.com/120090049/auto_landing/wiki) to setup PX4, mavros and VRX.

## Install auto_landing package

After then, cd to your work space.

```Python
git clone https://github.com/120090049/auto_landing.git  
catkin_make  
source /home/username/catkin_ws/devel/setup.bash   
```

or add it to the ~/.bashrc

## Some configurations before running


### Configuration for remote controller

In my code, **px4_controller.py** and **px4_controller_withcamera.py** work as a remote controller for UAV, we need to configure some control channels in the **QGroundControl**. 

- QGC installation
Please refer to [QGC](https://blog.csdn.net/Legendyyy/article/details/127177714?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166971163916800180668420%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166971163916800180668420&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-127177714-null-null.142%5Ev67%5Econtrol,201%5Ev3%5Eadd_ask,213%5Ev2%5Et3_control1&utm_term=ubuntu18.04%E5%AE%89%E8%A3%85qgc%E5%9C%B0%E9%9D%A2%E7%AB%99&spm=1018.2226.3001.4187)

- Setting channels for the remote controller, (please make sure your UAV is launched in the gazebo)
enter into vehicle setup => parameters => search and set as following


![image.png](https://github.com/120090049/auto_landing/blob/master/pic/image.png)


![image.png](https://github.com/120090049/auto_landing/blob/master/pic/image%201.png)

```Plain Text
 COM_FLTMODE1 = Position
 RC_CHAN_CNT = 8
 RC_MAP_FLTMODE = Channel 5
 RC_MAP_PITCH = Channel 3
 RC_MAP_ROLL= Channel 1
 RC_MAP_THROTTLE = Channel 2
 RC_MAP_YAW = Channel 4
 channel_1: roll; channel_2: throttle; channel_3: pitch; channel_4: yaw; channel_5: flight mode;
```

### Config PID parameters

set up simulation world

```Plain Text
roslaunch auto_landing launch_my_world.launch 
```

run remote controller

```Plain Text
rosrun auto_landing px4_controller.py
```

Take off the UAV, you can find the UAV is shaking violently. So open the QGC, choose vehicle setup => PID tuning => rate_controller. Disable "autotune", then, tune the UAV in roll/pitch/yaw channel.

![image.png](https://github.com/120090049/auto_landing/blob/master/pic/image%202.png)

# Function presentation


After setting all of these, we can happily run the code!

## QR-code tracking

- launch the environment and the detection & control module

```Plain Text
roslaunch auto_landing launch_my_world.launch
rosrun auto_landing px4_controller.py
rosrun auto_landing landpad_det
roslaunch auto_landing launch_mission.launch
```

- Then, take off our UAV, move the UAV roughly above the car so that the car can enter the field of view of UAV down-looking camera. Then we can enter in the offboard mode by entering 4 in px4_controller.py

- Now, UAV starts tracking. We can move the car with following command:

```Plain Text
rosrun auto_landing pad_car_controller.py
```

- Demo output (tracing a circle)
![](https://github.com/120090049/auto_landing/blob/master/pic/circlegif.gif)

## Landing in ocean environment

To see the detailed implementation, you may take a look at my paper.
The structure of the launch file can be seen in [launch_my_vrxworld.launch](https://gitmind.cn/app/docs/ml4da866).
The guideline of changing the stuff in the VRX like wind and waves can be be seen in the [link](https://github.com/osrf/vrx/wiki/vrx_api_tutorials).

- How to run

```Plain Text
cd /home/username/catkin_ws/src/auto_landing
./ocean_experiment.sh
```


In the **./ocean_experiment.sh**, we will 

- launch the simulation world, PX4, mavros 

- **px4_controller_withcamera.py** for UAV remote control (switch UAV to offboard mode after take off)

- **landpad_det** for down-looking camera, 

- **detect_with_front_camera.py** for front-looing camera

- **launch_ocean_mission.launch** for control loop

- **record_link.py** to record trajectory for later analysis

- Demo output
![](https://github.com/120090049/auto_landing/blob/master/pic/ocean_landing.gif)


# A Future plan
My academic advisor Prof. Qian has suggested me to study the usage of UVA for the inspection of offshore turbines, which is a development trend of new energy in the near future. I plan to work on this direction in the future, say, when I am a senior.  
Here is a rough environment I created.   
![](https://github.com/120090049/auto_landing/blob/master/pic/windturbines.jpg)
The turbine is designed and exported from solidworks.  
To launch the environment and control the turbine, you may run
```
roslaunch auto_landing launch_my_wind_motor_world.launch
rosrun auto_landing wind_motor_controller.py
```

