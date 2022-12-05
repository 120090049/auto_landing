# auto_landing
ros package for auto_landing

## Installation
Please refer to [environment_setup](https://github.com/120090049/auto_landing/wiki) to setup px4 and mavros.   
After then, cd to your work space.  
```
git clone https://github.com/120090049/auto_landing.git  
catkin_make  
source /home/username/catkin_ws/devel/setup.bash   
```   
or add it to the ~/.bashrc

## Initialize the aircraft with QGC
We need to set some configuration for the UAV in the QGroundControl. Otherwise, my code cannot work.  
* QGC installation  
Please refer to [QGC](https://blog.csdn.net/Legendyyy/article/details/127177714?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166971163916800180668420%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166971163916800180668420&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-127177714-null-null.142^v67^control,201^v3^add_ask,213^v2^t3_control1&utm_term=ubuntu18.04%E5%AE%89%E8%A3%85qgc%E5%9C%B0%E9%9D%A2%E7%AB%99&spm=1018.2226.3001.4187)

* QGC usage  
* * Set the channel of the remote controller   
enter into vehicle setup -> parameters -> search and set as following
```
    # COM_FLTMODE1 = Position
    # RC_CHAN_CNT = 8
    # RC_MAP_FLTMODE = Channel 5
    # RC_MAP_PITCH = Channel 3
    # RC_MAP_ROLL= Channel 1
    # RC_MAP_THROTTLE = Channel 2
    # RC_MAP_YAW = Channel 4
    # channel_1: roll; channel_2: throttle; channel_3: pitch; channel_4: yaw; channel_5: flight mode;
```

* * config PID parameters
set up simulation world  
`roslaunch auto_landing launch_my_world.launch `  
run remote controller  
`rosrun auto_landing px4_controller.py`  
Take off the UAV, you can find the UAV is shaking violently. So open the QGC, choose vehicle setup -> PID tuning rate_controller   
First,disable "autotune". Then, tune the UAV in roll/pitch/yaw channel.

## Demo of my ROS package
After setting all of these, we can happily run the code!
### QR-code tracking (which is the first stage of my plan)
* launch the environment
```
roslaunch auto_landing launch_my_world.launch
rosrun auto_landing px4_controller.py
rosrun auto_landing landpad_det
roslaunch auto_landing launch_mission.launch
```
* Then, take off our UAV, move the UAV roughly above the car so that the car can enter the field of vision of UAV camera. Then we can enter in the offboard mode by entering 4 in px4_controller.py  

* Now, UAV starts tracking. We can move the car with following command:  
```
rosrun auto_landing pad_car_controller.py
```
* Demo output
* * tracing a circle
`![tracing a cycle](https://github.com/120090049/auto_landing/blob/master/pic/circlegif.gif)`


vrx
https://github.com/osrf/vrx/wiki/vrx_api_tutorials
roslaunch auto_landing launch_my_vrxworld.launch
roslaunch vrx_gazebo usv_keydrive.launch
