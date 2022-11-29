# auto_landing
ros package for auto autolanding
# auto_landing
ros package for auto autolanding

vscode include directory 配置
Include path
gcc -v -E -x c++ -

${workspaceFolder}/**
/usr/include/c++/7
/usr/include/x86_64-linux-gnu/c++/7
/usr/include/c++/7/backward
/usr/lib/gcc/x86_64-linux-gnu/7/include
/usr/local/include
/usr/lib/gcc/x86_64-linux-gnu/7/include-fixed
/usr/include/x86_64-linux-gnu
/usr/include
/opt/ros/melodic/include
/home/clp/Prometheus/devel/include

set up vrx
略
use vrx
https://github.com/osrf/vrx/wiki/vrx_api_tutorials


source /home/clp/catkin_ws/devel/setup.bash

## 使用飞机前先进行gpc设置
qgc 下载
参考
https://blog.csdn.net/Legendyyy/article/details/127177714?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166971163916800180668420%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166971163916800180668420&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-127177714-null-null.142^v67^control,201^v3^add_ask,213^v2^t3_control1&utm_term=ubuntu18.04%E5%AE%89%E8%A3%85qgc%E5%9C%B0%E9%9D%A2%E7%AB%99&spm=1018.2226.3001.4187

qgc 使用
仿真中需要调的参数
1） 无人机的遥控器通道
    vehicle setup -> parameters -> 直接搜索
    # COM_FLTMODE1 = Position
    # RC_CHAN_CNT = 8
    # RC_MAP_FLTMODE = Channel 5
    # RC_MAP_PITCH = Channel 3
    # RC_MAP_ROLL= Channel 1
    # RC_MAP_THROTTLE = Channel 2
    # RC_MAP_YAW = Channel 4
    # channel_1: roll; channel_2: throttle; channel_3: pitch; channel_4: yaw; channel_5: flight mode;
    # com_flightmode1 = position #个人推测： chane

2） 无人机位置环pid控制
    
    启动仿真环境    roslaunch auto_landing launch_my_world.launch 
    启动键盘遥控器  rosrun auto_landing px4_controller.py
        按0解锁 按1起飞 按4进入定点模式 可以看到飞机晃得很厉害 需要进行pid调节
    vehicle setup -> PID tuning rate_controller 
    首先autotune disabled 然后在 roll/pitch/yaw进行调节

3） 使用qgc进行行路规划可参考 以下博客中的 六
https://blog.csdn.net/qq_38768959/article/details/123157315?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166971148816800182197350%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166971148816800182197350&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-123157315-null-null.142^v67^control,201^v3^add_ask,213^v2^t3_control1&utm_term=qgc&spm=1018.2226.3001.4187


运行二维码追踪程序
roslaunch auto_landing launch_my_world.launch
rosrun auto_landing px4_controller.py
rosrun auto_landing landpad_det
roslaunch auto_landing launch_mission.launch
这时候要把无人机切换到offboard模式下 进到px4_controller.py的窗口按4

rosrun auto_landing pad_car_controller.py
可以让小车进行移动

vrx
https://github.com/osrf/vrx/wiki/vrx_api_tutorials
roslaunch auto_landing launch_my_vrxworld.launch
roslaunch vrx_gazebo usv_keydrive.launch
