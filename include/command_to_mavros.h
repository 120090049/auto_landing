/***************************************************************************************************************************
* command_to_mavros.h
*
* Author: Qyp
*
* Update Time: 2020.08.12
*
* Introduction:  Drone control command send class using Mavros package
*         1. Ref to the Mavros plugins (setpoint_raw, loca_position, imu and etc..)
*         2. Ref to the Offboard Flight task in PX4 code: https://github.com/PX4/Firmware/blob/master/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp
*         3. Ref to the Mavlink module in PX4 code: https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp
*         4. Ref to the position control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control
*         5. Ref to the attitude control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control
*         6. 还需要考虑复合形式的输出情况
* 主要功能：
*    本库函数主要用于连接prometheus_control与mavros两个功能包。简单来讲，本代码提供飞控的状态量，用于控制或者监控，本代码接受控制指令，并将其发送至飞控。
* 1、发布prometheus_control功能包生成的控制量至mavros功能包，可发送期望位置、速度、角度、角速度、底层控制等。
* 2、订阅mavros功能包发布的飞控状态量（包括PX4中的期望位置、速度、角度、角速度、底层控制），用于检查飞控是否正确接收机载电脑的指令
* 3、解锁上锁、修改模式两个服务。

***************************************************************************************************************************/
#ifndef COMMAND_TO_MAVROS_H
#define COMMAND_TO_MAVROS_H

#include <ros/ros.h>
#include <math_utils.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/MountControl.h>
#include <sensor_msgs/Imu.h>
#include <prometheus_msgs/DroneState.h>
#include <bitset>
#include <prometheus_msgs/AttitudeReference.h>
#include <prometheus_msgs/DroneState.h>
using namespace std;

class command_to_mavros
{
public:
    //constructed function 1
    command_to_mavros(void) : command_nh("~")
    {
        command_nh.param<string>("uav_name", uav_name, "/uav0");

        if (uav_name == "/uav0")
        {
            uav_name = "";
        }

        pos_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        vel_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        accel_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        q_fcu_target = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        euler_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        rates_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        Thrust_target = 0.0;

        // 【订阅】无人机期望位置/速度/加速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为POSITION_TARGET_LOCAL_NED, 对应的飞控中的uORB消息为vehicle_local_position_setpoint.msg
        position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb, this);

        // 【订阅】无人机期望角度/角速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为ATTITUDE_TARGET (#83), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg
        attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb, this);

        // 【订阅】无人机底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力
        //  本话题来自飞控(通过Mavros功能包 /plugins/actuator_control.cpp读取), 对应Mavlink消息为ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_target_sub = command_nh.subscribe<mavros_msgs::ActuatorControl>(uav_name + "/mavros/target_actuator_control", 10, &command_to_mavros::actuator_target_cb, this);

        // 【发布】位置/速度/加速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
        setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);

        // 【发布】角度/角速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
        setpoint_raw_attitude_pub = command_nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 10);

        // 【发布】底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力 注意 这里是NED系的！！
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/actuator_control.cpp发送), 对应Mavlink消息为SET_ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>(uav_name + "/mavros/actuator_control", 10);

        //　本话题要发送至飞控(通过Mavros_extra功能包 /plugins/mount_control.cpp发送)
        mount_control_pub = command_nh.advertise<mavros_msgs::MountControl>(uav_name + "/mavros/mount_control/command", 1);

        // 【服务】解锁/上锁
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");

        // 【服务】修改系统模式
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    }

    //constructed function 2
    command_to_mavros(string Uav_name) : command_nh("")
    {
        uav_name = Uav_name;

        pos_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        vel_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        accel_drone_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        q_fcu_target = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        euler_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        rates_fcu_target = Eigen::Vector3d(0.0, 0.0, 0.0);
        Thrust_target = 0.0;

        // 【订阅】无人机期望位置/速度/加速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为POSITION_TARGET_LOCAL_NED, 对应的飞控中的uORB消息为vehicle_local_position_setpoint.msg
        position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb, this);

        // 【订阅】无人机期望角度/角速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为ATTITUDE_TARGET (#83), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg
        attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb, this);

        // 【订阅】无人机底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力
        //  本话题来自飞控(通过Mavros功能包 /plugins/actuator_control.cpp读取), 对应Mavlink消息为ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_target_sub = command_nh.subscribe<mavros_msgs::ActuatorControl>(uav_name + "/mavros/target_actuator_control", 10, &command_to_mavros::actuator_target_cb, this);

        // 【发布】位置/速度/加速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
        setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);

        // 【发布】角度/角速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
        setpoint_raw_attitude_pub = command_nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 10);

        // 【发布】底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力 注意 这里是NED系的！！
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/actuator_control.cpp发送), 对应Mavlink消息为SET_ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>(uav_name + "/mavros/actuator_control", 10);

        //　本话题要发送至飞控(通过Mavros_extra功能包 /plugins/mount_control.cpp发送)
        mount_control_pub = command_nh.advertise<mavros_msgs::MountControl>(uav_name + "/mavros/mount_control/command", 1);

        // 【服务】解锁/上锁
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");

        // 【服务】修改系统模式
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    }

    string uav_name;
    //Target pos of the drone [from fcu]
    Eigen::Vector3d pos_drone_fcu_target;
    //Target vel of the drone [from fcu]
    Eigen::Vector3d vel_drone_fcu_target;
    //Target accel of the drone [from fcu]
    Eigen::Vector3d accel_drone_fcu_target;
    //Target att of the drone [from fcu]
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d euler_fcu_target;
    Eigen::Vector3d rates_fcu_target;
    //Target thrust of the drone [from fcu]
    float Thrust_target;
    mavros_msgs::ActuatorControl actuator_target;

    //变量声明 - 服务
    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient arming_client;

    ros::ServiceClient set_mode_client;

    //Idle. Do nothing.
    void idle();

    //px4 takeoff
    void takeoff();

    //px4 loiter
    void loiter();

    // px4 land
    void land();

    //发送位置期望值至飞控（输入：期望xyz,期望yaw）
    void send_pos_setpoint(const Eigen::Vector3d &pos_sp, float yaw_sp);

    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    void send_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp);

    //发送速度期望值至飞控（输入：期望vxvy z,期望yaw）
    void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &state_sp, float yaw_sp);
    void send_vel_xy_pos_z_setpoint_yawrate(const Eigen::Vector3d &state_sp, float yaw_rate_sp);

    //发送速度期望值至飞控（机体系）（输入：期望vxvyvz,期望yaw）
    void send_vel_setpoint_body(const Eigen::Vector3d &vel_sp, float yaw_sp);

    void send_vel_setpoint_yaw_rate(const Eigen::Vector3d &vel_sp, float yaw_rate_sp);

    // 发送位置+速度期望值至飞控（机体系）（输入：期望xyz + vxvyvz,期望yaw）
    void send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp);

    //发送加速度期望值至飞控（输入：期望axayaz,期望yaw）
    void send_acc_xyz_setpoint(const Eigen::Vector3d &accel_sp, float yaw_sp);

    //发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
    //这是px4_pos_controller.cpp中目前使用的控制方式
    void send_attitude_setpoint(const prometheus_msgs::AttitudeReference &_AttitudeReference);

    //发送角度期望值至飞控（输入：期望角速度,期望推力）
    void send_attitude_rate_setpoint(const Eigen::Vector3d &attitude_rate_sp, float thrust_sp);

    void send_attitude_setpoint_yawrate(const prometheus_msgs::AttitudeReference &_AttitudeReference, float yaw_rate_sp);

    //发送底层至飞控（输入：MxMyMz,期望推力）[Not recommanded. Because the high delay between the onboard computer and Pixhawk]
    void send_actuator_setpoint(const Eigen::Vector4d &actuator_sp);

    //发送云台控制指令
    void send_mount_control_command(const Eigen::Vector3d &gimbal_att_sp);

private:
    ros::NodeHandle command_nh;

    ros::Subscriber position_target_sub;
    ros::Subscriber attitude_target_sub;
    ros::Subscriber actuator_target_sub;

    ros::Publisher setpoint_raw_local_pub;
    ros::Publisher setpoint_raw_attitude_pub;
    ros::Publisher actuator_setpoint_pub;
    ros::Publisher mount_control_pub;

    void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

        vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

        accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
    }

    void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
    {
        q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        //Transform the Quaternion to euler Angles
        euler_fcu_target = quaternion_to_euler(q_fcu_target);

        rates_fcu_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);

        Thrust_target = msg->thrust;
    }

    void actuator_target_cb(const mavros_msgs::ActuatorControl::ConstPtr &msg)
    {
        actuator_target = *msg;
    }
};

void command_to_mavros::send_mount_control_command(const Eigen::Vector3d &gimbal_att_sp)
{
    mavros_msgs::MountControl mount_setpoint;
    //
    mount_setpoint.mode = 2;
    mount_setpoint.pitch = gimbal_att_sp[0]; // Gimbal Pitch
    mount_setpoint.roll = gimbal_att_sp[1];  // Gimbal  Yaw
    mount_setpoint.yaw = gimbal_att_sp[2];   // Gimbal  Yaw

    mount_control_pub.publish(mount_setpoint);
}

void command_to_mavros::takeoff()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    //需要在QGC设置起飞高度
    pos_setpoint.type_mask = 0x1000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::land()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    //需要在QGC设置降落速度
    pos_setpoint.type_mask = 0x2000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::loiter()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x3000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::idle()
{
    // NOTE 新飞控无此模式，切入会一直答应警告
    // mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    // pos_setpoint.type_mask = 0x4000;

    // setpoint_raw_local_pub.publish(pos_setpoint);
    Eigen::Vector3d pos_sp{0, 0, 0};
    this->send_pos_setpoint(pos_sp, 0);
}

//发送位置期望值至飞控（输入：期望xyz,期望yaw）
void command_to_mavros::send_pos_setpoint(const Eigen::Vector3d &pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000; // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Pos_target [X Y Z] : " << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

//发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
void command_to_mavros::send_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

//发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
void command_to_mavros::send_vel_setpoint_yaw_rate(const Eigen::Vector3d &vel_sp, float yaw_rate_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b010111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw_rate = yaw_rate_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

//发送速度期望值至飞控（机体系）（输入：期望vxvyvz,期望yaw）
void command_to_mavros::send_vel_setpoint_body(const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_BODY_NED = 8
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.position.x = vel_sp[0];
    pos_setpoint.position.y = vel_sp[1];
    pos_setpoint.position.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

void command_to_mavros::send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &state_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011; // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = state_sp[0];
    pos_setpoint.velocity.y = state_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = state_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}
void command_to_mavros::send_vel_xy_pos_z_setpoint_yawrate(const Eigen::Vector3d &state_sp, float yaw_rate_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b010111000011; // 100 111 000 011  vx vy vz z + yawrate

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = state_sp[0];
    pos_setpoint.velocity.y = state_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = state_sp[2];

    pos_setpoint.yaw_rate = yaw_rate_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Vel_target [X Y Z] : " << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

void command_to_mavros::send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 2. position setpoint + velocity setpoint (velocity used as feedforward)
    // 控制方法请见 PositionControl.cpp
    pos_setpoint.type_mask = 0b100111000000; // 100 111 000 000  vx vy　vz x y z+ yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

//发送加速度期望值至飞控（输入：期望axayaz,期望yaw）
void command_to_mavros::send_acc_xyz_setpoint(const Eigen::Vector3d &accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100000111111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Acc_target [X Y Z] : " << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;
    // cout << "Yaw_target : " << euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
}

//发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
void command_to_mavros::send_attitude_setpoint(const prometheus_msgs::AttitudeReference &_AttitudeReference)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    att_setpoint.orientation.x = _AttitudeReference.desired_att_q.x;
    att_setpoint.orientation.y = _AttitudeReference.desired_att_q.y;
    att_setpoint.orientation.z = _AttitudeReference.desired_att_q.z;
    att_setpoint.orientation.w = _AttitudeReference.desired_att_q.w;

    att_setpoint.thrust = _AttitudeReference.desired_throttle;

    setpoint_raw_attitude_pub.publish(att_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg] "<<euler_fcu_target[1] * 180/M_PI << " [deg] "<< euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
    // cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;
}

//发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
void command_to_mavros::send_attitude_setpoint_yawrate(const prometheus_msgs::AttitudeReference &_AttitudeReference, float yaw_rate_sp)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000011;

    att_setpoint.orientation.x = _AttitudeReference.desired_att_q.x;
    att_setpoint.orientation.y = _AttitudeReference.desired_att_q.y;
    att_setpoint.orientation.z = _AttitudeReference.desired_att_q.z;
    att_setpoint.orientation.w = _AttitudeReference.desired_att_q.w;

    att_setpoint.thrust = _AttitudeReference.desired_throttle;

    att_setpoint.body_rate.x = 0.0;
    att_setpoint.body_rate.y = 0.0;
    att_setpoint.body_rate.z = yaw_rate_sp;

    setpoint_raw_attitude_pub.publish(att_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg] "<<euler_fcu_target[1] * 180/M_PI << " [deg] "<< euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
    // cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;
}

//发送角度期望值至飞控（输入：期望角速度,期望推力）
void command_to_mavros::send_attitude_rate_setpoint(const Eigen::Vector3d &attitude_rate_sp, float thrust_sp)
{
    mavros_msgs::AttitudeTarget att_setpoint;

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
    // msg.type_mask = 128;  // Ignore orientation messages
    att_setpoint.type_mask = 0b10000000;

    att_setpoint.body_rate.x = attitude_rate_sp[0];
    att_setpoint.body_rate.y = attitude_rate_sp[1];
    att_setpoint.body_rate.z = attitude_rate_sp[2];

    att_setpoint.thrust = thrust_sp;

    setpoint_raw_attitude_pub.publish(att_setpoint);

    // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // cout << "Att_rate_target [R P Y] : " << rates_fcu_target[0] * 180/M_PI <<" [deg/s] "<<rates_fcu_target[1] * 180/M_PI << " [deg/s] "<< rates_fcu_target[2] * 180/M_PI<<" [deg/s] "<<endl;
    // cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;
}

//发送底层至飞控（输入：MxMyMz,期望推力）
void command_to_mavros::send_actuator_setpoint(const Eigen::Vector4d &actuator_sp)
{
    mavros_msgs::ActuatorControl actuator_setpoint;

    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = actuator_sp(0);
    actuator_setpoint.controls[1] = actuator_sp(1);
    actuator_setpoint.controls[2] = actuator_sp(2);
    actuator_setpoint.controls[3] = actuator_sp(3);
    actuator_setpoint.controls[4] = 0.0;
    actuator_setpoint.controls[5] = 0.0;
    actuator_setpoint.controls[6] = 0.0;
    actuator_setpoint.controls[7] = 0.0;

    actuator_setpoint_pub.publish(actuator_setpoint);

    // // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // //ned to enu
    // cout << "actuator_target [0 1 2 3] : " << actuator_target.controls[0] << " [ ] "<< -actuator_target.controls[1] <<" [ ] "<<-actuator_target.controls[2]<<" [ ] "<<actuator_target.controls[3] <<" [ ] "<<endl;

    // cout << "actuator_target [4 5 6 7] : " << actuator_target.controls[4] << " [ ] "<< actuator_target.controls[5] <<" [ ] "<<actuator_target.controls[6]<<" [ ] "<<actuator_target.controls[7] <<" [ ] "<<endl;
}

#endif
