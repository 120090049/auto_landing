/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.31
* Description: Autonomous circular trajectory in offboard mode
***************************************************************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "mission_utils.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;


//parameters for basic control (before the uav starts tracking)
float desire_z = 1; //期望高度
float MoveTimeCnt = 0;
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
Eigen::Vector3d vel_target;//offboard模式下，发送给飞控的期望值

mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;

// ros::ServiceClient set_mode_client;

//parameters for detection
float camera_offset[3];
prometheus_msgs::DroneState _DroneState;    // 无人机姿态
Eigen::Matrix3f R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵
Detection_result landpad_det;               // 检测结果
float kp_land[3];         //控制参数 - 比例参数

//主状态
enum
{
    WAITING,		//等待offboard模式
	PREPARE,		//起飞特定高度
	FLY,			//开始飞（检测开始）

}FlyState = WAITING;//初始状态WAITING

//识别状态
enum 
{
    WAITING_RESULT,
    TRACKING,
    LOST,
    // LANDING,
}exec_state = WAITING_RESULT;

///////////////////////////////////////////////////////////////////////////////////////////////

// function before tracking
//接收来自飞控的当前飞机位置
Eigen::Vector3d pos_drone;                     
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}

//接收来自飞控的当前飞机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//发送位置期望值至飞控（输入：期望位置xyz(bool=1)/期望速度xyz(bool=0),期望yaw,bool）
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp, bool posit)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
	if (posit){
		cout << pos_sp << endl;
		pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

		pos_setpoint.coordinate_frame = 1;

		pos_setpoint.position.x = pos_sp[0];
		pos_setpoint.position.y = pos_sp[1];
		pos_setpoint.position.z = pos_sp[2];

		pos_setpoint.yaw = yaw_sp;

		setpoint_raw_local_pub.publish(pos_setpoint);
	}
	else{
		pos_setpoint.type_mask = 0b100111100011;  // 100 111 100 011  vx,vy, z + yaw

		pos_setpoint.coordinate_frame = 1;
		
		pos_setpoint.velocity.x = pos_sp[0];
		pos_setpoint.velocity.y = pos_sp[1];
		pos_setpoint.position.z = pos_sp[2];

		pos_setpoint.yaw = yaw_sp;

		setpoint_raw_local_pub.publish(pos_setpoint);
	}
}


// function for tracking
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置
    landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + camera_offset[0];
    landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + camera_offset[1];
    landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + camera_offset[2];
    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;
    // std::cout << landpad_det.pos_body_enu_frame << std::endl;

    // if(use_pad_height)
    // {
    //     //若已知降落板高度，则无需使用深度信息。
    //     landpad_det.pos_body_enu_frame[2] = pad_height - _DroneState.position[2];
    // }

    // 此降落方案不考虑偏航角 （高级版可提供）
    landpad_det.att_enu_frame[2] = 0.0;

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
    }

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg) //无人机姿态
{
    _DroneState = *msg;
	// cout << _DroneState << endl;
    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
}


//主状态机更新
void FlyState_update(void)
{

	switch(FlyState)
	{
		case WAITING:
			if(current_state.mode != "OFFBOARD")//等待offboard模式
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0, 1);
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0, 1);
				FlyState = PREPARE;
			}
			//cout << "WAITING" <<endl;
			break;
		case PREPARE:											//飞到目标高度	
			pos_target[0] = temp_pos_drone[0];
			pos_target[1] = temp_pos_drone[1];
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0, 1);
			MoveTimeCnt +=1;
			if(MoveTimeCnt >= 100)
			{
				MoveTimeCnt = 0;
				FlyState = FLY;
			}
			if(current_state.mode != "OFFBOARD")				//如果在REST途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
			}
			break;
		case FLY:
			{
				////////////////////////
				switch (exec_state)
				{
					// 初始状态，等待视觉检测结果
					case WAITING_RESULT:
					{
						if(landpad_det.is_detected)
						{
							exec_state = TRACKING;
							cout << "Get the detection result." <<endl;
							break;
						}
					}
					// 追踪状态
					case TRACKING:
					{
						// 丢失,进入LOST状态
						if(!landpad_det.is_detected)
						{
							exec_state = LOST;
							cout << "Lost the Landing Pad." <<endl;
							break;
						}   

						// 机体系速度控制
						// for (int i=0; i<3; i++)
						// {
						// 	vel_target[i] = kp_land[i] * landpad_det.pos_body_enu_frame[i];
						// }

                        vel_target[0] = kp_land[0] * landpad_det.pos_body_enu_frame[0];
                        vel_target[1] = kp_land[1] * landpad_det.pos_body_enu_frame[1];
						vel_target[2] = desire_z;
                        // std::cout << "x: " << landpad_det.pos_body_enu_frame[0] << " y: " <<  landpad_det.pos_body_enu_frame[0] << std::endl;
                        send_pos_setpoint(vel_target, 0, 0);


						break;
					}
					case LOST:
					{
						static int lost_time = 0;
						lost_time ++ ;
						
						// 重新获得信息,进入TRACKING
						if(landpad_det.is_detected)
						{
							exec_state = TRACKING;
							lost_time = 0;
							cout << "Regain the Landing Pad." <<endl;
							break;
						}   
						
						// 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则原地上升
						if(lost_time < 10.0)
						{
							vel_target[0] = 0.0;
							vel_target[1] = 0.0;
							vel_target[2] = desire_z+1.0;
							ros::Duration(0.4).sleep();
						}else
						{
							vel_target[0] = 0.0;
							vel_target[1] = 0.0;
							vel_target[2] = desire_z+1.0;
						}
						send_pos_setpoint(vel_target, 0, 0);
						break;
					}

				}
				/////////////////////////
				
				if(current_state.mode != "OFFBOARD")			//如果在飞圆形中途中切换到onboard，则跳到WAITING
				{
					FlyState = WAITING;
				}
			}
			//cout << "FLY" <<endl;
			break;

		default:
			cout << "error" <<endl;
	}	
}				


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb); //【订阅】无人机姿态
    // = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);//【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 【服务】修改系统模式
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	
   	nh.param<float>("desire_z", desire_z, 1.0);
	//追踪控制参数
    nh.param<float>("kpx_land", kp_land[0], 0.1);
    nh.param<float>("kpy_land", kp_land[1], 0.1);
    nh.param<float>("kpz_land", kp_land[2], 0.1);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    while(ros::ok())
    {
 		ros::spinOnce();
		FlyState_update();
        std::cout << FlyState << " & " << exec_state << std::endl;

        rate.sleep();
    }

    return 0;

}


