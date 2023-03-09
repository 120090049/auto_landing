#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Bool.h>

#include "mission_utils.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;
#define PI acos(-1)

float pre_x, pre_y;
int time_keep_moving = 0;
Eigen::Vector3d pre_vel_target;

bool Change_state;
ofstream outFile;
//parameters for basic control (before the uav starts tracking)
float MoveTimeCnt = 0;
float k = 0.2;
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d pos_target;//offboard模式下，发送给飞控的期望值
Eigen::Vector3d vel_target;//offboard模式下，发送给飞控的期望值

mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::Publisher front_cam_detect_switch;
ros::Publisher down_cam_detect_switch;

//parameters for detection
float camera_offset[3];
prometheus_msgs::DroneState _DroneState;    // 无人机姿态
Eigen::Matrix3f R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵

Detection_result boat_det;               // 前视摄像头检测结果
Detection_result landpad_det;               // 下视摄像头检测结果
float kp_land[2];         //控制参数 - 比例参数
float present_yaw;

double UAV_GPS[3];
double USV_GPS[3];
double BOUY_GPS_z;

float set_init_height; //无人机 offboard prepare to control 时无人机set to初始高度
float set_approach_height; //无人机进入approach stage时的目标高度
float init_height = 0.0; // initial height 对于无人机的高度控制很重要 
				   // 比如如果是在10m的高度无人机开机，那么set_height to 5, 则要给飞控发送-5米的指令，反正就是要-10m（initial height的高度）

//主状态
enum
{
    WAITING,		//等待offboard模式
	PREPARE,		//起飞特定高度
	FLY,			//开始飞（检测开始）

}FlyState = WAITING;//初始状态WAITING
const char* FlyState_name[] = {"WAITING", "PREPARE", "FLY"};

//识别状态
enum 
{
    APPROACH_GNSS,
	APPROACH_FRONT_CAMERA,
    TRACKING_AND_LANDING,
    LOST_AND_ASCENDING,
    // LANDING,
}exec_state = APPROACH_GNSS;
const char* exec_state_name[] = {"APPROACH_GNSS", "APPROACH_FRONT_CAMERA", "TRACKING_AND_LANDING", "LOST_AND_ASCENDING"};

///////////////////////////////////////////////////////////////////////////////////////////////

// // GPS数据回调函数
// Eigen::Vector3d current_gps;
// void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
//     current_gps = { msg->latitude, msg->longitude, msg->altitude };
	
// }

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
		pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw 基本用不着

		pos_setpoint.coordinate_frame = 1;

		pos_setpoint.position.x = pos_sp[0];
		pos_setpoint.position.y = pos_sp[1];
		pos_setpoint.position.z = pos_sp[2];

		pos_setpoint.yaw = present_yaw - yaw_sp;
		setpoint_raw_local_pub.publish(pos_setpoint);
	}
	else{
		pos_setpoint.type_mask = 0b100111100011;  // 100 111 100 011  vx,vy, z + yaw

		pos_setpoint.coordinate_frame = 1;
		
		pos_setpoint.velocity.x = pos_sp[0];
		pos_setpoint.velocity.y = pos_sp[1];
		pos_setpoint.position.z = pos_sp[2]-init_height; // 
		pos_setpoint.yaw = present_yaw - yaw_sp;
		setpoint_raw_local_pub.publish(pos_setpoint);
	}
}

void boat_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    boat_det.object_name = "boat";
    boat_det.Detection_info = *msg;
	
	// 偏行角
    boat_det.yaw_error = boat_det.Detection_info.yaw_error;

	boat_det.pos_body_frame[0] = cos(boat_det.yaw_error); // position[1];
    boat_det.pos_body_frame[1] = - sin(boat_det.yaw_error); // position[0];
    boat_det.pos_body_frame[2] = - set_approach_height; // position[2]
    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    boat_det.pos_body_enu_frame = R_Body_to_ENU * boat_det.pos_body_frame;

    if(boat_det.Detection_info.detected)
    {
        boat_det.num_regain++;
        boat_det.num_lost = 0;
    }else
    {
        boat_det.num_regain = 0;
        boat_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(boat_det.num_lost > VISION_THRES-5)
    {
        boat_det.is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(boat_det.num_regain > VISION_THRES+10)
    {
        boat_det.is_detected = true;
    }

}

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
	
	// 偏行角
    landpad_det.yaw_error = landpad_det.Detection_info.yaw_error;

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
	// 得到姿态旋转矩阵
    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
	
	// 计算偏航角
	cv::Vec3d rotvec(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);
	cv::Mat rotation_matrix;
	cv::Rodrigues(rotvec, rotation_matrix);
	rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
	float r11 = rotation_matrix.ptr<float>(0)[0];
	float r21 = rotation_matrix.ptr<float>(1)[0];
	float thetaz = atan2(r21, r11) / 3.1415 * 180;
	present_yaw = thetaz / 180 * CV_PI;

}

void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    // Print the link states to the console
    for (int i = 0; i < msg->name.size(); i++) {
		if (0 == strcmp(msg->name[i].c_str(), "p450_monocular::p450::base_link") ) {
			UAV_GPS[0] = msg->pose[i].position.x;
			UAV_GPS[1] = msg->pose[i].position.y;
			UAV_GPS[2] = msg->pose[i].position.z;
		}
		if (0 == strcmp(msg->name[i].c_str(), "boat::base_link") ) {
			USV_GPS[0] = msg->pose[i].position.x;
			USV_GPS[1] = msg->pose[i].position.y;
			USV_GPS[2] = msg->pose[i].position.z;
		}
		if (0 == strcmp(msg->name[i].c_str(), "bouy::bouy::base_link") ) {
			BOUY_GPS_z = msg->pose[i].position.z;
		}
		// float USV_GPS[3]; boat::base_link
		// float BOUY_GPS_z; bouy::bouy::base_link
    }
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
				cout << "Start offboard mode!" << endl;
			}
			break;

		case PREPARE: //飞到目标高度	

			if (init_height == 0.0) {
				init_height = UAV_GPS[2] - pos_drone[2];
				cout << UAV_GPS[2] << "  " << pos_drone[2] << endl;
			}				
			//这里的 pos_target其实是 vx,vy, z							
			pos_target[0] = 0;
			pos_target[1] = 0;
			pos_target[2] = set_init_height;
			send_pos_setpoint(pos_target, 0, 0);
			if(UAV_GPS[2] >= set_init_height-0.5)
			{
				cout << UAV_GPS[2] << ">=" <<  (set_init_height-0.5) << endl;
				cout << "Reach to initial height and ready for detection!" << endl;
				FlyState = FLY;
			}
			if(current_state.mode != "OFFBOARD")				//如果在REST途中切换到onboard，则跳到WAITING
			{
				FlyState = WAITING;
				cout << "Change back to the original mode" << endl;
			}
			
			break;
		case FLY:
			{
				switch (exec_state)
				{
					// 初始状态，等待视觉检测结果 APPROACH_GNSS
					case APPROACH_GNSS:
					{
						double digonal_side = sqrt( pow(USV_GPS[1]-UAV_GPS[1], 2) + pow(UAV_GPS[0]-USV_GPS[0], 2) );
                        vel_target[0] = (USV_GPS[0]-UAV_GPS[0])/digonal_side;
						vel_target[1] = (USV_GPS[1]-UAV_GPS[1])/digonal_side;
						vel_target[2] = set_approach_height;
						float target_theta = atan2((USV_GPS[0]-UAV_GPS[0]), (USV_GPS[1]-UAV_GPS[1])) + present_yaw - PI/2;

						send_pos_setpoint(vel_target, target_theta, 0);

						// open the front camera detection module
						std_msgs::Bool switch_front;
						switch_front.data = true;
						front_cam_detect_switch.publish(switch_front);

						// else open the down-looking camera detection module
						std_msgs::Bool switch_down;
						switch_down.data = true;
						down_cam_detect_switch.publish(switch_down);

						// whether the front camera detected
						if(boat_det.is_detected)
						{
							exec_state = APPROACH_FRONT_CAMERA;
							Change_state = true;
							cout << "Get the front camera detection result and get to the APPROACH_FRONT_CAMERA mode." <<endl;
							break;
						}
						

						// // whether the front camera detected
						else if(landpad_det.is_detected)
						{
							exec_state = TRACKING_AND_LANDING;
							Change_state = true;
							cout << "Get the down-looking camera detection result and get to the TRACKING_AND_LANDING." <<endl;
							std_msgs::Bool switch_front;
							switch_front.data = false;
							front_cam_detect_switch.publish(switch_front);
						}
						break;
					}

					case APPROACH_FRONT_CAMERA:
					{
						// submodule 1 front looking camera operation
						if(boat_det.is_detected) // whether the front camera detected
						{
							time_keep_moving = 0;
							float x = boat_det.pos_body_enu_frame[0];
							float y = boat_det.pos_body_enu_frame[1];
							if (Change_state) {
							pre_x = x;
							pre_y = y;
							}
							else {
								x = k*x + (1-k)*pre_x;
								y = k*y + (1-k)*pre_y;
								pre_x = x;
								pre_y = y;
							}
							// cout << "(" << x << ", " << y << ") and yaw = " << boat_det.yaw_error << endl; 
							vel_target[0] = kp_land[0] * x;
							vel_target[1] = kp_land[1] * y;
							vel_target[2] = set_approach_height;
							pre_vel_target[0] = vel_target[0];
							pre_vel_target[1] = vel_target[1];
							pre_vel_target[2] = vel_target[2];
							send_pos_setpoint(vel_target, boat_det.yaw_error, 0);
							Change_state = false;
						}
						else {
							if (time_keep_moving < 200) { // still keep moving for a while
								cout << "Although lost, still keep moving" << endl;
								time_keep_moving ++;
								send_pos_setpoint(pre_vel_target, 0, 0);
							}
							else {
								exec_state = APPROACH_GNSS;
								Change_state = true;
								cout << "Lost the front camera detection result back to the APPROACH_GNSS mode." <<endl;
								break;
							}
					
						}

						// submodule 2 down looking camera operation

						// open the down-looking camera detection module
						std_msgs::Bool switch_down;
						switch_down.data = true;
						down_cam_detect_switch.publish(switch_down);

						// // whether the front camera detected
						if(landpad_det.is_detected)
						{
							exec_state = TRACKING_AND_LANDING;
							Change_state = true;
							cout << "Get the down-looking camera detection result and get to the TRACKING_AND_LANDING." <<endl;
							std_msgs::Bool switch_front;
							switch_front.data = false;
							front_cam_detect_switch.publish(switch_front);
						}
						break;
					}
				
					
					// 追踪状态 
					case TRACKING_AND_LANDING: 
					{
						// 丢失,进入LOST状态
						if(!landpad_det.is_detected)
						{
							exec_state = LOST_AND_ASCENDING;
							cout << "Lost the Landing Pad and ascending." <<endl;
							break;
						}   

						float x = landpad_det.pos_body_enu_frame[0];
						float y = landpad_det.pos_body_enu_frame[1];
						
						if (Change_state) {
							pre_x = x;
							pre_y = y;
						}
						else {
							x = k*x + (1-k)*pre_x;
        					y = k*y + (1-k)*pre_y;
							pre_x = x;
							pre_y = y;
						}
                        vel_target[0] = kp_land[0] * x;
                        vel_target[1] = kp_land[1] * y;
						vel_target[2] = set_approach_height-1;

						send_pos_setpoint(vel_target, landpad_det.yaw_error, 0);
						Change_state = false;
						break;
					}

					case LOST_AND_ASCENDING:
					{
						static int lost_time = 0;
						lost_time ++ ;
						
						// 重新获得信息,进入TRACKING
						if(landpad_det.is_detected)
						{
							exec_state = TRACKING_AND_LANDING;
							lost_time = 0;
							cout << "Regain the Landing Pad." <<endl;
							Change_state = true;
							break;
						}   
						
						// 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则原地上升
						if(lost_time < 10.0)
						{
							vel_target[0] = 0.0;
							vel_target[1] = 0.0;
							vel_target[2] = set_approach_height;
							ros::Duration(0.4).sleep();
						}else
						{
							vel_target[0] = 0.0;
							vel_target[1] = 0.0;
							vel_target[2] = set_approach_height+1.0;
						}
						send_pos_setpoint(vel_target, present_yaw, 0);
						break;
					}

				}
				
				if(current_state.mode != "OFFBOARD")			//如果在飞圆形中途中切换到onboard，则跳到WAITING
				{
					FlyState = WAITING;
				}
			}
			break;

		default:
			cout << "error" <<endl;
	}	
}				


int main(int argc, char **argv)
{
	outFile.open("/home/clp/catkin_ws/src/auto_landing/file/xy.txt");

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
    ros::Subscriber boat_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/boat_det", 10, boat_det_cb);
	ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb); //【订阅】无人机姿态
	ros::Subscriber link_states_sub = nh.subscribe("/gazebo/link_states", 10, linkStatesCallback);
   
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    front_cam_detect_switch = nh.advertise<std_msgs::Bool>("/prometheus/switch/boat_det", 10);
	down_cam_detect_switch = nh.advertise<std_msgs::Bool>("/prometheus/switch/landpad_det", 10);
	
	// ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global",100, gps_cb);

    
	
   	nh.param<float>("set_init_height", set_init_height, 12.0);
	nh.param<float>("set_approach_height", set_approach_height, 5.0);
	
	//追踪控制参数
    nh.param<float>("kpx_land", kp_land[0], 0.1);
    nh.param<float>("kpy_land", kp_land[1], 0.1);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.2);
	int i = 0;
    while(ros::ok())
    {
		i ++;
 		ros::spinOnce();
		FlyState_update();
		if (i % 20 == 0) {
			std::cout << FlyState_name[FlyState] << " & " << exec_state_name[exec_state] << std::endl;
		}

        rate.sleep();
    }

    return 0;

}


