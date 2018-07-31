#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

#include <tag.h>

#include <cmath>
#include <Eigen/Geometry> 

#include <vector>
#include <iterator>
#include <algorithm>

#define APRILTAGS

ros::Subscriber gimbal_ori_sub;
ros::Subscriber local_position_sub;
ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher yaw_state_pub;
ros::Publisher x_state_pub;
ros::Publisher y_state_pub;
ros::Publisher z_state_pub;


double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double local_x;
double local_y;
double local_z;
double flight_height;

double error_yaw;

double heading_q0;
double heading_q1;
double heading_q2;
double heading_q3;

Eigen::Quaternion<double> drone_heading;
double yaw_state = 0;

double setpoint_x = 0;
double setpoint_y = 0;
double setpoint_z = 0;
double setpoint_yaw = 0;

int flight_status;

std_msgs::Float64 x_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 z_state_msg;


std_msgs::Float64 setpoint_x_msg;
std_msgs::Float64 setpoint_y_msg;
std_msgs::Float64 setpoint_yaw_msg;
std_msgs::Float64 yaw_state_msg;

std::vector<double> yaw_errors;

bool first_start = true;

bool landing_enabled = true;

void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
  gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
  gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
}

void localPositionCallback(const dji_sdk::LocalPosition::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->x;
  local_y = local_position_msg->y;
  local_z = local_position_msg->z;
}

void attitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr& attitude_quaternion_msg)
{
  heading_q0 = attitude_quaternion_msg->q0;
  heading_q1 = attitude_quaternion_msg->q1;
  heading_q2 = attitude_quaternion_msg->q2;
  heading_q3 = attitude_quaternion_msg->q3;

  drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
  yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;
}

void globalPositionCallback(const dji_sdk::GlobalPosition::ConstPtr& global_position_msg)
{
  flight_height = global_position_msg->height;
}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m100_track_setpoint_test");
  ROS_INFO("Starting position track setpoint publisher");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_x", 10);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_y", 10);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_yaw", 10);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/yaw_state", 10);
  x_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/x_state", 10);
  y_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/y_state", 10);
  z_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/z_state", 10);

  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attitudeQuaternionCallback ); 
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  global_position_sub = nh.subscribe("/dji_sdk/global_position", 10, globalPositionCallback);

  //camera to gimbal transformation
  Eigen::Matrix3d camera_to_gimbal_transformation;
  camera_to_gimbal_transformation << 0, 0, 1,
                                     1, 0, 0,
                                     0, 1, 0;

  Eigen::AngleAxisd yawAngle;
  Eigen::AngleAxisd pitchAngle;
  Eigen::AngleAxisd rollAngle;

  Eigen::Quaternion<double> q;

  //camera to drone transformation
  Eigen::Matrix3d camera_to_drone_transformation;

  ros::Rate loop_rate(200); 

  ros::spinOnce();
  while (ros::ok())
  {
	ros::spinOnce();

	if(flight_status != 3)
	{
		continue;
		ROS_INFO("flight_status != 3");
		// std::cout << flight_status << std::endl;
	}
	ROS_INFO("flight_status == 3");

	first_start = false;
	yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
	pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
	rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

	q = rollAngle * pitchAngle * yawAngle;

	camera_to_drone_transformation = q.matrix() * camera_to_gimbal_transformation;

	setpoint_yaw = 90;
	setpoint_yaw_msg.data = setpoint_yaw;
	setpoint_yaw_pub.publish(setpoint_yaw_msg);

	setpoint_x = 2;
	setpoint_y = 2;

	setpoint_x_msg.data = setpoint_x;
	setpoint_y_msg.data = setpoint_y;

	setpoint_x_pub.publish(setpoint_x_msg);
	setpoint_y_pub.publish(setpoint_y_msg);

	yaw_state_msg.data = yaw_state;
	yaw_state_pub.publish(yaw_state_msg);


	x_state_msg.data = local_x;
	y_state_msg.data = local_y;
	z_state_msg.data = flight_height;
	if(flight_height < 0)
	{
	ROS_DEBUG_THROTTLE(1, "Setpoint node: Flight height is negative, go to hell DJI");
	}

	x_state_pub.publish(x_state_msg); 
	y_state_pub.publish(y_state_msg); 
	z_state_pub.publish(z_state_msg); 
    
	loop_rate.sleep();
  }
}
