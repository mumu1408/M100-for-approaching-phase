#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/GimbalSpeedControl.h>
#include <dji_sdk/VelocityControl.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>
#include <iterator>
#include <string>
#include <iostream>

ros::Subscriber cameraframe_x_velocity_control_sub;
ros::Subscriber cameraframe_y_velocity_control_sub;
ros::Subscriber cameraframe_z_velocity_control_sub;
ros::Subscriber cameraframe_roll_velocity_control_sub;
ros::Subscriber cameraframe_pitch_velocity_control_sub;
ros::Subscriber cameraframe_yaw_velocity_control_sub;

ros::Subscriber gimbal_ori_sub;
ros::Subscriber drone_local_position_sub;
ros::Subscriber drone_quaternion_sub;
ros::Subscriber drone_global_position_sub;

ros::ServiceClient gimbal_velocity_control_service;
ros::ServiceClient drone_velocity_control_service;
ros::ServiceClient sdk_permission_control_service;

double cameraframe_x_velocity_control_effort;
double cameraframe_y_velocity_control_effort;
double cameraframe_z_velocity_control_effort;
double cameraframe_roll_velocity_control_effort;
double cameraframe_pitch_velocity_control_effort;
double cameraframe_yaw_velocity_control_effort;

double gimbal_roll;
double gimbal_pitch;
double gimbal_yaw;

void cameraframe_x_velocity_control_effort_callback(std_msgs::Float64 cameraframe_x_velocity_control_effort_msg)
{
    cameraframe_x_velocity_control_effort = cameraframe_x_velocity_control_effort_msg.data;
    std::cout << "cameraframe_x_velocity_control_effort: " << cameraframe_x_velocity_control_effort << std::endl;
}
void cameraframe_y_velocity_control_effort_callback(std_msgs::Float64 cameraframe_y_velocity_control_effort_msg)
{
    cameraframe_y_velocity_control_effort = cameraframe_y_velocity_control_effort_msg.data;
    std::cout << "cameraframe_y_velocity_control_effort: " << cameraframe_y_velocity_control_effort << std::endl;
}
void cameraframe_z_velocity_control_effort_callback(std_msgs::Float64 cameraframe_z_velocity_control_effort_msg)
{
    cameraframe_z_velocity_control_effort = cameraframe_z_velocity_control_effort_msg.data;
    std::cout << "cameraframe_z_velocity_control_effort: " << cameraframe_z_velocity_control_effort << std::endl;
}
void cameraframe_roll_velocity_control_effort_callback(std_msgs::Float64 cameraframe_roll_velocity_control_effort_msg)
{
    cameraframe_roll_velocity_control_effort = cameraframe_roll_velocity_control_effort_msg.data;
    std::cout << "cameraframe_roll_velocity_control_effort: " << cameraframe_roll_velocity_control_effort << std::endl;
}
void cameraframe_pitch_velocity_control_effort_callback(std_msgs::Float64 cameraframe_pitch_velocity_control_effort_msg)
{
    cameraframe_pitch_velocity_control_effort = cameraframe_pitch_velocity_control_effort_msg.data;
    std::cout << "cameraframe_pitch_velocity_control_effort: " << cameraframe_pitch_velocity_control_effort << std::endl;
}
void cameraframe_yaw_velocity_control_effort_callback(std_msgs::Float64 cameraframe_yaw_velocity_control_effort_msg)
{
    cameraframe_yaw_velocity_control_effort = cameraframe_yaw_velocity_control_effort_msg.data;
    std::cout << "cameraframe_yaw_velocity_control_effort: " << cameraframe_yaw_velocity_control_effort << std::endl;
}

void gimbal_ori_callback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
    gimbal_roll = gimbal_ori_msg->roll * M_PI / 180;
    gimbal_yaw = gimbal_ori_msg->yaw * M_PI / 180;
    gimbal_pitch = gimbal_ori_msg->pitch * M_PI / 180;
    //std::cout << "gimbal_roll: " << gimbal_roll << std::endl;
}
void drone_local_position_callback(const dji_sdk::LocalPosition::ConstPtr& drone_local_position_msg)
{

}
void drone_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr& drone_quaternion_msg)
{

}
void drone_global_position_callback(const dji_sdk::GlobalPosition::ConstPtr& drone_global_position_msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_approaching_velocity_controller");
    ros::NodeHandle nh;

    cameraframe_x_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_x_velocity_control_effort", 10, cameraframe_x_velocity_control_effort_callback);
    cameraframe_y_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_y_velocity_control_effort", 10, cameraframe_y_velocity_control_effort_callback);
    cameraframe_z_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_z_velocity_control_effort", 10, cameraframe_z_velocity_control_effort_callback);
    cameraframe_roll_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_roll_velocity_control_effort", 10, cameraframe_roll_velocity_control_effort_callback);
    cameraframe_pitch_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_pitch_velocity_control_effort", 10, cameraframe_pitch_velocity_control_effort_callback);
    cameraframe_yaw_velocity_control_sub = nh.subscribe("/m100_approaching/cameraframe_yaw_velocity_control_effort", 10, cameraframe_yaw_velocity_control_effort_callback);

    gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbal_ori_callback);
    drone_local_position_sub = nh.subscribe("/dji_sdk/local_position", 10, drone_local_position_callback);
    drone_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 10, drone_quaternion_callback);
    drone_global_position_sub = nh.subscribe("/dji_sdk/global_position", 10, drone_global_position_callback);

    gimbal_velocity_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("/dji_sdk/gimbal_speed_control");
    drone_velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("/dji_sdk/velocity_control");
    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control");

    dji_sdk::SDKPermissionControl sdk_permission_control;
    dji_sdk::VelocityControl velocity_control;
    dji_sdk::GimbalSpeedControl gimbal_speed_control;

    //将相机坐标系的速度转换到云台角速度和飞机线速度
    //定义坐标系的转换关系
    Eigen::Matrix3d camera_to_inneraxis_rotation, inneraxis_to_mediumaxis_rotation, mediumaxis_to_outeraxis_rotation, outeraxis_to_base_rotation, base_to_drone_rotation;
    Eigen::Matrix3d camera_to_drone_rotation;
    Eigen::AngleAxisd inneraxis_to_mediumaxis_pitch_angleaxis, mediumaxis_to_outeraxis_roll_angleaxis, outeraxis_to_base_yaw_angleaxis;

    camera_to_inneraxis_rotation << 0, 0, 1,
                                    1, 0, 0,
                                    0, 1, 0;
    base_to_drone_rotation << 1, 0, 0,
                              0, 1, 0,
                              0, 0, -1;
    //定义云台角速度值和飞行器线速度值
    double gimbal_inneraxis_pitch_speed;
    double gimbal_mediumaxis_roll_speed;
    double gimbal_outeraxis_yaw_speed;
    Eigen::Vector3d  drone_velocity(0, 0, 0);
    //中间变量值
    Eigen::Vector3d gimbal_inneraxisframe_speed(0, 0, 0);
    Eigen::Vector3d gimbal_mediumaxisframe_speed(0, 0, 0);
    Eigen::Vector3d gimbal_outeraxisframe_speed(0, 0, 0);
    Eigen::Vector3d cameraframe_angular_speed(0, 0, 0);
    Eigen::Vector3d cameraframe_linear_speed(0, 0, 0);
    
    sdk_permission_control.request.control_enable = 1;
    bool control_requested = false;
    while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
    {
        ROS_ERROR("Velocity controller: request control failed!");
    }

    while(ros::ok())
    {
        ros::spinOnce();

        //将相机坐标系的速度转换到云台角速度和飞机线速度
        cameraframe_angular_speed << cameraframe_roll_velocity_control_effort, cameraframe_pitch_velocity_control_effort, cameraframe_yaw_velocity_control_effort;
        cameraframe_linear_speed << cameraframe_x_velocity_control_effort, cameraframe_y_velocity_control_effort, cameraframe_z_velocity_control_effort;

        gimbal_inneraxisframe_speed = camera_to_inneraxis_rotation * cameraframe_angular_speed;
        gimbal_inneraxis_pitch_speed = gimbal_inneraxisframe_speed(1);//wy内
        
        inneraxis_to_mediumaxis_pitch_angleaxis = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
        inneraxis_to_mediumaxis_rotation = inneraxis_to_mediumaxis_pitch_angleaxis.matrix();
        gimbal_mediumaxisframe_speed = inneraxis_to_mediumaxis_rotation * Eigen::Vector3d(gimbal_inneraxisframe_speed(0), 0, gimbal_inneraxisframe_speed(2));
        gimbal_mediumaxis_roll_speed = gimbal_mediumaxisframe_speed(0);//wx中

        mediumaxis_to_outeraxis_roll_angleaxis = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());
        mediumaxis_to_outeraxis_rotation = mediumaxis_to_outeraxis_roll_angleaxis.matrix();
        gimbal_outeraxisframe_speed = mediumaxis_to_outeraxis_rotation * Eigen::Vector3d(0, 0, gimbal_mediumaxisframe_speed(2));
        gimbal_outeraxis_yaw_speed = gimbal_outeraxisframe_speed(2);//wz外

        outeraxis_to_base_yaw_angleaxis = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
        outeraxis_to_base_rotation = outeraxis_to_base_yaw_angleaxis.matrix();
        camera_to_drone_rotation = base_to_drone_rotation * outeraxis_to_base_rotation * mediumaxis_to_outeraxis_rotation * inneraxis_to_mediumaxis_rotation * camera_to_inneraxis_rotation;
        drone_velocity = camera_to_drone_rotation * cameraframe_linear_speed;

        //debug 
        std::cout << "drone_velocity_x: " << drone_velocity(0) << std::endl;
        std::cout << "drone_velocity_y: " << drone_velocity(1) << std::endl;
        std::cout << "drone_velocity_z: " << drone_velocity(2) << std::endl;
        std::cout << "gimbal_mediumaxis_roll_speed: " << gimbal_mediumaxis_roll_speed << std::endl;
        std::cout << "gimbal_inneraxis_pitch_speed: " << gimbal_inneraxis_pitch_speed << std::endl;
        std::cout << "gimbal_outeraxis_yaw_speed: " << gimbal_outeraxis_yaw_speed << std::endl;



        velocity_control.request.frame = 0;//body frame
        velocity_control.request.vx = drone_velocity(0);
        velocity_control.request.vy = drone_velocity(1);
        velocity_control.request.vz = drone_velocity(2);
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }

        gimbal_speed_control.request.roll_rate = gimbal_mediumaxis_roll_speed;
        gimbal_speed_control.request.pitch_rate = gimbal_inneraxis_pitch_speed;
        gimbal_speed_control.request.yaw_rate = gimbal_outeraxis_yaw_speed;

        if(!(gimbal_velocity_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result))
        {
            ROS_ERROR("Velocity controller: gimbal speed control failed!");
        }
    }
    sdk_permission_control.request.control_enable = 0;
    sdk_permission_control_service.call(sdk_permission_control);
}