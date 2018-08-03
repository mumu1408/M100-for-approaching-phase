#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

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

    gimbal_velocity_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("/dji_sdk/gimbal_speed_control")
    drone_velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("/dji_sdk/velocity_control");
    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control");

    dji_sdk::SDKPermissionControl sdk_permission_control;
    dji_sdk::VelocityControl velocity_control;
    dji_sdk::GimbalSpeedControl gimbal_speed_control;

    sdk_permission_control.request.control_enable = 1;
    bool control_requested = false;
    while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
    {
        ROS_ERROR("Velocity controller: request control failed!");
    }

    while(ros::ok())
    {
        ros::spinOnce();

        velocity_control.request.frame = 0;//body frame
        velocity_control.request.vx = ;
        velocity_control.request.vy = ;
        velocity_control.request.vz = ;
        velocity_control.request.yawRate = ;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }

        gimbal_speed_control.request.roll_rate = ;
        gimbal_speed_control.request.pitch_rate = ;
        gimbal_speed_control.request.yaw_rate = ;

        if(!(gimbal_velocity_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result))
        {
            ROS_ERROR("Velocity controller: gimbal speed control failed!");
        }
    }
    sdk_permission_control.request.control_enable = 0;
    sdk_permission_control_service.call(sdk_permission_control);
}