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

#include <apriltags/AprilTagDetections.h>

#include <cmath>
#include <Eigen/Geometry>

#include <vector>
#include <iterator>
#include <algorithm>

ros::Subscriber apriltags_36h11_sub;

ros::Publisher setpoint_target_x_incameraframe_pub;
ros::Publisher setpoint_target_y_incameraframe_pub;
ros::Publisher setpoint_target_z_incameraframe_pub;
ros::Publisher setpoint_target_roll_incameraframe_pub;
ros::Publisher setpoint_target_pitch_incameraframe_pub;
ros::Publisher setpoint_target_yaw_incameraframe_pub;

bool found_36h11 = false;
std::string tag_36h11_detection_topic;
Eigen::Matrix4d tag_to_camera_transformation;
Eigen::Vector3d target_position_incameraframe(0, 0, 0);
Eigen::Vector3d tag_euler_incameraframe(0, 0, 0);
Eigen::Vector3d target_euler_incameraframe(0, 0, 0);

std_msgs::Float64 target_x_incameraframe_msg;
std_msgs::Float64 target_y_incameraframe_msg;
std_msgs::Float64 target_z_incameraframe_msg;
std_msgs::Float64 target_roll_incameraframe_msg;
std_msgs::Float64 target_pitch_incameraframe_msg;
std_msgs::Float64 target_yaw_incameraframe_msg;

void apriltags36h11Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
    if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
    {
        found_36h11 = false;
        return;
    }
    else
    {
        found_36h11 = true;
        //tag_incameraframe是apriltags/detection的类型指针，该类型包括header,id,corners2d,tag_size,pose
        auto tag_incameraframe = std::begin(apriltag_pos_msg->detections);
        geometry_msgs::Pose tag_pose_incameraframe = tag_incameraframe->pose;
        //tag_incameraframe->pose，类型为geometry_msgs/Pose，包括位置和四元数姿态，将其转换为标签相对于相机的4×4转换矩阵
        Eigen::Vector3d tag_position_incameraframe = Eigen::Vector3d(tag_pose_incameraframe.position.x, tag_pose_incameraframe.position.y, tag_pose_incameraframe.position.z);
        Eigen::Quaternion<double> tag_quaternion_incameraframe = Eigen::Quaternion<double>(tag_pose_incameraframe.orientation.w, tag_pose_incameraframe.orientation.x, tag_pose_incameraframe.orientation.y, tag_pose_incameraframe.orientation.z);
        tag_to_camera_transformation.block(0,0,3,3) = tag_quaternion_incameraframe.toRotationMatrix();
        tag_to_camera_transformation.block(0,3,3,1) = tag_position_incameraframe;
        //最终逼近点相对于相机坐标系的位置
        Eigen::Vector4d position = tag_to_camera_transformation * Eigen::Vector4d(0,0,1,1);
        target_position_incameraframe = position.head(3);
        std::cout << "target_x_incameraframe = " << target_position_incameraframe(0) << std::endl;
        std::cout << "target_y_incameraframe = " << target_position_incameraframe(1) << std::endl;
        std::cout << "target_z_incameraframe = " << target_position_incameraframe(2) << std::endl;
        //标签坐标系相对于相机坐标系的欧拉角
        tag_euler_incameraframe = tag_quaternion_incameraframe.toRotationMatrix().eulerAngles(2, 1, 0);
        //std::cout << "tag_euler_incameraframe around x axis = " << tag_euler_incameraframe[2] / M_PI * 180 << std::endl;
	    //std::cout << "tag_euler_incameraframe around y axis = " << tag_euler_incameraframe[1] / M_PI * 180 << std::endl;
	    //std::cout << "tag_euler_incameraframe around z axis = " << tag_euler_incameraframe[0] / M_PI * 180 << std::endl;
        //最终逼近点坐标系相对于目前相机坐标系的欧拉角
        //标签坐标系围绕其X轴转180度极为最终逼近点坐标系的姿态
        Eigen::AngleAxisd tag_to_target_angleaxis = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d camera_to_target_rotation = tag_quaternion_incameraframe.toRotationMatrix() * tag_to_target_angleaxis.matrix();
        target_euler_incameraframe = camera_to_target_rotation.eulerAngles(2, 1, 0); 
        std::cout << "target_euler_incameraframe around x axis = " << target_euler_incameraframe[2] / M_PI * 180 << std::endl;
	    std::cout << "target_euler_incameraframe around y axis = " << target_euler_incameraframe[1] / M_PI * 180 << std::endl;
	    std::cout << "target_euler_incameraframe around z axis = " << target_euler_incameraframe[0] / M_PI * 180 << std::endl;
    }
}

void print_parameters()
{
    ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_approaching_setpoint");
    ros::NodeHandle nh;
    ros::NodeHandle node_priv("~");

    node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/apriltags/detections");
    print_parameters();

    apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1, apriltags36h11Callback);

    setpoint_target_x_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_x_incameraframe", 10);
    setpoint_target_y_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_y_incameraframe", 10);
    setpoint_target_z_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_z_incameraframe", 10);
    setpoint_target_roll_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_roll_incameraframe", 10);
    setpoint_target_pitch_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_pitch_incameraframe", 10);
    setpoint_target_yaw_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_yaw_incameraframe", 10);

    tag_to_camera_transformation << 0, 0, 0, 0,
                                    0, 0, 0, 0,
                                    0, 0, 0, 0,
                                    0, 0, 0, 1;

    ros::Rate loop_rate(200);

    ros::spinOnce();

    while (ros::ok())
    {
        ros::spinOnce();
        if(found_36h11)
        {
            target_x_incameraframe_msg.data = target_position_incameraframe(0);
            target_y_incameraframe_msg.data = target_position_incameraframe(1);
            target_z_incameraframe_msg.data = target_position_incameraframe(2);
            target_roll_incameraframe_msg.data = target_euler_incameraframe(2);
            target_pitch_incameraframe_msg.data = target_euler_incameraframe(1);
            target_yaw_incameraframe_msg.data = target_euler_incameraframe(0);

            setpoint_target_x_incameraframe_pub.publish(target_x_incameraframe_msg);
            setpoint_target_y_incameraframe_pub.publish(target_y_incameraframe_msg);
            setpoint_target_z_incameraframe_pub.publish(target_z_incameraframe_msg);
            setpoint_target_roll_incameraframe_pub.publish(target_roll_incameraframe_msg);
            setpoint_target_pitch_incameraframe_pub.publish(target_pitch_incameraframe_msg);
            setpoint_target_yaw_incameraframe_pub.publish(target_yaw_incameraframe_msg);

            std::cout << "pid target published." << std::endl;

        }
        else
        {
           
        }
        loop_rate.sleep();
    }

}