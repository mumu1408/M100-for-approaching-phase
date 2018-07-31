#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/GimbalAngleControl.h>
#include <dji_sdk/GimbalSpeedControl.h>

#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <apriltags/AprilTagDetections.h>
//#include <apriltags_ros/AprilTagDetectionArray.h>


ros::Subscriber apriltags_pos_sub;
ros::Subscriber gimbal_ori_sub;
ros::Subscriber filtered_x_sub;
ros::Subscriber filtered_y_sub;
ros::Subscriber filtered_z_sub;


ros::Publisher setpoint_yaw_pub;
ros::Publisher setpoint_pitch_pub;

ros::Publisher yaw_state_pub;
ros::Publisher pitch_state_pub;


std_msgs::Float64 setpoint_yaw;
std_msgs::Float64 setpoint_pitch;

double gimbal_roll;
double gimbal_yaw;
double gimbal_pitch;

double tag_x;
double tag_y;
double tag_z;

bool updated = false;
bool apriltag_in_sight = false;


const double z_threshold = 0.0001;

std::string tag_detection_topic;


void apriltagsPositionCallback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
   if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
   {
     apriltag_in_sight = false;
     return;
   }
   else
   {
     tag_x = apriltag_pos_msg->detections[0].pose.position.x;
     tag_y = -apriltag_pos_msg->detections[0].pose.position.y;
     tag_z = apriltag_pos_msg->detections[0].pose.position.z;

     if(fabs(tag_z) < 0.0001)
     {
       return;
     }

   }
   setpoint_yaw.data = gimbal_yaw + atan(tag_x / tag_z) * 180 / M_PI;
   setpoint_pitch.data = gimbal_pitch + atan(tag_y / tag_z) * 180 / M_PI;
}


void gimbalOrientationCallback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
  gimbal_roll = gimbal_ori_msg->roll;
  gimbal_yaw = gimbal_ori_msg->yaw;
  gimbal_pitch = gimbal_ori_msg->pitch;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal_track_setpoint_node");
  ROS_INFO("Starting gimbal track setpoint publisher");
  ros::NodeHandle nh;

  nh.param<std::string>("/gimbal_track_setpoint/tag_detection_topic", tag_detection_topic, "/apriltags/detections");

  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/gimbal_track/setpoint_yaw", 1);//
  setpoint_pitch_pub = nh.advertise<std_msgs::Float64>("/gimbal_track/setpoint_pitch", 1);//
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/gimbal_track/yaw_state", 10);
  pitch_state_pub = nh.advertise<std_msgs::Float64>("/gimbal_track/pitch_state", 10);

  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);

  std_msgs::Float64 yaw_state_msg;
  std_msgs::Float64 pitch_state_msg;

  //ros::Rate loop_rate(200); 
ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Publishing the setpoint data...");
    setpoint_yaw_pub.publish(setpoint_yaw);     
    setpoint_pitch_pub.publish(setpoint_pitch);

    yaw_state_msg.data = gimbal_yaw;
    pitch_state_msg.data = gimbal_pitch;

    yaw_state_pub.publish(yaw_state_msg);     
    pitch_state_pub.publish(pitch_state_msg);

    loop_rate.sleep();
  }
}
