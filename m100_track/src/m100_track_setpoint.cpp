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

ros::Subscriber apriltags_36h11_sub;
ros::Subscriber gimbal_ori_sub;
ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_z_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher landing_condition_met_pub;
ros::Publisher relanding_condition_met_pub;
ros::Publisher yaw_state_pub;
ros::Publisher x_state_pub;
ros::Publisher y_state_pub;
ros::Publisher z_state_pub;

//publish msg for debug
ros::Publisher tag_x_indroneframe_pub;
ros::Publisher tag_y_indroneframe_pub;
ros::Publisher tag_z_indroneframe_pub;
ros::Publisher tag_yaw_incameraframe_pub;


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

//msg for debug
double tag_x_indroneframe = 0;
double tag_y_indroneframe = 0;
double tag_z_indroneframe = 0;
double tag_yaw_incameraframe = 0;

double landing_threshold_36h11 = 0.3;
double reland_height_min_threshold = 1.3;
double reland_height_max_threshold = 2.5;

bool relanding = false;

int flight_status;


std_msgs::Bool landing_condition_met_msg;
std_msgs::Bool relanding_condition_met_msg;

std_msgs::Float64 x_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 z_state_msg;


std_msgs::Float64 setpoint_x_msg;
std_msgs::Float64 setpoint_y_msg;
std_msgs::Float64 setpoint_z_msg;
std_msgs::Float64 setpoint_yaw_msg;
std_msgs::Float64 yaw_state_msg;

//msg for debug
std_msgs::Float64 tag_x_indroneframe_msg;
std_msgs::Float64 tag_y_indroneframe_msg;
std_msgs::Float64 tag_z_indroneframe_msg;
std_msgs::Float64 tag_yaw_incameraframe_msg;

Tag *tag_36h11_6;

std::vector<int> found_tag_id;
std::vector<double> yaw_errors;

std::string tag_36h11_detection_topic;

bool found_36h11 = false;

bool first_start = true;

bool landing_enabled = true;

#ifdef APRILTAGS
#include <apriltags/AprilTagDetections.h>

void apriltags36h11Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 6)
      {
        tag_36h11_6->updateTagState((*it).pose);
      }
      
    }
    found_36h11 = true;

  }
}

#else

#include <apriltags_ros/AprilTagDetectionArray.h>

void apriltags36h11Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    tag_36h11_6->setMissing();
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id == 6)
      {
        tag_36h11_6->updateTagState((*it).pose.pose);
      }
      
    }

    found_36h11 = true;

  }
}

#endif

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

void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
  landing_enabled = landing_enable_msg.data;

}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

void descend()
{
  if(landing_enabled)
  {
    landing_condition_met_msg.data = true;
    landing_condition_met_pub.publish(landing_condition_met_msg);

    relanding_condition_met_msg.data = false;
    relanding_condition_met_pub.publish(relanding_condition_met_msg);

    relanding = false;
    std::cout << "descending" << std::endl;
    ROS_DEBUG_THROTTLE(2, "Descending");
  }
  
}

void ascend()
{
  if(landing_enabled)
  {
    landing_condition_met_msg.data = false;
    landing_condition_met_pub.publish(landing_condition_met_msg);

    relanding_condition_met_msg.data = true;
    relanding_condition_met_pub.publish(relanding_condition_met_msg);

    relanding = true;
    ROS_DEBUG_THROTTLE(2, "Ascending");
    std::cout << "ascending" << std::endl;
  }
}

void hover()
{
  landing_condition_met_msg.data = false;
  landing_condition_met_pub.publish(landing_condition_met_msg);

  relanding_condition_met_msg.data = false;
  relanding_condition_met_pub.publish(relanding_condition_met_msg);

  relanding = false;
  std::cout << "hovering" << std::endl;
}

void print_parameters()
{
  ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
  ROS_INFO("landing_threshold_36h11: %f", landing_threshold_36h11);
  ROS_INFO("reland_height_min_threshold: %f", reland_height_min_threshold);
  ROS_INFO("reland_height_max_threshold: %f", reland_height_max_threshold);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "m100_track_setpoint");
  ROS_INFO("Starting position track setpoint publisher");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/apriltags/detections");
  node_priv.param<double>("landing_threshold_36h11", landing_threshold_36h11, 0.3);
  node_priv.param<double>("reland_height_min_threshold", reland_height_min_threshold, 1.3);
  node_priv.param<double>("reland_height_max_threshold", reland_height_max_threshold, 2.5);

  print_parameters();

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_x", 10);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_y", 10);
  setpoint_z_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_z", 10);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/m100_track/setpoint_yaw", 10);
  landing_condition_met_pub = nh.advertise<std_msgs::Bool>("/m100_track/landing_condition_met", 10);
  relanding_condition_met_pub = nh.advertise<std_msgs::Bool>("/m100_track/relanding_condition_met", 10);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/yaw_state", 10);
  x_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/x_state", 10);
  y_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/y_state", 10);
  z_state_pub = nh.advertise<std_msgs::Float64>("/m100_track/z_state", 10);
  
  //publish msg for debug
  tag_x_indroneframe_pub = nh.advertise<std_msgs::Float64>("/m100_track/tag_x_indroneframe", 10);
  tag_y_indroneframe_pub = nh.advertise<std_msgs::Float64>("/m100_track/tag_y_indroneframe", 10);
  tag_z_indroneframe_pub = nh.advertise<std_msgs::Float64>("/m100_track/tag_z_indroneframe", 10);
  tag_yaw_incameraframe_pub = nh.advertise<std_msgs::Float64>("/m100_track/tag_yaw_incameraframe", 10);


  apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1, apriltags36h11Callback);
  gimbal_ori_sub = nh.subscribe("/dji_sdk/gimbal", 1000, gimbalOrientationCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  landing_enable_sub = nh.subscribe("/m100_track/landing_enable", 1, landingEnableCallback ); 
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attitudeQuaternionCallback ); 
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  global_position_sub = nh.subscribe("/dji_sdk/global_position", 10, globalPositionCallback);

  tag_36h11_6 = new Tag();
  tag_36h11_6->setToLandingCenterTranslation(Eigen::Vector3d(0.0,  0.485, 0.0));

  //camera to gimbal rotation
  Eigen::Matrix3d camera_to_gimbal_rotation, camera_to_inneraxis_rotation, base_to_drone_rotation;
  camera_to_gimbal_rotation << 0, 0, 1,
                                     1, 0, 0,
                                     0, 1, 0;
  camera_to_inneraxis_rotation << 0, 0, 1,
  										1, 0, 0,
  										0, 1, 0;
  base_to_drone_rotation << 1, 0, 0,
  								  0, 1, 0,
  								  0, 0, -1;										

  Eigen::AngleAxisd yawAngle, outeraxis_to_base_yawAngle;
  Eigen::AngleAxisd pitchAngle, inneraxis_to_mediumaxis_pitchAngle;
  Eigen::AngleAxisd rollAngle, mediumaxis_to_outeraxis_rollAngle;

  Eigen::Quaternion<double> q;

  //camera to drone rotation
  Eigen::Matrix3d camera_to_drone_rotation, camera_to_base_rotation;

  double yaw_sum = 0;
  double average_yaw_error = 0;
  double found_tag_count = 0;
  double tag_36h11_yaw_state;

  Eigen::Vector3d landing_center_position_sum(0, 0, 0);
  Eigen::Vector3d average_landing_center_position(0, 0, 0);
  Eigen::Vector3d landing_center_position;

  //ros::Rate loop_rate(200); 
  ros::Rate loop_rate(200);

  ros::spinOnce();
  if(!found_36h11)
  {
    setpoint_x = local_x;
    setpoint_y = local_y;
  }

  while (ros::ok())
  {
    ros::spinOnce();

    if(flight_status != 3)
    {
      continue;
      // std::cout << flight_status << std::endl;
    }

    if(found_36h11)
    {
      ROS_DEBUG("Setpoint node: Found Apriltag");
      first_start = false;
      //yawAngle = Eigen::AngleAxisd(-gimbal_yaw, Eigen::Vector3d::UnitZ());
      //pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
      //rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());

      //q = rollAngle * pitchAngle * yawAngle;
      //camera_to_drone_rotation = q.matrix() * camera_to_gimbal_rotation;
      
      inneraxis_to_mediumaxis_pitchAngle = Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY());
      mediumaxis_to_outeraxis_rollAngle = Eigen::AngleAxisd(gimbal_roll, Eigen::Vector3d::UnitX());
      outeraxis_to_base_yawAngle = Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ());
      
      //q = inneraxis_to_mediumaxis_pitchAngle * mediumaxis_to_outeraxis_rollAngle * outeraxis_to_base_yawAngle;
      //q = outeraxis_to_base_yawAngle * mediumaxis_to_outeraxis_rollAngle * inneraxis_to_mediumaxis_pitchAngle;

      //camera_to_base_rotation = q.matrix() * camera_to_inneraxis_rotation;

      camera_to_base_rotation = outeraxis_to_base_yawAngle.matrix() * mediumaxis_to_outeraxis_rollAngle.matrix() * inneraxis_to_mediumaxis_pitchAngle.matrix() * camera_to_inneraxis_rotation;
      camera_to_drone_rotation = base_to_drone_rotation * camera_to_base_rotation;

      //get the averaging landing center position
      landing_center_position_sum = Eigen::Vector3d(0, 0, 0);
      found_tag_count = 0;  
      
      found_tag_count ++;
      tag_36h11_6->calculateDroneFramePosition(camera_to_drone_rotation);
      tag_36h11_6->calculateDroneFrameOrientation(camera_to_drone_rotation);
      landing_center_position_sum += tag_36h11_6->getLandingCenterPosition();

        
      average_landing_center_position = landing_center_position_sum / found_tag_count;

      //get the averaging landing center yaw error
      yaw_sum = 0;
      yaw_errors.clear();

      yaw_errors.push_back(tag_36h11_6->getYawError());
        
      if(yaw_errors.size() > 0)
      {
        //Take the median as an estimator
        std::sort (yaw_errors.begin(), yaw_errors.end());
        average_yaw_error = yaw_errors[found_tag_count / 2] / M_PI * 180;
      }


      // if(fabs(average_landing_center_position(0)) < landing_threshold_36h11 && fabs(average_landing_center_position(1)) < landing_threshold_36h11)
      // {
      //     descend();
      // }
      // else
      // {
    	//   hover();
      // }
      
      //publish msg for debug
      tag_x_indroneframe = average_landing_center_position(0);
      tag_y_indroneframe = average_landing_center_position(1);
      tag_z_indroneframe = average_landing_center_position(2);
      tag_x_indroneframe_msg.data = tag_x_indroneframe;
      tag_y_indroneframe_msg.data = tag_y_indroneframe;
      tag_z_indroneframe_msg.data = tag_z_indroneframe;
      tag_x_indroneframe_pub.publish(tag_x_indroneframe_msg);
      tag_y_indroneframe_pub.publish(tag_y_indroneframe_msg);
      tag_z_indroneframe_pub.publish(tag_z_indroneframe_msg);
      
      tag_yaw_incameraframe = tag_36h11_6->getYaw_incameraframe();
      tag_yaw_incameraframe_msg.data = tag_yaw_incameraframe;
      tag_yaw_incameraframe_pub.publish(tag_yaw_incameraframe_msg);
      

      setpoint_yaw = yaw_state + average_yaw_error - 90;
      setpoint_yaw_msg.data = setpoint_yaw;
      setpoint_yaw_pub.publish(setpoint_yaw_msg);

      setpoint_x = average_landing_center_position(0) + local_x;
      setpoint_y = average_landing_center_position(1) + local_y;
      setpoint_z = average_landing_center_position(2) + local_z;

      setpoint_x_msg.data = setpoint_x;
      setpoint_y_msg.data = setpoint_y;
      setpoint_z_msg.data = setpoint_z;

      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
      setpoint_z_pub.publish(setpoint_z_msg);

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
    }
    else
    {

      if(relanding)
      {
        if(flight_height < reland_height_max_threshold)
        {
          ascend();
        }
        else
        {
          hover();
        }
      }
      else
      {
        hover();
      }

      if(first_start)
      {
        setpoint_x_msg.data = local_x;
        setpoint_y_msg.data = local_y;
        setpoint_z_msg.data = local_z;
        setpoint_yaw_msg.data = yaw_state;
        yaw_state_msg.data = yaw_state;
        
      }
      else
      {
        setpoint_x_msg.data = setpoint_x;
        setpoint_y_msg.data = setpoint_y;
        setpoint_z_msg.data = setpoint_z;
        //setpoint_x_msg.data = tag_x_indroneframe + local_x;
        //setpoint_y_msg.data = tag_y_indroneframe + local_y;
        setpoint_yaw_msg.data = yaw_state;
        yaw_state_msg.data = yaw_state;
      }
      
      //publish msg for debug
      tag_x_indroneframe_msg.data = tag_x_indroneframe;
      tag_y_indroneframe_msg.data = tag_y_indroneframe;
      tag_z_indroneframe_msg.data = tag_z_indroneframe;
      tag_x_indroneframe_pub.publish(tag_x_indroneframe_msg);
      tag_y_indroneframe_pub.publish(tag_y_indroneframe_msg);
      tag_z_indroneframe_pub.publish(tag_z_indroneframe_msg);
      tag_yaw_incameraframe_msg.data = tag_yaw_incameraframe;
      tag_yaw_incameraframe_pub.publish(tag_yaw_incameraframe_msg);
      

      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
      setpoint_z_pub.publish(setpoint_z_msg);
      setpoint_yaw_pub.publish(setpoint_yaw_msg);
      yaw_state_pub.publish(yaw_state_msg);

      x_state_msg.data = local_x;
      y_state_msg.data = local_y;
      z_state_msg.data = flight_height;

      x_state_pub.publish(x_state_msg); 
      y_state_pub.publish(y_state_msg); 
      z_state_pub.publish(z_state_msg); 
    }

    loop_rate.sleep();
  }
}
