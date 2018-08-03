#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ros/console.h>

ros::Publisher state_x_incameraframe_pub;
ros::Publisher state_y_incameraframe_pub;
ros::Publisher state_z_incameraframe_pub;
ros::Publisher state_roll_incameraframe_pub;
ros::Publisher state_pitch_incameraframe_pub;
ros::Publisher state_yaw_incameraframe_pub;

std_msgs::Float64 state_x_incameraframe_msg;
std_msgs::Float64 state_y_incameraframe_msg;
std_msgs::Float64 state_z_incameraframe_msg;
std_msgs::Float64 state_roll_incameraframe_msg;
std_msgs::Float64 state_pitch_incameraframe_msg;
std_msgs::Float64 state_yaw_incameraframe_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_approaching_state");
    ros::NodeHandle nh;

    state_x_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_x_incameraframe", 10);
    state_y_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_y_incameraframe", 10);
    state_z_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_z_incameraframe", 10);
    state_roll_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_roll_incameraframe", 10);
    state_pitch_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_pitch_incameraframe", 10);
    state_yaw_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/state_yaw_incameraframe", 10);

    ros::Rate loop_rate(200);
    ros::spinOnce();

    while(ros::ok())
    {
        ros::spinOnce();

        state_x_incameraframe_msg.data = 0;
        state_y_incameraframe_msg.data = 0;
        state_z_incameraframe_msg.data = 0;
        state_roll_incameraframe_msg.data = 0;
        state_pitch_incameraframe_msg.data = 0;
        state_yaw_incameraframe_msg.data = 0;

        state_x_incameraframe_pub.publish(state_x_incameraframe_msg);
        state_y_incameraframe_pub.publish(state_y_incameraframe_msg);
        state_z_incameraframe_pub.publish(state_z_incameraframe_msg);
        state_roll_incameraframe_pub.publish(state_roll_incameraframe_msg);
        state_pitch_incameraframe_pub.publish(state_pitch_incameraframe_msg);
        state_yaw_incameraframe_pub.publish(state_yaw_incameraframe_msg);

        loop_rate.sleep();
    }
}