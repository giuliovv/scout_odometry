#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "robotics_first/MotorSpeed.h"

#define R 0.1575

// float old = 0;
// int oldtempo = 1;

void callback(const robotics_first::MotorSpeedConstPtr& left, 
              const robotics_first::MotorSpeedConstPtr& right,
              const nav_msgs::OdometryConstPtr& odo,
              const ros::Publisher appa_baseline) {

    // ROS_INFO("Left RPM: %f", left -> rpm);

    float v_left = (left->rpm) * 2 * M_PI * R / 60;
    float v_right = (right->rpm) * 2 * M_PI * R / 60;

    float v_x = (v_left + v_right)/2;

    float w = odo->twist.twist.angular.z;
    std_msgs::Float64 appa;
    appa.data = (-v_left+v_right)/w;

    float v_x_read = odo->twist.twist.linear.x;

    // float v_x_ground = (scout -> pose.position.x - old)/(scout -> header.stamp.sec - oldtempo);

    // ROS_INFO("APPARENT: (%f, %f)", w, appa.data);

    // ROS_INFO("Vx: (%f, %f)", v_x, v_x_ground);
    ROS_INFO("Rapporto: %f", v_x/v_x_read);

    // ROS_INFO("tempo %f", scout->pose.position.x);

    // old = scout -> pose.position.x;
    // oldtempo = scout -> header.stamp.sec;

    appa_baseline.publish(appa);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "baseline_calculator");

    ros::NodeHandle n;

    ros::Publisher appa_baseline = n.advertise<std_msgs::Float64>("apparent_baseline", 1000);

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "motor_speed_rl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "motor_speed_rr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub3(n, "scout_odom", 1);
    // message_filters::Subscriber<geometry_msgs::PoseStamped> sub4(n, "gt_pose", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed,
                                        nav_msgs::Odometry
                                        // geometry_msgs::PoseStamped
                                        > sync(sub1, sub2, sub3, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, appa_baseline));

    ros::spin();

    return 0;
}
