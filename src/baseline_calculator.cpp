#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "robotics_first/MotorSpeed.h"

#define R 0.1575

void callback(const robotics_first::MotorSpeedConstPtr& left, 
              const robotics_first::MotorSpeedConstPtr& right,
              const nav_msgs::OdometryConstPtr& odo,
              const ros::Publisher appa_baseline) {

    float v_left = (left->rpm) * 2 * M_PI * R / 60;
    float v_right = (right->rpm) * 2 * M_PI * R / 60;

    float w = odo->twist.twist.linear.x;
    std_msgs::Float64 appa;
    appa.data = (-v_left+v_right)/w;

    ROS_INFO("APPARENT: (%f, %f)", w, appa);

    appa_baseline.publish(appa);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;

    ros::Publisher appa_baseline = n.advertise<std_msgs::Float64>("apparent_baseline", 1000);

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub3(n, "scout_odom", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed,
                                        nav_msgs::Odometry> sync(sub1, sub2, sub3, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, appa_baseline));

    ros::spin();

    return 0;
}
