#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>

#include "robotics_first/MotorSpeed.h"

void callback(const geometry_msgs::TwistStampedConstPtr& msg){
    ROS_INFO("I heard: [%f]", msg->twist.linear.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("twist", 1000, callback);

    ros::spin();

    return 0;
}
