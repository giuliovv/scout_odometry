#include "ros/ros.h"
#include <message_filters/subscriber.h>

#include "robotics_first/MotorSpeed.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "sync_left", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "sync_right", 1);

    ros::spin();

    return 0;
}
