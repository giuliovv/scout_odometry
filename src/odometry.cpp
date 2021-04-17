#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "robotics_first/MotorSpeed.h"

void callback(const robotics_first::MotorSpeedConstPtr& msg1, 
              const robotics_first::MotorSpeedConstPtr& msg2,
              const ros::Publisher odometry) {

    ROS_INFO ("Received two messages: (%f,%f)", 
        msg1->rpm, msg2->rpm);
    odometry.publish(msg1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;

    ros::Publisher odometry = n.advertise<robotics_first::MotorSpeed>("odometry", 1000);

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub3(n, "motr_speed_rl", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, odometry));

    ros::spin();

    return 0;
}
