#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/Float64.h"

#include "robotics_first/MotorSpeed.h"

void callback(const robotics_first::MotorSpeedConstPtr& msg1, 
              const robotics_first::MotorSpeedConstPtr& msg2,
              const ros::Publisher sync_left,
              const ros::Publisher sync_right) {

    ROS_INFO ("Received two messages: (%f,%f)", 
        msg1->rpm, msg2->rpm);
    sync_left.publish(msg1);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;

    ros::Publisher sync_left = n.advertise<robotics_first::MotorSpeed>("sync_left", 1000);
    ros::Publisher sync_right = n.advertise<robotics_first::MotorSpeed>("sync_right", 1000);

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub3(n, "motr_speed_rl", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, sync_left, sync_right));

    ros::spin();

    return 0;
}
