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
#define ALPHA 0.1

void callback(
            // const robotics_first::MotorSpeedConstPtr& rear_left, 
            //   const robotics_first::MotorSpeedConstPtr& rear_right,
              const robotics_first::MotorSpeedConstPtr& front_left, 
              const robotics_first::MotorSpeedConstPtr& front_right,
              const nav_msgs::OdometryConstPtr& odo,
              float& sum_of_gear_ratios,
              int& iteration_number,
              float& ema_apparent_baselines,
              int& apparent_baseline_iteration_number) {

    float v_left_no_gear_ratio = (front_left->rpm) * 2 * M_PI * R / 60;
    float v_right_no_gear_ratio = (front_right->rpm) * 2 * M_PI * R / 60;

    float v_x_no_gear_ratio = (- v_left_no_gear_ratio + v_right_no_gear_ratio)/2;

    float w = odo->twist.twist.angular.z;
    float v_x_read = odo->twist.twist.linear.x;

    float gear_ratio = v_x_no_gear_ratio/v_x_read;

    if (v_x_read != 0){
        sum_of_gear_ratios += gear_ratio;
        iteration_number +=1;

        float mean_gear_ratio = sum_of_gear_ratios / iteration_number;

        ROS_INFO("Gear ratio: %f", mean_gear_ratio);

        float v_left = v_left_no_gear_ratio / mean_gear_ratio;
        float v_right = v_right_no_gear_ratio / mean_gear_ratio;
        double apparent_baseline;
        apparent_baseline = (v_left+v_right)/w;

        if (v_left/v_right < 1.2 && v_left/v_right > 0.8 && v_left * v_right > 0){
            // Exponential moving average
            ema_apparent_baselines = (ALPHA * apparent_baseline) + (1.0 - ALPHA) * ema_apparent_baselines;
            apparent_baseline_iteration_number += 1;
            ROS_INFO("Vl: %f Vr: %f, w: %f, mean apparent baseline: %f", v_left, v_right, w, ema_apparent_baselines);
        }
    }

}

int main(int argc, char** argv) {

    float sum_of_gear_ratios = 0;
    int iteration_number = 0;

    float ema_apparent_baselines = 0;
    int apparent_baseline_iteration_number = 0;

    ros::init(argc, argv, "baseline_calculator");

    ros::NodeHandle n;

    // message_filters::Subscriber<robotics_first::MotorSpeed> rl(n, "motor_speed_rl", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> rr(n, "motor_speed_rr", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> fl(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> fr(n, "motor_speed_fr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom(n, "scout_odom", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed,
                                        // robotics_first::MotorSpeed,
                                        // robotics_first::MotorSpeed,
                                        nav_msgs::Odometry
                                        > sync(fl, fr, odom, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3,
                                         sum_of_gear_ratios, iteration_number,
                                         ema_apparent_baselines, apparent_baseline_iteration_number));

    ros::spin();

    return 0;
}
