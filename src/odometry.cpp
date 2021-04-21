#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define T_s 0.1

class Odometry{
    public:
        Odometry() {

            if (! n.getParam("x0", x_k)) {
                ROS_INFO("Error retrieving paramater x.");
            };

            if (! n.getParam("y0", y_k)) {
                ROS_INFO("Error retrieving paramater y.");
            };

            if (! n.getParam("theta0", theta_k)) {
                ROS_INFO("Error retrieving paramater theta.");
            };

            sub = n.subscribe("/twist", 1000, &Odometry::callback, this);

            odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);

        }

    
    void euler(const geometry_msgs::TwistStampedConstPtr& msg, double V_x, double omega, double time){

        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + V_x*time*cos(theta_k);
        y_k1 = y_k + V_x*time*sin(theta_k);

    }

<<<<<<< HEAD
    void kutta(const geometry_msgs::TwistStampedConstPtr& msg){
        
=======
    void kutta(const geometry_msgs::TwistStampedConstPtr& msg, double V_x, double omega, double time){

        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + V_x*time*cos(theta_k + omega*time/2);
        y_k1 = y_k + V_x*time*sin(theta_k + omega*time/2);

>>>>>>> 60e2b3efe89e23b33e1a81ea01783e102358cd2e
    }

    void callback(const geometry_msgs::TwistStampedConstPtr& msg){

        double V_x = msg -> twist.linear.x;
        double omega = msg -> twist.angular.z;
        double time = msg -> header.stamp.toSec();
        double delta_time = time - prv_time;

        euler(msg, V_x, omega, delta_time);

        // ROS_INFO("TEMPO: %f", delta_time);
        // ROS_INFO("X(k+1): %f", x_k1);

        odo_msg.child_frame_id = "world";
        odo_msg.header.frame_id = "robot_frame";
        odo_msg.header.stamp = ros::Time::now();
        odo_msg.pose.pose.orientation.z = theta_k1;
        odo_msg.pose.pose.position.x = x_k1;
        odo_msg.pose.pose.position.y = y_k1;
        

        odometry.publish(odo_msg);

        theta_k = theta_k1;
        x_k = x_k1;
        y_k = y_k1;
        prv_time = time;

    }

    private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odometry;
    nav_msgs::Odometry odo_msg; 
    int euler_kutta;
    double x_k;
    double y_k;
    double theta_k;
    double x_k1;
    double y_k1;
    double theta_k1;
    double prv_time = ros::Time::now().toSec();
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    
    Odometry odo;

    ros::spin();

    return 0;
}
