#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <robotics_first/IntegrationConfig.h>
#include "std_srvs/Empty.h"

class Odometry{
    public:
        Odometry(){

            if (! n.getParam("x0", x_k)) {
                ROS_INFO("Error retrieving paramater x.");
            };

            if (! n.getParam("y0", y_k)) {
                ROS_INFO("Error retrieving paramater y.");
            };

            if (! n.getParam("theta0", theta_k)) {
                ROS_INFO("Error retrieving paramater theta.");
            };

            f = boost::bind(&Odometry::setEulerKutta, this, _1, _2);
            server.setCallback(f);

            sub = n.subscribe("/twist", 1000, &Odometry::callback, this);

            odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);

            n.advertiseService("reset_odometry", &Odometry::resetOdometry, this);

        }

    
    void euler(const geometry_msgs::TwistStampedConstPtr& msg, double V_x, double omega, double time){

        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + V_x*time*cos(theta_k);
        y_k1 = y_k + V_x*time*sin(theta_k);

    }

    void kutta(const geometry_msgs::TwistStampedConstPtr& msg, double V_x, double omega, double time){

        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + V_x*time*cos(theta_k + omega*time/2);
        y_k1 = y_k + V_x*time*sin(theta_k + omega*time/2);

    }

    void callback(const geometry_msgs::TwistStampedConstPtr& msg){

        double V_x = msg -> twist.linear.x;
        double omega = msg -> twist.angular.z;
        double time = msg -> header.stamp.toSec();
        double delta_time = time - prv_time;

        if (euler_kutta == 0){
            ROS_INFO("EULER");
            euler(msg, V_x, omega, delta_time);
        } else {
            ROS_INFO("KUTTA");
            kutta(msg, V_x, omega, delta_time);
        };

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

    bool resetOdometry(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){

        x_k = 0;
        y_k = 0;
        theta_k = 0;
        ROS_INFO("Odometry setted at 0.");
        return true;

    }

    void setEulerKutta(robotics_first::IntegrationConfig &config, uint32_t level){

        ROS_INFO("Reconfigure Request: %d", config.method);

        euler_kutta = config.method;

    }

    private:
    ros::NodeHandle n;
    ros::Publisher odometry;
    ros::ServiceServer service;
    ros::Subscriber sub;
    nav_msgs::Odometry odo_msg; 
    dynamic_reconfigure::Server<robotics_first::IntegrationConfig> server;
    dynamic_reconfigure::Server<robotics_first::IntegrationConfig>::CallbackType f;
    int euler_kutta = 0;
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
