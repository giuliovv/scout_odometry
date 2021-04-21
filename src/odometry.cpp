#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

class odometry{
    public:
        odometry() {

            if (! n.getParam("x0", x0)) {
                ROS_INFO("Error retrieving paramater x.");
            };

            if (! n.getParam("y0", y0)) {
                ROS_INFO("Error retrieving paramater y.");
            };

            if (! n.getParam("theta0", theta0)) {
                ROS_INFO("Error retrieving paramater theta.");
            };

            ROS_INFO("%f %f %f", x0, y0, theta0);

            sub = n.subscribe("/twist", 1000, &odometry::callback, this);
        }

    
    void euler(const geometry_msgs::TwistStampedConstPtr& msg, double x, double y, double theta){

    }

    void kutta(const geometry_msgs::TwistStampedConstPtr& msg, double x, double y, double theta){

    }

    void callback(const geometry_msgs::TwistStampedConstPtr& msg){
        euler(msg, x0, y0, theta0);
    }

    private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    double x0;
    double y0;
    double theta0;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;
    
    odometry odo;

    ros::spin();

    return 0;
}
