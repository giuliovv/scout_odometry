#include <math.h>

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "robotics_first/CustomOdometry.h"
#include "robotics_first/IntegrationConfig.h"
#include "robotics_first/ResetToPose.h"

struct position_t {
  double x_k;
  double y_k;
  double theta_k;
  double x_k1;
  double y_k1;
  double theta_k1;
  double prv_time;
};

struct rosObjects_t {
    ros::NodeHandle node;
    ros::Publisher odometry;
    ros::Publisher custom_odometry;
    ros::Subscriber twist_subscriber;
    ros::ServiceServer reset_odom;
    ros::ServiceServer reset_odom_to_pose;

    dynamic_reconfigure::Server<robotics_first::IntegrationConfig> dynamic_server;
    std_msgs::String method_for_custom_odom;
    nav_msgs::Odometry odo_msg;
    robotics_first::CustomOdometry custom_odom_msg;
    tf2::Quaternion q;
    int euler_kutta = 0;
};


/////


void read_initial_parameters(position_t &position, rosObjects_t &rosObjects){
    // Read initial values from node parameters

    if (! rosObjects.node.getParam("x0", position.x_k)) {
        ROS_INFO("Error retrieving paramater x.");
    };

    if (! rosObjects.node.getParam("y0", position.y_k)) {
        ROS_INFO("Error retrieving paramater y.");
    };

    if (! rosObjects.node.getParam("theta0", position.theta_k)) {
        ROS_INFO("Error retrieving paramater theta.");
    };

    position.prv_time = ros::Time::now().toSec();
}


/////


void set_euler_or_kutta(robotics_first::IntegrationConfig &config,
                    uint32_t level,
                    rosObjects_t& rosObjects){

    rosObjects.euler_kutta = config.method;
    rosObjects.method_for_custom_odom.data = config.method == 0 ? "euler" : "rk";

    ROS_INFO("Reconfigure Request: %d, %s", config.method, rosObjects.method_for_custom_odom.data.c_str());

}

void start_dynamic_server(rosObjects_t &rosObjects){
    // Dynamic server to choose between integration methods.

    dynamic_reconfigure::Server<robotics_first::IntegrationConfig>::CallbackType callback_dynamic_server;

    callback_dynamic_server = boost::bind(&set_euler_or_kutta, _1, _2, boost::ref(rosObjects));
    rosObjects.dynamic_server.setCallback(callback_dynamic_server);

}


/////


void start_publishers(rosObjects_t &rosObjects){

    rosObjects.odometry = rosObjects.node.advertise<nav_msgs::Odometry>("odometry", 1000);
    rosObjects.custom_odometry = rosObjects.node.advertise<robotics_first::CustomOdometry>("custom_odometry", 1000);

}


/////


void euler(const geometry_msgs::TwistStampedConstPtr &msg,
            double V_x,
            double omega,
            double time,
            position_t &position){

    position.theta_k1 = position.theta_k + omega*time;
    position.x_k1 = position.x_k + V_x*time*cos(position.theta_k);
    position.y_k1 = position.y_k + V_x*time*sin(position.theta_k);

}

void kutta(const geometry_msgs::TwistStampedConstPtr& msg,
        double V_x,
        double omega,
        double time,
        position_t &position){

    position.theta_k1 = position.theta_k + omega*time;
    position.x_k1 = position.x_k + V_x*time*cos(position.theta_k + omega*time/2);
    position.y_k1 = position.y_k + V_x*time*sin(position.theta_k + omega*time/2);

}

void read_twist_calculate_odometry(const geometry_msgs::TwistStampedConstPtr &msg,
            position_t &position,
            rosObjects_t &rosObject){

        double V_x = msg -> twist.linear.x;
        double omega = msg -> twist.angular.z;
        double time = msg -> header.stamp.toSec();
        double delta_time = time - position.prv_time;

        if (rosObject.euler_kutta == 0){
            euler(msg, V_x, omega, delta_time, position);
        } else {
            kutta(msg, V_x, omega, delta_time, position);
        };

        rosObject.odo_msg.child_frame_id = "world";
        rosObject.odo_msg.header.frame_id = "robot_frame";
        rosObject.odo_msg.header.stamp = ros::Time::now();
        rosObject.odo_msg.pose.pose.position.x = position.x_k1;
        rosObject.odo_msg.pose.pose.position.y = position.y_k1;

        rosObject.q.setRPY(0,0,position.theta_k1);
        rosObject.odo_msg.pose.pose.orientation.x = rosObject.q.x();
        rosObject.odo_msg.pose.pose.orientation.y = rosObject.q.y();
        rosObject.odo_msg.pose.pose.orientation.z = rosObject.q.z();
        rosObject.odo_msg.pose.pose.orientation.w = rosObject.q.w();

        rosObject.odometry.publish(rosObject.odo_msg);

        rosObject.custom_odom_msg.odom = rosObject.odo_msg;
        rosObject.custom_odom_msg.method = rosObject.method_for_custom_odom;

        rosObject.custom_odometry.publish(rosObject.custom_odom_msg);

        position.theta_k = position.theta_k1;
        position.x_k = position.x_k1;
        position.y_k = position.y_k1;
        position.prv_time = time;

}

void start_subscriber(position_t &position,
                    rosObjects_t &rosObjects){

    auto callback_subscriber = boost::bind(&read_twist_calculate_odometry, _1, boost::ref(position), boost::ref(rosObjects));
    rosObjects.twist_subscriber = rosObjects.node.subscribe<geometry_msgs::TwistStamped>("/twist", 1000, callback_subscriber);

}


///// 


bool reset_odometry(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res,
                    position_t &position){

    position.x_k = 0;
    position.y_k = 0;
    position.theta_k = 0;

    ROS_INFO("Odometry setted at 0.");
    return true;

}

bool reset_odometry_to_pose(robotics_first::ResetToPose::Request &req,
                robotics_first::ResetToPose::Response &res,
                position_t &position){

    position.x_k = req.x;
    position.y_k = req.y;
    position.theta_k = req.theta;
    
    ROS_INFO("Odometry setted at x=%f y=%f theta=%f.", req.x, req.y, req.theta);
    return true;

}

void start_services(position_t &position,
                rosObjects_t &rosObjects){

    auto callback_reset_odometry = boost::bind(&reset_odometry, _1, _2, boost::ref(position));
    rosObjects.reset_odom = rosObjects.node.advertiseService<std_srvs::Empty::Request,
                                                            std_srvs::Empty::Response>
                                                            ("reset_odometry", callback_reset_odometry);
    auto callback_reset_to_pose = boost::bind(&reset_odometry_to_pose, _1, _2, boost::ref(position));
    rosObjects.reset_odom_to_pose = rosObjects.node.advertiseService<robotics_first::ResetToPose::Request,
                                                                    robotics_first::ResetToPose::Response>
                                                                    ("reset_odometry_to_pose", callback_reset_to_pose);
    
}


/////


int main(int argc, char** argv){

    ros::init(argc, argv, "odometry");

    position_t position;
    rosObjects_t rosObjects;
    
    read_initial_parameters(position, rosObjects);

    start_dynamic_server(rosObjects);

    start_publishers(rosObjects);

    start_subscriber(position, rosObjects);

    start_services(position, rosObjects);

    ros::spin();

    return 0;
}
