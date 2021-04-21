#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <robotics_first/IntegrationConfig.h>

void callback(robotics_first::IntegrationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", 
            config.method);

  ROS_INFO ("%d",level);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dyn_parameter");

  dynamic_reconfigure::Server<robotics_first::IntegrationConfig> server;
  dynamic_reconfigure::Server<robotics_first::IntegrationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  ros::spin();
  return 0;
}
