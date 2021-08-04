<h1 align="center">
  Agilex Scout odometry 
</h1>

Group project for Polimi Perception, localization and mapping for mobile robots course 20/21.

The files inside the archive are: 
1. Bags folder → contains the bags provided. 
2. Cfg folder → parameter.cfg that contains the configuration of the dynamic reconfigure server. 
3. Launch folder → robotics_first.launch is the launch file that starts everything. 
4. Msg folder → CustomOdometry.msg is the custom message & MotorSpeed.msg was provided to 
read the speeds from the bag.  
5. Src folder → agile_tf.cpp generates the tf of scout odom, baseline_calculator.cpp calculates the 
apparent baseline and the gear ratio, tf_publisher.cpp generates the tf of our odom, 
odometry_functional.cpp calculates the odom, does the dynamic reconfiguration and hosts the 
service to reset, twist.cpp synchronizes the messages of left and right front wheels. 
6. Srv folder → ResetToPose.srv is the definition of the service to reset to a given pose. 
