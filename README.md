<h1 align="center">
  :robot: Agilex Scout odometry :robot:
</h1>

<p align="center">Group project for Polimi Perception, localization and mapping for mobile robots course 20/21.</p>
<p>The second part of the project, SLAM based on LIDAR and odometry sensor fusion is here: <a href="https://github.com/giuliovv/robotics_second">robotics_second</a></p>

<h3>Files:</h3>

1. <b>Bags folder</b> → contains the bags provided. 
2. <b>Cfg folder</b> → parameter.cfg that contains the configuration of the dynamic reconfigure server. 
3. <b>Launch folder</b> → robotics_first.launch is the launch file that starts everything. 
4. <b>Msg folder</b> → CustomOdometry.msg is the custom message & MotorSpeed.msg was provided to 
read the speeds from the bag.  
5. <b>Src folder</b> → agile_tf.cpp generates the tf of scout odom, baseline_calculator.cpp calculates the 
apparent baseline and the gear ratio, tf_publisher.cpp generates the tf of our odom, 
odometry_functional.cpp calculates the odom, does the dynamic reconfiguration and hosts the 
service to reset, twist.cpp synchronizes the messages of left and right front wheels. 
6. <b>Srv folder</b> → ResetToPose.srv is the definition of the service to reset to a given pose. 

<h3>Launcher:</h3>

```shell
roslaunch robotics_first robotics_first.launch
```
