TEAM MEMBERS
Valerio - 10866026
Davide Osimo - 10673132
Ainoa Palomino - 10850326

FILES DESCRIPTION:
Inside the archive there are five main files:
1) odometry.cpp  executable that contains the code of the program
2) reset_odometry.cpp  client to run the reset position service
3) wheel_speed.cpp  it computes the wheel speed from the robot velocity
4) broadcaster_tf2.cpp  controls TF data, retrieving data from /odom and broadcasting it
5) calibration.cpp  used to calibrate ‘r’ (wheel radius) and ‘N’ (encoder CPR)

PARAMETERS DESCRIPTION:
1) pos_x0, pos_y0, theta0: they are the parameters used to set the initial position.
2) int_method is the parameter that can be dynamically reconfigured to change integration method

CUSTOM MESSAGES
The custom messages we are using is the WheelSpeed that contains:
	Header header
	float64 rpm_fl
	float64 rpm_fr
	float64 rpm_rr
	float64 rpm_rl
and it is published on “/wheel_states” topic.

DYNAMIC RECONFIGURATION:
Name of the parameter to change odometry source:
- "int_method": if set to zero node computes odometry with Euler; if set to one node computes odometry with Runge-Kutta.

PARAMETERS CHANGED WITH SERVICES:
- “new_posx”, “new_posy”, “new_theta” parameter: values for pose resetting.
- "new_r", "new_N" parameter: value for robot calibration.

TF TREE STRUCTURE:
The TF tree consists of three transformation frames (world, odom and base_link):
 				world
				  |
				  |
				odom
				  |
				  |
				base_link
The transformation world->odom is static while the one odom->base_link is performed dynamically.


HOW TO PLAY THE NODES, SERVICES AND LAUNCHFILE DESCRIPTION, AND OTHER INTERESTING INFO
To launch the project, we use “roslaunch project1 project1.launch”;
for the calibration, “rosrun project1 calibration <r> <N>” (they are respectively the wheel radius and the encoder resolution);
to reset the odometry, “rosrun project1 reset_odometry <x> <y> <theta>” (where we have three float parameters to be passed to set the new pose);
and finally, for the dynamic reconfigure we use “rosrun dynamic_reconfigure dynparam set odometry int_method <i>" (with i belonging to [0,1]).
We used the ROS graphical interface rviz to visualize some of the information and plotjuggler to plot the graphs of the velocities and the odometry and compare them with the given ones.
We performed the calibration manually but in a "smart way", comparing the plots with ros tools and dynamically changing the parameters with the calibration service, without the need of killing the nodes or recompiling the code.

