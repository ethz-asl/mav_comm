^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mav_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2015-08-09)
------------------
* Dropped "Command" from the names of all messages.
* Converted CommandPositionYawTrajectory message to MultiDOFJointTrajectory (with an additional option to use a geometry_msgs/PoseStamped instead) as the two ways to send trajectory/waypoint commands.
* Added conversions between the Eigen and ROS message types.
* Switched to using full orientation instead of just yaw where appropriate.
* Documented reference frame in the Eigen messages where possible.
* Contributors: Helen Oleynikova, Markus Achtelik

2.0.3 (2015-05-22)
------------------
* added install target for include
  Headers can be included outside of this package.
* Contributors: Fadri Furrer
