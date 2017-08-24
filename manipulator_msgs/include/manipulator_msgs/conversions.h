/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Helen Oleynikova, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Conversion functions between Eigen types and manipulator ROS message types.

#ifndef MANIPULATOR_MSGS_CONVERSIONS_H
#define MANIPULATOR_MSGS_CONVERSIONS_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>

#include "eigen_manipulator_msgs.h"
#include "mav_msgs/common.h"

namespace manipulator_msgs {

inline void eigenToStdVector(const Eigen::VectorXd& eig_vec, std::vector<double>* std_vec) {
  std_vec->clear();
  std_vec->resize(eig_vec.size());
  Eigen::VectorXd::Map(&(*std_vec)[0], eig_vec.size()) = eig_vec;
}


inline void eigenJointsStateFromMsg(const std::vector<sensor_msgs::JointState> joint_state_msgs, EigenJointsState* joints_state) {
  assert(joints_state != NULL);

  joints_state->angles.resize(joint_state_msgs.size());
  joints_state->angular_rates.resize(joint_state_msgs.size());
  for (unsigned int i = 0; i < joint_state_msgs.size(); i++) {
    joints_state->angles[i] = joint_state_msgs[i].position.front();
    joints_state->angular_rates[i] = joint_state_msgs[i].velocity.front();
  }
}


inline void eigenJointTrajectoryPointFromMsg(const trajectory_msgs::JointTrajectoryPoint& msg, EigenJointTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  if (msg.positions.empty()) {
    ROS_ERROR("JointTrajectoryPoint is empty.");
    return;
  }

  size_t joints_n = msg.positions.size();

  trajectory_point->time_from_start_ns = msg.time_from_start.toNSec();

  trajectory_point->angles = Eigen::VectorXd::Map(msg.positions.data(), joints_n);
  if (msg.velocities.size() == joints_n)
    trajectory_point->angular_rates = Eigen::VectorXd::Map(msg.velocities.data(), joints_n);
  else
    trajectory_point->angular_rates.setZero(joints_n);
  if (msg.accelerations.size() == joints_n)
    trajectory_point->angular_acc = Eigen::VectorXd::Map(msg.accelerations.data(), joints_n);
  else
    trajectory_point->angular_acc.setZero(joints_n);
  if (msg.effort.size() == joints_n)
    trajectory_point->torques = Eigen::VectorXd::Map(msg.effort.data(), joints_n);
  else
    trajectory_point->torques.setZero(joints_n);
}


inline void eigenJointTrajectoryPointFromVector3(const geometry_msgs::Vector3& msg, EigenJointTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  trajectory_point->angles = mav_msgs::vector3FromMsg(msg);
  trajectory_point->angular_rates.setZero(3);
  trajectory_point->angular_acc.setZero(3);
  trajectory_point->torques.setZero(3);
}


inline void msgJointTrajectoryPointFromEigen(const EigenJointTrajectoryPoint& trajectory_point, trajectory_msgs::JointTrajectoryPoint* msg) {
  assert(msg != NULL);

  msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
  eigenToStdVector(trajectory_point.angles, &(msg->positions));
  eigenToStdVector(trajectory_point.angular_rates, &(msg->velocities));
  eigenToStdVector(trajectory_point.angular_acc, &(msg->accelerations));
  eigenToStdVector(trajectory_point.torques, &(msg->effort));
}


// Convenience method to quickly create a trajectory from a single waypoint.
inline void msgJointTrajectoryFromEigen(const EigenJointTrajectoryPoint& trajectory, const std::vector<std::string>& link_names, trajectory_msgs::JointTrajectory* msg) {
  assert(msg != NULL);

  trajectory_msgs::JointTrajectoryPoint point_msg;
  msgJointTrajectoryPointFromEigen(trajectory, &point_msg);

  msg->joint_names.clear();
  for (const auto& link_name : link_names)
    msg->joint_names.push_back(link_name);

  msg->points.clear();
  msg->points.push_back(point_msg);
}


inline void msgJointTrajectoryFromEigen(const EigenJointTrajectoryPoint& trajectory, trajectory_msgs::JointTrajectory* msg) {
  std::vector<std::string> link_names;

  for (unsigned int i = 0; i<trajectory.angles.size(); i++)
    link_names.push_back("joint_"+std::to_string(i));

  msgJointTrajectoryFromEigen(trajectory, link_names, msg);
}


inline void msgJointTrajectoryFromEigen(const EigenJointTrajectoryPointVector& trajectory, const std::vector<std::string>& link_names, trajectory_msgs::JointTrajectory* msg) {
  assert(msg != NULL);

  if (trajectory.empty()) {
    ROS_ERROR("EigenJointTrajectoryPointVector is empty.");
    return;
  }

  msg->joint_names.clear();
  for (const auto& link_name : link_names)
    msg->joint_names.push_back(link_name);

  msg->points.clear();
  for (const auto& trajectory_point : trajectory) {
    trajectory_msgs::JointTrajectoryPoint point_msg;
    msgJointTrajectoryPointFromEigen(trajectory_point, &point_msg);
    msg->points.push_back(point_msg);
  }
}


inline void msgJointTrajectoryFromEigen(const EigenJointTrajectoryPointVector& trajectory, trajectory_msgs::JointTrajectory* msg) {
  std::vector<std::string> link_names;

  for (unsigned int i = 0; i<trajectory.front().angles.size(); i++)
    link_names.push_back("joint_"+std::to_string(i));

  msgJointTrajectoryFromEigen(trajectory, link_names, msg);
}


inline void msgJointTrajectoryFromAngles(const Eigen::VectorXd& angles_desired, trajectory_msgs::JointTrajectory* msg) {
  EigenJointTrajectoryPoint trajectory_point;
  trajectory_point.angles = angles_desired;
  msgJointTrajectoryFromEigen(trajectory_point, msg);
}

} // end namespace manipulator_msgs

#endif // MANIPULATOR_MSGS_CONVERSIONS_H
