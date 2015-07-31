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

// Conversion functions between Eigen types and MAV ROS message types.

#ifndef MAV_MSGS_CONVERSIONS_H
#define MAV_MSGS_CONVERSIONS_H

#include <deque>

#include <Eigen/StdVector>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_msgs/Actuators.h"
#include "mav_msgs/AttitudeThrust.h"
#include "mav_msgs/common.h"
#include "mav_msgs/eigen_mav_msgs.h"
#include "mav_msgs/RateThrust.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "mav_msgs/TorqueThrust.h"

namespace mav_msgs {

inline void eigenAttitudeThrustFromMsg(const AttitudeThrust& msg,
                                       EigenAttitudeThrust* attitude_thrust) {
  assert(attitude_thrust != NULL);

  attitude_thrust->attitude = quaternionFromMsg(msg.attitude);
  attitude_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenActuatorsFromMsg(const Actuators& msg, EigenActuators* actuators) {
  assert(actuators != NULL);

  // Positions.
  actuators->angles.resize(msg.angles.size());
  for (unsigned int i = 0; i < msg.angles.size(); ++i) {
    actuators->angles[i] = msg.angles[i];
  }

  // Angular velocities.
  actuators->angular_velocities.resize(msg.angular_velocities.size());
  for (unsigned int i = 0; i < msg.angular_velocities.size(); ++i) {
    actuators->angular_velocities[i] = msg.angular_velocities[i];
  }

  // Normalized.
  actuators->normalized.resize(msg.normalized.size());
  for (unsigned int i = 0; i < msg.normalized.size(); ++i) {
    actuators->normalized[i] = msg.normalized[i];
  }
}

inline void eigenRateThrustFromMsg(const RateThrust& msg,
                                   EigenRateThrust* rate_thrust) {
  assert(rate_thrust != NULL);

  rate_thrust->angular_rates = vector3FromMsg(msg.angular_rates);
  rate_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenTorqueThrustFromMsg(const TorqueThrust& msg,
                                     EigenTorqueThrust* torque_thrust) {
  assert(torque_thrust != NULL);

  torque_thrust->torque = vector3FromMsg(msg.torque);
  torque_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenRollPitchYawrateThrustFromMsg(
    const RollPitchYawrateThrust& msg,
    EigenRollPitchYawrateThrust* roll_pitch_yawrate_thrust) {
  assert(roll_pitch_yawrate_thrust != NULL);

  roll_pitch_yawrate_thrust->roll = msg.roll;
  roll_pitch_yawrate_thrust->pitch = msg.pitch;
  roll_pitch_yawrate_thrust->yaw_rate = msg.yaw_rate;
  roll_pitch_yawrate_thrust->thrust = vector3FromMsg(msg.thrust);
}

inline void eigenOdometryFromMsg(const nav_msgs::Odometry& msg, EigenOdometry* odometry) {
  assert(odometry != NULL);
  odometry->timestamp_ns = msg.header.stamp.toNSec();
  odometry->position = mav_msgs::vector3FromPointMsg(msg.pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg.pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg.twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg.twist.twist.angular);
}

inline void eigenTrajectoryPointFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
    EigenTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  if (msg.transforms.empty()) {
    ROS_ERROR("MultiDofJointTrajectoryPoint is empty.");
    return;
  }

  if (msg.transforms.size() > 1) {
    ROS_WARN("MultiDofJointTrajectoryPoint message should have one joint, but has %lu. Using first joint.",
             msg.transforms.size());
  }

  trajectory_point->time_from_start_ns = msg.time_from_start.toNSec();
  trajectory_point->position = vector3FromMsg(msg.transforms[0].translation);
  trajectory_point->velocity = vector3FromMsg(msg.velocities[0].linear);
  trajectory_point->acceleration = vector3FromMsg(msg.accelerations[0].linear);
  trajectory_point->jerk.setZero();
  trajectory_point->snap.setZero();
  trajectory_point->orientation = quaternionFromMsg(msg.transforms[0].rotation);
  trajectory_point->angular_velocity = vector3FromMsg(msg.velocities[0].angular);
}

inline void msgActuatorsFromEigen(const EigenActuators& actuators, Actuators* msg) {
  assert(msg != NULL);

  msg->angles.resize(actuators.angles.size());
  for (unsigned int i = 0; i < actuators.angles.size(); ++i) {
    msg->angles[i] = actuators.angles[i];
  }

  msg->angular_velocities.resize(actuators.angular_velocities.size());
  for (unsigned int i = 0; i < actuators.angular_velocities.size(); ++i) {
    msg->angular_velocities[i] = actuators.angular_velocities[i];
  }

  msg->normalized.resize(actuators.normalized.size());
  for (unsigned int i = 0; i < actuators.normalized.size(); ++i) {
    msg->normalized[i] = actuators.normalized[i];
  }
}


inline void msgAttitudeThrustFromEigen(const EigenAttitudeThrust& attitude_thrust,
                                       AttitudeThrust* msg) {
  assert(msg != NULL);
  tf::quaternionEigenToMsg(attitude_thrust.attitude, msg->attitude);
  tf::vectorEigenToMsg(attitude_thrust.thrust, msg->thrust);
}

inline void msgRateThrustFromEigen(EigenRateThrust& rate_thrust,
                                   RateThrust* msg) {
  assert(msg != NULL);
  tf::vectorEigenToMsg(rate_thrust.angular_rates, msg->angular_rates);
  tf::vectorEigenToMsg(rate_thrust.thrust, msg->thrust);
}

inline void msgTorqueThrustFromEigen(EigenTorqueThrust& torque_thrust,
                                     TorqueThrust* msg) {
  assert(msg != NULL);
  tf::vectorEigenToMsg(torque_thrust.torque, msg->torque);
  tf::vectorEigenToMsg(torque_thrust.thrust, msg->thrust);
}

inline void msgRollPitchYawrateThrustFromEigen(
    const EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust,
    RollPitchYawrateThrust* msg) {
  assert(msg != NULL);
  msg->roll = roll_pitch_yawrate_thrust.roll;
  msg->pitch = roll_pitch_yawrate_thrust.pitch;
  msg->yaw_rate = roll_pitch_yawrate_thrust.yaw_rate;
  tf::vectorEigenToMsg(roll_pitch_yawrate_thrust.thrust, msg->thrust);
}

inline void msgOdometryFromEigen(const EigenOdometry& odometry, nav_msgs::Odometry* msg) {
  assert(msg != NULL);

  msg->header.stamp.fromNSec(odometry.timestamp_ns);
  tf::pointEigenToMsg(odometry.position, msg->pose.pose.position);
  tf::quaternionEigenToMsg(odometry.orientation, msg->pose.pose.orientation);

  tf::vectorEigenToMsg(odometry.velocity, msg->twist.twist.linear);
  tf::vectorEigenToMsg(odometry.angular_velocity, msg->twist.twist.angular);
}

inline void msgMultiDofJointTrajectoryPointFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectoryPoint* msg) {
  assert(msg != NULL);

  msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
  msg->transforms.resize(1);
  msg->velocities.resize(1);
  msg->accelerations.resize(1);

  tf::vectorEigenToMsg(trajectory_point.position, msg->transforms[0].translation);
  tf::quaternionEigenToMsg(trajectory_point.orientation, msg->transforms[0].rotation);
  tf::vectorEigenToMsg(trajectory_point.velocity, msg->velocities[0].linear);
  tf::vectorEigenToMsg(trajectory_point.angular_velocity, msg->velocities[0].angular);
  tf::vectorEigenToMsg(trajectory_point.acceleration, msg->accelerations[0].linear);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);
  trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
  msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);

  msg->joint_names.clear();
  msg->points.clear();
  msg->joint_names.push_back(link_name);
  msg->points.push_back(point_msg);
}

inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectory* msg){
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

// TODO(helenol): replaced with aligned allocator headers from Simon.
#define MAV_MSGS_CONCATENATE(x, y) x ## y
#define MAV_MSGS_CONCATENATE2(x, y) MAV_MSGS_CONCATENATE(x, y)
#define MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE) \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Vector); \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Deque); \

MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenAttitudeThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenActuators)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenRateThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPoint)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenRollPitchYawrateThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenOdometry)

} // end namespace mav_msgs

#endif // MAV_MSGS_CONVERSIONS_H
