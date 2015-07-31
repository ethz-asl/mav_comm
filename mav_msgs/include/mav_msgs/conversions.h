/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef MAV_MSGS_CONVERSIONS_H
#define MAV_MSGS_CONVERSIONS_H

#include <deque>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_msgs/Actuators.h"
#include "mav_msgs/AttitudeThrust.h"
#include "mav_msgs/eigen_mav_msgs.h"
#include "mav_msgs/RateThrust.h"
#include "mav_msgs/RollPitchYawrateThrust.h"

namespace mav_msgs {

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& msg) {
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

/**
 * \brief Extracts the yaw part from a quaternion, using RPY / euler (z-y'-z'') angles.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 */
template<class T>
T yawFromQuaternion(const Eigen::Quaternion<T> & q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline void eigenAttitudeThrustFromMsg(const AttitudeThrust& msg,
                                       EigenAttitudeThrust* attitude_thrust) {
  assert(attitude_thrust != NULL);

  attitude_thrust->attitude = quaternionFromMsg(msg.attitude);
  attitude_thrust->thrust = msg.thrust.z;
}

inline void eigenActuatorsFromMsg(const Actuators& msg, EigenActuators* actuators) {
  assert(actuators != NULL);

  actuators->angular_velocities.resize(msg.angular_velocities.size());
  for (unsigned int i = 0; i < msg.angular_velocities.size(); ++i) {
    actuators->angular_velocities[i] = msg.angular_velocities[i];
  }
}

inline void eigenRateThrustFromMsg(const RateThrust& msg,
                                   EigenRateThrust* rate_thrust) {
  assert(rate_thrust != NULL);

  rate_thrust->angular_rates = vector3FromMsg(msg.angular_rates);
  rate_thrust->thrust = msg.thrust.z;
}

inline void eigenRollPitchYawrateThrustFromMsg(
    const RollPitchYawrateThrust& msg,
    EigenRollPitchYawrateThrust* roll_pitch_yawrate_thrust) {
  assert(roll_pitch_yawrate_thrust != NULL);

  roll_pitch_yawrate_thrust->roll = msg.roll;
  roll_pitch_yawrate_thrust->pitch = msg.pitch;
  roll_pitch_yawrate_thrust->yaw_rate = msg.yaw_rate;
  roll_pitch_yawrate_thrust->thrust = msg.thrust.z;
}

inline void eigenOdometryFromMsg(const nav_msgs::Odometry& msg, EigenOdometry* odometry) {
  assert(odometry != NULL);
  odometry->timestamp_ns = msg.header.stamp.toNSec();
  odometry->position = mav_msgs::vector3FromPointMsg(msg.pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg.pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg.twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg.twist.twist.angular);
}

inline void eigenTrajectoryPointFromMultiDofJointTrajectoryPointMsg(
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
  trajectory_point->yaw = yawFromQuaternion(quaternionFromMsg(msg.transforms[0].rotation));
  trajectory_point->yaw_rate = msg.velocities[0].angular.z;
}

inline void msgActuatorsFromEigen(const EigenActuators& actuators, Actuators* msg) {
  assert(msg != NULL);
  msg->angular_velocities.resize(actuators.angular_velocities.size());

  for (unsigned int i = 0; i < actuators.angular_velocities.size(); ++i) {
    msg->angular_velocities[i] = actuators.angular_velocities[i];
  }
}


inline void msgAttitudeThrustFromEigen(const EigenAttitudeThrust& attitude_thrust,
                                       AttitudeThrust* msg) {
  assert(msg != NULL);
  tf::quaternionEigenToMsg(attitude_thrust.attitude, msg->attitude);
  msg->thrust.z = attitude_thrust.thrust;
}

inline void msgRateThrustFromEigen(EigenRateThrust& rate_thrust,
                                   RateThrust* msg) {
  assert(msg != NULL);
  tf::vectorEigenToMsg(rate_thrust.angular_rates, msg->angular_rates);
  msg->thrust.z = rate_thrust.thrust;
}

inline void msgRollPitchYawrateThrustFromEigen(
    const EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust,
    RollPitchYawrateThrust* msg) {
  assert(msg != NULL);
  msg->roll = roll_pitch_yawrate_thrust.roll;
  msg->pitch = roll_pitch_yawrate_thrust.pitch;
  msg->yaw_rate = roll_pitch_yawrate_thrust.yaw_rate;
  msg->thrust.z = roll_pitch_yawrate_thrust.thrust;
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
  tf::quaternionEigenToMsg(quaternionFromYaw(trajectory_point.yaw), msg->transforms[0].rotation);
  tf::vectorEigenToMsg(trajectory_point.velocity, msg->velocities[0].linear);
  msg->velocities[0].angular.z = trajectory_point.yaw_rate;
  tf::vectorEigenToMsg(trajectory_point.acceleration, msg->accelerations[0].linear);
}

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
