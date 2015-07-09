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

#include <Eigen/StdVector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_msgs/CommandAttitudeThrust.h"
#include "mav_msgs/CommandMotorSpeed.h"
#include "mav_msgs/CommandRateThrust.h"
#include "mav_msgs/CommandRollPitchYawrateThrust.h"
#include "mav_msgs/CommandTrajectoryPositionYaw.h"
#include "mav_msgs/eigen_mav_msgs.h"

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

inline void eigenCommandAttitudeThrustFromMsg(const CommandAttitudeThrust& msg,
                                              EigenCommandAttitudeThrust* command_attitude_thrust) {
  assert(command_attitude_thrust != NULL);

  command_attitude_thrust->attitude = quaternionFromMsg(msg.attitude);
  command_attitude_thrust->thrust = msg.thrust;
}

inline void eigenCommandMotorSpeedFromMsg(const CommandMotorSpeed& msg,
                                          EigenCommandMotorSpeed* command_motor_speed) {
  assert(command_motor_speed != NULL);

  command_motor_speed->motor_speeds.resize(msg.motor_speed.size());
  for (unsigned int i = 0; i < msg.motor_speed.size(); ++i) {
    command_motor_speed->motor_speeds[i] = msg.motor_speed[i];
  }
}

inline void eigenCommandRateThrustFromMsg(const CommandRateThrust& msg,
                                          EigenCommandRateThrust* command_rate_thrust) {
  assert(command_rate_thrust != NULL);

  command_rate_thrust->angular_rates = vector3FromMsg(msg.angular_rates);
  command_rate_thrust->thrust = msg.thrust;
}

inline void eigenCommandRollPitchYawrateThrustFromMsg(
    const CommandRollPitchYawrateThrust& msg,
    EigenCommandRollPitchYawrateThrust* command_roll_pitch_yawrate_thrust) {
  assert(command_roll_pitch_yawrate_thrust != NULL);

  command_roll_pitch_yawrate_thrust->roll = msg.roll;
  command_roll_pitch_yawrate_thrust->pitch = msg.pitch;
  command_roll_pitch_yawrate_thrust->yaw_rate = msg.yaw_rate;
  command_roll_pitch_yawrate_thrust->thrust = msg.thrust;
}

inline void eigenCommandTrajectoryPositionYawFromMsg(const CommandTrajectoryPositionYaw& msg,
                                                     EigenCommandTrajectoryPositionYaw* command_trajectory) {
  assert(command_trajectory != NULL);

  command_trajectory->position = vector3FromMsg(msg.position);
  command_trajectory->velocity = vector3FromMsg(msg.velocity);
  command_trajectory->acceleration = vector3FromMsg(msg.acceleration);
  command_trajectory->jerk = vector3FromMsg(msg.jerk);
  command_trajectory->snap = vector3FromMsg(msg.snap);
  command_trajectory->yaw = msg.yaw;
  command_trajectory->yaw_rate = msg.yaw_rate;
}

inline void eigenOdometryFromMsg(const nav_msgs::Odometry& msg, EigenOdometry* odometry) {
  assert(odometry != NULL);
  odometry->position = mav_msgs::vector3FromPointMsg(msg.pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg.pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg.twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg.twist.twist.angular);
}

inline void eigenCommandTrajectoryPositionYawFromMultiDofJointTrajectoryPointMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
    EigenCommandTrajectoryPositionYaw* command_trajectory) {
  assert(command_trajectory != NULL);

  if (msg.transforms.size() > 1) {
    ROS_WARN("MultiDofJointTrajectoryPoint message should have one joint, but has %lu. Using first joint.",
             msg.transforms.size());
  }

  if (msg.transforms.empty()) {
    ROS_ERROR("MultiDofJointTrajectoryPoint is empty.");
    return;
  }

  command_trajectory->position = vector3FromMsg(msg.transforms[0].translation);
  command_trajectory->velocity = vector3FromMsg(msg.velocities[0].linear);
  command_trajectory->acceleration = vector3FromMsg(msg.accelerations[0].linear);
  command_trajectory->jerk.setZero();
  command_trajectory->snap.setZero();
  command_trajectory->yaw = yawFromQuaternion(quaternionFromMsg(msg.transforms[0].rotation));
  command_trajectory->yaw_rate = msg.velocities[0].angular.z;
}

#define MAV_MSGS_CONCATENATE(x, y) x ## y
#define MAV_MSGS_CONCATENATE2(x, y) MAV_MSGS_CONCATENATE(x, y)
#define MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE) \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Vector); \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Deque); \

MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenCommandAttitudeThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenCommandMotorSpeed)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenCommandRateThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenCommandTrajectoryPositionYaw)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenCommandRollPitchYawrateThrust)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenOdometry)

} // end namespace mav_msgs

#endif // MAV_MSGS_CONVERSIONS_H
