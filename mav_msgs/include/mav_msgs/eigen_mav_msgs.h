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

#ifndef MAV_MSGS_EIGEN_MAV_MSGS_H
#define MAV_MSGS_EIGEN_MAV_MSGS_H

#include <Eigen/Eigen>

#include "mav_msgs/common.h"

namespace mav_msgs {

struct EigenAttitudeThrust {
  EigenAttitudeThrust()
      : attitude(Eigen::Quaterniond::Identity()),
        thrust(Eigen::Vector3d::Zero()) {};
  EigenAttitudeThrust(const Eigen::Quaterniond& _attitude,
                      const Eigen::Vector3d& _thrust) {
    attitude = _attitude;
    thrust = _thrust;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond attitude;
  Eigen::Vector3d thrust;
};

struct EigenActuators {
  //TODO(ffurrer): Find a proper way of initializing :)

  EigenActuators(const Eigen::VectorXd& _angular_velocities) {
    angular_velocities = _angular_velocities;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd angles;             // In rad.
  Eigen::VectorXd angular_velocities; // In rad/s.
  Eigen::VectorXd normalized;         // Everything else, normalized [-1 to 1].
};

struct EigenRateThrust {
  EigenRateThrust()
      : angular_rates(Eigen::Vector3d::Zero()),
        thrust(Eigen::Vector3d::Zero()) {};

  EigenRateThrust(const Eigen::Vector3d& _angular_rates, const Eigen::Vector3d _thrust) {
    angular_rates = _angular_rates;
    thrust = _thrust;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d angular_rates;
  Eigen::Vector3d thrust;
};

struct EigenTorqueThrust {
  EigenTorqueThrust()
      : torque(Eigen::Vector3d::Zero()),
        thrust(Eigen::Vector3d::Zero()) {};

  EigenTorqueThrust(const Eigen::Vector3d& _torque, const Eigen::Vector3d _thrust) {
    torque = _torque;
    thrust = _thrust;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d torque;
  Eigen::Vector3d thrust;
};

struct EigenRollPitchYawrateThrust {
  EigenRollPitchYawrateThrust()
      : roll(0.0),
        pitch(0.0),
        yaw_rate(0.0),
        thrust(Eigen::Vector3d::Zero()) {};

  EigenRollPitchYawrateThrust(double _roll,
                              double _pitch,
                              double _yaw_rate,
                              const Eigen::Vector3d& _thrust)
      : roll(_roll),
        pitch(_pitch),
        yaw_rate(_yaw_rate),
        thrust(_thrust) {};

  double roll;
  double pitch;
  double yaw_rate;
  Eigen::Vector3d thrust;
};

struct EigenTrajectoryPoint {
  EigenTrajectoryPoint()
      : time_from_start_ns(0),
        position(Eigen::Vector3d::Zero()),
        velocity(Eigen::Vector3d::Zero()),
        acceleration(Eigen::Vector3d::Zero()),
        jerk(Eigen::Vector3d::Zero()),
        snap(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        angular_velocity(Eigen::Vector3d::Zero()) {};

  EigenTrajectoryPoint(int64_t _time_from_start_ns,
                       const Eigen::Vector3d& _position,
                       const Eigen::Vector3d& _velocity,
                       const Eigen::Vector3d& _acceleration,
                       const Eigen::Vector3d& _jerk,
                       const Eigen::Vector3d& _snap,
                       const Eigen::Quaterniond& _orientation,
                       const Eigen::Vector3d& _angular_velocity)
      : time_from_start_ns(_time_from_start_ns),
        position(_position),
        velocity(_velocity),
        acceleration(_acceleration),
        jerk(_jerk),
        snap(_snap),
        orientation(_orientation),
        angular_velocity(_angular_velocity) {};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t time_from_start_ns;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;

  Eigen::Quaterniond orientation;
  Eigen::Vector3d angular_velocity;

  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const {
    return yawFromQuaternion(orientation);
  }
  inline double getYawRate() const {
    return angular_velocity.z();
  }
  // WARNING: sets roll and pitch to 0.
  inline void setYaw(double yaw) {
    orientation = quaternionFromYaw(yaw);
  }
  inline void setYawRate(double yaw_rate) {
    angular_velocity.z() = yaw_rate;
  }
};

struct EigenOdometry {
  EigenOdometry()
      : timestamp_ns(-1),
        position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()),
        angular_velocity(Eigen::Vector3d::Zero()) {
  }

  EigenOdometry(const Eigen::Vector3d& _position, const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity, const Eigen::Vector3d& _angular_velocity)
      : position(_position),
        orientation(_orientation),
        velocity(_velocity),
        angular_velocity(_angular_velocity) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp_ns; // Time since epoch, negative value = invalid timestamp.
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;  // Velocity in expressed in the Body frame!
  Eigen::Vector3d angular_velocity;

  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const {
    return yawFromQuaternion(orientation);
  }
  inline double getYawRate() const {
    return angular_velocity.z();
  }
  // WARNING: sets roll and pitch to 0.
  inline void setYaw(double yaw) {
    orientation = quaternionFromYaw(yaw);
  }
  inline void setYawRate(double yaw_rate) {
    angular_velocity.z() = yaw_rate;
  }

  // TODO(helenol): add accessors for body frame vs. world frame velocities.
};

}

#endif // MAV_MSGS_EIGEN_MAV_MSGS_H
