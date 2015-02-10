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

namespace mav_msgs {

struct EigenCommandAttitudeThrust {
  EigenCommandAttitudeThrust()
      : attitude(1.0, 0.0, 0.0, 0.0),
        thrust(0.0) {};
  EigenCommandAttitudeThrust(const Eigen::Quaterniond& _attitude,
                             const double& _thrust) {
    attitude = _attitude;
    thrust = _thrust;
  }
  Eigen::Quaterniond attitude;
  double thrust;
};

struct EigenCommandMotorSpeed {
  EigenCommandMotorSpeed()
      : motor_speeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {};

  EigenCommandMotorSpeed(const Eigen::VectorXd& _motor_speeds) {
    motor_speeds = _motor_speeds;
  };

  Eigen::VectorXd motor_speeds;
};

struct EigenCommandRateThrust {
  EigenCommandRateThrust()
      : angular_rates(0.0, 0.0, 0.0);
        thrust(0.0) {};

  EigenCommandRateThrust(const Eigen::Vector3d& _angular_rates, const double& _thrust) {
    angular_rates = _angular_rates;
    thrust = _thrust;
  };

  Eigen::Vector3d angular_rates;
  double thrust;
};

struct EigenCommandRollPitchYawrateThrust {
  EigenCommandRollPitchYawrateThrust()
      : roll(0.0),
        pitch(0.0),
        yaw_rate(0.0),
        thrust(0.0) {};

  EigenCommandRollPitchYawrateThrust(const double& _roll,
                                     const double& _pitch,
                                     const double& _yaw_rate,
                                     const double& _thrust) {
    roll = _roll;
    pitch = _pitch;
    yaw_rate = _yaw_rate;
    thrust = _thrust;
  };

  double roll;
  double pitch;
  double yaw_rate;
  double thrust;
};

struct EigenCommandTrajectory {
  EigenCommandTrajectory()
      : position(0.0, 0.0, 0.0),
        velocity(0.0, 0.0, 0.0),
        acceleration(0.0, 0.0, 0.0),
        jerk(0.0, 0.0, 0.0),
        snap(0.0, 0.0, 0.0),
        yaw(0.0),
        yaw_rate(0.0) {};

  EigenCommandTrajectory(const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _velocity,
                         const Eigen::Vector3d& _acceleration,
                         const Eigen::Vector3d& _jerk,
                         const Eigen::Vector3d& _snap,
                         const double& _yaw,
                         const double& _yaw_rate) {
    position = _position;
    velocity = _velocity;
    acceleration = _acceleration;
    jerk = _jerk;
    snap = _snap;
    yaw = _yaw;
    yaw_rate = _yaw_rate;
  };

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;
  double yaw;
  double yaw_rate;
};

}

#endif // MAV_MSGS_EIGEN_MAV_MSGS_H
