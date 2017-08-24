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

#ifndef MANIPULATOR_MSGS_EIGEN_MANIPULATOR_MSGS_H
#define MANIPULATOR_MSGS_EIGEN_MANIPULATOR_MSGS_H

#include <deque>
#include <Eigen/Eigen>

#include "mav_msgs/common.h"

namespace manipulator_msgs {

struct EigenJointsState {
  EigenJointsState() {}
  EigenJointsState(const Eigen::VectorXd& _angles,
                   const Eigen::VectorXd& _angular_rates) {
    angles = _angles;
    angular_rates = _angular_rates;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd angles;
  Eigen::VectorXd angular_rates;
};


struct EigenJointTrajectoryPoint {
  EigenJointTrajectoryPoint() : time_from_start_ns(0) {}
  EigenJointTrajectoryPoint(int64_t _time_from_start_ns,
                            const Eigen::VectorXd& _angles,
                            const Eigen::VectorXd& _angular_rates,
                            const Eigen::VectorXd& _angular_acc,
                            const Eigen::VectorXd& _torques) {
    time_from_start_ns = _time_from_start_ns;
    angles = _angles;
    angular_rates = _angular_rates;
    angular_acc = _angular_acc;
    torques = _torques;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t time_from_start_ns;
  Eigen::VectorXd angles;
  Eigen::VectorXd angular_rates;
  Eigen::VectorXd angular_acc;
  Eigen::VectorXd torques;
};


#define MANIPULATOR_MSGS_CONCATENATE(x, y) x ## y
#define MANIPULATOR_MSGS_CONCATENATE2(x, y) MANIPULATOR_MSGS_CONCATENATE(x, y)
#define MANIPULATOR_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE) \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MANIPULATOR_MSGS_CONCATENATE2(EIGEN_TYPE, Vector); \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE> > MANIPULATOR_MSGS_CONCATENATE2(EIGEN_TYPE, Deque); \

MANIPULATOR_MSGS_MAKE_ALIGNED_CONTAINERS(EigenJointsState)
MANIPULATOR_MSGS_MAKE_ALIGNED_CONTAINERS(EigenJointTrajectoryPoint)

}

#endif // MANIPULATOR_MSGS_EIGEN_MANIPULATOR_MSGS_H
