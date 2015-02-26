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

#ifndef PLANNING_MSGS_CONVERSIONS_H
#define PLANNING_MSGS_CONVERSIONS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "planning_msgs/WayPoint.h"
#include "planning_msgs/WayPointArray.h"
#include "planning_msgs/WaypointType.h"
#include "planning_msgs/eigen_planning_msgs.h"

namespace planning_msgs {

/// Converts a WayPoint double array to an Eigen::VectorXd.
inline void vectorFromMsgArray(const WayPoint::_x_type& array, Eigen::VectorXd* x) {
  *x = Eigen::Map<const Eigen::VectorXd>(&(array[0]), array.size());
}

/// Converts an Eigen::VectorXd to a WayPoint double array.
inline void msgArrayFromVector(const Eigen::VectorXd& x, WayPoint::_x_type* array) {
  array->resize(x.size());
  Eigen::Map<Eigen::VectorXd> map = Eigen::Map<Eigen::VectorXd>(&((*array)[0]), array->size());
  map = x;
}

/// Converts a WayPoint message to an EigenWayPoint structure.
inline void eigenWaypointFromMsg(const WayPoint& msg, EigenWayPoint* waypoint) {
  assert(waypoint != NULL);

  vectorFromMsgArray(msg.x, &(waypoint->x));
  vectorFromMsgArray(msg.y, &(waypoint->y));
  vectorFromMsgArray(msg.z, &(waypoint->z));
  vectorFromMsgArray(msg.yaw, &(waypoint->yaw));

  waypoint->time = msg.time;
  waypoint->type = msg.type;
}

/// Converts a WayPointArray message to a EigenWayPointArray
inline void eigenWaypointArrayFromMsg(const WayPointArray& msg, EigenWaypointArray* waypoint_array) {
  assert(waypoint_array != NULL);
  waypoint_array->clear();
  waypoint_array->reserve(msg.waypoints.size());
  for (WayPointArray::_waypoints_type::const_iterator it = msg.waypoints.begin(); it != msg.waypoints.end();
      ++it) {
    EigenWayPoint wp;
    eigenWaypointFromMsg(*it, &wp);
    waypoint_array->push_back(wp);
  }
}

/// Converts an EigenWayPoint to a WayPoint message. Does NOT set the header!
inline void wayPointMsgFromEigen(const EigenWayPoint& waypoint, WayPoint* msg) {
  assert(msg != NULL);
  msgArrayFromVector(waypoint.x, &(msg->x));
  msgArrayFromVector(waypoint.y, &(msg->y));
  msgArrayFromVector(waypoint.z, &(msg->z));
  msgArrayFromVector(waypoint.yaw, &(msg->yaw));

  msg->time = waypoint.time;
  msg->type = waypoint.type;
}

/// Converts an EigenWayPointArray to a WayPointArray message. Does NOT set the header!
inline void waypointArrayMsgFromEigen(const EigenWaypointArray& waypoint_array, WayPointArray* msg) {
  assert(msg != NULL);
  msg->waypoints.reserve(waypoint_array.size());
  for (EigenWaypointArray::const_iterator it = waypoint_array.begin(); it != waypoint_array.end(); ++it) {
    WayPoint wp;
    wayPointMsgFromEigen(*it, &wp);
    msg->waypoints.push_back(wp);
  }
}

}

#endif
