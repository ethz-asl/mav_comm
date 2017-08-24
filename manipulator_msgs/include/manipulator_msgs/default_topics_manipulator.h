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

#ifndef DEFAULT_TOPICS_MANIPULATOR_H_
#define DEFAULT_TOPICS_MANIPULATOR_H_


namespace manipulator_msgs {
namespace default_topics {

static constexpr char MOTOR_PITCHING_JOINT_STATE[] = "delta_manipulator/motor_pitching_joint_state";
static constexpr char MOTOR_LEFT_JOINT_STATE[] = "delta_manipulator/motor_left_joint_state";
static constexpr char MOTOR_RIGHT_JOINT_STATE[] = "delta_manipulator/motor_right_joint_state";
static constexpr char MOTOR_JOINT_STATE[] = "delta_manipulator/motor_joint_state";

static constexpr char COMMAND_MOTOR_PITCHING_TORQUE[] = "delta_manipulator/command/motor_pitching_torque";
static constexpr char COMMAND_MOTOR_LEFT_TORQUE[] = "delta_manipulator/command/motor_left_torque";
static constexpr char COMMAND_MOTOR_RIGHT_TORQUE[] = "delta_manipulator/command/motor_right_torque";
static constexpr char COMMAND_MOTOR_TORQUE[] = "delta_manipulator/command/motor_torque";

static constexpr char COMMAND_JOINT_TRAJECTORY[] = "delta_manipulator/command/joint_trajectory";
static constexpr char COMMAND_JOINT_ANGLES[] = "delta_manipulator/command/joint_angles";

static constexpr char COMMAND_EE_TRAJECTORY[] = "delta_manipulator/command/end_eff_trajectory";
static constexpr char COMMAND_EE_POSE[] = "delta_manipulator/command/end_eff_pose";

static constexpr char FORCE_SENSOR_LINEAR[] = "delta_manipulator/force_sensor/linear";

}
}

#endif /* DEFAULT_TOPICS_MANIPULATOR_H_ */
