/*
 * default_values.h
 *
 *  Created on: 16.03.2016
 *      Author: burrimi
 */

#ifndef MAV_MSGS_DEFAULT_VALUES_H_
#define MAV_MSGS_DEFAULT_VALUES_H_

#include <mav_msgs/common.h>

namespace mav_msgs {

const double kZurichLatitude = 0.8267;
const double kZurichHeight = 405.94;
const double kGravity = MagnitudeOfGravity(kZurichHeight, kZurichLatitude);
}

#endif /* MAV_MSGS_DEFAULT_VALUES_H_ */
