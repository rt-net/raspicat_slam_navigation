// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright 2022 RT Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef LIGHTSENSORS_TO_LASERSCAN_H
#define LIGHTSENSORS_TO_LASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <string>

namespace lightsensors_to_laserscan
{
class LightsensorsToLaserscan
{
 public:
  LightsensorsToLaserscan(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nodeHandle);

  ~LightsensorsToLaserscan();

 private:
  ros::NodeHandle &nh_, &pnh_;
  ros::Publisher ls_pub_, lf_pub_, rf_pub_, rs_pub_;
  ros::Subscriber lightsensors_subscriber_;
  ros::Time initial_time_, latest_sub_time_;
  ros::Timer check_move_base_param_timer_, check_sub_lightsensors_timer_;

  std::string ls_frame_id_, lf_frame_id_, rf_frame_id_, rs_frame_id_;
  int analog_hight_noise_th_, analog_max_th_, analog_min_th_;
  double sub_tolerance_, analog_error_value_;

  inline bool checkInvalidValue(int16_t &analog_value)
  {
    return (analog_value < analog_max_th_ && analog_value > analog_min_th_) ? true : false;
  }

  inline auto convertAnalogToMeter(int16_t &analog_value) { return analog_value * 0.001; }

  sensor_msgs::LaserScan convertAnalogDistanceSensorToLaserscan(int16_t analog_value,
                                                                std::string &frame_id);

  inline auto publishScan(sensor_msgs::LaserScan ls_scan, sensor_msgs::LaserScan lf_scan,
                          sensor_msgs::LaserScan rf_scan, sensor_msgs::LaserScan rs_scan)
  {
    ls_pub_.publish(ls_scan);
    lf_pub_.publish(lf_scan);
    rf_pub_.publish(rf_scan);
    rs_pub_.publish(rs_scan);
  }

  void initPub();
  void setParam();
  void initTimerCb();
  void run();
};

}  // namespace lightsensors_to_laserscan

#endif  // LIGHTSENSORS_TO_LASERSCAN_H