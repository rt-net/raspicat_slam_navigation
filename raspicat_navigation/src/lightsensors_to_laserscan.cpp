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

#include "raspimouse_msgs/LightSensorValues.h"
#include "raspicat_navigation/lightsensors_to_laserscan.hpp"

namespace lightsensors_to_laserscan
{
LightsensorsToLaserscan::LightsensorsToLaserscan(ros::NodeHandle &nodeHandle,
                                                 ros::NodeHandle &private_nodeHandle)
    : nh_(nodeHandle), pnh_(private_nodeHandle), latest_sub_time_(ros::Time::now())
{
  initPub();
  setParam();
  initTimerCb();
  run();
}

LightsensorsToLaserscan::~LightsensorsToLaserscan()
{
  check_sub_lightsensors_timer_.stop();
  nh_.shutdown();
}

sensor_msgs::LaserScan LightsensorsToLaserscan::convertAnalogDistanceSensorToLaserscan(
    int16_t analog_value, std::string &frame_id)
{
  latest_sub_time_ = ros::Time::now();

  sensor_msgs::LaserScan out_scan;
  if (checkInvalidValue(analog_value))
  {
    for (auto i = 0; i < 15; i++) out_scan.ranges.push_back(convertAnalogToMeter(analog_value));
  }
  else
    for (auto i = 0; i < 15; i++) out_scan.ranges.push_back(analog_error_value_);

  out_scan.header.frame_id = frame_id;
  out_scan.header.stamp = ros::Time::now();
  out_scan.scan_time = 0.1;
  out_scan.time_increment = 0.000001;
  out_scan.angle_min = -0.1309;
  out_scan.angle_max = 0.1309;
  out_scan.range_min = 0.1;
  out_scan.range_max = 4.0;
  out_scan.angle_increment = 0.0174533;
  return out_scan;
}

void LightsensorsToLaserscan::setParam()
{
  pnh_.param("left_side_usensor_frame_id", ls_frame_id_, std::string("left_side_usensor_link"));
  pnh_.param("left_front_usensor_frame_id", lf_frame_id_, std::string("left_front_usensor_link"));
  pnh_.param("right_front_usensor_frame_id", rf_frame_id_, std::string("right_front_usensor_link"));
  pnh_.param("right_side_usensor_frame_id", rs_frame_id_, std::string("right_side_usensor_link"));
  pnh_.param("usensor_max_threshold", analog_max_th_, 500);
  pnh_.param("usensor_min_threshold", analog_min_th_, 100);
  pnh_.param("usensor_error_value", analog_error_value_, static_cast<double>(INFINITY));
  pnh_.param("usensor_topic_receive_tolerance", sub_tolerance_, 1.0);
}

void LightsensorsToLaserscan::initPub()
{
  ls_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/ls_scan", 1);
  lf_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/lf_scan", 1);
  rf_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/rf_scan", 1);
  rs_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/rs_scan", 1);
}

void LightsensorsToLaserscan::initTimerCb()
{
  check_sub_lightsensors_timer_ = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
    if ((ros::Time::now().toSec() - latest_sub_time_.toSec()) > sub_tolerance_)
      ROS_WARN(
          "Not receiving /lightsensors within %f seconds. It actually "
          "takes %f seconds.",
          sub_tolerance_, (ros::Time::now().toSec() - latest_sub_time_.toSec()));
  });
}

void LightsensorsToLaserscan::run()
{
  lightsensors_subscriber_ =
      nh_.subscribe<raspimouse_msgs::LightSensorValues>("/lightsensors", 10, [&](const auto &msg) {
        raspimouse_msgs::LightSensorValues in_scan;
        in_scan = *msg;

        publishScan(convertAnalogDistanceSensorToLaserscan(in_scan.left_side, ls_frame_id_),
                    convertAnalogDistanceSensorToLaserscan(in_scan.left_forward, lf_frame_id_),
                    convertAnalogDistanceSensorToLaserscan(in_scan.right_forward, rf_frame_id_),
                    convertAnalogDistanceSensorToLaserscan(in_scan.right_side, rs_frame_id_));
      });
}
}  // namespace lightsensors_to_laserscan