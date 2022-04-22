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
#include "raspicat_navigation/lightsensors_to_laserscan.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lightsensors_to_laserscan");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Time::waitForValid();

  lightsensors_to_laserscan::LightsensorsToLaserscan lts(nh, pnh);

  ros::AsyncSpinner spinner(pnh.param("num_callback_threads", 4));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}