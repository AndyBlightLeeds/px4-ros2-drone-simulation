// Copyright 2020 University of Leeds.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DRONE_HPP_
#define DRONE_HPP_

#include <memory>

#include "auto_pilot_interface.hpp"

/**
 * @brief The drone control software.
 * @note This is a first hack so only tests the autopilot code.
 * @todo Define the real interface and then implement it!
 */
class Drone
{
public:
  Drone(
    std::shared_ptr<AutoPilotInterface> auto_pilot,
    rclcpp::executors::SingleThreadedExecutor & exec);
  ~Drone();

  /**
   * @brief Starts the control software.  The drone will do some amazing
   * things all by itself!
   */
  void Run();

private:
  void TestAutoModes();
  void TestMission();

  std::shared_ptr<AutoPilotInterface> auto_pilot_;
  rclcpp::executors::SingleThreadedExecutor & exec_;
};

#endif  // DRONE_HPP_
