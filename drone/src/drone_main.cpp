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

#define AUTO_PILOT_PX4

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "drone.hpp"

#if defined(AUTO_PILOT_PX4)
#include "auto_pilot_px4.hpp"
#endif

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Dependency injection to allow testing and different implementations.
#if defined(AUTO_PILOT_PX4)
  auto auto_pilot_node = std::make_shared<AutoPilotPX4>();
#endif
  // Inject the dependency. Also send the executor so it can be cancelled.
  rclcpp::executors::SingleThreadedExecutor exec;
  Drone drone(auto_pilot_node, exec);
  // Start the auto pilot.
  exec.add_node(auto_pilot_node);
  // Start the main program on a thread to allow the nodes etc. to spin.
  auto drone_thread = new std::thread(&Drone::Run, drone);
  // Spin until done.
  exec.spin();
  // Tidy up thread.
  exec.cancel();
  drone_thread->join();
  delete drone_thread;
  auto_pilot_node = nullptr;
  rclcpp::shutdown();
  return 0;
}
