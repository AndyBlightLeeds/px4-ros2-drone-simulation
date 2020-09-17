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

#include "drone.hpp"
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "auto_pilot_interface.hpp"


Drone::Drone(
  std::shared_ptr<AutoPilotInterface> auto_pilot,
  rclcpp::executors::SingleThreadedExecutor & exec)
: auto_pilot_(auto_pilot),
  exec_(exec)
{
  rcutils_ret_t result = rcutils_logging_set_logger_level(
    "Drone", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) result;
}

Drone::~Drone()
{
}

void Drone::Run()
{
  RCUTILS_LOG_INFO_NAMED("Drone", "%s: started", __FUNCTION__);
  // It is best to work on each test individually as the previous tests
  // may not leave everything pristine.
  TestAutoModes();
  TestMission();
  exec_.cancel();
}

void Drone::TestAutoModes()
{
  RCUTILS_LOG_INFO_NAMED("Drone", "%s: started", __FUNCTION__);
  using namespace std::chrono_literals;
  // Execute take off and landing using auto modes.
  // Hold between each command so that the change is visible.
  std::chrono::seconds hold_time_s(3);
  int result = auto_pilot_->PreFlightChecks();
  if (result == 0) {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: Pre-flight checks OK", __FUNCTION__);
    result = auto_pilot_->TakeOff(0.f);
    // The drone will loiter automatically after take off.
    rclcpp::sleep_for(hold_time_s);
    // Always land if off the ground.
    result = auto_pilot_->ReturnToLaunch();
  }

  if (result == 0) {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: success", __FUNCTION__);
  } else {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: fail", __FUNCTION__);
  }
}

void Drone::TestMission()
{
  RCUTILS_LOG_INFO_NAMED("Drone", "%s: started", __FUNCTION__);
  using namespace std::chrono_literals;
  // Execute a mission.
  int result = auto_pilot_->PreFlightChecks();
  if (result == 0) {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: Pre-flight checks OK", __FUNCTION__);
    AutoPilotInterface::GeographicPointLocation start;
    result = auto_pilot_->GetHomeLocation(&start);
    if (result == 0) {
      // Now that we have the start position, set up a mission using waypoints
      // relative to the start position.
      std::vector<AutoPilotInterface::Waypoint> waypoints;
      // Use the same waypoint to make the code easier.
      AutoPilotInterface::Waypoint waypoint;
      // Hold for 3 seconds at each waypoint.
      waypoint.hold_time_ms = 3000;
      // Make position changes relative to the start location.
      waypoint.location = start;
      // Use 3 waypoints.
      for (int i = 0; i < 5; ++i) {
        switch (i) {
          case 0:
            // Ascend to 5 metres.
            waypoint.location.altitude += (5 * 1000);
            break;
          case 1:
            // Change latitude.
            // Should be a small change so it stays in range of Gazebo but big
            // enough to be visible.
            waypoint.location.latitude += 5;
            break;
          case 2:
            // Change longitude.
            waypoint.location.longitude += 5;
            break;
          case 3:
            // Return to above start location.
            waypoint.location.latitude = start.latitude;
            waypoint.location.longitude = start.longitude;
            break;
          case 4:
            // Land
            waypoint.location.altitude = start.altitude;
            break;
          default:
            RCUTILS_LOG_ERROR_NAMED("Drone", "%s: Should never reach here", __FUNCTION__);
            break;
        }
        waypoints.push_back(waypoint);
      }

      // Fly the mission returning to launch site when done.
      result = auto_pilot_->FlyMission(waypoints, true);
    }
  }

  if (result == 0) {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: success", __FUNCTION__);
  } else {
    RCUTILS_LOG_INFO_NAMED("Drone", "%s: fail", __FUNCTION__);
  }
}
