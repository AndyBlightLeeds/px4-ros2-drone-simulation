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

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "src/drone.hpp"
#include "src/auto_pilot_px4.hpp"


// This delay is set to the smallest value that allows the tests to pass every time.
static const std::chrono::milliseconds kConnectionDelayMs(250);


/**
 * @brief Test AutoPilotInterface class using AutoPilotPX4 implementation.
 */
class TestAutoPilotPX4 : public ::testing::Test
{
protected:
  void SetUp();
  void auto_pilot_spin();
  void TearDown();
  std::shared_ptr<AutoPilotPX4> auto_pilot_node_;
  rclcpp::executors::SingleThreadedExecutor * auto_pilot_exec_;
  std::thread * auto_pilot_thread_;
};

void TestAutoPilotPX4::SetUp()
{
  // Set up rclcpp stuff.
  rclcpp::init(0, nullptr);
  rcutils_ret_t result = rcutils_logging_set_logger_level(
    "TestAutoPilotPX4", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) result;
  // Create PX4 auto pilot node.
  auto_pilot_node_ = std::make_shared<AutoPilotPX4>();
  // Add the auto pilot node to the executor.
  auto_pilot_exec_ = new rclcpp::executors::SingleThreadedExecutor();
  auto_pilot_exec_->add_node(auto_pilot_node_);
  // Create thread so that the auot pilot runs while the tests are taking place.
  auto_pilot_thread_ = new std::thread(&TestAutoPilotPX4::auto_pilot_spin, this);
  // Wait for the node to get started.
  std::this_thread::sleep_for(std::chrono::milliseconds(kConnectionDelayMs));
}

void TestAutoPilotPX4::auto_pilot_spin()
{
  auto_pilot_exec_->spin();
}

void TestAutoPilotPX4::TearDown()
{
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s", __FUNCTION__);
  // Stop auto pilot.
  auto_pilot_exec_->cancel();
  auto_pilot_thread_->join();
  delete auto_pilot_exec_;
  delete auto_pilot_thread_;
  // Destroy node.
  auto_pilot_node_ = nullptr;
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s joined", __FUNCTION__);
  rclcpp::shutdown();
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s done", __FUNCTION__);
}

TEST_F(TestAutoPilotPX4, AutoModes)
{
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s: started", __FUNCTION__);
  // Execute take off and landing using auto modes.
  // Hold between each command so that the change is visible.
  std::chrono::seconds hold_time_s(3);
  int result = auto_pilot_node_->PreFlightChecks();
  ASSERT_EQ(result, 0);
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s: Pre-flight checks OK", __FUNCTION__);
  result = auto_pilot_node_->TakeOff(0.f);
  ASSERT_EQ(result, 0);
  // The drone will loiter automatically after take off.
  rclcpp::sleep_for(hold_time_s);
  // Always land if off the ground.
  result = auto_pilot_node_->ReturnToLaunch();
  ASSERT_EQ(result, 0);
}

TEST_F(TestAutoPilotPX4, Mission)
{
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s: started", __FUNCTION__);
  // Execute a mission.
  int result = auto_pilot_node_->PreFlightChecks();
  ASSERT_EQ(result, 0);
  RCUTILS_LOG_INFO_NAMED("TestAutoPilotPX4", "%s: Pre-flight checks OK", __FUNCTION__);
  AutoPilotInterface::GeographicPointLocation start;
  result = auto_pilot_node_->GetHomeLocation(&start);
  ASSERT_EQ(result, 0);
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
        RCUTILS_LOG_ERROR_NAMED("TestAutoPilotPX4", "%s: Should never reach here", __FUNCTION__);
        break;
    }
    waypoints.push_back(waypoint);
  }

  // Fly the mission returning to launch site when done.
  result = auto_pilot_node_->FlyMission(waypoints, true);
  ASSERT_EQ(result, 0);
}
