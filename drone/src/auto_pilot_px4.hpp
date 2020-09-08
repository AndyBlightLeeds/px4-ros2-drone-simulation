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


#ifndef AUTO_PILOT_PX4_HPP_
#define AUTO_PILOT_PX4_HPP_

#include "auto_pilot_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <thread>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/telemetry_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

class AutoPilotPX4 : public AutoPilotInterface
{
public:
  AutoPilotPX4();
  ~AutoPilotPX4();
  // See AutoPilotInterface for details.
  int PreFlightChecks() override;
  int GetHomeLocation(GeographicPointLocation * location) override;
  int TakeOff(float height) override;
  int Land() override;
  int Loiter(const std::chrono::milliseconds duration_ms);
  int ReturnToLaunch() override;
  int FlyTo(const GeographicPointLocation & location) override;
  int FlyMission(const std::vector<Waypoint> & waypoints, bool return_to_launch_site) override;

private:
  // A copy of some of the data sent by the GPS message.
  struct GpsData
  {
    uint64_t timestamp;
    GeographicPointLocation location;
    GpsData()
    : timestamp(0) {}
  };

  // A copy of some of the data sent by the landed message.
  struct LandedData
  {
    uint64_t timestamp;
    bool landed;
    LandedData()
    : timestamp(0), landed(false) {}
  };

  // A copy of some of the data sent by the vehicle status message.
  struct VehicleStatusData
  {
    uint64_t timestamp;
    uint8_t arming_state;
    uint8_t navigation_state;
    uint8_t system_id;
    uint8_t component_id;
    VehicleStatusData()
    : timestamp(0), arming_state(0), navigation_state(0), system_id(0), component_id(0) {}
  };

  // Waypoint
  struct PX4Waypoint
  {
    float hold_time_s;
    float accept_radius_m;
    float pass_radius_m;
    float yaw_degrees;
    double latitude;
    double longitude;
    float altitude_m;
    PX4Waypoint()
    : hold_time_s(std::nanf("")), accept_radius_m(std::nanf("")),
      pass_radius_m(std::nanf("")), yaw_degrees(std::nanf("")),
      latitude(std::nan("")), longitude(std::nan("")),
      altitude_m(std::nanf("")) {}
  };

  enum class FlightState
  {
    kDisarmed, kArmed, kTakingOff, kHovering, kEnRoute,
    kReturnToLaunch, kLanding
  };

  // A subset of the available modes.
  enum class DoSetMode
  {
    kAutoTakeOff, kAutoLoiter, kAutoMission, kAutoRtl, kAutoLand, kOffboard
  };

  int WaitForGPS();
  int WaitForVehicleStatus();
  bool TakeoffHeightAchieved(float takeoff_height_m);
  int WaitForLanded(bool landed);
  void RunRCSimulator();
  // Messages
  int SendArm(bool arm);
  void SendDoSetMode(DoSetMode new_mode);
  void SendLand();
  void SendManualControlSetpoint();
  void SendOffboardControlMode();
  void SendPositionSetpointTriplet(uint8_t type, const Waypoint &waypoint);
  void SendRTL();
  void SendSetHome();
  void SendTakeoff();
  // The PX4 code tests against the NaN values.
  void SendVehicleCommand(
    const uint16_t command, float param1 = std::nanf(""),
    float param2 = std::nanf(""), float param3 = std::nanf(""),
    float param4 = std::nanf(""), double param5 = std::nan(""),
    double param6 = std::nan(""), float param7 = std::nanf(""));
  void SetCommandMessageDefaults(px4_msgs::msg::VehicleCommand * msg_vehicle_command);
  // Callbacks
  void BatteryStatusCallback(px4_msgs::msg::BatteryStatus::ConstSharedPtr msg);
  void VehicleCommandAckCallback(px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg);
  void VehicleGpsPositionCallback(px4_msgs::msg::VehicleGpsPosition::ConstSharedPtr msg);
  void VehicleLandDetectedCallback(px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg);
  void VehicleStatusCallback(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);

  FlightState flight_state_;
  float heading_;
  int source_system_;
  int target_system_;
  int source_component_;
  int target_component_;
  GpsData gps_data_;
  GeographicPointLocation home_location_;
  GeographicPointLocation target_location_;
  LandedData landed_data_;
  VehicleStatusData vehicle_status_data_;
  std::thread * rc_simulator_thread_;
  bool rc_simulator_running_;
  // Pubs.
  rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr position_setpoint_triplet_pub_;
  rclcpp::Publisher<px4_msgs::msg::TelemetryStatus>::SharedPtr telemetry_status_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  // Subs.
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr vehicle_gps_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
};

#endif  // AUTO_PILOT_PX4_HPP_
