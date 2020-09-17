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

#include "auto_pilot_px4.hpp"

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/position_setpoint_triplet.hpp"
#include "px4_msgs/msg/telemetry_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_gps_position.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "auto_pilot_interface.hpp"

const std::chrono::milliseconds kWaitLoopTime(100);


AutoPilotPX4::AutoPilotPX4()
: flight_state_(FlightState::kDisarmed),
  heading_(0.0),
  source_system_(0),
  target_system_(0),
  source_component_(0),
  target_component_(0),
  rc_simulator_thread_(nullptr),
  rc_simulator_running_(false)
{
  rcutils_ret_t result = rcutils_logging_set_logger_level(
    get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  (void) result;
  // Target system - this is for dealing with multiple drones.
  declare_parameter("target_system");
  get_parameter("target_system", target_system_);
  RCLCPP_DEBUG(get_logger(), "%s target_system %d\n", get_namespace(), target_system_);
  // The source and target systems cannot be the same!  The drone system
  // numbers (target) typically start at 1 going up.  The controllers (source)
  // start at 255 and work down.
  source_system_ = 255;
  target_system_ = 1;
  // The component numbers allow fine grained analysis about which component
  // did what and when (log files?)
  // In our usage, there is only one PX4 autopilot and only one controller.
  source_component_ = 1;
  target_component_ = 1;
  // IMPORTANT NOTE
  // The topic names MUST match.  The best way to ensure this is to start the
  // MicroRTPS agent and use `ros2 topic list` to get the topic names.
  // Create publishers.
  manual_control_setpoint_pub_ = create_publisher<px4_msgs::msg::ManualControlSetpoint>(
    "ManualControlSetpoint_PubSubTopic", 10);
  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "OffboardControlMode_PubSubTopic", 10);
  position_setpoint_triplet_pub_ = create_publisher<px4_msgs::msg::PositionSetpointTriplet>(
    "PositionSetpointTriplet_PubSubTopic", 10);
  telemetry_status_pub_ = create_publisher<px4_msgs::msg::TelemetryStatus>(
    "TelemetryStatus_PubSubTopic", 10);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "VehicleCommand_PubSubTopic", 10);
  // Create subscribers.
  battery_status_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "BatteryStatus_PubSubTopic", 10,
    std::bind(&AutoPilotPX4::BatteryStatusCallback, this, std::placeholders::_1));
  vehicle_command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "VehicleCommandAck_PubSubTopic", 10,
    std::bind(&AutoPilotPX4::VehicleCommandAckCallback, this, std::placeholders::_1));
  vehicle_gps_position_sub_ = create_subscription<px4_msgs::msg::VehicleGpsPosition>(
    "VehicleGpsPosition_PubSubTopic", 10,
    std::bind(&AutoPilotPX4::VehicleGpsPositionCallback, this, std::placeholders::_1));
  vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "VehicleLandDetected_PubSubTopic", 10,
    std::bind(&AutoPilotPX4::VehicleLandDetectedCallback, this, std::placeholders::_1));
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "VehicleStatus_PubSubTopic", 10,
    std::bind(&AutoPilotPX4::VehicleStatusCallback, this, std::placeholders::_1));
}

AutoPilotPX4::~AutoPilotPX4()
{
  if (rc_simulator_running_) {
    rc_simulator_running_ = false;
    rc_simulator_thread_->join();
  }
}

int AutoPilotPX4::PreFlightChecks()
{
  int result = 1;
  // Heart beat has to be running before arming.
  rc_simulator_thread_ = new std::thread(&AutoPilotPX4::RunRCSimulator, this);
  if (rc_simulator_thread_) {
    // Wait to make sure a couple of heart beats have been received.
    // FIXME(AJB): Replace sleep with something better.
    rclcpp::sleep_for(std::chrono::seconds(1));
    result = WaitForGPS();
    if (result == 0) {
      // Set home position.
      home_location_ = gps_data_.location;
      // Other callbacks required for flight.
      result = WaitForVehicleStatus();
    }
  }
  return result;
}

int AutoPilotPX4::GetHomeLocation(GeographicPointLocation * location)
{
  int result = 0;
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  if (gps_data_.timestamp == 0) {
    RCLCPP_ERROR(get_logger(), "%s: GPS NOT AVAILABLE", __FUNCTION__);
    result = 1;
  } else {
    *location = home_location_;
  }
  return result;
}

int AutoPilotPX4::TakeOff(float height_m)
{
  int result = 0;
  RCLCPP_INFO(get_logger(), "%s: called: height %f", __FUNCTION__, height_m);

  // Disarm if armed.
  // Arm
  result = SendArm(true);
  if (result == 0) {
    // FIXME(AJB): Replace sleep with something better.
    rclcpp::sleep_for(std::chrono::seconds(1));
    // Default to auto take off height.
    float takeoff_height_m = 2.5;
    if (height_m == 0.f) {
      // Use auto takeoff.
      // Take off to 2.5m.
      SendDoSetMode(DoSetMode::kAutoTakeOff);
    } else {
      // Take off to given height.
      RCLCPP_ERROR(get_logger(), "%s: NOT IMPLEMENTED", __FUNCTION__);
      // TODO(AJB): Use single step mission?
      takeoff_height_m = height_m;
    }
    // Block until target height achieved.
    while (rclcpp::ok()) {
      bool takeoff_complete = TakeoffHeightAchieved(takeoff_height_m);
      if (takeoff_complete) {
        RCLCPP_INFO(get_logger(), "%s: Takeoff complete", __FUNCTION__);
        break;
      }
      rclcpp::sleep_for(kWaitLoopTime);
    }
  } else {
    RCLCPP_ERROR(get_logger(), "%s: Arming failed", __FUNCTION__);
  }
  return result;
}

int AutoPilotPX4::Land()
{
  RCLCPP_INFO(get_logger(), "%s: called.", __FUNCTION__);
  SendLand();
  int result = WaitForLanded(true);
  if (result == 0) {
    result = SendArm(false);
    if (result == 0) {
      RCLCPP_INFO(get_logger(), "%s: landed.", __FUNCTION__);
    }
  }
  return result;
}

int AutoPilotPX4::Loiter(const std::chrono::milliseconds duration_ms)
{
  RCLCPP_INFO(get_logger(), "%s: called.", __FUNCTION__);
  SendDoSetMode(DoSetMode::kAutoLoiter);
  rclcpp::Time end_time = this->now() + duration_ms;
  while (rclcpp::ok() && (this->now() < end_time)) {
    rclcpp::sleep_for(kWaitLoopTime);
  }
  return rclcpp::ok() ? 0 : 1;
}

int AutoPilotPX4::ReturnToLaunch()
{
  RCLCPP_INFO(get_logger(), "%s: called", __FUNCTION__);
  SendRTL();
  int result = WaitForLanded(true);
  if (result == 0) {
    result = SendArm(false);
    if (result == 0) {
      RCLCPP_INFO(get_logger(), "%s: landed.", __FUNCTION__);
    }
  }
  return result;
}

int AutoPilotPX4::FlyTo(const GeographicPointLocation & location)
{
  int result = 0;
  RCLCPP_INFO(get_logger(), "%s: called: latitude %i, longitude %i, altitude %i",
    __FUNCTION__, location.latitude, location.longitude, location.altitude);
  RCLCPP_ERROR(get_logger(), "%s: called: Write me!", __FUNCTION__);
  return result;
}

int AutoPilotPX4::FlyMission(const std::vector<Waypoint> & waypoints, bool return_to_launch_site)
{
  int result = 0;
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  SendArm(true);
  // Send offboard command 5 times a second.
  rclcpp::WallRate loop_rate(5);
  Waypoint current_waypoint;
  // Start with waypoint 0.
  size_t waypoint_index = 0;
  size_t max_waypoints = waypoints.size();
  bool waypoint_sent = false;
  while (rclcpp::ok() && waypoint_index < max_waypoints) {
    // Process each waypoint.
    if (!waypoint_sent) {
      current_waypoint = waypoints[waypoint_index];
      uint8_t type = px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_POSITION;
      if (waypoint_index == 0) {
        type = px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_TAKEOFF;
      } else if (waypoint_index == max_waypoints - 1) {
        type = px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_LAND;
      }
      SendPositionSetpointTriplet(type, current_waypoint);
      waypoint_sent = true;
    }
    if (waypoint_sent) {
      bool waypoint_arrived = gps_data_.location.is_at(current_waypoint.location);
      if (waypoint_arrived) {
        ++waypoint_index;
        waypoint_sent = false;
      }
    }
    SendOffboardControlMode();
    loop_rate.sleep();
  }
  // RTL if requested, loiter otherwise.
  if (return_to_launch_site) {
    result = ReturnToLaunch();
  }
  SendArm(false);
  return result;
}


// Simulate the presence of an user operating a RC transmitter.
void AutoPilotPX4::RunRCSimulator()
{
  // Signal that the simulator has started.
  rc_simulator_running_ = true;
  // This message has to be sent at least 2 times a second.
  // Using 5Hz ensures that message meets this requirement in case the PX4
  // instance is busy.
  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok() && rc_simulator_running_) {
    SendManualControlSetpoint();
    loop_rate.sleep();
  }
}

int AutoPilotPX4::WaitForGPS()
{
  int result = 0;
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  // GPS publishing rate is 2.5Hz so should not have to wait long.
  // If timestamp == 0, no messages have been received.
  int wait_count = 0;
  while (rclcpp::ok() && gps_data_.timestamp == 0) {
    if (wait_count > 20) {
      break;
    }
    rclcpp::sleep_for(kWaitLoopTime);
    ++wait_count;
    RCLCPP_DEBUG(get_logger(), "%s: wait_count %d", __FUNCTION__, wait_count);
  }
  if (gps_data_.timestamp == 0) {
    RCLCPP_ERROR(get_logger(), "%s: GPS NOT AVAILABLE", __FUNCTION__);
    result = 1;
  } else {
    RCLCPP_DEBUG(get_logger(), "%s: GPS available", __FUNCTION__);
  }
  return result;
}

int AutoPilotPX4::WaitForVehicleStatus()
{
  int result = 0;
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  // If timestamp == 0, no messages have been received.
  // Rate is 1 second.
  int wait_count = 0;
  while (rclcpp::ok() && vehicle_status_data_.timestamp == 0) {
    if (wait_count > 10) {
      break;
    }
    rclcpp::sleep_for(kWaitLoopTime);
    ++wait_count;
  }
  if (vehicle_status_data_.timestamp == 0) {
    RCLCPP_ERROR(get_logger(), "%s: VEHICLE STATUS NOT AVAILABLE", __FUNCTION__);
    result = 1;
  } else {
    RCLCPP_DEBUG(get_logger(), "%s: Vehicle status available", __FUNCTION__);
  }
  return result;
}

bool AutoPilotPX4::TakeoffHeightAchieved(float takeoff_height_m)
{
  bool result = false;
  int32_t altidude_diff = gps_data_.location.altitude - home_location_.altitude;
  float reported_height_m = static_cast<float>(altidude_diff * 1000);
  float radius_m = 0.1;
  if ((reported_height_m + radius_m) > takeoff_height_m) {
    result = true;
  }
  return result;
}

int AutoPilotPX4::WaitForLanded(bool landed)
{
  int result = 0;
  // Wait for first landed callback.  Rate is 1 second. Timeout 1.5s.
  int wait_count = 0;
  while (rclcpp::ok() && landed_data_.timestamp == 0) {
    if (wait_count > 15) {
      result = 1;
      break;
    }
    rclcpp::sleep_for(kWaitLoopTime);
    ++wait_count;
  }
  if (result == 0) {
    // Landing takes a lot longer than take off.
    int landed_count = landed ? 500 : 50;
    // Wait for desired state change.
    while (rclcpp::ok() && landed_data_.landed != landed) {
      if (wait_count > landed_count) {
        result = 1;
      }
    }
  }
  if (result == 0) {
    if (landed_data_.landed) {
      RCLCPP_DEBUG(get_logger(), "%s: Landed", __FUNCTION__);
    } else {
      RCLCPP_DEBUG(get_logger(), "%s: Off the ground", __FUNCTION__);
    }
  } else {
    RCLCPP_ERROR(get_logger(), "%s: TIMEOUT!", __FUNCTION__);
  }
  return result;
}

int AutoPilotPX4::SendArm(bool arm)
{
  int result = 0;
  RCLCPP_DEBUG(get_logger(), "%s: called: arm: %d", __FUNCTION__, arm);
  SendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm);
  // Simulate service behaviour. 2 is armed, 1 is standby.
  uint8_t requested_state = arm ? 2 : 1;
  int wait_count = 0;
  while (rclcpp::ok() && vehicle_status_data_.arming_state != requested_state) {
    if (wait_count > 20) {
      result = 1;
      break;
    }
    rclcpp::sleep_for(kWaitLoopTime);
    ++wait_count;
    RCLCPP_DEBUG(get_logger(), "%s: wait_count %d", __FUNCTION__, wait_count);
  }
  return result;
}

// Code based on:
// PX4/Firmware/src/modules/commander/Commander.cpp: Lines 348 - 390.
// The MavLink command is not relevant as we are dealing with uORB messages.
// FIXME(AJB): Hardcoding the mode values will introduce errors later.
// The mode values are hard coded but are based on the values in the
// file: PX4/Firmware/src/modules/commander/px4_custom_mode.h
void AutoPilotPX4::SendDoSetMode(DoSetMode new_mode)
{
  RCLCPP_DEBUG(get_logger(), "%s: called, %d",
    __FUNCTION__, static_cast<int>(new_mode));
  float mode = 1.0;  // All cases use this.
  float custom_mode = 0.0;
  float custom_submode = 0.0;
  switch (new_mode) {
    case DoSetMode::kAutoTakeOff:
      // PX4_CUSTOM_MAIN_MODE_AUTO = 4
      custom_mode = 4.0;
      // PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
      custom_submode = 2.0;
      break;
    case DoSetMode::kAutoLoiter:
      // PX4_CUSTOM_MAIN_MODE_AUTO = 4
      custom_mode = 4.0;
      // PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
      custom_submode = 2.0;
      break;
    case DoSetMode::kAutoMission:
      // PX4_CUSTOM_MAIN_MODE_AUTO = 4
      custom_mode = 4.0;
      // PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
      custom_submode = 4.0;
      break;
    case DoSetMode::kAutoRtl:
      // PX4_CUSTOM_MAIN_MODE_AUTO = 4
      custom_mode = 4.0;
      // PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
      custom_submode = 5.0;
      break;
    case DoSetMode::kAutoLand:
      // PX4_CUSTOM_MAIN_MODE_AUTO = 4
      custom_mode = 4.0;
      // PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
      custom_submode = 6.0;
      break;
    case DoSetMode::kOffboard:
      // PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
      custom_mode = 6.0;
      // No sub mode.
      break;
  }
  SendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    static_cast<float>(mode), static_cast<float>(custom_mode),
    static_cast<float>(custom_submode));
}

void AutoPilotPX4::SendLand()
{
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  float pitch = 0.1;  // Pitch is fixed on multi-copters so just set a value.
  SendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, pitch, std::nanf(""), std::nanf(""),
    heading_, target_location_.latitude, target_location_.longitude);
}

// This message is sent regularly to prevent failsafes being activated.
void AutoPilotPX4::SendManualControlSetpoint()
{
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  // Fill out the message.
  px4_msgs::msg::ManualControlSetpoint msg;
  // Timestamp is not needed as this is done by the client.
  msg.x = 0.f;
  msg.y = 0.f;
  msg.z = 0.f;
  msg.r = 0.f;
  // Only fill out the mandatory switches.
  msg.mode_switch = 0;
  msg.return_switch = 0;
  // Send the message.
  manual_control_setpoint_pub_->publish(msg);
}

void AutoPilotPX4::SendOffboardControlMode()
{
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  // Fill out the message.
  px4_msgs::msg::OffboardControlMode msg;
  // Timestamp is not needed as this is done by the client.
  // May need to revisit these values.
  // For now we are just doing position.
  msg.ignore_thrust = true;
  msg.ignore_attitude = true;
  msg.ignore_bodyrate_x = true;
  msg.ignore_bodyrate_y = true;
  msg.ignore_bodyrate_z = true;
  msg.ignore_position = false;
  msg.ignore_velocity = true;
  msg.ignore_acceleration_force = true;
  msg.ignore_alt_hold = true;
  // Send the message.
  offboard_control_mode_pub_->publish(msg);
}

void AutoPilotPX4::SendPositionSetpointTriplet(uint8_t type, const Waypoint & waypoint)
{
  RCLCPP_INFO(get_logger(), "%s: called, type %d: %dmm, %d, %d", __FUNCTION__, type,
    waypoint.location.altitude, waypoint.location.latitude, waypoint.location.longitude);
  if (type == px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_TAKEOFF) {
    RCLCPP_INFO(get_logger(), "%s: taking off", __FUNCTION__);
  } else if (type == px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_LAND) {
    RCLCPP_INFO(get_logger(), "%s: landing", __FUNCTION__);
  }
  // Fill out the message.
  // According comments in FlightTaskAuto::_evaluateTriplets(), the triplet
  // should be filled out as follows:
  //   previous = current GPS location.
  //   current = new setpoint.
  //   next = new setpoint.
  // To see what effect each param has, see:
  // Firmware/src/lib/flight_tasks/tasks/Offboard/FlightTaskOffboard.cpp
  px4_msgs::msg::PositionSetpointTriplet msg;
  // Timestamp is done by the client.
  // Fill out previous setpoint.
  px4_msgs::msg::PositionSetpoint previous;
  previous.valid = true;
  previous.yaw_valid = false;
  previous.alt = gps_data_.location.altitude;
  previous.lat = gps_data_.location.latitude;
  previous.lon = gps_data_.location.longitude;
  // Fill out current and next setpoint (they are the same so just use next).
  px4_msgs::msg::PositionSetpoint next;
  next.valid = true;
  next.type = type;
  next.alt = waypoint.location.altitude;
  next.lat = waypoint.location.latitude;
  next.lon = waypoint.location.longitude;
  // Ignore everything else.
  next.yaw_valid = false;
  // Send the message.
  msg.previous = previous;
  msg.current = next;
  msg.next = next;
  position_setpoint_triplet_pub_->publish(msg);
}

void AutoPilotPX4::SendRTL()
{
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  SendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
}

void AutoPilotPX4::SendSetHome()
{
  RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
  SendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 1.0);
}

void AutoPilotPX4::SendTakeoff()
{
  RCLCPP_DEBUG(get_logger(), "%s: taking off", __FUNCTION__);
  SendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.1, std::nanf(""), std::nanf(""),
    heading_, target_location_.latitude, target_location_.longitude,
    target_location_.altitude);
}

// void AutoPilotPX4::SendWaypoint(const PX4Waypoint & waypoint)
// {
//   RCLCPP_DEBUG(get_logger(), "%s: called", __FUNCTION__);
//   SendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_WAYPOINT,
//     waypoint.hold_time_s, waypoint.accept_radius_m, waypoint.pass_radius_m,
//     waypoint.yaw_degrees, waypoint.latitude, waypoint.longitude,
//     waypoint.altitude_m);
// }

/* See PX4/Firmware/src/modules/navigator/navigator_main.cpp for how the
 * values are used.  Each message is different.
 */
void AutoPilotPX4::SendVehicleCommand(
  const uint16_t command, float param1,
  float param2, float param3, float param4,
  double param5, double param6, float param7)
{
  RCLCPP_DEBUG(get_logger(), "%s: command %d", __FUNCTION__, command);
  px4_msgs::msg::VehicleCommand msg_vehicle_command;
  SetCommandMessageDefaults(&msg_vehicle_command);
  msg_vehicle_command.command = command;
  msg_vehicle_command.param1 = param1;
  msg_vehicle_command.param2 = param2;
  msg_vehicle_command.param3 = param3;
  msg_vehicle_command.param4 = param4;
  msg_vehicle_command.param5 = param5;
  msg_vehicle_command.param6 = param6;
  msg_vehicle_command.param7 = param7;
  vehicle_command_pub_->publish(msg_vehicle_command);
}

void AutoPilotPX4::SetCommandMessageDefaults(px4_msgs::msg::VehicleCommand * msg_vehicle_command)
{
  // Comments from VehicleCommand.msg file.
  msg_vehicle_command->timestamp = get_clock()->now().nanoseconds() / 1000;
  // 0: First transmission of this command. 1-255: Confirmation transmissions
  // (e.g. for kill command)
  msg_vehicle_command->confirmation = 0;
  // System sending the command
  msg_vehicle_command->source_system = source_system_;
  // System which should execute the command
  msg_vehicle_command->target_system = target_system_;
  // Component which should execute the command, 0 for all components
  msg_vehicle_command->target_component = target_component_;
  // Component sending the command
  msg_vehicle_command->source_component = source_component_;
  // No comment!
  msg_vehicle_command->from_external = true;
}

void AutoPilotPX4::BatteryStatusCallback(px4_msgs::msg::BatteryStatus::ConstSharedPtr msg)
{
  // The average publishing rate is aboout 80Hz. Print every 100 calls.
  const int target_count = 100;
  static int call_count = target_count;
  if (call_count >= target_count) {
    RCLCPP_DEBUG(get_logger(), "%s: called: remaining %f%%", __FUNCTION__, msg->remaining * 100);
    call_count = 0;
  }
  ++call_count;
}

void AutoPilotPX4::VehicleCommandAckCallback(px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "%s: called: command %d, result %d",
    __FUNCTION__, msg->command, msg->result);
}

void AutoPilotPX4::VehicleGpsPositionCallback(px4_msgs::msg::VehicleGpsPosition::ConstSharedPtr msg)
{
  gps_data_.timestamp = msg->timestamp;
  gps_data_.location.latitude = msg->lat;
  gps_data_.location.longitude = msg->lon;
  gps_data_.location.altitude = msg->alt;
  // GPS call rate is 2.5Hz. Print every 10th call.
  const int target_count = 10;
  static int call_count = target_count;
  if (call_count >= target_count) {
    RCLCPP_DEBUG(
      get_logger(), "%s: called: lat %d, long %d, alt %d", __FUNCTION__, msg->lat, msg->lon,
      msg->alt);
    call_count = 0;
  }
  ++call_count;
}


void AutoPilotPX4::VehicleLandDetectedCallback(
  px4_msgs::msg::VehicleLandDetected::ConstSharedPtr msg)
{
  // Rate is 1 per second.
  RCLCPP_DEBUG(get_logger(), "%s: called: landed %d", __FUNCTION__, msg->landed);
  landed_data_.timestamp = msg->timestamp;
  landed_data_.landed = msg->landed;
}


void AutoPilotPX4::VehicleStatusCallback(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  // Rate is 1 per second.
  RCLCPP_DEBUG(get_logger(),
    "%s: called: arming_state %d, nav_state %d, hil_state %d, failsafe %d",
    __FUNCTION__, msg->arming_state, msg->nav_state, msg->hil_state, msg->failsafe);
  RCLCPP_DEBUG(get_logger(),
    "%s: called: system_type %d, system_id %d, component_id %d, vehicle_type %d",
    __FUNCTION__, msg->system_type, msg->system_id, msg->component_id, msg->vehicle_type);

  vehicle_status_data_.arming_state = msg->arming_state;
  vehicle_status_data_.component_id = msg->component_id;
  vehicle_status_data_.navigation_state = msg->nav_state;
  vehicle_status_data_.system_id = msg->system_id;
  vehicle_status_data_.timestamp = msg->timestamp;
}
