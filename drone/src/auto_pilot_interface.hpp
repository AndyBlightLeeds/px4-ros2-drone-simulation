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


#ifndef AUTO_PILOT_INTERFACE_HPP_
#define AUTO_PILOT_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>

class AutoPilotInterface : public rclcpp::Node
{
public:
  /** A WGS84 location using scaled integers. */
  struct GeographicPointLocation
  {
    GeographicPointLocation()
    : altitude(0), latitude(0), longitude(0) {}

    bool is_at(const GeographicPointLocation & location)
    {
      bool result = false;
      const int32_t max_altitude_diff_mm = 200;  // 20cm
      // 1 degree of lattitude is 111.2 km. 1 unit is 11.1cm.
      // 2 units is 22.2cm.
      const int32_t max_latitude_diff = 2;
      // Take value at 45 North, 1 unit = 7.8cm.  This will fail near the poles!
      // 3 units is 23.4cm.
      const int32_t max_longitude_diff = 3;
      int32_t altitude_diff = abs(location.altitude - altitude);
      int32_t latitude_diff = abs(location.latitude - latitude);
      int32_t longitude_diff = abs(location.longitude - longitude);
      if ((altitude_diff < max_altitude_diff_mm) &&
        (latitude_diff < max_latitude_diff) &&
        (longitude_diff < max_longitude_diff))
      {
        result = true;
      }
      return result;
    }
    //< Altitude (AMSL, NOT WGS84), in millimeters (positive for up).
    int32_t altitude;
    //< Degrees * 1E7
    int32_t latitude;
    //< Degrees * 1E7
    int32_t longitude;
  };

  /** A location with a hold time. */
  struct Waypoint
  {
    GeographicPointLocation location;
    int32_t hold_time_ms;
    Waypoint()
    : hold_time_ms(0) {}
  };

  AutoPilotInterface()
  : Node("auto_pilot") {}

  ~AutoPilotInterface() {}

  /**
   * @brief Perform pre-flight checks.
   *
   * @return int 0 = success, any other value failure.
   * @note Blocks until success or timeout.  May take more than a second.
   */
  virtual int PreFlightChecks() = 0;

  /**
   * @brief Get the home location.
   * Should be called after PreFlightChecks() to be useful.
   * @param location The home location.
   * @return int 0 on success.  1 for no GPS and location is unchanged.
   */
  virtual int GetHomeLocation(GeographicPointLocation * location) = 0;

  /**
   * @brief Cause the vehicle to ascend to the given height above the take off
   * site.
   * @param height Height in metres above the take off site. When height = 0.0
   * use auto takeoff.
   * @return int 0 = success, any other value failure.
   * @note Blocks until target height is achieved.
   */
  virtual int TakeOff(float height) = 0;

  /**
   * @brief The vehicle loiters for the given duration.
   * @param duration_ms The time to loiter in milliseconds.
   * @return int 0 = success.
   */
  virtual int Loiter(const std::chrono::milliseconds duration_ms) = 0;

  /**
   * @brief The vehicle descends until it lands.  If the vehicle was moving
   * horizontally, no effort will be made to cancel out the horizontal momentum
   * during descent.
   * @note Blocks until landing complete.
   * @return int 0 = success.
   */
  virtual int Land() = 0;

  /**
   * @brief The vehicle returns to launch site.
   *
   * @return int 0 = success.
   * @note Blocks until landing complete.
   */
  virtual int ReturnToLaunch() = 0;

  /**
   * @brief Causes the vehicle to fly towards the location specified by
   * latitude and longitude at the given altitude.  Will loiter on arrival.
   * @param location The location to fly to.
   * @return int 0 = success.
   * @note Blocks until arrived at destination.
   */
  virtual int FlyTo(const GeographicPointLocation & location) = 0;

  /**
   * @brief Causes the vehicle to fly a mission.  A mission is an ordered
   * sequence of waypoints defined in the waypoints parameter. All missions
   * have the following sequence:
   *  Automatic takeoff.
   *  for waypoint in waypoints:
   *    fly to waypoint
   *  end for
   *  Loiter at the last waypoint or return to launch site depending on value
   *  of return_to_launch_site.
   * @param waypoints A sequence of waypoints. The waypoints are executed in
   * order starting with index 0 first and then incrementing.
   * @param return_to_launch_site At the end of the mission, true causes the
   * vehicle to return to the launch site, false causes the vehicle to loiter
   * until the next command is received.
   * @return int 0 = success.
   * @note Blocks until complete.
   */
  virtual int FlyMission(const std::vector<Waypoint> & waypoints, bool return_to_launch_site) = 0;
};

#endif  // AUTO_PILOT_INTERFACE_HPP_
