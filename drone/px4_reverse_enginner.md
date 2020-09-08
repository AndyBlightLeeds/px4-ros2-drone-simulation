# PX4 Commander

I needed to reverse engineer the commander code to work out how to control the drone reliably.

## Commands

The commands that I send are:

* VEHICLE_CMD_COMPONENT_ARM_DISARM
* VEHICLE_CMD_DO_SET_HOME
* VEHICLE_CMD_NAV_TAKEOFF
* VEHICLE_CMD_NAV_LAND
* VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

These are the commands I will focus on.

The starting point is Commander::handle() command.

I am ignoring things like parameter checking and error checking as they cause
obvious failures.

Each command also sets the appropriate value of cmd_result that should update
the caller if the command was successful or not.

### VEHICLE_CMD_COMPONENT_ARM_DISARM

This does the following:

1. Calls Commander::arm_disarm.
   This calls the function arming_state_transition in state_machine_helper.cpp.
      This checks that the transtion is valid (defined by arming_transitions[])
      and them makes the transition.  Does some extra stuff when using HIL.
      No side effects.
2. If successful, calls Commander::set_home_position.
   If global and local positions are valid, set the home position.
      Get the global and local positions from uORB subscriber.
      Create a home position message (home_position_s), fill it out and
      publish it using uORB.

### VEHICLE_CMD_DO_SET_HOME

If use_current is set, calls Commander::set_home_position.
Else creates a home position message (home_position_s), fills it out with the
given data and publishes it using uORB.

### VEHICLE_CMD_NAV_TAKEOFF

Calls `main_state_transition(*status_local, commander_state_s::MAIN_STATE_AUTO_TAKEOFF, status_flags, &_internal_state)`.
If it passes, returns `vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED`.
This will only fail if the local position (determined by status_flags) is not valid.

If that fails, and `(_internal_state.main_state == commander_state_s::MAIN_STATE_AUTO_TAKEOFF)`
then returns `vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED`.

Else changing state fails.

`commander_state_s` is used in main_state_transition().

Appears to have no side-effect of causing the vehicle to takeoff.  There must
be something else involved.

### VEHICLE_CMD_NAV_LAND

Identical to VEHICLE_CMD_NAV_TAKEOFF except for the values.

### VEHICLE_CMD_NAV_RETURN_TO_LAUNCH

Identical to VEHICLE_CMD_NAV_TAKEOFF except for the values and that it must
have valid global position info.

### Summary

The messages being printed out by the commander module are a side-effect of
the uORB commands being sent, e.g. "Takeoff detected" comes from a while loop
in `Commander::run`.

`Commander::handle_command` only appears to cause the state to change.  The
real work is done elsewhere.  The static function `send_vehicle_command`
publishes a uORB command.

```cpp
   uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
   return vcmd_pub.publish(vcmd);
```

So now I need to find out how the uORB commands are handled.

## uORB messages

The micrortps_client modules publishes received commands to uORB and subscribes
to uORB messages and fowards them onwards to the px4_ros_com agent.

So that I have something to focus on, I'm going to work out how the navigator
module debug appears on successful take offs.

The string "minimum takeoff altitude" is found in
`Takeoff::set_takeoff_position`.  This function is called by
`Takeoff::on_activation` and `Takeoff::on_active`.
The class `Takeoff` is a kind of `MissionBlock`.  There seem to be many types
of `MissionBlocks`, e.g. EngineFailure, Land, Loiter.
This `Takeoff` class is a part of the `Navigator` class along with all the
other `MissionBlocks`.

The class `MulticopterPositionControl` appears to be the master controller for
multi-copters.  It listens to the following uORB messages:

* vehicle_status
* vehicle_land_detected
* vehicle_control_mode
* parameter_update
* vehicle_attitude
* home_position
* hover_thrust_estimate

It owns an instance of the `Takeoff` class (not the same one owned by
`Navigator`) that controls the smooth speed up of the rotors during take off.

At a guess, it seems that the auto pilot is set up to traverse a set
of way points rather than take off and land.

Really useful but not always accurate for uORB.
<https://mavlink.io/en/messages/common.html>

### DO_SET_MODE related posts

This is very interesting!  Not what the documents say at all!
<https://discuss.px4.io/t/mav-cmd-do-set-mode-all-possible-modes/8495/2>

The Arm mode has been deprecated.
<https://discuss.px4.io/t/mav-cmd-do-set-mode-mav-mode-auto-armed-unsupported/10272/2>
