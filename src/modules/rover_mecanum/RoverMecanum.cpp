/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RoverMecanum.hpp"

using namespace time_literals;

RoverMecanum::RoverMecanum() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	updateParams();
}

bool RoverMecanum::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverMecanum::updateParams()
{
	ModuleParams::updateParams();
}

void RoverMecanum::Run()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		runSanityChecks();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);

		// Run sanity checks if the control mode changes (Note: This has to be done this way, because the topic is periodically updated and not on changes)
		if (vehicle_control_mode.flag_control_position_enabled != _vehicle_control_mode.flag_control_position_enabled ||
		    vehicle_control_mode.flag_control_velocity_enabled != _vehicle_control_mode.flag_control_velocity_enabled ||
		    vehicle_control_mode.flag_control_attitude_enabled != _vehicle_control_mode.flag_control_attitude_enabled ||
		    vehicle_control_mode.flag_control_rates_enabled != _vehicle_control_mode.flag_control_rates_enabled) {
			_vehicle_control_mode = vehicle_control_mode;
			runSanityChecks();

		} else {
			_vehicle_control_mode = vehicle_control_mode;
		}

	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		// Reset all controllers if the navigation state changes
		if (vehicle_status.nav_state != _nav_state) {
			_mecanum_pos_control.reset();
			_mecanum_vel_control.reset();
			_mecanum_att_control.reset();
			_mecanum_rate_control.reset();
		}

		_nav_state = vehicle_status.nav_state;
	}

	if (_vehicle_control_mode.flag_armed && _sanity_checks_passed) {
		// Generate setpoints
		if (_vehicle_control_mode.flag_control_manual_enabled) {
			manualControl();

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {
			_mecanum_pos_control.autoPositionMode();

		} else if (_vehicle_control_mode.flag_control_offboard_enabled) {
			offboardControl();
		}

		updateControllers();

	} else if (_was_armed) { // Reset all controllers and stop the vehicle
		_mecanum_pos_control.reset();
		_mecanum_vel_control.reset();
		_mecanum_att_control.reset();
		_mecanum_rate_control.reset();
		_mecanum_act_control.stopVehicle();
		_was_armed = false;
	}

}

void RoverMecanum::manualControl()
{
	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_mecanum_act_control.manualManualMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_mecanum_rate_control.manualAcroMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_mecanum_att_control.manualStabMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_mecanum_pos_control.manualPositionMode();
		break;
	}
}

void RoverMecanum::offboardControl()
{
	offboard_control_mode_s offboard_control_mode{};
	_offboard_control_mode_sub.copy(&offboard_control_mode);

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (offboard_control_mode.position) {
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
		rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else if (offboard_control_mode.velocity) {
		const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = hrt_absolute_time();
		rover_velocity_setpoint.speed = velocity_ned.norm();
		rover_velocity_setpoint.bearing = atan2f(velocity_ned(1), velocity_ned(0));
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	} else if (offboard_control_mode.attitude) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = trajectory_setpoint.yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else if (offboard_control_mode.body_rate) {
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = trajectory_setpoint.yawspeed;
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
	}
}

void RoverMecanum::updateControllers()
{
	if (_vehicle_control_mode.flag_control_position_enabled) {
		_mecanum_pos_control.updatePosControl();
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled) {
		_mecanum_vel_control.updateVelControl();
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		_mecanum_att_control.updateAttControl();
	}

	if (_vehicle_control_mode.flag_control_rates_enabled) {
		_mecanum_rate_control.updateRateControl();
	}

	if (_vehicle_control_mode.flag_control_allocation_enabled) {
		_mecanum_act_control.updateActControl();
	}
}

void RoverMecanum::runSanityChecks()
{
	if (_vehicle_control_mode.flag_control_rates_enabled && !_mecanum_rate_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled && !_mecanum_att_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled && !_mecanum_vel_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_position_enabled && !_mecanum_pos_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	_sanity_checks_passed = true;
}

int RoverMecanum::task_spawn(int argc, char *argv[])
{
	RoverMecanum *instance = new RoverMecanum();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverMecanum::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverMecanum::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover mecanum module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_mecanum", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_mecanum_main(int argc, char *argv[])
{
	return RoverMecanum::main(argc, argv);
}
