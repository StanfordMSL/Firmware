/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file LandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include "LandDetector.h"

#include <float.h>
#include <math.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

namespace land_detector
{

LandDetector::LandDetector() :
	ScheduledWorkItem(px4::wq_configurations::hp_default)
{
}

LandDetector::~LandDetector()
{
	perf_free(_cycle_perf);
}

void LandDetector::_check_params(const bool force)
{
	parameter_update_s paramUpdate;

	if (_parameter_sub.update(&paramUpdate) || force) {
		_update_params();
		uint32_t flight_time;
		param_get(_paramHandle.total_flight_time_high, (int32_t *)&flight_time);
		_total_flight_time = ((uint64_t)flight_time) << 32;
		param_get(_paramHandle.total_flight_time_low, (int32_t *)&flight_time);
		_total_flight_time |= flight_time;
	}
}

void LandDetector::Run()
{
	perf_begin(_cycle_perf);

	if (_object.load() == nullptr) { // not initialized yet
		// Advertise the first land detected uORB.
		_land_detected.timestamp = hrt_absolute_time();
		_land_detected.freefall = false;
		_land_detected.landed = true;
		_land_detected.ground_contact = false;
		_land_detected.maybe_landed = false;

		_paramHandle.total_flight_time_high = param_find("LND_FLIGHT_T_HI");
		_paramHandle.total_flight_time_low = param_find("LND_FLIGHT_T_LO");

		_check_params(true);

		_object.store(this);
	}

	_check_params(false);

	_actuator_armed_sub.update(&_actuator_armed);

	_update_topics();
	_update_state();

	const bool freefallDetected       = (_state == LandDetectionState::FREEFALL);
	const bool ground_contactDetected = (_state == LandDetectionState::GROUND_CONTACT);
	const bool land_detected          = (_state == LandDetectionState::LANDED);
	const bool maybe_landed_detected  = (_state == LandDetectionState::MAYBE_LANDED);

	const bool in_ground_effect = _ground_effect_hysteresis.get_state();

	const float alt_max = _get_max_altitude();

	const hrt_abstime now = hrt_absolute_time();

	// publish at 1 Hz, very first time, or when the result has changed
	if ((hrt_elapsed_time(&_land_detected.timestamp) >= 1_s) ||
	    (_land_detected_pub == nullptr) ||
	    (_land_detected.landed != land_detected) ||
	    (_land_detected.freefall != freefallDetected) ||
	    (_land_detected.maybe_landed != maybe_landed_detected) ||
	    (_land_detected.ground_contact != ground_contactDetected) ||
	    (_land_detected.in_ground_effect != in_ground_effect) ||
	    (fabsf(_land_detected.alt_max - alt_max) > FLT_EPSILON)) {

		if (!land_detected && _land_detected.landed) {
			// We did take off
			_takeoff_time = now;
		}

		_land_detected.timestamp = hrt_absolute_time();
		_land_detected.alt_max = alt_max;
		_land_detected.freefall = freefallDetected;
		_land_detected.ground_contact = ground_contactDetected;
		_land_detected.in_ground_effect = in_ground_effect;
		_land_detected.landed = land_detected;
		_land_detected.maybe_landed = maybe_landed_detected;

		int instance = 0;
		orb_publish_auto(ORB_ID(vehicle_land_detected), &_land_detected_pub, &_land_detected,
				 &instance, ORB_PRIO_DEFAULT);
	}

	// set the flight time when disarming (not necessarily when landed, because all param changes should
	// happen on the same event and it's better to set/save params while not in armed state)
	if (_takeoff_time != 0 && !_actuator_armed.armed && _previous_armed_state) {
		_total_flight_time += now - _takeoff_time;
		_takeoff_time = 0;
		uint32_t flight_time = (_total_flight_time >> 32) & 0xffffffff;
		param_set_no_notification(_params.total_flight_time_high, &flight_time);
		flight_time = _total_flight_time & 0xffffffff;
		param_set_no_notification(_params.total_flight_time_low, &flight_time);
	}

	_previous_armed_state = _actuator_armed.armed;

	perf_end(_cycle_perf);

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}
}

int LandDetector::start()
{
	ScheduleOnInterval(LAND_DETECTOR_UPDATE_INTERVAL, 10000);

	return PX4_OK;
}

void LandDetector::_update_params()
{
	param_get(_paramHandle.total_flight_time_high, &_params.total_flight_time_high);
	param_get(_paramHandle.total_flight_time_low, &_params.total_flight_time_low);
}

void LandDetector::_update_topics()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_sensor_bias_sub.update(&_sensor_bias);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
}

void LandDetector::_update_state()
{
	/* when we are landed we also have ground contact for sure but only one output state can be true at a particular time
	 * with higher priority for landed */
	const hrt_abstime now_us = hrt_absolute_time();
	_freefall_hysteresis.set_state_and_update(_get_freefall_state(), now_us);
	_landed_hysteresis.set_state_and_update(_get_landed_state(), now_us);
	_maybe_landed_hysteresis.set_state_and_update(_get_maybe_landed_state(), now_us);
	_ground_contact_hysteresis.set_state_and_update(_get_ground_contact_state(), now_us);
	_ground_effect_hysteresis.set_state_and_update(_get_ground_effect_state(), now_us);

	if (_freefall_hysteresis.get_state()) {
		_state = LandDetectionState::FREEFALL;

	} else if (_landed_hysteresis.get_state()) {
		_state = LandDetectionState::LANDED;

	} else if (_maybe_landed_hysteresis.get_state()) {
		_state = LandDetectionState::MAYBE_LANDED;

	} else if (_ground_contact_hysteresis.get_state()) {
		_state = LandDetectionState::GROUND_CONTACT;

	} else {
		_state = LandDetectionState::FLYING;
	}
}

} // namespace land_detector
