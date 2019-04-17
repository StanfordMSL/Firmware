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

/**
 * @file FixedwingLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#include "FixedwingLandDetector.h"

#include <cmath>

#include <px4_config.h>
#include <px4_defines.h>

namespace land_detector
{

FixedwingLandDetector::FixedwingLandDetector()
{
	_paramHandle.max_air_speed = param_find("LNDFW_AIRSPD_MAX");
	_paramHandle.max_climb_rate = param_find("LNDFW_VEL_Z_MAX");
	_paramHandle.max_velocity = param_find("LNDFW_VEL_XY_MAX");
	_paramHandle.max_XY_accel = param_find("LNDFW_XYACC_MAX");

	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_landed_hysteresis.set_hysteresis_time_from(false, LANDED_TRIGGER_TIME_US);
	_landed_hysteresis.set_hysteresis_time_from(true, FLYING_TRIGGER_TIME_US);
}

bool FixedwingLandDetector::_get_landed_state()
{
	// only trigger flight conditions if we are armed
	if (!_actuator_armed.armed) {
		return true;
	}

	bool landDetected = false;

	if (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 500_ms) {

		// horizontal velocity complimentary filter.
		float val = 0.97f * _velocity_xy_filtered + 0.03f *
			    sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
				  _vehicle_local_position.vy * _vehicle_local_position.vy);

		if (PX4_ISFINITE(val)) {
			_velocity_xy_filtered = val;
		}

		// vertical velocity complimentary filter.
		val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_vehicle_local_position.vz);

		if (PX4_ISFINITE(val)) {
			_velocity_z_filtered = val;
		}

		_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed.true_airspeed_m_s;

		// a leaking lowpass prevents biases from building up, but
		// gives a mostly correct response for short impulses
		const float acc_hor = sqrtf(_sensor_bias.accel_x * _sensor_bias.accel_x +
					    _sensor_bias.accel_y * _sensor_bias.accel_y);
		_accel_horz_lp = _accel_horz_lp * 0.8f + acc_hor * 0.18f;

		// crude land detector for fixedwing
		landDetected = _accel_horz_lp < _params.max_XY_accel
			       && _airspeed_filtered < _params.max_air_speed
			       && _velocity_xy_filtered < _params.max_velocity
			       && _velocity_z_filtered < _params.max_climb_rate;

	} else {
		// Control state topic has timed out and we need to assume we're landed.
		landDetected = true;
	}

	return landDetected;
}

void FixedwingLandDetector::_update_params()
{
	LandDetector::_update_params();

	param_get(_paramHandle.max_velocity, &_params.max_velocity);
	param_get(_paramHandle.max_climb_rate, &_params.max_climb_rate);
	param_get(_paramHandle.max_air_speed, &_params.max_air_speed);
	param_get(_paramHandle.max_XY_accel, &_params.max_XY_accel);
}

void FixedwingLandDetector::_update_topics()
{
	LandDetector::_update_topics();

	_airspeed_sub.update(&_airspeed);
}

} // namespace land_detector
