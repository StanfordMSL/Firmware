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
 * @file FixedwingLandDetector.h
 * Land detector implementation for fixedwing.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>

#include "LandDetector.h"

using namespace time_literals;

namespace land_detector
{

class FixedwingLandDetector final : public LandDetector
{
public:
	FixedwingLandDetector();

protected:
	void _update_params() override;
	void _update_topics() override;

	bool _get_landed_state() override;

private:

	/** Time in us that landing conditions have to hold before triggering a land. */
	static constexpr hrt_abstime LANDED_TRIGGER_TIME_US = 2_s;
	static constexpr hrt_abstime FLYING_TRIGGER_TIME_US = 0_us;

	struct {
		param_t max_air_speed;
		param_t max_climb_rate;
		param_t max_velocity;
		param_t max_XY_accel;
	} _paramHandle{};

	struct {
		float max_air_speed;
		float max_climb_rate;
		float max_velocity;
		float max_XY_accel;
	} _params{};

	airspeed_s _airspeed{};

	float _accel_horz_lp{0.0f};
	float _airspeed_filtered{0.0f};
	float _velocity_xy_filtered{0.0f};
	float _velocity_z_filtered{0.0f};

	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
};

} // namespace land_detector
