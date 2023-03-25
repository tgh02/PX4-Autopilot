/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file RateControl.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D, const Vector3f &FF)
{
    _gain_p = P;
    _gain_i = I;
    _gain_d = D;
    _gain_ff = FF;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
                                      const Vector<bool, 3> &saturation_negative)
{
    _control_allocator_saturation_positive = saturation_positive;
    _control_allocator_saturation_negative = saturation_negative;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
                             const float dt, const bool landed)
{
    // angular rates error
    Vector3f rate_error = rate_sp - rate;

    // PID control with feed forward
    const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

    // update integral only if we are not landed
    if (!landed) {
        updateIntegral(rate_error, dt);
    }

    return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
    for (int i = 0; i < 3; i++) {
        // Check if the control signal is saturated
        bool is_saturated = false;
        if (_control_allocator_saturation_positive(i)) {
            is_saturated = (_rate_int(i) >= _lim_int(i));
        } else if (_control_allocator_saturation_negative(i)) {
            is_saturated = (_rate_int(i) <= -_lim_int(i));
        }

        // Compute the I term factor
        float i_factor = rate_error(i) / math::radians(400.f);
        i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

        // Only update the integral term if the control signal is not saturated
        if (!is_saturated) {
            float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;
            _rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
        }
    }
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
    rate_ctrl_status.rollspeed_integ = _rate_int(0);
    rate_ctrl_status.pitchspeed_integ = _rate_int(1);
    rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
