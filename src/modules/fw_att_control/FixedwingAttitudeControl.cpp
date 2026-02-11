/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingAttitudeControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::radians;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
	_landing_gear_wheel_pub.advertise();
	_attitude_sp_pub.advertise();
	_debug_vect_pub.advertise();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedwingAttitudeControl::parameters_update()
{
	_proportional_gain = matrix::Vector3f(1.0f / _param_fw_r_tc.get(),
					      1.0f / _param_fw_p_tc.get(),
					      1.0f / _param_fw_y_tc.get());

	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));

	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));

	_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));
	_wheel_ctrl.set_time_constant(0.1f);
}

void
FixedwingAttitudeControl::vehicle_manual_poll(const float yaw_body)
{
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vcontrol_mode.flag_control_climb_rate_enabled && _vcontrol_mode.flag_control_attitude_enabled) {

				// STABILIZED mode generate the attitude setpoint from manual user inputs

				const float roll_body = _manual_control_setpoint.roll * radians(_param_fw_man_r_max.get());

				float pitch_body = -_manual_control_setpoint.pitch * radians(_param_fw_man_p_max.get())
						   + radians(_param_fw_psp_off.get());
				pitch_body = constrain(pitch_body,
						       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

				_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.timestamp = hrt_absolute_time();

				_attitude_sp_pub.publish(_att_sp);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_constrained()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	return math::constrain(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_max.get());
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	if (_att_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		const bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_attitude_s att{};

		if (_att_sub.copy(&att)) {
			dt = math::constrain((att.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = att.timestamp_sample;

			// get current rotation matrix from control state quaternions
			_R = matrix::Quatf(att.q);
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		if (_vehicle_status.is_vtol_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = _R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = _R(0, 2);
			R_adapted(1, 0) = _R(1, 2);
			R_adapted(2, 0) = _R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = _R(0, 0);
			R_adapted(1, 2) = _R(1, 0);
			R_adapted(2, 2) = _R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			_R = R_adapted;
		}

		const matrix::Eulerf euler_angles(_R);

		vehicle_manual_poll(euler_angles.psi());

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		if (_vcontrol_mode.flag_control_rates_enabled) {

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL or tailsitter)
			 */
			if (_landed
			    || !_in_fw_or_transition_wo_tailsitter_transition) {

				_rates_sp.reset_integral = true;
				_wheel_ctrl.reset_integrator();

			} else {
				_rates_sp.reset_integral = false;
			}

			/* Run attitude controllers */

			if (_vcontrol_mode.flag_control_attitude_enabled && _in_fw_or_transition_wo_tailsitter_transition) {
				const Quatf q_sp(_att_sp.q_d);

				if (q_sp.isAllFinite()) {

					///////////////////////////////////
					const Eulerf euler_sp(q_sp);
					const float roll_sp = euler_sp.phi();
					const float pitch_sp = euler_sp.theta();

					_roll_ctrl.control_roll(roll_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								euler_angles.theta());
					_pitch_ctrl.control_pitch(pitch_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								  euler_angles.theta());
					_yaw_ctrl.control_yaw(roll_sp, _pitch_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
							      euler_angles.theta(), get_airspeed_constrained());

					/* Update input data for rate controllers */
					Vector3f _euler_body_rates_sp = Vector3f(_roll_ctrl.get_body_rate_setpoint(), _pitch_ctrl.get_body_rate_setpoint(),
									_yaw_ctrl.get_body_rate_setpoint());

					_euler_rates_sp.timestamp = hrt_absolute_time();
					_euler_rates_sp.roll_rate = _euler_body_rates_sp(0);
					_euler_rates_sp.pitch_rate = _euler_body_rates_sp(1);
					_euler_rates_sp.yaw_rate = _euler_body_rates_sp(2);
					///////////////////////////////////


					///////////////////////////////////////////////////////////////////////////
					// Quaternion / geometric tilt-priority attitude controller
					///////////////////////////////////////////////////////////////////////////

					// Current attitude
					const Quatf q_current(att.q);

					// Desired attitude setpoint (full)
					const Quatf q_desired = q_sp;

					// World "up" axis (keep as in your code)
					const Vector3f ez_world(0.f, 0.f, 1.f);

					// Current body axes expressed in world frame (from quaternion body->world)
					const Vector3f ez_current = q_current.dcm_z(); // body +Z in world
					const Vector3f ex_current = q_current.dcm_x(); // body +X in world

					// ------------------------------------------------------------
					// 0) Blend weight: level (lift) vs vertical (thrust)
					// ------------------------------------------------------------
					// cos_tilt ~ 1 in upright level-ish flight, ~0 when vertical.
					// (Uses current attitude only; can also incorporate airspeed if you want.)
					const float cos_tilt = ez_current.dot(ez_world);
					const float w_lift = math::constrain((fabsf(cos_tilt) - 0.2f) / (0.6f - 0.2f), 0.f, 1.f);
					// w_lift = 1 -> track lift axis (-Zb) like normal FW tilt control
					// w_lift = 0 -> track thrust axis (+Xb) for vertical flight

					// ------------------------------------------------------------
					// 1) Define primary "force" axis in BODY frame and rotate to world
					// ------------------------------------------------------------
					// PX4 body frame is typically FRD: +X fwd, +Y right, +Z down.
					const Vector3f ex_b(1.f, 0.f, 0.f);
					const Vector3f ez_b(0.f, 0.f, 1.f);

					// Lift axis (direction of aerodynamic lift in body coordinates) ~ "up" in body = -Zb
					const Vector3f lift_axis_b = -ez_b;

					// Thrust axis (direction of prop thrust) ~ +Xb
					const Vector3f thrust_axis_b = ex_b;

					// Primary controlled axis in BODY frame (blend lift/thrust)
					Vector3f a_body = lift_axis_b * w_lift + thrust_axis_b * (1.f - w_lift);
					const float a_norm = a_body.norm();
					if (a_norm > 1e-4f) {
						a_body /= a_norm;
					} else {
						a_body = lift_axis_b;
					}

					// Primary axis in WORLD frame (current & desired)
					const Vector3f a_current_w = q_current.rotateVector(a_body);
					const Vector3f a_desired_w = q_desired.rotateVector(a_body);

					// ------------------------------------------------------------
					// 2) Primary-axis geometric error -> "tilt" error in BODY frame
					// ------------------------------------------------------------
					// Raw error axis in world: a_current x a_desired
					Vector3f e_primary_b = q_current.rotateVectorInverse(a_current_w.cross(a_desired_w));

					// Remove component about the primary axis (yaw-about-axis handled separately)
					const float e_along = e_primary_b.dot(a_body);
					Vector3f e_tilt = e_primary_b - a_body * e_along;

					// Debug (optional)
					debug_vect_s dbg_e_tilt{};
					dbg_e_tilt.timestamp = hrt_absolute_time();
					strncpy(dbg_e_tilt.name, "e_tilt", sizeof(dbg_e_tilt.name));
					dbg_e_tilt.name[sizeof(dbg_e_tilt.name) - 1] = '\0';
					dbg_e_tilt.x = e_tilt(0);
					dbg_e_tilt.y = e_tilt(1);
					dbg_e_tilt.z = e_tilt(2);
					_debug_vect_pub.publish(dbg_e_tilt);

					// ------------------------------------------------------------
					// 3) Tilt-rate command from tilt error (BODY rates)
					// ------------------------------------------------------------
					// Extract roll component, scale down to prioritize pitch, then recombine
					const float roll_error = e_tilt(0);
					Vector3f e_tilt_scaled = e_tilt;
					e_tilt_scaled(0) = roll_error * _param_fw_geom_r_scale.get();

					Vector3f body_rates_setpoint;
					body_rates_setpoint(0) = _proportional_gain(0) * e_tilt_scaled(0);
					body_rates_setpoint(1) = _proportional_gain(1) * e_tilt_scaled(1);
					body_rates_setpoint(2) = _proportional_gain(2) * e_tilt_scaled(2);

					// ------------------------------------------------------------
					// 4) Yaw control about the PRIMARY axis (generalized yaw)
					// ------------------------------------------------------------
					auto yaw_error_about_axis = [](const Vector3f &axis_world,
								const Vector3f &ref_current_world,
								const Vector3f &ref_desired_world) -> float
					{
						Vector3f ax = axis_world;
						const float a_nor = ax.norm();
						if (a_nor < 1e-4f) { return 0.f; }
						ax /= a_nor;

						// Project references into plane orthogonal to axis
						Vector3f u = ref_current_world - ax * (ref_current_world.dot(ax));
						Vector3f v = ref_desired_world - ax * (ref_desired_world.dot(ax));

						const float un = u.norm();
						const float vn = v.norm();
						if (un < 1e-4f || vn < 1e-4f) { return 0.f; }

						u /= un;
						v /= vn;

						const float sinang = ax.dot(u.cross(v));
						const float cosang = u.dot(v);
						return atan2f(sinang, cosang);
					};

					// Secondary reference axis in BODY to define yaw-about-primary.
					// Level (lift) -> use nose direction (+Xb) for heading
					// Vertical (thrust) -> use "up" (-Zb) to define rotation about +Xb
					Vector3f b_body = ex_b * w_lift + (-ez_b) * (1.f - w_lift);

					// Orthogonalize b_body against a_body (Gram-Schmidt)
					b_body -= a_body * (b_body.dot(a_body));
					const float b_norm = b_body.norm();
					if (b_norm > 1e-4f) {
						b_body /= b_norm;
					} else {
						// Fallback: choose Y axis, then orthogonalize once more
						b_body = Vector3f(0.f, 1.f, 0.f);
						b_body -= a_body * (b_body.dot(a_body));
						const float b2 = b_body.norm();
						if (b2 > 1e-4f) { b_body /= b2; }
					}

					// Rotate b_body to world for current/desired
					const Vector3f b_current_w = q_current.rotateVector(b_body);
					const Vector3f b_desired_w = q_desired.rotateVector(b_body);

					// Yaw error about desired primary axis
					const float yaw_err = yaw_error_about_axis(a_desired_w, b_current_w, b_desired_w);

					// Yaw-hold about primary axis (used strongly near vertical)
					const float K_yaw = _proportional_gain(2);

					Vector3f yaw_axis_b = ez_b * w_lift + a_body * (1.f - w_lift);
					yaw_axis_b.normalize();

					body_rates_setpoint += yaw_axis_b * (K_yaw * yaw_err);

					// ------------------------------------------------------------
					// 5) Turn coordination feedforward (gated to level flight)
					// ------------------------------------------------------------
					// Keep your existing heading-aligned bank estimation (good!).
					const float V = math::max(get_airspeed_constrained(), 0.1f);

					float tan_bank = 0.f;
					float tc_gate = 0.f;

					// heading-aligned horizontal frame using current forward axis
					Vector3f x_h = ex_current - ez_world * ex_current.dot(ez_world);
					const float x_h_norm = x_h.norm();

					if (x_h_norm > 1e-3f) {
						x_h /= x_h_norm;
						const Vector3f y_h = ez_world.cross(x_h);

						// Reuse cos_tilt from above
						// For FRD body in NED world, sign may need inversion; keep your original sign convention.
						const float sin_bank = -ez_current.dot(y_h);

						if (fabsf(cos_tilt) > 0.1f) {
							tan_bank = sin_bank / cos_tilt;
						}

						// Gate TC out near vertical (same shape as w_lift, but keep separate if you like)
						tc_gate = math::constrain((fabsf(cos_tilt) - 0.2f) / (0.6f - 0.2f), 0.f, 1.f);
					}

					const float r_tc_ff = (9.81f / V) * tan_bank;

					// Add TC as classic yaw-rate about BODY Z (valid near level flight)
					body_rates_setpoint(2) += tc_gate * r_tc_ff;

					// ------------------------------------------------------------
					// 6) Clamp body rates (unchanged)
					// ------------------------------------------------------------
					body_rates_setpoint(0) = math::constrain(body_rates_setpoint(0),
						-radians(_param_fw_r_rmax.get()), radians(_param_fw_r_rmax.get()));

					body_rates_setpoint(1) = math::constrain(body_rates_setpoint(1),
						-radians(_param_fw_p_rmax_neg.get()), radians(_param_fw_p_rmax_pos.get()));

					body_rates_setpoint(2) = math::constrain(body_rates_setpoint(2),
						-radians(_param_fw_y_rmax.get()), radians(_param_fw_y_rmax.get()));


					///////////////////////////////////////////////////////////////////////////
					// End: Quaternion / geometric controller
					///////////////////////////////////////////////////////////////////////////





					autotune_attitude_control_status_s pid_autotune;
					matrix::Vector3f bodyrate_autotune_ff;

					if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
						if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
						    && ((hrt_absolute_time() - pid_autotune.timestamp) < 1_s)) {

							bodyrate_autotune_ff = matrix::Vector3f(pid_autotune.rate_sp);
							body_rates_setpoint += bodyrate_autotune_ff;
						}
					}

					/* add yaw rate setpoint from sticks in all attitude-controlled modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						body_rates_setpoint(2) += math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
									  -radians(_param_fw_y_rmax.get()), radians(_param_fw_y_rmax.get()));
					}

					// Tailsitter: transform from FW to hover frame (all interfaces are in hover (body) frame)
					if (_vehicle_status.is_vtol_tailsitter) {
						body_rates_setpoint = Vector3f(body_rates_setpoint(2), body_rates_setpoint(1), -body_rates_setpoint(0));
					}

					/* Publish the rate setpoint for analysis once available */
					_rates_sp.roll = body_rates_setpoint(0);
					_rates_sp.pitch = body_rates_setpoint(1);
					_rates_sp.yaw = body_rates_setpoint(2);

					_rates_sp.timestamp = hrt_absolute_time();

					_rate_sp_pub.publish(_rates_sp);
					_euler_rates_sp_pub.publish(_euler_rates_sp);
				}
			}
		}

		// steering wheel control
		fixed_wing_runway_control_s runway_control{};
		_fixed_wing_runway_control_sub.copy(&runway_control);
		const bool runway_control_recent = hrt_elapsed_time(&runway_control.timestamp) < 1_s;
		const bool wheel_controller_enabled = _param_fw_w_en.get() && _vcontrol_mode.flag_control_auto_enabled
						      && runway_control_recent && runway_control.wheel_steering_enabled;

		float groundspeed_scale = 1.f;
		float wheel_u = 0.f;

		if (wheel_controller_enabled) {
			if (_local_pos_sub.updated()) {
				vehicle_local_position_s vehicle_local_position;

				if (_local_pos_sub.copy(&vehicle_local_position)) {
					_groundspeed = sqrtf(vehicle_local_position.vx * vehicle_local_position.vx + vehicle_local_position.vy *
							     vehicle_local_position.vy);
				}
			}

			// Use stall airspeed to calculate ground speed scaling region. Don't scale below gspd_scaling_trim
			float gspd_scaling_trim = (_param_fw_airspd_stall.get());

			if (_groundspeed > gspd_scaling_trim) {
				groundspeed_scale = gspd_scaling_trim / _groundspeed;

			}

			// set now yaw setpoint once we're entering the first time
			if (!PX4_ISFINITE(_steering_wheel_yaw_setpoint)) {
				_steering_wheel_yaw_setpoint = euler_angles.psi();
			}

			_wheel_ctrl.control_attitude(_steering_wheel_yaw_setpoint, euler_angles.psi());

			vehicle_angular_velocity_s angular_velocity{};
			_vehicle_rates_sub.copy(&angular_velocity);

			const float wheel_controller_output = wheel_controller_enabled ? _wheel_ctrl.control_bodyrate(dt,
							      angular_velocity.xyz[2], _groundspeed,
							      groundspeed_scale) : 0.f;

			wheel_u = wheel_controller_output + runway_control.wheel_steering_nudging_rate;

		} else {
			_wheel_ctrl.reset_integrator();
			_steering_wheel_yaw_setpoint = NAN;
			wheel_u = _manual_control_setpoint.yaw; // direct yaw stick to wheel steering
		}

		_landing_gear_wheel.normalized_wheel_setpoint = PX4_ISFINITE(wheel_u) ? wheel_u : 0.f;
		_landing_gear_wheel.timestamp = hrt_absolute_time();
		_landing_gear_wheel_pub.publish(_landing_gear_wheel);
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

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

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
