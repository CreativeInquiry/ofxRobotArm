/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


int XArmAPI::set_collision_sensitivity(int sensitivity) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_collis_sens(sensitivity);
}

int XArmAPI::set_teach_sensitivity(int sensitivity) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_teach_sens(sensitivity);
}

int XArmAPI::set_gravity_direction(fp32 gravity_dir[3]) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_gravity_dir(gravity_dir);
}

int XArmAPI::clean_conf(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->clean_conf();
}

int XArmAPI::save_conf(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->save_conf();
}

int XArmAPI::set_tcp_offset(fp32 pose_offset[6]) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 offset[6];
	for (int i = 0; i < 6; i++) {
		offset[i] = (float)(default_is_radian || i < 3 ? pose_offset[i] : pose_offset[i] / RAD_DEGREE);
	}
	_wait_move(NO_TIMEOUT);
	return core->set_tcp_offset(offset);
}

int XArmAPI::set_tcp_load(fp32 weight, fp32 center_of_gravity[3]) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	float _gravity[3];
	if (compare_version(version_number, new int[3]{ 0, 2, 0 })) {
		_gravity[0] = center_of_gravity[0];
		_gravity[1] = center_of_gravity[1];
		_gravity[2] = center_of_gravity[2];
	}
	else {
		_gravity[0] = (float)(center_of_gravity[0] / 1000.0);
		_gravity[1] = (float)(center_of_gravity[1] / 1000.0);
		_gravity[2] = (float)(center_of_gravity[2] / 1000.0);
	}
	return core->set_tcp_load(weight, _gravity);
}

int XArmAPI::set_tcp_jerk(fp32 jerk) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->set_tcp_jerk(jerk);
}

int XArmAPI::set_tcp_maxacc(fp32 acc) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->set_tcp_maxacc(acc);
}

int XArmAPI::set_joint_jerk(fp32 jerk) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->set_joint_jerk(default_is_radian ? jerk : (float)(jerk / RAD_DEGREE));
}

int XArmAPI::set_joint_maxacc(fp32 acc) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->set_joint_maxacc(default_is_radian ? acc : (float)(acc / RAD_DEGREE));
}

int XArmAPI::get_reduced_mode(int *mode) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->get_reduced_mode(mode);
}

int XArmAPI::get_reduced_states(int *on, int *xyz_list, float *tcp_speed, float *joint_speed, float jrange[14], int *fense_is_on, int *collision_rebound_is_on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_reduced_states(on, xyz_list, tcp_speed, joint_speed, jrange, fense_is_on, collision_rebound_is_on, _version_is_ge() ? 79 : 21);
	if (!default_is_radian) {
		*joint_speed = (float)(*joint_speed * RAD_DEGREE);
	}
	if (_version_is_ge()) {
		if (jrange != NULL && !default_is_radian) {
			for (int i = 0; i < 14; i++) {
				jrange[i] = (float)(jrange[i] * RAD_DEGREE);
			}
		}
	}
	return ret;
}

int XArmAPI::set_reduced_mode(bool on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_reduced_mode(int(on));
}

int XArmAPI::set_reduced_max_tcp_speed(float speed) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_reduced_linespeed(speed);
}

int XArmAPI::set_reduced_max_joint_speed(float speed) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_reduced_jointspeed(default_is_radian ? speed : (float)(speed / RAD_DEGREE));
}


int XArmAPI::set_reduced_tcp_boundary(int boundary[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_xyz_limits(boundary);
}

int XArmAPI::set_reduced_joint_range(float jrange[14]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	float joint_range[14];
	for (int i = 0; i < 14; i++) {
		joint_range[i] = default_is_radian ? jrange[i] : (float)(jrange[i] / RAD_DEGREE);
	}
	return core->set_reduced_jrange(joint_range);
}

int XArmAPI::set_fense_mode(bool on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_fense_on(int(on));
}

int XArmAPI::set_collision_rebound(bool on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_collis_reb(int(on));
}

int XArmAPI::set_world_offset(float pose_offset[6]) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 offset[6];
	for (int i = 0; i < 6; i++) {
		offset[i] = default_is_radian || i < 3 ? pose_offset[i] : (float)(pose_offset[i] / RAD_DEGREE);
	}
	return core->set_world_offset(offset);
}

int XArmAPI::config_tgpio_reset_when_stop(bool on_off) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->config_io_stop_reset(1, int(on_off));
}

int XArmAPI::config_cgpio_reset_when_stop(bool on_off) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->config_io_stop_reset(0, int(on_off));
}


int XArmAPI::get_report_tau_or_i(int *tau_or_i) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->get_report_tau_or_i(tau_or_i);
}

int XArmAPI::set_report_tau_or_i(int tau_or_i) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_report_tau_or_i(tau_or_i);
}

int XArmAPI::set_self_collision_detection(bool on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_self_collision_detection((int)on);
}

int XArmAPI::set_collision_tool_model(int tool_type, int n, ...) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (tool_type < COLLISION_TOOL_TYPE::USE_PRIMITIVES) {
		return core->set_collision_tool_model(tool_type);
	}
	assert(n > (tool_type == COLLISION_TOOL_TYPE::BOX ? 2 : tool_type == COLLISION_TOOL_TYPE::CYLINDER ? 1 : 0));
	fp32 *params = new fp32[n];
	va_list args;
	va_start(args, n);
	int inx = 0;
	while (inx < n)
	{
		params[inx] = (fp32)va_arg(args, double);
		inx++;
	}
	va_end(args);
	int ret = core->set_collision_tool_model(tool_type, n, params);
	delete[] params;
	return ret;
}