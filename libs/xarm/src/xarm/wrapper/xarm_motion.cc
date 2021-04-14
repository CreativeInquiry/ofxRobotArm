/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


int XArmAPI::set_position(fp32 pose[6], fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	int ret = 0;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 mvpose[6];
	for (int i = 0; i < 6; i++) {
		last_used_position[i] = pose[i];
		mvpose[i] = (float)(default_is_radian || i < 3 ? last_used_position[i] : last_used_position[i] / RAD_DEGREE);
	}

	if (radius >= 0) {
		ret = core->move_lineb(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, radius);
	}
	else {
		ret = core->move_line(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);
	}
	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}

	return ret;
}

int XArmAPI::set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout) {
	return set_position(pose, radius, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_position(fp32 pose[6], bool wait, fp32 timeout) {
	return set_position(pose, -1, 0, 0, 0, wait, timeout);
}

int XArmAPI::set_tool_position(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 mvpose[6];
	for (int i = 0; i < 6; i++) {
		mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : pose[i] / RAD_DEGREE);
	}
	int ret = core->move_line_tool(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime);

	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}

	return ret;
}

int XArmAPI::set_tool_position(fp32 pose[6], bool wait, fp32 timeout) {
	return set_tool_position(pose, 0, 0, 0, wait, timeout);
}


int XArmAPI::set_servo_angle(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	last_used_joint_speed = speed > 0 ? speed : last_used_joint_speed;
	last_used_joint_acc = acc > 0 ? acc : last_used_joint_acc;
	fp32 mvjoint[7];
	for (int i = 0; i < 7; i++) {
		last_used_angles[i] = angs[i];
		mvjoint[i] = (float)(default_is_radian ? last_used_angles[i] : last_used_angles[i] / RAD_DEGREE);
	}
	fp32 speed_ = (float)(default_is_radian ? last_used_joint_speed : last_used_joint_speed / RAD_DEGREE);
	fp32 acc_ = (float)(default_is_radian ? last_used_joint_acc : last_used_joint_acc / RAD_DEGREE);

	int ret = 0;
	if (_version_is_ge(1, 5, 20) && radius >= 0) {
		ret = core->move_jointb(mvjoint, speed_, acc_, radius);
	}
	else {
		ret = core->move_joint(mvjoint, speed_, acc_, mvtime);
	}
	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}
	return ret;
}

int XArmAPI::set_servo_angle(fp32 angs[7], bool wait, fp32 timeout, fp32 radius) {
	return set_servo_angle(angs, 0, 0, 0, wait, timeout, radius);
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius) {
	assert(servo_id > 0 && servo_id <= 7);
	last_used_angles[servo_id - 1] = angle;
	return set_servo_angle(last_used_angles, speed, acc, mvtime, wait, timeout, radius);
}

int XArmAPI::set_servo_angle(int servo_id, fp32 angle, bool wait, fp32 timeout, fp32 radius) {
	return set_servo_angle(servo_id, angle, 0, 0, 0, wait, timeout, radius);
}

int XArmAPI::set_servo_angle_j(fp32 angs[7], fp32 speed, fp32 acc, fp32 mvtime) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 mvjoint[7];
	for (int i = 0; i < 7; i++) {
		mvjoint[i] = (float)(default_is_radian ? angs[i] : angs[i] / RAD_DEGREE);
	}
	return core->move_servoj(mvjoint, last_used_joint_speed, last_used_joint_acc, mvtime);
}

int XArmAPI::set_servo_cartesian(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 mvpose[6];
	for (int i = 0; i < 6; i++) {
		mvpose[i] = (float)(i < 3 || default_is_radian ? pose[i] : pose[i] / RAD_DEGREE);
	}
	mvtime = (float)(is_tool_coord ? 1.0 : 0.0);
	return core->move_servo_cartesian(mvpose, speed, acc, mvtime);
}

int XArmAPI::move_circle(fp32 pose1[6], fp32 pose2[6], fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 pose_1[6];
	fp32 pose_2[6];
	for (int i = 0; i < 6; i++) {
		pose_1[i] = (float)(default_is_radian || i < 3 ? pose1[i] : pose1[i] / RAD_DEGREE);
		pose_2[i] = (float)(default_is_radian || i < 3 ? pose2[i] : pose2[i] / RAD_DEGREE);
	}
	int ret = core->move_circle(pose_1, pose_2, last_used_tcp_speed, last_used_tcp_acc, mvtime, percent);
	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}

	return ret;
}

int XArmAPI::move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	fp32 speed_ = (float)(default_is_radian ? speed : speed / RAD_DEGREE);
	fp32 acc_ = (float)(default_is_radian ? acc : acc / RAD_DEGREE);
	speed_ = speed_ > 0 ? speed_ : (float)0.8726646259971648; // 50 °/s
	acc_ = acc_ > 0 ? acc_ : (float)17.453292519943297; // 1000 °/s^2
	int ret = core->move_gohome(speed_, acc_, mvtime);
	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}

	return ret;
}

int XArmAPI::move_gohome(bool wait, fp32 timeout) {
	return move_gohome(0, 0, 0, wait, timeout);
}

void XArmAPI::reset(bool wait, fp32 timeout) {
	int err_warn[2];
	int state_;
	if (!is_tcp_) {
		get_err_warn_code(err_warn);
		get_state(&state_);
	}
	if (warn_code != 0) {
		clean_warn();
	}
	if (error_code != 0) {
		clean_error();
		motion_enable(true, 8);
		set_mode(0);
		set_state(0);
	}
	if (!is_ready_) {
		motion_enable(true, 8);
		set_mode(0);
		set_state(0);
	}
	move_gohome(wait, timeout);
}

int XArmAPI::set_position_aa(fp32 pose[6], fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, bool relative, bool wait, fp32 timeout) {
	_check_is_pause();
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	last_used_tcp_speed = speed > 0 ? speed : last_used_tcp_speed;
	last_used_tcp_acc = acc > 0 ? acc : last_used_tcp_acc;
	fp32 mvpose[6];
	for (int i = 0; i < 6; i++) {
		mvpose[i] = (float)(default_is_radian || i < 3 ? pose[i] : pose[i] / RAD_DEGREE);
	}
	int ret = core->move_line_aa(mvpose, last_used_tcp_speed, last_used_tcp_acc, mvtime, (int)is_tool_coord, (int)relative);
	ret = _check_code(ret, true);
	if (wait && ret == 0) {
		ret = _wait_move(timeout);
		_sync();
	}

	return ret;
}

int XArmAPI::set_position_aa(fp32 pose[6], bool is_tool_coord, bool relative, bool wait, fp32 timeout) {
	return set_position_aa(pose, 0, 0, 0, is_tool_coord, relative, wait, timeout);
}

int XArmAPI::set_servo_cartesian_aa(fp32 pose[6], fp32 speed, fp32 acc, bool is_tool_coord, bool relative) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 mvpose[6];
	for (int i = 0; i < 6; i++) {
		mvpose[i] = (float)(i < 3 || default_is_radian ? pose[i] : pose[i] / RAD_DEGREE);
	}
	return core->move_servo_cart_aa(mvpose, speed, acc, (int)is_tool_coord, (int)relative);
}

int XArmAPI::set_servo_cartesian_aa(fp32 pose[6], bool is_tool_coord, bool relative) {
	return set_servo_cartesian_aa(pose, 0, 0, is_tool_coord, relative);
}

int XArmAPI::vc_set_cartesian_velocity(fp32 speeds[6], bool is_tool_coord) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 line_v[6];
	for (int i = 0; i < 6; i++) {
		line_v[i] = (float)((i < 3 || default_is_radian) ? speeds[i] : speeds[i] / RAD_DEGREE);
	}
	return core->vc_set_linev(line_v, is_tool_coord ? 1 : 0);
}

int XArmAPI::vc_set_joint_velocity(fp32 speeds[7], bool is_sync) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 jnt_v[7];
	for (int i = 0; i < 7; i++) {
		jnt_v[i] = (float)(default_is_radian ? speeds[i] : speeds[i] / RAD_DEGREE);
	}
	return core->vc_set_jointv(jnt_v, is_sync ? 1 : 0);
}