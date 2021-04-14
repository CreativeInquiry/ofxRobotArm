/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

int XArmAPI::get_gripper_version(unsigned char versions[3]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char val1[5], val2[5], val3[5];
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	int ret1 = core->gripper_modbus_r16s(0x0801, 1, val1);
	int ret2 = core->gripper_modbus_r16s(0x0802, 1, val2);
	int ret3 = core->gripper_modbus_r16s(0x0803, 1, val3);
	ret1 = _check_modbus_code(ret1);
	ret2 = _check_modbus_code(ret2);
	ret3 = _check_modbus_code(ret3);
	if (ret1 == 0) { 
		versions[0] = (unsigned char)bin8_to_16(&val1[4]); 
		gripper_version_numbers_[0] = version[0];
	}
	else { code = ret1; }
	if (ret2 == 0) { 
		versions[1] = (unsigned char)bin8_to_16(&val2[4]);
		gripper_version_numbers_[1] = version[1];
	}
	else { code = ret2; }
	if (ret3 == 0) { 
		versions[2] = (unsigned char)bin8_to_16(&val3[4]);
		gripper_version_numbers_[2] = version[2];
	}
	else { code = ret3; }
	return code;
}


int XArmAPI::set_gripper_enable(bool enable) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_set_en(int(enable));
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	if (ret == 0 && xarm_gripper_error_code_ == 0) gripper_is_enabled_ = true;
	return xarm_gripper_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::set_gripper_mode(int mode) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_set_mode(mode);
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	return xarm_gripper_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::set_gripper_speed(fp32 speed) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_set_posspd(speed);
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	return xarm_gripper_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::get_gripper_position(fp32 *pos) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_get_pos(pos);
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	return xarm_gripper_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}

int XArmAPI::get_gripper_err_code(int *err) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_get_errcode(err);
	ret = _check_modbus_code(ret);
	if (ret == 0) {
		if (*err < 128) {
			xarm_gripper_error_code_ = *err;
			if (xarm_gripper_error_code_ != 0)
				gripper_is_enabled_ = false;
		}
	}
	return ret;
}

bool XArmAPI::_gripper_is_support_status(void) {
	if (gripper_version_numbers_[0] == -1 || gripper_version_numbers_[1] == -1 || gripper_version_numbers_[2] == -1) {
		unsigned char ver[3];
		get_gripper_version(ver);
	}
	return gripper_version_numbers_[0] > 3
		|| (gripper_version_numbers_[0] == 3 && gripper_version_numbers_[1] > 4)
		|| (gripper_version_numbers_[0] == 3 && gripper_version_numbers_[1] == 4 && gripper_version_numbers_[2] >= 3);
}

int XArmAPI::_get_gripper_status(int *status) {
	unsigned char val[5];
	int ret = core->gripper_modbus_r16s(0x0000, 1, val);
	ret = _check_modbus_code(ret);
	if (ret == 0) {
		*status = bin8_to_16(&val[4]);
	}
	return ret;
}

int XArmAPI::_check_gripper_position(fp32 target_pos, fp32 timeout) {
	int ret2 = 0;
	float last_pos = 0, pos_tmp, cur_pos;
	bool is_add = true;
	ret2 = get_gripper_position(&pos_tmp);
	if (ret2 == 0) {
		last_pos = pos_tmp;
		if (int(last_pos) == int(target_pos))
			return 0;
		is_add = target_pos > last_pos ? true : false;
	}

	int cnt = 0;
	int cnt2 = 0;
	int failed_cnt = 0;
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	while (timeout <= 0 || get_system_time() < expired) {
		ret2 = get_gripper_position(&pos_tmp);
		if (xarm_gripper_error_code_ != 0) return API_CODE::END_EFFECTOR_HAS_FAULT;
		failed_cnt = ret2 == 0 ? 0 : failed_cnt + 1;
		if (ret2 == 0) {
			cur_pos = pos_tmp;
			if (fabs(target_pos - cur_pos) < 1) {
				last_pos = cur_pos;
				return 0;
			}
			if (is_add) {
				if (cur_pos <= last_pos) {
					cnt += 1;
				}
				else if (cur_pos <= target_pos) {
					last_pos = cur_pos;
					cnt = 0;
					cnt2 = 0;
				}
				else {
					cnt2 += 1;
					if (cnt2 >= 10) {
						return 0;
					}
				}
			}
			else {
				if (cur_pos >= last_pos) {
					cnt += 1;
				}
				else if (cur_pos >= target_pos) {
					last_pos = cur_pos;
					cnt = 0;
					cnt2 = 0;
				}
				else {
					cnt2 += 1;
					if (cnt2 >= 10) {
						return 0;
					}
				}

			}
			if (cnt >= 8) {
				return 0;
			}
		}
		else {
			if (failed_cnt > 10) return API_CODE::CHECK_FAILED;
		}
		sleep_milliseconds(200);
	}
	return code;
}

int XArmAPI::_check_gripper_status(fp32 timeout) {
	bool start_move = false;
	int not_start_move_cnt = 0;
	int failed_cnt = 0;
	int ret;
	int status;
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	while (timeout <= 0 || get_system_time() < expired) {
		ret = _get_gripper_status(&status);
		failed_cnt = ret == 0 ? 0 : failed_cnt + 1;
		if (ret == 0) {
			if ((status & 0x03) == 0 || (status & 0x03) == 2) {
				if (start_move) return 0;
				not_start_move_cnt += 1;
				if (not_start_move_cnt > 20) return 0;
			}
			else if (!start_move) {
				not_start_move_cnt = 0;
				start_move = true;
			}
		}
		else {
			if (failed_cnt > 10) return API_CODE::CHECK_FAILED;
		}
		sleep_milliseconds(100);
	}
	return code;
}

int XArmAPI::set_gripper_position(fp32 pos, bool wait, fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	bool has_error = error_code != 0;
	bool is_stop = state == 4 || state == 5;
	int code = _wait_move(NO_TIMEOUT);
	if (!(code == 0 || (is_stop && code == API_CODE::EMERGENCY_STOP) || (has_error && code == API_CODE::HAS_ERROR))) {
		return code;
	}
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_set_pos(pos);
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	if (xarm_gripper_error_code_ != 0) return API_CODE::END_EFFECTOR_HAS_FAULT;
	if (wait && ret == 0) {
		if (_gripper_is_support_status()) {
			return _check_gripper_status(timeout);
		}
		else {
			return _check_gripper_position(pos, timeout);
		}
	}
	return ret;
}

int XArmAPI::clean_gripper_error(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = core->gripper_modbus_clean_err();
	int err;
	get_gripper_err_code(&err);
	ret = _check_modbus_code(ret);
	return xarm_gripper_error_code_ != 0 ? API_CODE::END_EFFECTOR_HAS_FAULT : ret;
}
