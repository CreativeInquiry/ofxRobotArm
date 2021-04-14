/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


int XArmAPI::_bio_gripper_send_modbus(unsigned char *send_data, int length, unsigned char *ret_data, int ret_length) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(2000000) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	int ret = getset_tgpio_modbus_data(send_data, length, ret_data, ret_length);
	return ret;
}

int XArmAPI::_get_bio_gripper_register(unsigned char *ret_data, int address, int number_of_registers) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x08, 0x03, (unsigned char)(address >> 8), (unsigned char)address, (unsigned char)(number_of_registers >> 8), (unsigned char)number_of_registers };
	return _bio_gripper_send_modbus(params, 6, ret_data, 3 + 2 * number_of_registers);
}

int XArmAPI::_bio_gripper_wait_motion_completed(fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	int status = BIO_STATE::IS_MOTION;
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = get_bio_gripper_status(&status);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0) {
			code = (status & 0x03) == BIO_STATE::IS_MOTION ? code : (status & 0x03) == BIO_STATE::IS_FAULT ? API_CODE::END_EFFECTOR_HAS_FAULT : 0;
		}
		else {
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(100);
	}
	if (code == 0 && !bio_gripper_is_enabled_) code = API_CODE::END_EFFECTOR_NOT_ENABLED;
	return code;
}

int XArmAPI::_bio_gripper_wait_enable_completed(fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	int status = BIO_STATE::IS_NOT_ENABLED;
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = get_bio_gripper_status(&status);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0) {
			code = bio_gripper_is_enabled_ ? 0 : code;
		}
		else {
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(100);
	}
	return code;
}

int XArmAPI::set_bio_gripper_enable(bool enable, bool wait, fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x08, 0x06, 0x01, 0x00, 0x00, (unsigned char)enable };
	unsigned char rx_data[6] = { 0 };
	int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
	if (ret == 0 && enable && wait) { ret = _bio_gripper_wait_enable_completed(timeout); }
	return ret;
}

int XArmAPI::set_bio_gripper_speed(int speed) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned short tmp = speed;
	unsigned char params[6] = { 0x08, 0x06, 0x03, 0x03, (unsigned char)(tmp >> 8), (unsigned char)tmp };
	unsigned char rx_data[6] = { 0 };
	int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
	if (ret == 0) { bio_gripper_speed_ = speed; }
	return ret;
}

int XArmAPI::_set_bio_gripper_position(int pos, int speed, bool wait, fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (speed > 0 && speed != bio_gripper_speed_) { set_bio_gripper_speed(speed); }
	unsigned char params[11] = { 0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04 };
	params[7] = (unsigned char)(pos >> 24);
	params[8] = (unsigned char)(pos >> 16);
	params[9] = (unsigned char)(pos >> 8);
	params[10] = (unsigned char)(pos);
	unsigned char rx_data[6] = { 0 };
	bool has_error = error_code != 0;
	bool is_stop = state == 4 || state == 5;
	int code = _wait_move(NO_TIMEOUT);
	if (!(code == 0 || (is_stop && code == API_CODE::EMERGENCY_STOP) || (has_error && code == API_CODE::HAS_ERROR))) {
		return code;
	}
	int ret = _bio_gripper_send_modbus(params, 11, rx_data, 6);
	if (ret == 0 && wait) { ret = _bio_gripper_wait_motion_completed(timeout); }
	return ret;
}

int XArmAPI::_set_bio_gripper_position(int pos, bool wait, fp32 timeout) {
	return _set_bio_gripper_position(pos, 0, wait, timeout);
}

int XArmAPI::open_bio_gripper(int speed, bool wait, fp32 timeout) {
	return _set_bio_gripper_position(130, speed, wait, timeout);
}

int XArmAPI::open_bio_gripper(bool wait, fp32 timeout) {
	return open_bio_gripper(bio_gripper_speed_, wait, timeout);
}

int XArmAPI::close_bio_gripper(int speed, bool wait, fp32 timeout) {
	return _set_bio_gripper_position(50, speed, wait, timeout);
}

int XArmAPI::close_bio_gripper(bool wait, fp32 timeout) {
	return close_bio_gripper(bio_gripper_speed_, wait, timeout);
}

int XArmAPI::get_bio_gripper_status(int *status) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char rx_data[5] = { 0 };
	int ret = _get_bio_gripper_register(rx_data, 0x00);
	*status = (rx_data[3] << 8) + rx_data[4];
	if (ret == 0) {
		if ((*status & 0x03) == BIO_STATE::IS_FAULT) {
			int err;
			get_bio_gripper_error(&err);
		}
		bio_gripper_error_code_ = (*status & 0x03) == BIO_STATE::IS_FAULT ? bio_gripper_error_code_ : 0;
		bio_gripper_is_enabled_ = ((*status >> 2) & 0x03) == BIO_STATE::IS_ENABLED ? true : false;
	}
	return ret;
}

int XArmAPI::get_bio_gripper_error(int *err) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char rx_data[5] = { 0 };
	int ret = _get_bio_gripper_register(rx_data, 0x0F);
	*err = (rx_data[3] << 8) + rx_data[4];
	if (ret == 0) bio_gripper_error_code_ = *err;
	return ret;
}

int XArmAPI::clean_bio_gripper_error(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x08, 0x06, 0x00, 0x0F, 0x00, 0x00 };
	unsigned char rx_data[6] = { 0 };
	int ret = _bio_gripper_send_modbus(params, 6, rx_data, 6);
	int status;
	get_bio_gripper_status(&status);
	return ret;
}