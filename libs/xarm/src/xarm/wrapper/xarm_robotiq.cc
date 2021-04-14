/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

int XArmAPI::_robotiq_set(unsigned char *params, int length, unsigned char ret_data[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(115200) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	unsigned char *send_data = new unsigned char[7 + length];
	send_data[0] = 0x09;
	send_data[1] = 0x10;
	send_data[2] = 0x03;
	send_data[3] = 0xE8;
	send_data[4] = 0x00;
	send_data[5] = 0x03;
	send_data[6] = (unsigned char)length;
	for (int i = 0; i < length; i++) { send_data[7+i] = params[i]; }
	int ret = getset_tgpio_modbus_data(send_data, length + 7, ret_data, 6);
	delete[] send_data;
	return ret;
}
int XArmAPI::_robotiq_get(unsigned char ret_data[9], unsigned char number_of_registers) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (_checkset_modbus_baud(115200) != 0) return API_CODE::MODBUS_BAUD_NOT_CORRECT;
	unsigned char *send_data = new unsigned char[6];
	send_data[0] = 0x09;
	send_data[1] = 0x03;
	send_data[2] = 0x07;
	send_data[3] = 0xD0;
	send_data[4] = 0x00;
	send_data[5] = number_of_registers;
	int ret = getset_tgpio_modbus_data(send_data, 6, ret_data, 3 + 2 * number_of_registers);
	delete[] send_data;
	if (ret == 0) {
		if (number_of_registers >= 0x01) {
			robotiq_status.gOBJ = (ret_data[3] & 0xC0) >> 6;
			robotiq_status.gSTA = (ret_data[3] & 0x30) >> 4;
			robotiq_status.gGTO = (ret_data[3] & 0x08) >> 3;
			robotiq_status.gACT = ret_data[3] & 0x01;
		}
		if (number_of_registers >= 0x02) {
			robotiq_status.kFLT = (ret_data[5] & 0xF0) >> 4;
			robotiq_status.gFLT = ret_data[5] & 0x0F;
			robotiq_status.gPR = ret_data[6];
			robotiq_error_code_ = robotiq_status.gFLT;
		}
		if (number_of_registers >= 0x03) {
			robotiq_status.gPO = ret_data[7];
			robotiq_status.gCU = ret_data[8];
		}

		if (robotiq_status.gSTA == 3 && (robotiq_status.gFLT == 0 || robotiq_status.gFLT == 9)) {
			robotiq_is_activated_ = true;
		}
		else {
			robotiq_is_activated_ = false;
		}
	}
	return ret;
}

int XArmAPI::_robotiq_wait_activation_completed(fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	unsigned char rx_data[9] = { 0 };
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = robotiq_get_status(rx_data);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0) {
			code = (robotiq_status.gFLT != 0 && !(robotiq_status.gFLT == 5 && robotiq_status.gSTA == 1)) ? API_CODE::END_EFFECTOR_HAS_FAULT : robotiq_status.gSTA == 3 ? 0 : code;
		}
		else {
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(50);
	}
	return code;
}

int XArmAPI::_robotiq_wait_motion_completed(fp32 timeout, bool check_detected) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int failed_cnt = 0;
	long long expired = get_system_time() + (long long)(timeout * 1000);
	int code = API_CODE::WAIT_FINISH_TIMEOUT;
	unsigned char rx_data[9] = { 0 };
	while (timeout <= 0 || get_system_time() < expired) {
		int code2 = robotiq_get_status(rx_data);
		failed_cnt = code2 == 0 ? 0 : failed_cnt + 1;
		if (code2 == 0) {
			code = (robotiq_status.gFLT != 0 && !(robotiq_status.gFLT == 5 && robotiq_status.gSTA == 1)) ? API_CODE::END_EFFECTOR_HAS_FAULT :
				((check_detected && (robotiq_status.gOBJ == 1 || robotiq_status.gOBJ == 2)) || (robotiq_status.gOBJ == 1 || robotiq_status.gOBJ == 2 || robotiq_status.gOBJ == 3)) ? 0 : code;
		}
		else {
			code = code2 == API_CODE::NOT_CONNECTED ? API_CODE::NOT_CONNECTED : failed_cnt > 10 ? API_CODE::CHECK_FAILED : code;
		}
		if (code != API_CODE::WAIT_FINISH_TIMEOUT) break;
		sleep_milliseconds(50);
	}
	if (code == 0 && !robotiq_is_activated_) code = API_CODE::END_EFFECTOR_NOT_ENABLED;
	return code;
}

int XArmAPI::robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return _robotiq_get(ret_data, number_of_registers);
}

int XArmAPI::robotiq_reset(unsigned char ret_data[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	unsigned char rx_data[6] = { 0 };
	int ret = _robotiq_set(params, 6, rx_data);
	if (ret_data != NULL) { memcpy(ret_data, rx_data, 6); }
	return ret;
}

int XArmAPI::robotiq_set_activate(bool wait, fp32 timeout, unsigned char ret_data[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
	unsigned char rx_data[6] = { 0 };
	int ret = _robotiq_set(params, 6, rx_data);
	if (ret_data != NULL) { memcpy(ret_data, rx_data, 6); }
	if (wait && ret == 0) { ret = _robotiq_wait_activation_completed(timeout); }
	if (ret == 0) robotiq_is_activated_ = true;
	return ret;
}

int XArmAPI::robotiq_set_activate(bool wait, unsigned char ret_data[6]) {
	return robotiq_set_activate(wait, 3, ret_data);
}
int XArmAPI::robotiq_set_activate(unsigned char ret_data[6]) {
	return robotiq_set_activate(true, ret_data);
}

int XArmAPI::robotiq_set_position(unsigned char pos, unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char params[6] = { 0x09, 0x00, 0x00, pos, speed, force };
	unsigned char rx_data[6] = { 0 };
	bool has_error = error_code != 0;
	bool is_stop = state == 4 || state == 5;
	int code = _wait_move(NO_TIMEOUT);
	if (!(code == 0 || (is_stop && code == API_CODE::EMERGENCY_STOP) || (has_error && code == API_CODE::HAS_ERROR))) {
		return code;
	}
	int ret = _robotiq_set(params, 6, rx_data);
	if (ret_data != NULL) { memcpy(ret_data, rx_data, 6); }
	if (wait && ret == 0) { ret = _robotiq_wait_motion_completed(timeout); }
	return ret;
}

int XArmAPI::robotiq_set_position(unsigned char pos, bool wait, fp32 timeout, unsigned char ret_data[6]) {
	return robotiq_set_position(pos, 0xFF, 0xFF, wait, timeout, ret_data);
}
int XArmAPI::robotiq_set_position(unsigned char pos, bool wait, unsigned char ret_data[6]) {
	return robotiq_set_position(pos, wait, 5, ret_data);
}

int XArmAPI::robotiq_set_position(unsigned char pos, unsigned char ret_data[6]) {
	return robotiq_set_position(pos, true, ret_data);
}

int XArmAPI::robotiq_open(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6]) {
	return robotiq_set_position(0x00, speed, force, wait, timeout, ret_data);
}

int XArmAPI::robotiq_open(bool wait, fp32 timeout, unsigned char ret_data[6]) {
	return robotiq_set_position(0x00, wait, timeout, ret_data);
}

int XArmAPI::robotiq_open(bool wait, unsigned char ret_data[6]) {
	return robotiq_open(wait, 5, ret_data);
}

int XArmAPI::robotiq_open(unsigned char ret_data[6]) {
	return robotiq_open(true, ret_data);
}

int XArmAPI::robotiq_close(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6]) {
	return robotiq_set_position(0xFF, speed, force, wait, timeout, ret_data);
}

int XArmAPI::robotiq_close(bool wait, fp32 timeout, unsigned char ret_data[6]) {
	return robotiq_set_position(0xFF, wait, timeout, ret_data);
}

int XArmAPI::robotiq_close(bool wait, unsigned char ret_data[6]) {
	return robotiq_close(wait, 5, ret_data);
}

int XArmAPI::robotiq_close(unsigned char ret_data[6]) {
	return robotiq_close(true, ret_data);
}