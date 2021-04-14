/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


static int BAUDRATES[13] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000 };

static int get_baud_inx(int baud) {
	for (int i = 0; i < 13; i++) { if (BAUDRATES[i] == baud) return i; }
	return -1;
}


int XArmAPI::get_tgpio_version(unsigned char versions[3]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	float val1, val2, val3;
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	int ret1 = core->tgpio_addr_r16(0x0801, &val1);
	int ret2 = core->tgpio_addr_r16(0x0802, &val2);
	int ret3 = core->tgpio_addr_r16(0x0803, &val3);
	if (ret1 == 0) { versions[0] = (unsigned char)val1; }
	else { code = ret1; }
	if (ret2 == 0) { versions[1] = (unsigned char)val2; }
	else { code = ret2; }
	if (ret3 == 0) { versions[2] = (unsigned char)val3; }
	else { code = ret3; }
	return code;
}


int XArmAPI::get_tgpio_digital(int *io0, int *io1) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->tgpio_get_digital(io0, io1);
}

int XArmAPI::set_tgpio_digital(int ionum, int value, float delay_sec) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	assert(ionum == 0 || ionum == 1);
	if (delay_sec > 0) {
		return core->tgpio_delay_set_digital(ionum + 1, value, delay_sec);
	}
	else {
		return core->tgpio_set_digital(ionum + 1, value);
	}
}

int XArmAPI::get_tgpio_analog(int ionum, float *value) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	assert(ionum == 0 || ionum == 1);
	if (ionum == 0) {
		return core->tgpio_get_analog1(value);
	}
	else {
		return core->tgpio_get_analog2(value);
	}
}

int XArmAPI::get_cgpio_digital(int *digitals, int *digitals2) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int tmp;
	int ret = core->cgpio_get_auxdigit(&tmp);
	for (int i = 0; i < 8; i++) {
		digitals[i] = tmp >> i & 0x0001;
	}
	if (digitals2 != NULL) {
		for (int i = 8; i < 16; i++) {
			digitals2[i-8] = tmp >> i & 0x0001;
		}
	}
	return ret;
}

int XArmAPI::get_cgpio_analog(int ionum, fp32 *value) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	assert(ionum == 0 || ionum == 1);
	if (ionum == 0) {
		return core->cgpio_get_analog1(value);
	}
	else {
		return core->cgpio_get_analog2(value);
	}
}

int XArmAPI::set_cgpio_digital(int ionum, int value, float delay_sec) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	assert(ionum >= 0 && ionum < 16);
	if (delay_sec > 0) {
		return core->cgpio_delay_set_digital(ionum, value, delay_sec);
	}
	else {
		return core->cgpio_set_auxdigit(ionum, value);
	}
}

int XArmAPI::set_cgpio_analog(int ionum, fp32 value) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	assert(ionum == 0 || ionum == 1);
	if (ionum == 0) {
		return core->cgpio_set_analog1(value);
	}
	else {
		return core->cgpio_set_analog2(value);
	}
}

int XArmAPI::set_cgpio_digital_input_function(int ionum, int fun) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	assert(ionum >= 0 && ionum < 16);
	return core->cgpio_set_infun(ionum, fun);
}

int XArmAPI::set_cgpio_digital_output_function(int ionum, int fun) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	assert(ionum >= 0 && ionum < 16);
	return core->cgpio_set_outfun(ionum, fun);
}

int XArmAPI::get_cgpio_state(int *state_, int *digit_io, fp32 *analog, int *input_conf, int *output_conf, int *input_conf2, int *output_conf2) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->cgpio_get_state(state_, digit_io, analog, input_conf, output_conf, input_conf2, output_conf2);
}


int XArmAPI::get_suction_cup(int *val) {
	int io1;
	return get_tgpio_digital(val, &io1);
}

int XArmAPI::set_suction_cup(bool on, bool wait, float timeout, float delay_sec) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	int code1, code2;
	if (on) {
		code1 = set_tgpio_digital(0, 1, delay_sec);
		code2 = set_tgpio_digital(1, 0, delay_sec);
	}
	else {
		code1 = set_tgpio_digital(0, 0, delay_sec);
		code2 = set_tgpio_digital(1, 1, delay_sec);
	}
	int code = code1 == 0 ? code2 : code1;
	if (code == 0 && wait) {
		long long start_time = get_system_time();
		int val, ret;
		code = API_CODE::SUCTION_CUP_TOUT;
		while (get_system_time() - start_time < timeout * 1000) {
			ret = get_suction_cup(&val);
			if (ret == UXBUS_STATE::ERR_CODE) {
				code = UXBUS_STATE::ERR_CODE;
				break;
			}
			if (ret == 0) {
				if (on && val == 1) {
					code = 0;
					break;
				}
				if (!on && val == 0) {
					code = 0;
					break;
				}
			}
			sleep_milliseconds(100);
		}
	}

	return code;
}

int XArmAPI::set_tgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->tgpio_position_set_digital(ionum, value, xyz, tol_r);
}

int XArmAPI::set_cgpio_digital_with_xyz(int ionum, int value, float xyz[3], float tol_r) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->cgpio_position_set_digital(ionum, value, xyz, tol_r);
}

int XArmAPI::set_cgpio_analog_with_xyz(int ionum, float value, float xyz[3], float tol_r) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	return core->cgpio_position_set_analog(ionum, value, xyz, tol_r);
}

int XArmAPI::_check_modbus_code(int ret, unsigned char *rx_data) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (ret == 0 || ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
		if (rx_data != NULL && rx_data[0] != UXBUS_CONF::TGPIO_ID)
			return API_CODE::TGPIO_ID_ERR;
		if (ret != 0) {
			if (error_code != 19 && error_code != 28) {
				int err_warn[2] = { 0 };
				get_err_warn_code(err_warn);
			}
			return (error_code != 19 && error_code != 28) ? 0 : ret;
		}
	}
	return ret;
}

int XArmAPI::_get_modbus_baudrate(int *baud_inx) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	float val;
	int ret = core->tgpio_addr_r16(SERVO3_RG::MODBUS_BAUDRATE & 0x0FFF, &val);
	*baud_inx = (int)val;
	if (ret == UXBUS_STATE::ERR_CODE || ret == UXBUS_STATE::WAR_CODE) {
		if (error_code != 19 && error_code != 28) {
			int err_warn[2] = { 0 };
			get_err_warn_code(err_warn);
		}
		ret = (error_code != 19 && error_code != 28) ? 0 : ret;
	}
	if (ret == 0 && *baud_inx >= 0 && *baud_inx < 13) modbus_baud_ = BAUDRATES[*baud_inx];
	return ret;
}

int XArmAPI::_checkset_modbus_baud(int baudrate, bool check) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	if (check && modbus_baud_ == baudrate)
		return 0;
	int baud_inx = get_baud_inx(baudrate);
	if (baud_inx == -1) return API_CODE::MODBUS_BAUD_NOT_SUPPORT;
	int cur_baud_inx;
	int ret = _get_modbus_baudrate(&cur_baud_inx);
	if (ret == 0) {
		if (cur_baud_inx != baud_inx) {
			try {
				ignore_error_ = true;
				ignore_state_ = (state != 4 && state != 5) ? true : false;
				int state_ = state;
				// core->tgpio_addr_w16(SERVO3_RG::MODBUS_BAUDRATE, (float)baud_inx);
				core->tgpio_addr_w16(0x1a0b, (float)baud_inx);
				sleep_milliseconds(300);
				core->tgpio_addr_w16(SERVO3_RG::SOFT_REBOOT, 1);
				int err_warn[2] = { 0 };
				get_err_warn_code(err_warn);
				if (error_code == 19 || error_code == 28) {
					clean_error();
					if (ignore_state_) set_state(state_ >= 3 ? state_ : 0);
					sleep_milliseconds(1000);
				}
			}
			catch (...) {
				ignore_error_ = false;
				ignore_state_ = false;
				return API_CODE::API_EXCEPTION;
			}
			ignore_error_ = false;
			ignore_state_ = false;
			ret = _get_modbus_baudrate(&cur_baud_inx);
		}
		if (ret == 0 && cur_baud_inx < 13) modbus_baud_ = BAUDRATES[cur_baud_inx];
	}
	return modbus_baud_ == baudrate ? 0 : API_CODE::MODBUS_BAUD_NOT_CORRECT;
}

int XArmAPI::set_tgpio_modbus_timeout(int timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_modbus_timeout(timeout);
}

int XArmAPI::set_tgpio_modbus_baudrate(int baud) {
	return _checkset_modbus_baud(baud, false);
}

int XArmAPI::get_tgpio_modbus_baudrate(int *baud) {
	int cur_baud_inx;
	int ret = _get_modbus_baudrate(&cur_baud_inx);
	if (ret == 0 && cur_baud_inx < 13) 
		modbus_baud_ = BAUDRATES[cur_baud_inx];
	*baud = modbus_baud_;
	return ret;
}

int XArmAPI::getset_tgpio_modbus_data(unsigned char *modbus_data, int modbus_length, unsigned char *ret_data, int ret_length) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	unsigned char *rx_data = new unsigned char[ret_length + 1];
	int ret = core->tgpio_set_modbus(modbus_data, modbus_length, rx_data);
	ret = _check_modbus_code(ret, rx_data);
	memcpy(ret_data, rx_data + 1, ret_length);
	delete[] rx_data;
	return ret;
}
