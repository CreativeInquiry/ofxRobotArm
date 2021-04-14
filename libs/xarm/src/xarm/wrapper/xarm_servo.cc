/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"

int XArmAPI::get_servo_version(unsigned char versions[3], int servo_id) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	float val1, val2, val3;
	int code;
	versions[0] = 0;
	versions[1] = 0;
	versions[2] = 0;
	int ret1 = core->servo_addr_r16(servo_id, 0x0801, &val1);
	int ret2 = core->servo_addr_r16(servo_id, 0x0802, &val2);
	int ret3 = core->servo_addr_r16(servo_id, 0x0803, &val3);
	if (ret1 == 0) { versions[0] = (unsigned char)val1; }
	else { code = ret1; }
	if (ret2 == 0) { versions[1] = (unsigned char)val2; }
	else { code = ret2; }
	if (ret3 == 0) { versions[2] = (unsigned char)val3; }
	else { code = ret3; }
	return code;
}
