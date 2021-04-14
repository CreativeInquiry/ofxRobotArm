/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


int XArmAPI::start_record_trajectory(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_record_traj(1);
}

int XArmAPI::stop_record_trajectory(char* filename) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_record_traj(0);
	if (ret == 0 && filename != NULL) {
		int ret2 = save_record_trajectory(filename, 10);
		return ret2;
	}
	return ret;
}

int XArmAPI::save_record_trajectory(char* filename, float timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->save_traj(filename);
	if (ret == 0) {
		int ret2 = 0;
		int status = 0;
		long long start_time = get_system_time();
		while (get_system_time() - start_time < timeout * 1000) {
			ret2 = get_trajectory_rw_status(&status);
			if (ret2 == 0) {
				if (status == TRAJ_STATE::IDLE) {
					return API_CODE::TRAJ_RW_FAILED;
				}
				else if (status == TRAJ_STATE::SAVE_SUCCESS) {
					return 0;
				}
				else if (status == TRAJ_STATE::SAVE_FAIL) {
					return API_CODE::TRAJ_RW_FAILED;
				}
			}
			sleep_milliseconds(100);
		}
		return API_CODE::TRAJ_RW_TOUT;
	}
	return ret;
}

int XArmAPI::load_trajectory(char* filename, float timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->load_traj(filename);
	if (ret == 0) {
		int ret2 = 0;
		int status = 0;
		long long start_time = get_system_time();
		while (get_system_time() - start_time < timeout * 1000) {
			ret2 = get_trajectory_rw_status(&status);
			if (ret2 == 0) {
				if (status == TRAJ_STATE::IDLE) {
					return API_CODE::TRAJ_RW_FAILED;
				}
				else if (status == TRAJ_STATE::LOAD_SUCCESS) {
					return 0;
				}
				else if (status == TRAJ_STATE::LOAD_FAIL) {
					return API_CODE::TRAJ_RW_FAILED;
				}
			}
			sleep_milliseconds(100);
		}
		return API_CODE::TRAJ_RW_TOUT;
	}
	return ret;
}

int XArmAPI::playback_trajectory(int times, char* filename, bool wait, int double_speed) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = 0;
	if (filename != NULL) {
		ret = load_trajectory(filename, 10);
		if (ret != 0) {
			return ret;
		}
	}
	if (state == 4) return API_CODE::NOT_READY;
	ret = core->playback_traj(times, double_speed);
	if (ret == 0 && wait) {
		long long start_time = get_system_time();
		while (state != 1) {
			if (state == 4) return API_CODE::NOT_READY;
			if (get_system_time() - start_time > 5000) return API_CODE::TRAJ_PLAYBACK_TOUT;
			sleep_milliseconds(100);
		}
		int max_count = int((get_system_time() - start_time) * 100);
		max_count = max_count > 10 ? max_count : 10;
		start_time = get_system_time();
		while (mode != 11) {
			if (state == 1) {
				start_time = get_system_time();
				sleep_milliseconds(100);
				continue;
			}
			if (state == 4) {
				return API_CODE::NOT_READY;
			}
			if (get_system_time() - start_time > 5000) {
				return API_CODE::TRAJ_PLAYBACK_TOUT;
			}
			sleep_milliseconds(100);
		}
		sleep_milliseconds(100);
		int cnt = 0;
		while (state != 4) {
			if (state == 2) {
				if (times == 1) break;
				cnt += 1;
			}
			else {
				cnt = 0;
			}
			if (cnt > max_count) break;
			sleep_milliseconds(100);
		}
		if (state != 4) {
			set_mode(0);
			set_state(0);
		}
	}
	return ret;
}

int XArmAPI::get_trajectory_rw_status(int *status) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->get_traj_rw_status(status);
}
