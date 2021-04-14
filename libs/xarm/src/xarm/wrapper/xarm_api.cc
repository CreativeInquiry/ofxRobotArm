/*
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#include "xarm/wrapper/xarm_api.h"


bool compare_version(int v1[3], int v2[3]) {
	for (int i = 0; i < 3; i++) {
		if (v1[i] > v2[i]) {
			return true;
		}
		else if (v1[i] < v2[i]) {
			return false;
		}
	}
	return false;
}

XArmAPI::XArmAPI(
	const std::string &port,
	bool is_radian,
	bool do_not_open,
	bool check_tcp_limit,
	bool check_joint_limit,
	bool check_cmdnum_limit,
	bool check_robot_sn,
	bool check_is_ready,
	bool check_is_pause,
	int max_callback_thread_count,
	int max_cmdnum,
	int init_axis,
	bool debug,
	std::string report_type)
	: default_is_radian(is_radian), port_(port),
	check_tcp_limit_(check_tcp_limit), check_joint_limit_(check_joint_limit),
	check_cmdnum_limit_(check_cmdnum_limit), check_robot_sn_(check_robot_sn),
	check_is_ready_(check_is_ready), check_is_pause_(check_is_pause) {
	// default_is_radian = is_radian;
	// check_tcp_limit_ = check_tcp_limit;
	pool_.set_max_thread_count(max_callback_thread_count);
	callback_in_thread_ = max_callback_thread_count != 0;
	max_cmdnum_ = max_cmdnum > 0 ? max_cmdnum : 256;
	axis = init_axis;
	report_type_ = report_type;
	debug_ = debug;
	_init();
	printf("SDK_VERSION: %s\n", SDK_VERSION);
	if (!do_not_open) {
		connect();
	}
}

XArmAPI::~XArmAPI() {
	disconnect();
}

void XArmAPI::_init(void) {
	core = NULL;
	stream_tcp_ = NULL;
	stream_tcp_report_ = NULL;
	stream_ser_ = NULL;
	is_ready_ = true;
	is_tcp_ = true;
	is_old_protocol_ = false;
	is_first_report_ = true;
	is_sync_ = false;
	arm_type_is_1300_ = false;
	control_box_type_is_1300_ = false;

	major_version_number_ = 0;
	minor_version_number_ = 0;
	revision_version_number_ = 0;
	version_number = new int[3]{ major_version_number_, minor_version_number_, revision_version_number_ };

	mt_brake_ = 0;
	mt_able_ = 0;
	min_tcp_speed_ = (float)0.1;    // mm/s
	max_tcp_speed_ = 1000;   // mm/s
	min_tcp_acc_ = 1.0;      // mm/s^2
	max_tcp_acc_ = 50000;    // mm/s^2
	min_joint_speed_ = (float)0.01; // rad/s
	max_joint_speed_ = 4.0;  // rad/s
	min_joint_acc_ = (float)0.01;   // rad/s^2
	max_joint_acc_ = 20.0;   // rad/s^2
	count = -1;

	sleep_finish_time_ = get_system_time();

	angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	last_used_angles = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	tcp_offset = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
	if (default_is_radian) {
		joint_speed_limit = new fp32[2]{ min_joint_speed_, max_joint_speed_ };
		joint_acc_limit = new fp32[2]{ min_joint_acc_, max_joint_acc_ };
		last_used_joint_speed = (float)0.3490658503988659; // rad/s (20°/s);
		last_used_joint_acc = (float)8.726646259971648;    // rad/s^2 (500°/s^2);
		position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
		last_used_position = new fp32[6]{ 201.5, 0, 140.5, (fp32)3.1415926, 0, 0 };
	}
	else {
		joint_speed_limit = new fp32[2]{ (fp32)(min_joint_speed_ * RAD_DEGREE), (fp32)(max_joint_speed_ * RAD_DEGREE) };
		joint_acc_limit = new fp32[2]{ (fp32)(min_joint_acc_ * RAD_DEGREE), (fp32)(max_joint_acc_ * RAD_DEGREE) };
		last_used_joint_speed = (fp32)(0.3490658503988659 * RAD_DEGREE); // rad/s (20°/s);
		last_used_joint_acc = (fp32)(8.726646259971648 * RAD_DEGREE);    // rad/s^2 (500°/s^2);
		position = new fp32[6]{ 201.5, 0, 140.5, (fp32)(3.1415926 * RAD_DEGREE), 0, 0 };
		last_used_position = new fp32[6]{ 201.5, 0, 140.5, (fp32)(3.1415926 * RAD_DEGREE), 0, 0 };
	}

	state = 4;
	mode = 0;
	cmd_num = 0;
	joints_torque = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	motor_brake_states = new bool[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };
	motor_enable_states = new bool[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };
	error_code = 0;
	warn_code = 0;
	tcp_load = new fp32[4]{ 0, 0, 0, 0 };
	collision_sensitivity = 0;
	teach_sensitivity = 0;
	device_type = 7;
	axis = 7;
	master_id = 0;
	slave_id = 0;
	motor_tid = 0;
	motor_fid = 0;
	tcp_jerk = 1000;        // mm/s^3
	joint_jerk = default_is_radian ? 20 : (fp32)(20 * RAD_DEGREE); // 20 rad/s^3
	rot_jerk = (float)2.3;
	max_rot_acc = (float)2.7;
	tcp_speed_limit = new fp32[2]{ min_tcp_speed_, max_tcp_speed_ };
	tcp_acc_limit = new fp32[2]{ min_tcp_acc_, max_tcp_acc_ };
	last_used_tcp_speed = 100;  // mm/s
	last_used_tcp_acc = 2000;   // mm/s^2
	gravity_direction = new fp32[3]{ 0, 0, -1 };
	realtime_tcp_speed = 0;
	realtime_joint_speeds = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	world_offset = new fp32[6]{ 0, 0, 0, 0, 0, 0 };
	temperatures = new fp32[7]{ 0, 0, 0, 0, 0, 0 };
	gpio_reset_config = new unsigned char[2]{0, 0};
	modbus_baud_ = -1;
	ignore_error_ = false;
	ignore_state_ = false;

	gripper_is_enabled_ = false;
	bio_gripper_is_enabled_ = false;
	robotiq_is_activated_ = false;
	last_report_time_ = get_system_time();
	max_report_interval_ = 0;
	voltages = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	currents = new fp32[7]{ 0, 0, 0, 0, 0, 0, 0 };
	is_simulation_robot = 0;
	is_collision_detection = 0;
	collision_tool_type = 0;
	collision_model_params = new fp32[6]{ 0, 0, 0, 0, 0, 0};
	cgpio_state = 0;
	cgpio_code = 0;
	cgpio_input_digitals = new int[2]{ 0, 0 };
	cgpio_output_digitals = new int[2]{ 0, 0 };
	cgpio_intput_anglogs = new fp32[2]{ 0, 0 };
	cgpio_output_anglogs = new fp32[2]{ 0, 0 };
	cgpio_input_conf = new int[16]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cgpio_output_conf = new int[16]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cmd_timeout_ = -1;

	xarm_gripper_error_code_ = 0;
	bio_gripper_error_code_ = 0;
	robotiq_error_code_ = 0;
	gripper_version_numbers_[0] = -1;
	gripper_version_numbers_[1] = -1;
	gripper_version_numbers_[2] = -1;

	report_data_ptr_ = new XArmReportData(report_type_);
}

bool XArmAPI::has_err_warn(void) {
	return has_error() || has_warn();
}

bool XArmAPI::has_error(void) {
	return error_code != 0;
}

bool XArmAPI::has_warn(void) {
	return warn_code != 0;
}

bool XArmAPI::is_connected(void) {
	return is_tcp_ ? (stream_tcp_ == NULL ? false : stream_tcp_->is_ok() == 0) : (stream_ser_ == NULL ? false : stream_ser_->is_ok() == 0);
}

bool XArmAPI::is_reported(void) {
	return is_tcp_ ? (stream_tcp_report_ == NULL ? false : stream_tcp_report_->is_ok() == 0) : false;
}

static void report_thread_handle_(void *arg) {
	XArmAPI *my_this = (XArmAPI *)arg;
	my_this->_recv_report_data();
}

void XArmAPI::_sync(void) {
	memcpy(last_used_position, position, 24);
	memcpy(last_used_angles, angles, 28);
}

void XArmAPI::_check_version(void) {
	int cnt = 5;
	unsigned char version_[40];
	int ret = -1;
	while ((ret < 0 || ret > 2) && cnt > 0) {
		ret = get_version(version_);
		sleep_milliseconds(100);
		cnt -= 1;
	}
	std::string v((const char *)version_);
	std::regex pattern_new(".*(\\d+),(\\d+),(\\S+),(\\S+),.*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
	std::regex pattern(".*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
	// std::regex pattern(".*[vV](\\d+)[.](\\d+)[.](\\d+).*");
	std::smatch result;
	int arm_type = 0;
	int control_type = 0;
	if (std::regex_match(v, result, pattern_new)) {
		auto it = result.begin();
		sscanf(std::string(*++it).data(), "%d", &axis);
		sscanf(std::string(*++it).data(), "%d", &device_type);
		sscanf(std::string(*++it).substr(2, 4).data(), "%d", &arm_type);
		sscanf(std::string(*++it).substr(2, 4).data(), "%d", &control_type);

		arm_type_is_1300_ = arm_type >= 1300;
		control_box_type_is_1300_ = control_type >= 1300;

		sscanf(std::string(*++it).data(), "%d", &major_version_number_);
		sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
		sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
	}
	else if (std::regex_match(v, result, pattern)) {
		auto it = result.begin();
		sscanf(std::string(*++it).data(), "%d", &major_version_number_);
		sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
		sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
	}
	else {
		std::vector<std::string> tmpList = split(v, "-");
		int size = tmpList.size();
		if (size >= 3) {
			int year = atoi(tmpList[size - 3].c_str());
			int month = atoi(tmpList[size - 2].c_str());
			if (year < 2019) is_old_protocol_ = true;
			else if (year == 2019) {
				is_old_protocol_ = month >= 2 ? false : true;
			}
			else {
				is_old_protocol_ = false;
			}
		}
		if (is_old_protocol_) {
			major_version_number_ = 0;
			minor_version_number_ = 0;
			revision_version_number_ = 1;
		}
		else {
			major_version_number_ = 0;
			minor_version_number_ = 1;
			revision_version_number_ = 0;
		}
	}
	version_number[0] = major_version_number_;
	version_number[1] = minor_version_number_;
	version_number[2] = revision_version_number_;
	printf("FIRMWARE_VERSION: %d.%d.%d, PROTOCOL: V%d\n", major_version_number_, minor_version_number_, revision_version_number_, is_old_protocol_ ? 0 : 1);
	printf("HARDWARE_TYPE: %d, CONTROL_BOX_TYPE: %d\n", arm_type, control_type);
	if (check_robot_sn_) {
		cnt = 5;
		int err_warn[2];
		ret = -1;
		while ((ret < 0 || ret > 2) && cnt > 0 && warn_code == 0) {
			ret = get_robot_sn(version_);
			get_err_warn_code(err_warn);
			sleep_milliseconds(100);
			cnt -= 1;
		}
		printf("robot_sn: %s\n", sn);
	}
}

void XArmAPI::_check_is_pause(void) {
	if (check_is_pause_ && state == 3) {
		std::unique_lock<std::mutex> locker(mutex_);
		cond_.wait(locker, [this] { return state != 3 || !is_connected(); });
		locker.unlock();
	}
}

int XArmAPI::_wait_until_cmdnum_lt_max(void) {
	if (!check_cmdnum_limit_) return 0;
	while (cmd_num >= max_cmdnum_) {
		if (!is_connected()) return API_CODE::NOT_CONNECTED;
		if (error_code != 0) return API_CODE::HAS_ERROR;
		if (state == 4 || state == 5) return API_CODE::NOT_READY;
		sleep_milliseconds(50);
	}
	return 0;
}

int XArmAPI::_check_code(int code, bool is_move_cmd) {
	if (is_move_cmd) {
		return ((code == 0 || code == UXBUS_STATE::WAR_CODE) && core->state_is_ready) ? 0 : !core->state_is_ready ? UXBUS_STATE::STATE_NOT_READY : code;
	}
	else {
		return (code == 0 || code == UXBUS_STATE::ERR_CODE || code == UXBUS_STATE::WAR_CODE || code == UXBUS_STATE::STATE_NOT_READY) ? 0 : code;
	}
}

bool XArmAPI::_version_is_ge(int major, int minor, int revision) {
	if (major_version_number_ == 0 && minor_version_number_ == 0 && revision_version_number_ == 0) {
		unsigned char version_[40];
		get_version(version_);

		std::string v((const char *)version_);
		std::regex pattern_new(".*(\\d+),(\\d+),(\\S+),(\\S+),.*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
		std::regex pattern(".*[vV](\\d+)\\.(\\d+)\\.(\\d+)");
		std::smatch result;
		int arm_type = 0;
		int control_type = 0;
		if (std::regex_match(v, result, pattern_new)) {
			auto it = result.begin();
			sscanf(std::string(*++it).data(), "%d", &axis);
			sscanf(std::string(*++it).data(), "%d", &device_type);
			sscanf(std::string(*++it).substr(2, 4).data(), "%d", &arm_type);
			sscanf(std::string(*++it).substr(2, 4).data(), "%d", &control_type);

			arm_type_is_1300_ = arm_type >= 1300;
			control_box_type_is_1300_ = control_type >= 1300;

			sscanf(std::string(*++it).data(), "%d", &major_version_number_);
			sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
			sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
		}
		else if (std::regex_match(v, result, pattern)) {
			auto it = result.begin();
			sscanf(std::string(*++it).data(), "%d", &major_version_number_);
			sscanf(std::string(*++it).data(), "%d", &minor_version_number_);
			sscanf(std::string(*++it).data(), "%d", &revision_version_number_);
		}
		else {
			std::vector<std::string> tmpList = split(v, "-");
			int size = tmpList.size();
			if (size >= 3) {
				int year = atoi(tmpList[size - 3].c_str());
				int month = atoi(tmpList[size - 2].c_str());
				if (year < 2019) is_old_protocol_ = true;
				else if (year == 2019) {
					is_old_protocol_ = month >= 2 ? false : true;
				}
				else {
					is_old_protocol_ = false;
				}
			}
			if (is_old_protocol_) {
				major_version_number_ = 0;
				minor_version_number_ = 0;
				revision_version_number_ = 1;
			}
			else {
				major_version_number_ = 0;
				minor_version_number_ = 1;
				revision_version_number_ = 0;
			}
		}
		version_number[0] = major_version_number_;
		version_number[1] = minor_version_number_;
		version_number[2] = revision_version_number_;
	}
	return major_version_number_ > major || (major_version_number_ == major && minor_version_number_ > minor) || (major_version_number_ == major && minor_version_number_ == minor && revision_version_number_ >= revision);
}

int XArmAPI::connect(const std::string &port) {
	if (is_connected()) return 0;
	if (port != "" && port != port_) {
		port_ = port;
	}
	if (port_ == "") {
		printf("can not connect to port/ip: %s\n", port_.data());
		return API_CODE::NOT_CONNECTED;
	}
	// std::regex pattern("(\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})[.](\\d|\\d{1,2}|(1\\d{1,2})|2[0-5]{1,2})");
	std::regex pattern("(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)[.]){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)");
	is_ready_ = true;
	if (port_ == "localhost" || std::regex_match(port_, pattern)) {
		is_tcp_ = true;
		stream_tcp_ = new SocketPort((char *)port_.data(), XARM_CONF::TCP_PORT_CONTROL, 3, 128);
		if (stream_tcp_->is_ok() != 0) {
			printf("Error: Tcp control connection failed\n");
			return -2;
		}
		core = new UxbusCmdTcp((SocketPort *)stream_tcp_);
		printf("Tcp control connection successful\n");

		sleep_milliseconds(200);
		_check_version();

		stream_tcp_report_ = connect_tcp_report((char *)port_.data(), report_type_);
		_report_connect_changed_callback();
		if (!is_reported()) { return -3; }
		report_thread_ = std::thread(report_thread_handle_, this);
		report_thread_.detach();
	}
	else {
		is_tcp_ = false;
		stream_ser_ = new SerialPort((const char *)port_.data(), XARM_CONF::SERIAL_BAUD, 3, 128);
		core = new UxbusCmdSer((SerialPort *)stream_ser_);
		_report_connect_changed_callback();
		sleep_milliseconds(200);
		_check_version();
	}
	if (cmd_timeout_ > 0)
		set_timeout(cmd_timeout_);

	return 0;
}

void XArmAPI::disconnect(void) {
	if (stream_tcp_ != NULL) {
		stream_tcp_->close_port();
	}
	if (stream_ser_ != NULL) {
		stream_ser_->close_port();
	}
	if (stream_tcp_report_ != NULL) {
		stream_tcp_report_->close_port();
	}
	_report_connect_changed_callback();
	is_ready_ = false;
}

int XArmAPI::set_timeout(fp32 timeout) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	assert(timeout > 0);
	cmd_timeout_ = timeout;
	return core->set_timeout(timeout);
}

int XArmAPI::get_version(unsigned char version_[40]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_version(version_);
	return _check_code(ret);
}

int XArmAPI::get_robot_sn(unsigned char robot_sn[40]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	char str[40];
	int ret = core->get_robot_sn((unsigned char*)str);
	ret = _check_code(ret);
	if (ret == 0) {
		int arm_type = 0;
		int control_type = 0;
		char *control_box_sn = strchr(str, '\0') + 1;
		sscanf(std::string(str).substr(2, 4).data(), "%d", &arm_type);
		sscanf(std::string(control_box_sn).substr(2, 4).data(), "%d", &control_type);
		arm_type_is_1300_ = arm_type >= 1300;
		control_box_type_is_1300_ = control_type >= 1300;
		memcpy(robot_sn, sn, 40);
		memcpy(sn, robot_sn, 40);
	}
	return ret;
}

int XArmAPI::shutdown_system(int value) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->shutdown_system(value);
	return _check_code(ret);
}

int XArmAPI::get_state(int *state_) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_state(state_);
	ret = _check_code(ret);
	if (ret == 0) {
		state = *state_;
	}
	return ret;
}

int XArmAPI::get_cmdnum(int *cmdnum_) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_cmdnum(cmdnum_);
	ret = _check_code(ret);
	if (ret == 0) {
		cmd_num = *cmdnum_;
	}
	return ret;
}

int XArmAPI::get_err_warn_code(int err_warn[2]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_err_code(err_warn);
	ret = _check_code(ret);
	if (ret == 0) {
		error_code = err_warn[0];
		warn_code = err_warn[1];
	}
	return ret;
}

int XArmAPI::get_position(fp32 pose[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_tcp_pose(pose);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 6; i++) {
			if (!default_is_radian && i > 2) {
				pose[i] = (float)(pose[i] * RAD_DEGREE);
			}
			position[i] = pose[i];
		}
	}
	return ret;
}

int XArmAPI::get_servo_angle(fp32 angs[7]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_joint_pose(angs);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 7; i++) {
			if (!default_is_radian) {
				angs[i] = (float)(angs[i] * RAD_DEGREE);
			}
			angles[i] = angs[i];
		}
	}
	return ret;
}

int XArmAPI::motion_enable(bool enable, int servo_id) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->motion_en(servo_id, int(enable));
	ret = _check_code(ret);
	get_state(&state);
	if (state == 4 || state == 5) {
		sleep_finish_time_ = 0;
		if (debug_ && is_ready_) {
			printf("[motion_enable], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (debug_ && !is_ready_) {
			printf("[motion_enable], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::set_state(int state_) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_state(state_);
	ret = _check_code(ret);
	get_state(&state);
	if (state == 4 || state == 5) {
		// is_sync_ = false;
		sleep_finish_time_ = 0;
		if (debug_ && is_ready_) {
			printf("[set_state], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (debug_ && !is_ready_) {
			printf("[set_state], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::set_mode(int mode_) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_mode(mode_);
	return _check_code(ret);
}

int XArmAPI::set_servo_attach(int servo_id) {
	// if (!is_connected()) return API_CODE::NOT_CONNECTED;
	// return core->set_brake(servo_id, 0);
	return motion_enable(true, servo_id);
}

int XArmAPI::set_servo_detach(int servo_id) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->set_brake(servo_id, 1);
}

int XArmAPI::clean_error(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->clean_err();
	get_state(&state);
	if (state == 4 || state == 5) {
		sleep_finish_time_ = 0;
		if (debug_ && is_ready_) {
			printf("[clean_error], xArm is not ready to move\n");
		}
		is_ready_ = false;
	}
	else {
		if (debug_ && !is_ready_) {
			printf("[clean_error], xArm is ready to move\n");
		}
		is_ready_ = true;
	}
	return ret;
}

int XArmAPI::clean_warn(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	return core->clean_war();
}

int XArmAPI::set_pause_time(fp32 sltime) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	int ret = core->sleep_instruction(sltime);
	if (get_system_time() >= sleep_finish_time_) {
		sleep_finish_time_ = get_system_time() + (long long)(sltime * 1000);
	}
	else {
		sleep_finish_time_ = sleep_finish_time_ + (long long)(sltime * 1000);
	}
	return ret;
}

int XArmAPI::_wait_move(fp32 timeout) {
	long long start_time = get_system_time();
	long long expired = timeout <= 0 ? 0 : (get_system_time() + timeout * 1000 + sleep_finish_time_ > start_time ? sleep_finish_time_ : 0);
	int cnt = 0;
	int state_;
	int err_warn[2];
	int ret = get_state(&state_);
	int max_cnt = (ret == 0 && state_ == 1) ? 2 : 10;
	while (timeout <= 0 || get_system_time() < expired) {
		if (!is_connected()) return API_CODE::NOT_CONNECTED;
		if (get_system_time() - last_report_time_ > 400) {
			get_state(&state_);
			get_err_warn_code(err_warn);
		}
		if (error_code != 0) {
			return API_CODE::HAS_ERROR;
		}
		if (state == 4 || state == 5) {
			ret = get_state(&state_);
			if (ret != 0 || (state_ != 4 && state_ != 5)) {
				sleep_milliseconds(20);
				continue;
			}
			sleep_finish_time_ = 0;
			return API_CODE::EMERGENCY_STOP;
		}
		if (get_system_time() < sleep_finish_time_ || state == 3) {
			sleep_milliseconds(20);
			cnt = 0;
			continue;
		}
		if (state != 1) {
			cnt += 1;
			if (cnt >= max_cnt) {
				ret = get_state(&state_);
				get_err_warn_code(err_warn);
				if (ret == 0 && state_ != 1) {
					return 0;
				}
				else {
					cnt = 0;
				}
			}
		}
		else {
			cnt = 0;
		}
		sleep_milliseconds(50);
	}
	return API_CODE::WAIT_FINISH_TIMEOUT;
}

void XArmAPI::emergency_stop(void) {
	long long start_time = get_system_time();
	while (state != 4 && state != 5 && get_system_time() - start_time < 3000) {
		set_state(4);
		sleep_milliseconds(100);
	}
	sleep_finish_time_ = 0;
	// motion_enable(true, 8);
	// while ((state == 0 || state == 3 || state == 4) && get_system_time() - start_time < 3000) {
	//     set_state(0);
	//     sleep_milliseconds(100);
	// }
}

int XArmAPI::get_inverse_kinematics(fp32 source_pose[6], fp32 target_angles[7]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 pose[6];
	for (int i = 0; i < 6; i++) {
		pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : source_pose[i] / RAD_DEGREE);
	}
	fp32 angs[7];
	int ret = core->get_ik(pose, angs);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 7; i++) {
			target_angles[i] = (float)(default_is_radian ? angs[i] : angs[i] * RAD_DEGREE);
		}
	}
	return ret;
}

int XArmAPI::get_forward_kinematics(fp32 source_angles[7], fp32 target_pose[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 angs[7];
	for (int i = 0; i < 7; i++) {
		angs[i] = (float)(default_is_radian ? source_angles[i] : source_angles[i] / RAD_DEGREE);
	}
	fp32 pose[6];
	int ret = core->get_fk(angs, pose);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 6; i++) {
			target_pose[i] = (float)(default_is_radian || i < 3 ? pose[i] : pose[i] * RAD_DEGREE);
		}
	}
	return ret;
}

int XArmAPI::is_tcp_limit(fp32 source_pose[6], int *limit) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 pose[6];
	for (int i = 0; i < 6; i++) {
		pose[i] = (float)(default_is_radian || i < 3 ? source_pose[i] : source_pose[i] / RAD_DEGREE);
	}
	int ret = core->is_tcp_limit(pose, limit);
	return _check_code(ret);
}

int XArmAPI::is_joint_limit(fp32 source_angles[7], int *limit) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 angs[7];
	for (int i = 0; i < 7; i++) {
		angs[i] = (float)(default_is_radian ? source_angles[i] : source_angles[i] / RAD_DEGREE);
	}
	int ret = core->is_joint_limit(angs, limit);
	return _check_code(ret);
}

int XArmAPI::reload_dynamics(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->reload_dynamics();
	return _check_code(ret);
}

int XArmAPI::set_counter_reset(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	int ret = core->cnter_reset();
	return _check_code(ret);
}

int XArmAPI::set_counter_increase(void) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int wait_code = _wait_until_cmdnum_lt_max();
	if (wait_code != 0) return wait_code;
	int ret = core->cnter_plus();
	return _check_code(ret);
}

int XArmAPI::get_pose_offset(float pose1[6], float pose2[6], float offset[6], int orient_type_in, int orient_type_out) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 p1[6], p2[6];
	for (int i = 0; i < 6; i++) {
		p1[i] = (float)(default_is_radian || i < 3 ? pose1[i] : pose1[i] / RAD_DEGREE);
		p2[i] = (float)(default_is_radian || i < 3 ? pose2[i] : pose2[i] / RAD_DEGREE);
	}
	int ret = core->get_pose_offset(p1, p2, offset, orient_type_in, orient_type_out);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 6; i++) {
			offset[i] = (float)(default_is_radian || i < 3 ? offset[i] : offset[i] * RAD_DEGREE);
		}
	}
	return ret;
}

int XArmAPI::get_position_aa(fp32 pose[6]) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->get_position_aa(pose);
	ret = _check_code(ret);
	if (ret == 0) {
		for (int i = 0; i < 6; i++) {
			pose[i] = (!default_is_radian && i > 2) ? (float)(pose[i] * RAD_DEGREE) : pose[i];
		}
	}
	return ret;
}

int XArmAPI::set_simulation_robot(bool on) {
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	int ret = core->set_simulation_robot((int)on);
	return _check_code(ret);
}

int XArmAPI::calibrate_tcp_coordinate_offset(float four_points[4][6], float ret_xyz[3])
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 points[4][6];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 6; j++) {
			points[i][j] = (float)((j < 3 || default_is_radian) ? four_points[i][j] : four_points[i][j] / RAD_DEGREE);
		}
	}
	int ret = core->cali_tcp_pose(points, ret_xyz);
	return _check_code(ret);
}

int XArmAPI::calibrate_tcp_orientation_offset(float rpy_be[3], float rpy_bt[3], float ret_rpy[3])
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 rpy_be_[3];
	fp32 rpy_bt_[3];
	for (int i = 0; i < 3; i++) {
		rpy_be_[i] = (float)(default_is_radian ? rpy_be[i] : rpy_be[i] / RAD_DEGREE);
		rpy_bt_[i] = (float)(default_is_radian ? rpy_bt[i] : rpy_bt[i] / RAD_DEGREE);
	}
	int ret = core->cali_tcp_orient(rpy_be_, rpy_bt_, ret_rpy);
	for (int i = 0; i < 3; i++) {
		ret_rpy[i] = (float)(default_is_radian ? ret_rpy[i] : ret_rpy[i] * RAD_DEGREE);
	}
	return _check_code(ret);
}

int XArmAPI::calibrate_user_orientation_offset(float three_points[3][6], float ret_rpy[3], int mode, int trust_ind)
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 points[3][6];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 6; j++) {
			points[i][j] = (float)((j < 3 || default_is_radian) ? three_points[i][j] : three_points[i][j] / RAD_DEGREE);
		}
	}
	int ret = core->cali_user_orient(points, ret_rpy, mode, trust_ind);
	for (int i = 0; i < 3; i++) {
		ret_rpy[i] = (float)(default_is_radian ? ret_rpy[i] : ret_rpy[i] * RAD_DEGREE);
	}
	return _check_code(ret);
}

int XArmAPI::calibrate_user_coordinate_offset(float rpy_ub[3], float pos_b_uorg[3], float ret_xyz[3])
{
	if (!is_connected()) return API_CODE::NOT_CONNECTED;
	fp32 rpy_ub_[3];
	for (int i = 0; i < 3; i++) {
		rpy_ub_[i] = (float)(default_is_radian ? rpy_ub[i] : rpy_ub[i] / RAD_DEGREE);
	}
	int ret = core->cali_user_pos(rpy_ub_, pos_b_uorg, ret_xyz);
	return _check_code(ret);
}

