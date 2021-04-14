/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#include "xarm/core/connect.h"

#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/xarm_config.h"

UxbusCmdSer *connect_rs485_control(const char *com) {
  SerialPort *arm_port = new SerialPort(com, XARM_CONF::SERIAL_BAUD, 3, 128);
  if (arm_port->is_ok() != 0) {
    printf("Error: Serial RS485 connection failed\n");
    return NULL;
  }
  UxbusCmdSer *arm_cmd = new UxbusCmdSer(arm_port);
  printf("Serial RS485 connection successful\n");
  return arm_cmd;
}

UxbusCmdTcp *connect_tcp_control(char *server_ip) {
  SocketPort *arm_port =
    new SocketPort(server_ip, XARM_CONF::TCP_PORT_CONTROL, 3, 128);
  if (arm_port->is_ok() != 0) {
    printf("Error: Tcp Control connection failed\n");
    return NULL;
  }
  UxbusCmdTcp *arm_cmd = new UxbusCmdTcp(arm_port);
  printf("Tcp Control connection successful\n");
  return arm_cmd;
}

SocketPort *connect_tcp_report_norm(char *server_ip) {
  SocketPort *arm_report =
    new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_NORM, 5, 149); // 145 + 4
  if (arm_report->is_ok() != 0) {
    printf("Error: Tcp Report Norm connection failed, ip: %s\n", server_ip);
    return NULL;
  }
  printf("Tcp Report Norm connection successful\n");
  return arm_report;
}

SocketPort *connect_tcp_report_rich(char *server_ip) {
  SocketPort *arm_report =
    new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_RICH, 5, 512);
  if (arm_report->is_ok() != 0) {
    printf("Error: Tcp Report Rich connection failed\n");
    return NULL;
  }
  printf("Tcp Report Rich connection successful\n");
  return arm_report;
}

SocketPort *connect_tcp_report_devl(char *server_ip) {
  SocketPort *arm_report =
    new SocketPort(server_ip, XARM_CONF::TCP_PORT_REPORT_DEVL, 10, 91); // 87 + 4
  if (arm_report->is_ok() != 0) {
    printf("Error: Tcp Report develop connection failed\n");
    return NULL;
  }
  printf("Tcp Report develop connection successful\n");
  return arm_report;
}

SocketPort *connect_tcp_report(char *server_ip, std::string report_type) {
  if (report_type == "dev")
    return connect_tcp_report_devl(server_ip);
  else if (report_type == "rich")
    return connect_tcp_report_rich(server_ip);
  else
    return connect_tcp_report_norm(server_ip);
}
