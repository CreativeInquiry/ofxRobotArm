/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef OS_NETWORK_H_
#define OS_NETWORK_H_

#if !defined(SOL_TCP) && defined(IPPROTO_TCP)
#define SOL_TCP IPPROTO_TCP
#endif
#if !defined(TCP_KEEPIDLE) && defined(TCP_KEEPALIVE)
#define TCP_KEEPIDLE TCP_KEEPALIVE
#endif

#include "xarm/core/common/data_type.h"

int socket_init(char *local_ip, int port, int is_server);
int socket_send_data(int client_fp, unsigned char *data, int len);
int socket_connect_server(int *socket, char server_ip[], int server_port);

#endif // OS_NETWORK_H_
