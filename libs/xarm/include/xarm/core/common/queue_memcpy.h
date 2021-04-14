/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef CORE_COMMON_QUEUE_MEMCPY_H_
#define CORE_COMMON_QUEUE_MEMCPY_H_

#include <mutex>

class QueueMemcpy {
public:
	QueueMemcpy(long n, long n_size);
	~QueueMemcpy(void);
	char flush(void);
	char push(void *data);
	char pop(void *data);
	char get(void *data);
	long size(void);
	long node_size(void);
	int is_full(void);

protected:
private:
	long total_;
	long annode_size_;

	long cnt_;
	long head_;
	long tail_;
	char *buf_;
	
	std::mutex mutex_;
};
#endif
