/* vim: set noai ts=4 sw=4:
 *
 * pulse-cntr.h
 *
 * (C) 2018 T-platforms. All right reserved.
 */
#ifndef __PULSE_CNTR__H__
#define __PULSE_CNTR__H__

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>

#include "config.h"

#define PULSE_CMD_POL_REG	0
#define PULSE_CMD_DEB_REG	1
#define PULSE_CMD_CNTR_REG	2
#define PULSE_CMD_MAX_REG	PULSE_CMD_CNTR_REG
#define PULSE_CMD_MAX_PIN	3
#define PULSE_CMD_MAX_POL	1
#define PULSE_CMD_MAX_DEB	65535

#define PULSE_CMD_RD_LEN	2
#define PULSE_CMD_POL_LEN	3
#define PULSE_CMD_DEB_LEN	4
#define PULSE_CMD_CNTR_LEN	6
#define PULSE_CMD_MAX_LEN	PULSE_CMD_CNTR_LEN
#define PULSE_CMD_SILENCE	100

#define PULSE_TTY_BAUD			B115200
#define PULSE_DEFAULT_NAME		DEFAULT_TTY
#define PULSE_DEFAULT_WRITE		false
#define PULSE_DEFAULT_REG		0
#define PULSE_DEFAULT_PIN		0

#define PULSE_VERSION			PROJECT_VERSION

#define PULSE_MSG_VERSION		 2
#define PULSE_MSG_USAGE			 1
#define PULSE_SUCCESS			 0
#define PULSE_ERR_MISSING_ARG	-1
#define PULSE_ERR_INVALID_ARG	-2
#define PULSE_ERR_OPEN_FILE		-3
#define PULSE_ERR_WRITE_OPS		-4
#define PULSE_ERR_READ_OPS		-5
#define PULSE_ERR_BAD_CRC		-6

enum pulse_msg_mode {
	PULSE_PRINT_BOTH,
	PULSE_PRINT_MESSAGE,
	PULSE_PRINT_USAGE
};

struct pulse_data {
	bool write;

	uint8_t reg;
	uint8_t pin;

	uint32_t data;

	uint8_t buf[PULSE_CMD_MAX_LEN];
	int len;

	struct termios tty;
	const char *fname;
	int fd;
};

#endif /* __PULSE_CNTR__H__ */
