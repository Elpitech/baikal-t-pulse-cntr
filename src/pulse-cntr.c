/* vim: set noai ts=4 sw=4:
 *
 * pulse-cntr.c
 *
 * (C) 2018 T-platforms. All right reserved.
 */
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#include "pulse-cntr.h"
#include "crc8.h"

static const char *pulse_get_name(uint8_t reg)
{
	switch (reg) {
	case PULSE_CMD_POL_REG:
		return "polarity";
	case PULSE_CMD_DEB_REG:
		return "debounce";
	case PULSE_CMD_CNTR_REG:
		return "counter";
	}

	return "unknown";
}

static int pulse_get_len(uint8_t reg)
{
	switch (reg) {
	case PULSE_CMD_POL_REG:
		return PULSE_CMD_POL_LEN;
	case PULSE_CMD_DEB_REG:
		return PULSE_CMD_DEB_LEN;
	case PULSE_CMD_CNTR_REG:
		return PULSE_CMD_CNTR_LEN;
	}

	/* make sure it's impossible */
	return 0;
}

static void pulse_status_msg(enum pulse_msg_mode mode, const char *fmt, ...)
{
	if (mode == PULSE_PRINT_MESSAGE || mode == PULSE_PRINT_BOTH) {
		va_list argptr;

		va_start(argptr, fmt);
		vprintf(fmt, argptr);
		va_end(argptr);

		putchar('\n');
	}

	if (mode == PULSE_PRINT_USAGE || mode == PULSE_PRINT_BOTH) {
		printf("Usage: pulse-cntr [OPTIONS]\n"
			   "Read/Write MRBT1 BMC pulse counter information.\n\n"
			   "Options:\n");
		printf(" -d, --device=path      Serial device of the counter (default %s)\n",
			   PULSE_DEFAULT_NAME);
		printf(" -w, --write=number     Hex/Dec/Oct data number for write operations\n");
		printf(" -r, --register=number  Register identifier (default %d)\n",
			   PULSE_DEFAULT_REG);
		printf(" -p, --pin=id           Pin identifier (default %d)\n",
			   PULSE_DEFAULT_PIN);
		printf(" -h, --help             Print this help message and exit\n");
		printf(" -v, --version          Output version information and exit\n\n");
	}
}

static int pulse_parse_args(int argc, char *argv[], struct pulse_data *pulse)
{
	static const char *optstring = ":d:w:r:p:hv";
	static struct option longopts[] = {
		{"device", required_argument, NULL, 'd'},
		{"write", required_argument, NULL, 'w'},
		{"register", required_argument, NULL, 'r'},
		{"pin", required_argument, NULL, 'p'},
		{"help", no_argument, NULL, 'h'},
		{"version", no_argument, NULL, 'v'},
		{0}
	};
	int longindex = 0;
	char *endptr;
	int opt;

	while (1) {
		opt = getopt_long(argc, argv, optstring, longopts, &longindex);

		switch (opt) {
		case 'd':
			pulse->fname = optarg;
			break;
		case 'w':
			errno = 0;
			pulse->write = true;
			pulse->data = strtoul(optarg, &endptr, 0);
			if (errno || optarg == endptr) {
				pulse_status_msg(PULSE_PRINT_BOTH,
								 "Error: Couldn't convert data '%s' to integer\n", optarg);
				return PULSE_ERR_INVALID_ARG;
			}
			break;
		case 'r':
			errno = 0;
			pulse->reg = strtoul(optarg, &endptr, 0);
			if (errno || optarg == endptr) {
				pulse_status_msg(PULSE_PRINT_BOTH,
								 "Error: Couldn't convert register id '%s' to integer\n", optarg);
				return PULSE_ERR_INVALID_ARG;
			}
			break;
		case 'p':
			errno = 0;
			pulse->pin = strtoul(optarg, &endptr, 0);
			if (errno || optarg == endptr) {
				pulse_status_msg(PULSE_PRINT_BOTH,
								 "Error: Couldn't convert pin id '%s' to integer\n", optarg);
				return PULSE_ERR_INVALID_ARG;
			}
			break;
		case 'h':
			pulse_status_msg(PULSE_PRINT_USAGE, NULL);
			return PULSE_MSG_USAGE;
		case 'v':
			pulse_status_msg(PULSE_PRINT_MESSAGE, "pulse-cntr v%s", PULSE_VERSION);
			return PULSE_MSG_VERSION;
		case ':':
			pulse_status_msg(PULSE_PRINT_BOTH, "Error: Option '-%c' requires an argument\n", optopt);
			return PULSE_ERR_MISSING_ARG;
		case '?':
			pulse_status_msg(PULSE_PRINT_BOTH, "Error: Invalid argument '%c' detected\n", optopt);
			return PULSE_ERR_INVALID_ARG;
		case -1:
			return PULSE_SUCCESS;
		default:
			break;
		}
	}

	return PULSE_SUCCESS;
}

static int pulse_check_args(struct pulse_data *pulse)
{
	if (pulse->reg > PULSE_CMD_MAX_REG) {
		pulse_status_msg(PULSE_PRINT_BOTH, "Error: Invalid register id %hhu (0 <= id <= %d)\n",
						 pulse->reg, PULSE_CMD_MAX_REG);
		return PULSE_ERR_INVALID_ARG;
	}

	if (pulse->pin > PULSE_CMD_MAX_PIN) {
		pulse_status_msg(PULSE_PRINT_BOTH, "Error: Invalid pin id %hhu (0 <= id <= %d)\n",
						 pulse->pin, PULSE_CMD_MAX_PIN);
		return PULSE_ERR_INVALID_ARG;
	}

	if (pulse->write && pulse->reg == PULSE_CMD_POL_REG && pulse->data > PULSE_CMD_MAX_POL) {
		pulse_status_msg(PULSE_PRINT_BOTH, "Error: Invalid data %u (0 <= id <= %d)\n",
						 pulse->data, PULSE_CMD_MAX_POL);
		return PULSE_ERR_INVALID_ARG;
	}

	if (pulse->write && pulse->reg == PULSE_CMD_DEB_REG && pulse->data > PULSE_CMD_MAX_DEB) {
		pulse_status_msg(PULSE_PRINT_BOTH, "Error: Invalid data %u (0 <= id <= %d)\n",
						 pulse->data, PULSE_CMD_MAX_DEB);
		return PULSE_ERR_INVALID_ARG;
	}

	return PULSE_SUCCESS;
}

static int pulse_open_serial(struct pulse_data *pulse)
{
	struct termios tty;
	int status;

	pulse->fd = open(pulse->fname, O_RDWR | O_SYNC | O_NOCTTY);
	if (pulse->fd < 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: File '%s' couldn't be opened",
						 pulse->fname);
		return PULSE_ERR_OPEN_FILE;
	}

	memset(&tty, 0, sizeof(tty));

	status = tcgetattr(pulse->fd, &tty);
	if (status < 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Could't get term attributes");
		goto err_close_fd;
	}

	memcpy(&pulse->tty, &tty, sizeof(tty));

	status = cfsetispeed(&tty, PULSE_TTY_BAUD);
	if (status < 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Could't set term inbound speed");
		goto err_close_fd;
	}
	status = cfsetospeed(&tty, PULSE_TTY_BAUD);
	if (status < 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Could't set term outbound speed");
		goto err_close_fd;
	}

	/* set raw mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ECHOE | ECHOK| ICANON | ISIG | IEXTEN);
	tty.c_cflag &= ~(CSIZE | PARENB);
	tty.c_cflag |= CS8;

	/* set the read blocking paramters */
	if (!pulse->write) {
		tty.c_cc[VMIN] = pulse_get_len(pulse->reg);
		tty.c_cc[VTIME] = 1; /* wait for just 10 msec */ 
	}

	status = tcsetattr(pulse->fd, TCSANOW, &tty);
	if (status < 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Could't set term attributes");
		goto err_close_fd;
	}

	return PULSE_SUCCESS;

err_close_fd:
	close(pulse->fd);

	return PULSE_ERR_OPEN_FILE;
}

static void pulse_close_serial(struct pulse_data *pulse)
{
	(void)tcsetattr(pulse->fd, TCSANOW, &pulse->tty);

	close(pulse->fd);
}

static void pulse_init_buf(struct pulse_data *pulse)
{
	pulse->buf[0] = (pulse->reg << 2) | (pulse->pin);

	if (!pulse->write) {
		pulse->len = PULSE_CMD_RD_LEN;
	} else {
		pulse->buf[0] |= (1 << 7);
		pulse->len = pulse_get_len(pulse->reg);

		switch (pulse->reg) {
		case PULSE_CMD_POL_REG:
			pulse->buf[1] = pulse->data;
			break;
		case PULSE_CMD_DEB_REG:
			pulse->buf[1] = pulse->data;
			pulse->buf[2] = pulse->data >> 8;
			break;
		case PULSE_CMD_CNTR_REG:
			pulse->buf[1] = pulse->data;
			pulse->buf[2] = pulse->data >> 8;
			pulse->buf[3] = pulse->data >> 16;
			pulse->buf[4] = pulse->data >> 24;
			break;
		}
	}

	pulse->buf[pulse->len - 1] = crc8_dallas(pulse->buf, pulse->len - 1);
}

static int pulse_send_data(struct pulse_data *pulse)
{
	ssize_t status;

	status = write(pulse->fd, pulse->buf, pulse->len);
	(void)tcdrain(pulse->fd);
	if (status != pulse->len) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Could't send the request\n");
		return PULSE_ERR_WRITE_OPS;
	}

	return PULSE_SUCCESS;
}

static int pulse_recv_data(struct pulse_data *pulse)
{
	ssize_t status;

	/* skip if write operation requested */
	if (pulse->write)
		return PULSE_SUCCESS;

	pulse->len = pulse_get_len(pulse->reg);
	status = read(pulse->fd, pulse->buf, pulse->len);
	if (status != pulse->len) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Recv operation failed (%d != %d)\n",
						 status, pulse->len);
		return PULSE_ERR_READ_OPS;
	}

	status = crc8_dallas(pulse->buf, pulse->len);
	if (status != 0) {
		pulse_status_msg(PULSE_PRINT_MESSAGE, "Error: Recv data with bad CRC\n");
		return PULSE_ERR_BAD_CRC;
	}

	pulse->pin = pulse->buf[0] & 0x3;
	pulse->reg = (pulse->buf[0] >> 2) & 0x3;

	switch(pulse->reg) {
		case PULSE_CMD_POL_REG:
			pulse->data = pulse->buf[1];
			break;
		case PULSE_CMD_DEB_REG:
			pulse->data = pulse->buf[2] << 8 | pulse->buf[1];
			break;
		case PULSE_CMD_CNTR_REG:
			pulse->data = pulse->buf[4] << 24 | pulse->buf[3] << 16 | pulse->buf[2] << 8 | pulse->buf[1];
			break;
	}

	if (pulse->reg != PULSE_CMD_POL_REG) {
		printf("Pin %hhu %s: %u\n", pulse->pin, pulse_get_name(pulse->reg), pulse->data);
	} else {
		printf("Pin %hhu polarity: %s\n", pulse->pin, pulse->data ? "negative" : "positive");
	}

	return PULSE_SUCCESS;
}

int main(int argc, char *argv[], char *env[])
{
	struct pulse_data pulse = {
		.write = PULSE_DEFAULT_WRITE,
		.reg = PULSE_DEFAULT_REG,
		.pin = PULSE_DEFAULT_PIN,
		.fname = PULSE_DEFAULT_NAME,
		.data = 0,
		.fd = 0
	};
	int status;

	/* Parse input arguments */
	status = pulse_parse_args(argc, argv, &pulse);
	if (status == PULSE_MSG_USAGE || status == PULSE_MSG_VERSION)
		return PULSE_SUCCESS;
	else if (status != PULSE_SUCCESS)
		return status;

	/* Check the passed arguments */
	status = pulse_check_args(&pulse);
	if (status != PULSE_SUCCESS)
		return status;

	/* Open the pulse-counter serial console */
	status = pulse_open_serial(&pulse);
	if (status != PULSE_SUCCESS)
		return status;

	/* Initialize the buffer with requested message */
	pulse_init_buf(&pulse);

	/* Send requested message to the counter */
	status = pulse_send_data(&pulse);
	if (status != PULSE_SUCCESS)
		goto err_close_serial;

	/* Recv data from the counter */
	status = pulse_recv_data(&pulse);

	/* Close the serial console restoring the previous settings */
err_close_serial:
	pulse_close_serial(&pulse);

	return status;
}
