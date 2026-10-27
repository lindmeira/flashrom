/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Driver for Thomas Fischl's USBasp programmer and Chinese clones.
 */

#include <stdlib.h>
#include <stdint.h>
#include "platform/string.h"
#include <errno.h>
#include <libusb.h>

#include "flash.h"
#include "programmer.h"
#include "spi.h"
#include "helpers.h"
#include "log.h"
#include "usb_device.h"

#define USBASP_VID 0x16c0
#define USBASP_PID 0x05dc

static const struct dev_entry devs_usbasp[] = {
	{ USBASP_VID, USBASP_PID, OK, "Thomas Fischl", "USBasp" },
	{ 0 }
};

/* USBasp function request codes */
#define USBASP_FUNC_CONNECT        1
#define USBASP_FUNC_DISCONNECT     2
#define USBASP_FUNC_TRANSMIT       3
#define USBASP_FUNC_READFLASH      4
#define USBASP_FUNC_ENABLEPROG     5
#define USBASP_FUNC_WRITEFLASH     6
#define USBASP_FUNC_READEEPROM     7
#define USBASP_FUNC_WRITEEEPROM    8
#define USBASP_FUNC_SETLONGADDRESS 9
#define USBASP_FUNC_SETISPSCK     10

/* USBasp ISP clock speed options */
#define USBASP_ISP_SCK_AUTO        0
#define USBASP_ISP_SCK_0_5         1 /* 500 Hz */
#define USBASP_ISP_SCK_1           2 /* 1 kHz */
#define USBASP_ISP_SCK_2           3 /* 2 kHz */
#define USBASP_ISP_SCK_4           4 /* 4 kHz */
#define USBASP_ISP_SCK_8           5 /* 8 kHz */
#define USBASP_ISP_SCK_16          6 /* 16 kHz */
#define USBASP_ISP_SCK_32          7 /* 32 kHz */
#define USBASP_ISP_SCK_93_75       8 /* 93.75 kHz */
#define USBASP_ISP_SCK_187_5       9 /* 187.5 kHz */
#define USBASP_ISP_SCK_375        10 /* 375 kHz */
#define USBASP_ISP_SCK_750        11 /* 750 kHz */
#define USBASP_ISP_SCK_1500       12 /* 1.5 MHz */

#define USB_TIMEOUT 2000

struct usbasp_spispeed {
	const char *name;
	uint8_t sck_index;
};

static const struct usbasp_spispeed spispeeds[] = {
	{ "auto",    USBASP_ISP_SCK_AUTO },
	{ "1.5M",    USBASP_ISP_SCK_1500 },
	{ "1500k",   USBASP_ISP_SCK_1500 },
	{ "750k",    USBASP_ISP_SCK_750 },
	{ "375k",    USBASP_ISP_SCK_375 },
	{ "187.5k",  USBASP_ISP_SCK_187_5 },
	{ "93.75k",  USBASP_ISP_SCK_93_75 },
	{ "32k",     USBASP_ISP_SCK_32 },
	{ "16k",     USBASP_ISP_SCK_16 },
	{ "8k",      USBASP_ISP_SCK_8 },
	{ "4k",      USBASP_ISP_SCK_4 },
	{ "2k",      USBASP_ISP_SCK_2 },
	{ "1k",      USBASP_ISP_SCK_1 },
	{ "500",     USBASP_ISP_SCK_0_5 },
	{ NULL,      0 },
};

struct usbasp_data {
	struct libusb_device_handle *handle;
};

static int usbasp_control(struct libusb_device_handle *handle,
			  uint8_t request, uint16_t value, uint16_t index,
			  unsigned char *buf, uint16_t length)
{
	int ret = libusb_control_transfer(
		handle,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
		request,
		value,
		index,
		buf,
		length,
		USB_TIMEOUT);

	if (ret < 0) {
		msg_perr("%s: libusb_control_transfer failed for request %d: %s\n",
			 __func__, request, libusb_error_name(ret));
		return -1;
	}
	return ret;
}

static int usbasp_spi_send_command(const struct flashctx *flash,
				   unsigned int writecnt, unsigned int readcnt,
				   const unsigned char *writearr, unsigned char *readarr)
{
	struct usbasp_data *data = flash->mst->spi.data;
	unsigned int total_len = writecnt + readcnt;
	if (total_len == 0)
		return 0;

	/* Assert Chip Select: pulses RESET line HIGH then LOW while keeping /SS as output */
	unsigned char dummy[4] = { 0 };
	if (usbasp_control(data->handle, USBASP_FUNC_CONNECT, 0, 0, dummy, sizeof(dummy)) < 0) {
		msg_perr("USBasp: Failed to assert CS# (connect)\n");
		return SPI_GENERIC_ERROR;
	}

	size_t alloc_len = (total_len + 3) & ~3;
	uint8_t *tx_buf = calloc(1, alloc_len);
	uint8_t *rx_buf = calloc(1, alloc_len);
	if (!tx_buf || !rx_buf) {
		msg_perr("USBasp: Out of memory\n");
		free(tx_buf);
		free(rx_buf);
		usbasp_control(data->handle, USBASP_FUNC_CONNECT, 0, 0, dummy, sizeof(dummy));
		return SPI_GENERIC_ERROR;
	}

	memcpy(tx_buf, writearr, writecnt);

	int ret = 0;
	size_t pos = 0;
	while (pos < total_len) {
		uint8_t cmd[4];
		uint8_t res[4] = { 0 };

		memcpy(cmd, tx_buf + pos, 4);

		uint16_t wValue = cmd[0] | ((uint16_t)cmd[1] << 8);
		uint16_t wIndex = cmd[2] | ((uint16_t)cmd[3] << 8);

		if (usbasp_control(data->handle, USBASP_FUNC_TRANSMIT, wValue, wIndex, res, 4) < 0) {
			msg_perr("USBasp: SPI transmit failed at offset %zu\n", pos);
			ret = SPI_GENERIC_ERROR;
			break;
		}

		memcpy(rx_buf + pos, res, 4);
		pos += 4;
	}

	/*
	 * Pulse Chip Select HIGH via USBASP_FUNC_CONNECT instead of USBASP_FUNC_DISCONNECT.
	 * In USBasp firmware, ispDisconnect() sets PB2 (/SS) as a low input before
	 * disabling SPCR, which triggers an AVR hardware SPI slave-mode fault and
	 * latches SPIF=1 in SPSR, corrupting the next SPI transfer.
	 */
	if (usbasp_control(data->handle, USBASP_FUNC_CONNECT, 0, 0, dummy, sizeof(dummy)) < 0) {
		msg_perr("USBasp: Failed to de-assert CS#\n");
		ret = SPI_GENERIC_ERROR;
	}

	if (ret == 0 && readcnt > 0) {
		memcpy(readarr, rx_buf + writecnt, readcnt);
	}

	free(tx_buf);
	free(rx_buf);
	return ret;
}

static int usbasp_spi_shutdown(void *data)
{
	struct usbasp_data *usbasp_data = data;
	if (usbasp_data) {
		if (usbasp_data->handle) {
			unsigned char dummy[4] = { 0 };
			usbasp_control(usbasp_data->handle, USBASP_FUNC_DISCONNECT, 0, 0, dummy, sizeof(dummy));
			libusb_release_interface(usbasp_data->handle, 0);
			libusb_attach_kernel_driver(usbasp_data->handle, 0);
			libusb_close(usbasp_data->handle);
		}
		free(usbasp_data);
	}
	libusb_exit(NULL);
	return 0;
}

static const struct spi_master spi_master_usbasp = {
	.features       = SPI_MASTER_4BA,
	.max_data_read  = MAX_DATA_READ_UNLIMITED,
	.max_data_write = 256,
	.command        = usbasp_spi_send_command,
	.read           = default_spi_read,
	.write_256      = default_spi_write_256,
	.shutdown       = usbasp_spi_shutdown,
};

static int usbasp_spi_init(const struct programmer_cfg *cfg)
{
	int ret = libusb_init(NULL);
	if (ret < 0) {
		msg_perr("%s: couldn't initialize libusb!\n", __func__);
		return -1;
	}

#if LIBUSB_API_VERSION < 0x01000106
	libusb_set_debug(NULL, 3);
#else
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

	struct usbasp_data *data = calloc(1, sizeof(*data));
	if (!data) {
		msg_perr("Out of memory!\n");
		libusb_exit(NULL);
		return 1;
	}

	uint16_t vid = devs_usbasp[0].vendor_id;
	uint16_t pid = devs_usbasp[0].device_id;
	data->handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
	if (data->handle == NULL) {
		msg_perr("Couldn't open device %04x:%04x.\n", vid, pid);
		free(data);
		libusb_exit(NULL);
		return 1;
	}

	if (usb_dev_claim_and_describe(data->handle, 0) != 0) {
		libusb_close(data->handle);
		free(data);
		libusb_exit(NULL);
		return 1;
	}

	/* Parse spispeed parameter if supplied */
	uint8_t sck_index = USBASP_ISP_SCK_AUTO;
	char *speed_str = extract_programmer_param_str(cfg, "spispeed");
	if (speed_str) {
		bool found = false;
		for (size_t i = 0; spispeeds[i].name != NULL; i++) {
			if (!strcasecmp(speed_str, spispeeds[i].name)) {
				sck_index = spispeeds[i].sck_index;
				found = true;
				break;
			}
		}
		if (!found) {
			msg_perr("Invalid spispeed parameter: '%s'\n", speed_str);
			free(speed_str);
			usbasp_spi_shutdown(data);
			return 1;
		}
		free(speed_str);
	}

	uint8_t sck_res = 0;
	ret = usbasp_control(data->handle, USBASP_FUNC_SETISPSCK, sck_index, 0, &sck_res, 1);
	if (ret != 1 || sck_res != 0) {
		msg_pwarn("USBasp: Warning: cannot set SCK period. Please check for USBasp firmware update.\n");
	}

	/*
	 * Clear any stale SPIF flag in SPSR left over from a previous disconnect
	 * by connecting and clocking a dummy 0xFF transfer, then re-pulsing CS#.
	 */
	unsigned char dummy[4] = { 0 };
	usbasp_control(data->handle, USBASP_FUNC_CONNECT, 0, 0, dummy, sizeof(dummy));
	usbasp_control(data->handle, USBASP_FUNC_TRANSMIT, 0xffff, 0xffff, dummy, sizeof(dummy));
	usbasp_control(data->handle, USBASP_FUNC_CONNECT, 0, 0, dummy, sizeof(dummy));

	return register_spi_master(&spi_master_usbasp, data);
}

const struct programmer_entry programmer_usbasp = {
	.name           = "usbasp",
	.type           = USB,
	.devs.dev       = devs_usbasp,
	.init           = usbasp_spi_init,
};
