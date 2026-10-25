/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "lifecycle.h"

#if CONFIG_USBASP == 1

#define USBASP_FUNC_CONNECT        1
#define USBASP_FUNC_DISCONNECT     2
#define USBASP_FUNC_TRANSMIT       3
#define USBASP_FUNC_SETISPSCK     10

static int usbasp_libusb_init(void *state, libusb_context **ctx)
{
	*ctx = not_null();
	return 0;
}

static int usbasp_libusb_control_transfer(void *state,
					  libusb_device_handle *devh,
					  uint8_t bmRequestType,
					  uint8_t bRequest,
					  uint16_t wValue,
					  uint16_t wIndex,
					  unsigned char *data,
					  uint16_t wLength,
					  unsigned int timeout)
{
	if (bRequest == USBASP_FUNC_CONNECT || bRequest == USBASP_FUNC_DISCONNECT) {
		return 0;
	}

	if (bRequest == USBASP_FUNC_SETISPSCK) {
		if (data && wLength >= 1)
			data[0] = 0;
		return 1;
	}

	if (bRequest == USBASP_FUNC_TRANSMIT) {
		uint8_t opcode = wValue & 0xFF;
		memset(data, 0, wLength);
		if (opcode == 0x9F && wLength >= 4) {
			data[0] = 0x00;
			data[1] = 0xEF;
			data[2] = 0x40;
			data[3] = 0x17;
		}
		return wLength;
	}

	return 0;
}

static struct io_mock_fallback_open_state usbasp_fallback_open_state = {
	.noc = 0,
	.paths = { NULL },
};

void usbasp_basic_lifecycle_test_success(void **state)
{
	const struct io_mock usbasp_io = {
		.libusb_init = usbasp_libusb_init,
		.libusb_control_transfer = usbasp_libusb_control_transfer,
		.fallback_open_state = &usbasp_fallback_open_state,
	};

	run_basic_lifecycle(state, &usbasp_io, &programmer_usbasp, "");
}

#else
	SKIP_TEST(usbasp_basic_lifecycle_test_success)
#endif /* CONFIG_USBASP */
