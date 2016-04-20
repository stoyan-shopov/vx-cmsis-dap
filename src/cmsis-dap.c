/*
Copyright (c) 2015-2016 stoyan shopov

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <libopencm3/stm32/gpio.h>


#include <stdint.h>
#include <string.h>
#include "cmsis-dap.h"
#include "swd.h"

enum CMSIS_DAP_COMMAND
{
	ID_DAP_Info                     =	0x00,
	ID_DAP_LED                      =	0x01,
	ID_DAP_Connect                  =	0x02,
	ID_DAP_Disconnect               =	0x03,
	ID_DAP_TransferConfigure        =	0x04,
	ID_DAP_Transfer                 =	0x05,
	ID_DAP_TransferBlock            =	0x06,
	ID_DAP_TransferAbort            =	0x07,
	ID_DAP_WriteABORT               =	0x08,
	ID_DAP_Delay                    =	0x09,
	ID_DAP_ResetTarget              =	0x0A,
	ID_DAP_SWJ_Pins                 =	0x10,
	ID_DAP_SWJ_Clock                =	0x11,
	ID_DAP_SWJ_Sequence             =	0x12,
	ID_DAP_SWD_Configure            =	0x13,
	ID_DAP_JTAG_Sequence            =	0x14,
	ID_DAP_JTAG_Configure           =	0x15,
	ID_DAP_JTAG_IDCODE              =	0x16,
};

enum CMSIS_DAP_INFO_ID
{
	DAP_INFO_VENDOR_ID			= 0x01, /* string */
	DAP_INFO_PRODUCT_ID			= 0x02, /* string */
	DAP_INFO_SERIAL_NUMBER			= 0x03, /* string */
	DAP_INFO_CMSIS_DAP_FIRMWARE_VERSION	= 0x04, /* string */
	DAP_INFO_TARGET_DEVICE_VENDOR		= 0x05, /* string */
	DAP_INFO_TARGET_DEVICE_NAME		= 0x06, /* string */
	DAP_INFO_CAPABILITIES			= 0xf0, /* byte */
	DAP_INFO_MAX_PACKET_COUNT		= 0xfe, /* byte */
	DAP_INFO_MAX_PACKET_SIZE		= 0xff, /* short */
};

/* miscellaneous constants */
enum
{
	/* dap port number for ID_DAP_Connect */
	DAP_PORT_DEFAULT	= 0,
	DAP_PORT_SWD		= 1,
	DAP_PORT_JTAG		= 2,
	/* led constants for ID_DAP_LED */
	CONNECT_LED		= 0,
	RUNNING_LED		= 1,
	LED_OFF			= 0,
	LED_ON			= 1,
};

enum
{
	DAP_OK		= 0x00,
	DAP_ERROR	= 0xff,
};

struct __attribute__((packed)) cmsis_dap_request
{
	/* this is an enumerator from CMSIS_DAP_COMMAND */
	uint8_t		command_id;
	union
	{
		/* ID_DAP_Info request */
		/* this is an enumerator from CMSIS_DAP_INFO_ID */
		uint8_t		info_id;
		/* ID_DAP_Connect request - port number */
		uint8_t		dap_port;
		/* ID_DAP_SWJ_Clock request */
		uint32_t	swj_clock;
		/* ID_DAP_TransferConfigure request */
		struct __attribute__((packed))
		{
			uint8_t		idle_cycles;
			uint16_t	wait_retry_count;
			uint16_t	match_retry_count;
		};
		/* ID_DAP_SWD_Configure request - port number */
		struct __attribute__((packed))
		{
			uint8_t		turnaround_clock_periods	: 2;
			uint8_t		is_dataphase_generated		: 1;
		};
		/* ID_DAP_LED request - port number */
		struct __attribute__((packed))
		{
			uint8_t		led_id;
			uint8_t		led_state;
		};
		/* ID_DAP_SWJ_Sequence request */
		struct __attribute__((packed))
		{
			uint8_t		sequence_bit_count;
			uint8_t		sequence_bytes[0];
		};
		/* ID_DAP_WriteABORT request */
		struct __attribute__((packed))
		{
			/* ignored for swd */
			uint8_t		abort_dap_index;
			uint32_t	abort_value;
		};
		/* ID_DAP_TransferBlock request */
		struct __attribute__((packed))
		{
			/* ignored for swd */
			uint8_t		block_dap_index;
			uint16_t	block_transfer_count;
			uint8_t		block_transfer_request;
			uint32_t	block_data[0];
		};
		/* ID_DAP_SWJ_Pins request */
		struct __attribute__((packed))
		{
			uint8_t		pin_output;
			uint8_t		pin_select;
			uint32_t	pin_wait_time;
		};
		/* ID_DAP_Transfer request */
		struct __attribute__((packed))
		{
			/* ignored for swd */
			uint8_t		dap_index;
			uint8_t		transfer_count;
			uint8_t		transfer_data[0];
#if 0
			struct __attribute__((packed))
			{
				/* APnDP */
				uint8_t		apndp	: 1;
				/* RnW */
				uint8_t		rnw	: 1;
				/* register address bit 2 */
				uint8_t		a2	: 1;
				/* register address bit 3 */
				uint8_t		a3	: 1;
				/* !!! only valid for reading a register !!!
				 * read register with value match - if this
				 * flag is nonzero, */
				uint8_t		is_value_match_used	: 1;
				uint8_t		is_write_match_mask	: 1;
			};
#endif
		};
	};
};

struct cmsis_dap_response __attribute__((packed));
struct __attribute__((packed)) cmsis_dap_response
{
	uint8_t		command_id;
	union __attribute__((packed))
	{
		/* generic status reply */
		uint8_t	status;
		/* ID_DAP_Info response */
		struct __attribute__((packed))
		{
			uint8_t	info_len;
			union __attribute__((packed))
			{
				uint8_t info_byte;
				uint16_t info_short;
				uint8_t	data[0];
			};
		};
		/* ID_DAP_Connect response - port number */
		uint8_t	dap_port;
		/* ID_DAP_SWJ_Pins response */
		struct __attribute__((packed))
		{
			uint8_t		pin_input;
		};
		/* ID_DAP_Transfer response */
		struct __attribute__((packed))
		{
			uint8_t		transfer_count;
			uint8_t		transfer_response;
			uint8_t		transfer_data[0];
		};
		/* ID_DAP_TransferBlock response */
		struct __attribute__((packed))
		{
			uint16_t	block_transfer_count;
			uint8_t		block_transfer_response;
			uint32_t	block_transfer_data[0];
		};
	};
};

int dap_xfer_req_cnt = 10;
int dap_xfer_err_cnt;
int block_cnt;

void report_error(void)
{
	dap_xfer_err_cnt ++;
}

bool cmsis_dap_process_request(void * request, void * response)
{
uint32_t fetch_data(uint8_t * p) { return * p | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }
void store_data(uint8_t * p, uint32_t data) { p[0] = data, p[1] = data >> 8, p[2] = data >> 16, p[3] = data >> 24; }
struct cmsis_dap_request * req = (struct cmsis_dap_request *) request;
struct cmsis_dap_response * res = (struct cmsis_dap_response *) response;
bool status = false;

	memset(res, 0, 64);
	res->command_id = req->command_id;
	if (req->block_transfer_count == 14)
		report_error();
	switch (req->command_id)
	{
		case ID_DAP_Info:
			switch (req->info_id)
			{
				case DAP_INFO_PRODUCT_ID:
				case DAP_INFO_SERIAL_NUMBER:
				case DAP_INFO_CMSIS_DAP_FIRMWARE_VERSION:
				case DAP_INFO_VENDOR_ID:
					res->info_len = 0;
					status = true;
					break;
				case DAP_INFO_MAX_PACKET_SIZE:
					res->info_len = 2;
					res->info_short = 64;
					status = true;
					break;
				case DAP_INFO_MAX_PACKET_COUNT:
					res->info_len = 1;
					res->info_short = 1;
					status = true;
					break;
			}
			break;
		case ID_DAP_Connect:
			switch (req->dap_port)
			{
				case DAP_PORT_SWD:
					init_sw_hardware();
					res->dap_port = DAP_PORT_SWD;
					status = true;
					break;
			}
			break;
		case ID_DAP_SWJ_Clock:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_TransferConfigure:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_SWD_Configure:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_LED:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_SWJ_Sequence:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_Disconnect:
			/* ignored for now */
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_WriteABORT:
			/*! \todo	make use of the abort register symbollic address for better maintainability */
			write_dp(0, req->abort_value);
			res->status = DAP_OK;
			status = true;
			break;
		case ID_DAP_SWJ_Pins:
			/* only control the reset signal for now */
			if (req->pin_select & (1 << 7))
			{
				(req->pin_output & (1 << 7)) ? gpio_set(GPIOB, GPIO6) : gpio_clear(GPIOB, GPIO6);
				if (req->pin_output & (1 << 7))
				{
					volatile int i;
					for (i = 0; i < 1000; i++);
					init_sw_hardware();
					/* set boot block mapping on lpc1754 */
					write_ap(1, 0x400fc040);
					write_ap(3, 1);
				}
			}
			res->pin_input = req->pin_output;
			status = true;
			break;
		case ID_DAP_Transfer:
			dap_xfer_req_cnt ++;
			if (1)
			{
				static uint32_t match_mask;
				uint8_t * data_in = req->transfer_data, * data_out = res->transfer_data, swq;
				uint32_t x;
				res->transfer_count = 0;
				res->transfer_response = SW_ACK_OK;
				while (req->transfer_count --)
				{
					swq = * data_in ++;
					if (swq & (1 << 1))
					{
						/* read access */
						if (swq & (1 << 4))
						{
							uint32_t match_value = fetch_data(data_in);
							data_in += sizeof(uint32_t);
							/* read register with value match */
							while (1)
							{
								res->transfer_response = ((swq & (1 << 0)) ? read_ap : read_dp)((swq >> 2) & 3, & x);
								if (res->transfer_response != SW_ACK_OK)
									goto report_error;
								if ((x & match_mask) == match_value)
									break;
							}
res->transfer_count ++;
							continue;
						}
						res->transfer_response = ((swq & (1 << 0)) ? read_ap : read_dp)((swq >> 2) & 3, & x);
						store_data(data_out, x);
						data_out += sizeof(uint32_t);
						if (res->transfer_response != SW_ACK_OK)
						{
report_error:
							report_error();
							if (res->transfer_response == SW_ACK_PROTOCOL_ERROR)
							{
								res->transfer_response = SW_ACK_FAULT;
								res->transfer_response |= 1 << 3;
							}
							break;
						}
					}
					else
					{
						/* write access */
						if (swq & (1 << 5))
						{
							/* write match mask */
							match_mask = fetch_data(data_in);
							data_in += sizeof(uint32_t);
res->transfer_count ++;
							continue;
						}
						/* normal write access */
						x = fetch_data(data_in);
						data_in += sizeof(uint32_t);
						res->transfer_response = ((swq & (1 << 0)) ? write_ap : write_dp)((swq >> 2) & 3, x);
						if (res->transfer_response != SW_ACK_OK)
						{
							report_error();
							if (res->transfer_response == SW_ACK_PROTOCOL_ERROR)
							{
								res->transfer_response = SW_ACK_FAULT;
								res->transfer_response |= 1 << 3;
							}
							break;
						}
					}
					res->transfer_count ++;
				}
				if (res->transfer_response != SW_ACK_OK)
				{
					if (1) init_sw_hardware();
					while(0);
				}
				status = true;
				break;
			}
			status = true;
			break;
		case ID_DAP_TransferBlock:
			{
				uint32_t x;
				int idx = 0;
				res->block_transfer_count = 0;
				res->block_transfer_response = SW_ACK_OK;
				while (req->block_transfer_count --)
				{
					res->block_transfer_count ++;
					if (req->block_transfer_request & (1 << 1))
					{
						/* read access */
						res->block_transfer_response = ((req->block_transfer_request & (1 << 0)) ? read_ap : read_dp)((req->block_transfer_request >> 2) & 3, & x);
						res->block_transfer_data[idx ++] = x;
						if (res->block_transfer_response != SW_ACK_OK)
						{
							report_error();
							if (res->block_transfer_response == SW_ACK_PROTOCOL_ERROR)
							{
								res->block_transfer_response = SW_ACK_FAULT;
								res->block_transfer_response |= 1 << 3;
							}
							break;
						}
					}
					else
					{
						/* write access */
						res->block_transfer_response = ((req->block_transfer_request & (1 << 0)) ? write_ap : write_dp)((req->block_transfer_request >> 2) & 3, req->block_data[idx ++]);
						if (res->block_transfer_response != SW_ACK_OK)
						{
							report_error();
							if (res->block_transfer_response == SW_ACK_PROTOCOL_ERROR)
							{
								res->block_transfer_response = SW_ACK_FAULT;
								res->block_transfer_response |= 1 << 3;
							}
							break;
						}
					}
				}
				if (res->block_transfer_response != SW_ACK_OK)
				{
					if (1) init_sw_hardware();
					while(0);
				}
				status = true;
				break;
			}
		default:
			while (1);
	}
	return status;
}
