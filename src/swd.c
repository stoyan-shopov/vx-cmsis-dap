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

#include "swd.h"
#include <stdbool.h>

#if 1
#define DBGMSG(x)
#define usbprint(x)
#define dprintint(x)
#endif

#include "swd-hw.c"

volatile struct
{
volatile int write_ap_cnt, wait_cnt, write_ap_err_cnt, bitseq_errors, bitseq_waits, bitseq_faults, bitseq_nacks, bitseq_parity_errors, bitseq_xfers_total;
}
counters;

volatile int switch_to_sw_flag = 1;
volatile int reset_swd_bus_flag = 1;
volatile int read_dp_idcode = 1;
volatile int sw_debug = 1;

/* on the vortex (serial-wire driving part
 * is an stm32f103c8t6), swdiotms is pb14(pin27),
 * swclktck is pb13(pin26) */

enum
{
	ENABLE_SW_DELAYS	= 0,
};


/*! an enumeration of the debug port (dp) register addresses
 *
 * note that some of the register addresses values are numerically equal
 * for different registers - this is because exactly which
 * register is being accessed is determined by the register
 * request type (read or write)
 *
 * \note	these register addresses values are intended
 *		to be passed as the 'a32' parameter of the
 *		sw_bitseq_xfer() routine - and as such,
 *		they represent address bits [3:2] of the
 *		corresponding dp register address (i.e., they
 *		equal the corresponding dp register address
 *		shifted left by two bits) */
enum SW_DP_REG_ADDRESS_ENUM
{
	/*! the identification code register (IDCODE) - read only access - dp reg address 0 */
	SW_DP_REG_IDCODE = 0 >> 2,
	/*! the access port (ap) abort register (ABORT) - write only access - dp reg address 0 */
	SW_DP_REG_AP_ABORT = 0 >> 2,
	/*! the control/status register (CTRL/STAT) - read/write access - dp reg address 4
	 *
	 * \note	bit CTRLSEL bit in the SELECT register (below)
	 *		determines if this, or the WCR register (below)
	 *		is being accessed; to access the CTRL/STAT register,
	 *		the CTRLSEL bit must equal 0 */
	SW_DP_REG_CTRLSTAT = 4 >> 2,
	/*! the wire control register (WCR) - read/write access - dp reg address 4
	 *
	 * \note	bit CTRLSEL bit in the SELECT register (below)
	 *		determines if this, or the CTRL/STAT register (above)
	 *		is being accessed; to access the WCR register,
	 *		the CTRLSEL bit must equal 1 */
	SW_DP_REG_WCR = 4 >> 2,
	/*! the read resend register (RESEND) - read only access - dp reg address 8 */
	SW_DP_REG_RESEND = 8 >> 2,
	/*! the ap select register (SELECT) - write only access - dp reg address 8 */
	SW_DP_REG_SELECT = 8 >> 2,
	/*! the read buffer register (RDBUFF) - read only access - dp reg address 0xc */
	SW_DP_REG_RDBUFF = 0xc >> 2,

};



/*! an enumeration for the mem-ap (access port) register addresses */
enum SW_MEM_AP_REG_ADDR_ENUM
{
	/*! control/status word (CSW) register - read/write access */
	SW_MEM_AP_REG_CSW = 0x00,
	/*! transfer address register (TAR) register - read/write access */
	SW_MEM_AP_REG_TAR = 0x04,
	/*! data read/write (DRW) register - read/write access */
	SW_MEM_AP_REG_DRW = 0x0c,
	/*! banked data register 0 (BD0) register - read/write access */
	SW_MEM_AP_REG_BD0 = 0x10,
	/*! banked data register 1 (BD1) register - read/write access */
	SW_MEM_AP_REG_BD1 = 0x14,
	/*! banked data register 2 (BD2) register - read/write access */
	SW_MEM_AP_REG_BD2 = 0x18,
	/*! banked data register 3 (BD3) register - read/write access */
	SW_MEM_AP_REG_BD3 = 0x1c,
	/*! configuration register (CFG) - read only access */
	SW_MEM_AP_REG_CFG = 0xf4,
	/*! debug base address register (BASE) - read only access */
	SW_MEM_AP_REG_BASE = 0xf8,
	/*! identification register (IDR) - read only access */
	SW_MEM_AP_REG_IDR = 0xfc,
};


/*! this variable holds the last written value to the SELECT debug port register
 *
 * some of the fields of this register are significant when addressing
 * specific ap or dp registers - this is kept here for reference in
 * order to avoid unneeded reloads of this register */
static union
{
	struct
	{
		/*! the ctrlsel bit */
		uint32_t	ctrlsel	: 1;
		uint32_t	: 3;
		/*! the apbanksel field */
		uint32_t	apbanksel : 4;
		uint32_t	: 16;
		/*! the apsel field */
		uint32_t	apsel : 8;
	};
	/*! the whole 32 bit SELECT register value, for convenient access */
	uint32_t	select_reg;
}
sw_select_reg;

/*! this variable holds the last known value of the mem-ap transfer address register (TAR)
 *
 * this is useful for lowering traffic to the TAR register when it already
 * contains a value that does not need updating; when used with automatic
 * address incrementing (field 'addrinc' in the mem-ap CSW register),
 * updating the TAR register can be avoided in most of the cases;
 * note that in the 'IHI0031A_ARM_debug_interface_v5.pdf' document,
 * in section 5.4.4, it is stated that automatic address increment
 * is only guaranteed to operate on the bottom 10 bits of the address
 * held in the TAR register - so be careful to take that in account
 * when handling the TAR register */
static uint32_t last_known_tar;

static enum SW_ACK_ENUM sw_write_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t data);
static enum SW_ACK_ENUM sw_write_ap(enum SW_MEM_AP_REG_ADDR_ENUM ap_reg_addr, uint32_t data);
static enum SW_ACK_ENUM sw_read_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t * data);
static bool read_dp_ctrl_stat_reg(uint32_t * val);
static bool write_dp_abort_reg(uint32_t val);

uint32_t nr_swd_idle_cycles = 4;

/*!
 *	\fn	static inline enum SW_ACK_ENUM sw_set_transfer_addr_reg(uint32_t tar)
 *	\brief	sets the target mem-ap TAR register to a requested value
 *
 *	\param	tar	new value to write to the TAR register; this value
 *			*must* specify a word aligned target address
 *	\return	the acknowledge value received in the acknowledge phase in
 *		the serial wire protocol (an enumerator value from the SW_ACK_ENUM
 *		enumeration)
 */ 
static inline enum SW_ACK_ENUM sw_set_transfer_addr_reg(uint32_t tar)
{
enum SW_ACK_ENUM ack;
	ack = sw_write_ap(SW_MEM_AP_REG_TAR, tar);
	if (ack == SW_ACK_OK)
		last_known_tar = tar;
	return ack;
}

/*!
 *	\fn	static inline bool is_tar_reg_reload_needed(void)
 *	\brief	increments the last known address held in the target TAR address with one word address
 *
 *	\return	increments the last known address held in the
 *		target TAR address with one word address; returns true
 *		if the least significant 10 bits of the address value
 *		in the TAR register wrap around and therefore a reload
 *		of the TAR register is necessary; returns false if reload of
 *		the target TAR register is not necessary
 *
 *	\note	it is mandatory to call this function after each successfull
 *		mem-ap data transfer that automatically increments the TAR
 *		register (i.e. when the 'addrinc' field in the mem-ap
 *		csw register is set to 'single'), and if this routine
 *		returns 'true', it is mandatory to reload the target TAR
 *		register
 *	\note	the value held in the 'last_known_tar' variable
 *		*must* specify a word aligned target address
 */ 
static inline bool is_tar_reg_reload_needed(void)
{
	if (last_known_tar & 3)
		return true;
	last_known_tar += sizeof(uint32_t);
	if (!(last_known_tar & ((1 << 10) - 1)))
		return true;
	else
		return false;
}
static inline enum SW_ACK_ENUM sw_wordinc_transfer_addr_reg(void)
{
enum SW_ACK_ENUM ack;
	if (last_known_tar & 3)
		return SW_ACK_FAULT;
	ack = SW_ACK_OK;
	last_known_tar += sizeof(uint32_t);
	if (!(last_known_tar & ((1 << 10) - 1)))
	{
		/* low 10 bit address wraparound - read the comments
		 * about the 'last_known_tar' variable in this file */
		ack = sw_write_ap(SW_MEM_AP_REG_TAR, last_known_tar);
		if (ack != SW_ACK_OK)
			last_known_tar -= sizeof(uint32_t);
	}
	return ack;
}

/*!
 *	\fn	static inline int sw_compute_even_parity_bit(uint32_t x)
 *	\brief	computes the even parity checksum bit of a 32 bit input word
 *
 *	\note	this function can be used for computing the even parity checksum
 *		bit for any word width less than 32 - just pad the word
 *		with zeros on the left
 *
 *	\param	x	the 32 bit word for which to compute the even parity
 *			checksum bit
 *	\return	1, if the number of bits set to 1 in the 32 bit 'x' parameter
 *		is odd, 0 - if this number is even */
static inline int sw_compute_even_parity_bit(uint32_t x)
{
	return (x ^= x >> 16, x ^= x >> 8, x ^= x >> 4, x ^= x >> 2, x ^= x >> 1) & 1;
}




/*!
 *	\fn	static void sw_report_wire_error(enum SW_ACK_ENUM ack, bool is_ap_access, bool is_read_access, uint32_t a32)
 *	\brief	reports error details in case of a serial wire transfer error
 *
 *	this function prints error details in case of a serial wire
 *	error - whenever the acknowledge received from the target
 *	is different from SW_ACK_OK
 *
 *	\param	ack	the value of the last acknowledge received from the target
 *	\param	is_ap_access	true, if the last transaction was targetting the
 *				target access port, false if it was targeting the
 *				target debug port
 *	\param	is_read_access	true, if the last transaction was a read request,
 *				false if it was a write request
 *	\param	a32	the value of the a32 field of the last transaction
 *	\return	none */
static void sw_report_wire_error(enum SW_ACK_ENUM ack, bool is_ap_access, bool is_read_access, uint32_t a32)
{
uint32_t x;
return;
	DBGMSG("warning: sw ack received is "); dprintint(ack); usbprint("\n");
	if (ack == SW_ACK_FAULT)
		usbprint("serial wire fault response; ");
	else if (ack == SW_ACK_WAIT)
		usbprint("serial wire wait response; ");
	else
	{
		usbprint("bad serial wire response; aborting transfer\n");
		sw_config_swdio_output();
		sw_insert_idle_cycles(10);
		return;
	}
	usbprint(is_ap_access ? "ap " : "dp ");
	usbprint(is_read_access ? "read " : "write ");
	usbprint("transaction; ");
	usbprint("; a32 is ");
	dprintint(a32);

	sw_config_swdio_output();
	sw_insert_idle_cycles(10);

	usbprint("; idcode is: ");
	dprintint(sw_read_dp_idcode());

	usbprint("; ctrl/stat is: ");
	usbprint("take1 ");
	read_dp_ctrl_stat_reg(&x);
	dprintint(x);
	usbprint("take2 ");
	read_dp_ctrl_stat_reg(&x);
	dprintint(x);
	usbprint("\n");

}


/*!
 *	\fn	static enum SW_ACK_ENUM sw_bitseq_xfer(bool is_ap_access, bool is_read_access, int ctrlsel, int a32, uint32_t * data)
 *	\brief	performs a low level serial wire transaction on the serial line hardware
 *
 *	for details on the low level serial wire protocol details,
 *	refer to the "DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf"
 *	document available for download on the arm site
 *
 *	\note	it is assumed, that on entry to this function, the swdio hardware
 *		signal is configured as an output, and the swclk hardware signal
 *		is also configured as an output - and it is in a high logic
 *		level state; these assertions are also guaranteed to remain
 *		true on exit from this function
 *
 *	\param	is_ap_access	if true, then this transaction is an access port
 *				(ap) access; otherwise (if false), then this is 
 *				a debug port (dp) access; this flag determines
 *				the value of the APnDP bit in the serial wire
 *				packet request phase
 *	\param	is_read_access	if true, then this transaction is a read request;
 *				otherwise (if false), then this transaction is
 *				a write request; this flag determines the value
 *				of the RnW bit in the serial wire packet request
 *				phase
 *	\param	ctrlsel		the value of the CTRLSEL bit in the SELECT dp
 *				register for this access; permitted values are:
 *					-1 - for do not care,
 *					0  - if the CTRLSEL bit should be 0
 *					1  - if the CTRLSEL bit should be 1
 *	\param	a32		the a[3:2] address field for the dp or ap
 *				register access
 *	\param	data		in case of read accesses (is_read_access == true),
 *				the location at which to store the data sampled
 *				in the data transfer phase of the serial wire
 *				protocol; in case of write accesses
 *				(is_read_access == false), the location of the
 *				value which to send over the serial wire during
 *				the data transfer phase of the sw protocol
 *	\return	the acknowledge value received in the acknowledge phase in
 *		the serial wire protocol (an enumerator value from the SW_ACK_ENUM
 *		enumeration)
 *	\note	the 'ctrlsel' parameter seems to be highly redundant, as (right now)
 *		it is never to be used; i(sgs) realised that a bit late, but i will
 *		leave it be; that is probably because i am not too smart...
 *	\warning	if the acknowledge value received during the acknowledge
 *			phase of the sw protocol is not equal to SW_ACK_OK,
 *			then the sw transaction is aborted immediately and
 *			a data transfer phase is *not* performed by this routine */

volatile struct bitseq_log
{
	enum {	DP_ACCESS = 0, AP_ACCESS, } port;
	enum {	WRITE_ACCESS = 0, READ_ACCESS, } access;
	int	a32;
	uint32_t data;
	enum SW_ACK_ENUM ack;
}
bitseq_log[8];
int bitseq_idx;

static enum SW_ACK_ENUM sw_bitseq_xfer(bool is_ap_access, bool is_read_access, int ctrlsel, int a32, uint32_t * data)
{
int i;
uint32_t x, ack, rdata, parity;

	bitseq_log[bitseq_idx] = (struct bitseq_log) { .port = is_ap_access & 1, .access = is_read_access & 1, .a32 = a32, .data = (is_read_access ? 0 : *data), };

counters.bitseq_xfers_total ++;

if (1 || !nr_swd_idle_cycles)
{

	/* first of all, see if the CTRLSEL bit in the SELECT
	 * dp register needs to be updated - this is the case
	 * if either of the dp CTRLSTAT or WCR register is
	 * being accessed - for other dp registers this is
	 * a do-not-care case */
	if (ctrlsel != -1 && (ctrlsel & 1) != sw_select_reg.ctrlsel)
	{
		/* the ctrlsel bit must be updated */
		sw_select_reg.ctrlsel = ctrlsel & 1;
		ack = sw_bitseq_xfer(false, false, -1, SW_DP_REG_SELECT, & sw_select_reg.select_reg);
		if (ack != SW_ACK_OK)
			return (enum SW_ACK_ENUM) ack;
	}
	/* if this is a write to the dp SELECT register, update
	 * the cached value for this register */
	if (a32 == SW_DP_REG_SELECT && !is_read_access)
	{
		sw_select_reg.select_reg = * data;
	}
	/* construct the packet request sequence */
	x = 0;
	if (is_ap_access)
		x |= 1 << 1;
	if (is_read_access)
		x |= 1 << 2;
	/* insert a32 field */
	x |= a32 << 3;
	/* add parity */
	x |= sw_compute_even_parity_bit(x) << 5;
	/* add start and park bits */
	x |= 0x81;

	ack = clock_header_out_get_ack(x);

	if (ack != SW_ACK_OK)
		sw_report_wire_error(ack, is_ap_access, is_read_access, a32), counters.bitseq_nacks ++;

	if (is_read_access)
	{
		uint64_t x;
		x = clock_word_and_parity_in();
		* data = x;

		if (x >> 32)
		{
			DBGMSG("error: bad parity bit received on a sw read transaction\n");
			counters.bitseq_parity_errors ++;
			ack = SW_ACK_PROTOCOL_ERROR;
		}
	}
	else
	{
		clock_word_and_parity_out(* data);

	}
	/* issue a couple of idle cycles to make sure the sw transfers
	 * have completed */
	sw_insert_idle_cycles(10);
	switch (ack)
	{
		case SW_ACK_OK:
			if (0)
		case SW_ACK_WAIT:
			counters.bitseq_waits ++;
			if (0)
		case SW_ACK_FAULT:
			counters.bitseq_faults ++;
			break;
		default:
			counters.bitseq_errors ++;
			ack = SW_ACK_PROTOCOL_ERROR;
	}

	bitseq_log[bitseq_idx ++].ack = ack;
	bitseq_idx &= 7;
	return (enum SW_ACK_ENUM) ack;
}
else
{

	/* first of all, see if the CTRLSEL bit in the SELECT
	 * dp register needs to be updated - this is the case
	 * if either of the dp CTRLSTAT or WCR register is
	 * being accessed - for other dp registers this is
	 * a do-not-care case */
	if (ctrlsel != -1 && (ctrlsel & 1) != sw_select_reg.ctrlsel)
	{
		/* the ctrlsel bit must be updated */
		sw_select_reg.ctrlsel = ctrlsel & 1;
		ack = sw_bitseq_xfer(false, false, -1, SW_DP_REG_SELECT, & sw_select_reg.select_reg);
		if (ack != SW_ACK_OK)
			return (enum SW_ACK_ENUM) ack;
	}
	/* if this is a write to the dp SELECT register, update
	 * the cached value for this register */
	if (a32 == SW_DP_REG_SELECT && !is_read_access)
	{
		sw_select_reg.select_reg = * data;
	}
	/* construct the packet request sequence */
	x = 0;
	if (is_ap_access)
		x |= 1 << 1;
	if (is_read_access)
		x |= 1 << 2;
	/* insert a32 field */
	x |= a32 << 3;
	/* add parity */
	x |= sw_compute_even_parity_bit(x) << 5;
	/* add start and park bits */
	x |= 0x81;

	ack = clock_header_out_get_ack_delay(x);

	if (ack != SW_ACK_OK)
		sw_report_wire_error(ack, is_ap_access, is_read_access, a32);

	if (is_read_access)
	{
		uint64_t x;
		x = clock_word_and_parity_in_delay();
		* data = x;

		if (x >> 32)
		{
			DBGMSG("error: bad parity bit received on a sw read transaction\n");
			/*! \todo	is this a good idea??? */
			ack = SW_ACK_FAULT;
		}
	}
	else
	{
		clock_word_and_parity_out_delay(* data);

	}
	/* issue a couple of idle cycles to make sure the sw transfers
	 * have completed */
	sw_insert_idle_cycles(10);

	return (enum SW_ACK_ENUM) ack;
}
}


/*!
 *	\fn	static bool read_dp_ctrl_stat_reg(uint32_t * val)
 *	\brief	reads the debug port (dp) 'ctrl/stat' register
 *
 *	\param	val	a pointer to where to store the value read
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
static bool read_dp_ctrl_stat_reg(uint32_t * val)
{
	return sw_read_dp(SW_DP_REG_CTRLSTAT, val) == SW_ACK_OK;
}

/*!
 *	\fn	static bool write_dp_abort_reg(uint32_t val)
 *	\brief	writes a value to the 'abort' register of a debug port (dp)
 *
 *	\param	val	the value to write to the 'abort' dp register
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
static bool write_dp_abort_reg(uint32_t val)
{
	return sw_write_dp(SW_DP_REG_AP_ABORT, val) == SW_ACK_OK;
}


static inline enum SW_ACK_ENUM sw_xfer_write_ap_word(uint32_t data)
{
uint32_t x, ack;
int retry_cnt;

if (1 || !nr_swd_idle_cycles)
{
	retry_cnt = 0;

retry:

	ack = clock_header_out_get_ack(0xbb);

	if (ack == SW_ACK_WAIT)
	{
		if (retry_cnt ++ < 4)
		{
			DBGMSG("warning: 'wait' sw ack received, retrying transaction"); usbprint("\n");
			goto retry;
		}
		else
		{
			DBGMSG("warning: too many sw ack 'wait' responses, aborting transaction"); usbprint("\n");
			return ack;
		}
	}

	/* shift data out */
	clock_word_and_parity_out(data);

	if (ack != SW_ACK_OK)
	{
		sw_report_wire_error(ack, true, false, SW_MEM_AP_REG_DRW >> 2);
	}

	return (enum SW_ACK_ENUM) ack;
}
else
{
	retry_cnt = 0;

retry_delay:

	ack = clock_header_out_get_ack_delay(0xbb);

	if (ack == SW_ACK_WAIT)
	{
		if (retry_cnt ++ < 4)
		{
			DBGMSG("warning: 'wait' sw ack received, retrying transaction"); usbprint("\n");
			goto retry_delay;
		}
		else
		{
			DBGMSG("warning: too many sw ack 'wait' responses, aborting transaction"); usbprint("\n");
			return ack;
		}
	}

	/* shift data out */
	clock_word_and_parity_out_delay(data);

	if (ack != SW_ACK_OK)
	{
		sw_report_wire_error(ack, true, false, SW_MEM_AP_REG_DRW >> 2);
	}

	return (enum SW_ACK_ENUM) ack;
}
}



static inline enum SW_ACK_ENUM sw_xfer_read_ap_word(uint32_t * data)
{
uint32_t y, ack;
uint64_t x;

if (!nr_swd_idle_cycles)
{
	ack = clock_header_out_get_ack(0x9f);

	/* read request */
	/* shift data in */
	x = clock_word_and_parity_in();
	* data = x;

	/* check parity */
	if (x >> 32)
	{
		DBGMSG("error: bad parity bit received on a sw read transaction\n");
		/*! \todo	is this a good idea??? */
		ack = SW_ACK_FAULT;
	}

	if (ack != SW_ACK_OK)
		sw_report_wire_error(ack, true, true, SW_MEM_AP_REG_DRW >> 2);

	return (enum SW_ACK_ENUM) ack;
}
else
{
	ack = clock_header_out_get_ack_delay(0x9f);

	/* read request */
	/* shift data in */
	x = clock_word_and_parity_in_delay();
	* data = x;

	/* check parity */
	if (x >> 32)
	{
		DBGMSG("error: bad parity bit received on a sw read transaction\n");
		/*! \todo	is this a good idea??? */
		ack = SW_ACK_FAULT;
	}

	if (ack != SW_ACK_OK)
		sw_report_wire_error(ack, true, true, SW_MEM_AP_REG_DRW >> 2);

	return (enum SW_ACK_ENUM) ack;
}

}



/*!
 *	\fn	static enum SW_ACK_ENUM sw_read_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t * data)
 *	\brief	reads a debug port (dp) register
 *
 *	\param	dp_reg_addr	the dp register address to read
 *	\param	data		a pointer to where to store the data read
 *	\return	the acknowledge value received in the acknowledge phase in
 *		the serial wire protocol (an enumerator value from the SW_ACK_ENUM
 *		enumeration)
 *
 *	\note	this function does not currently support control
 *		over the ctrlsel bit in the dp SELECT register,
 *		if you need such control, you should directly
 *		invoke sw_bitseq_xfer() */
static enum SW_ACK_ENUM sw_read_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t * data)
{
	return sw_bitseq_xfer(false, true, -1, dp_reg_addr, data);
}

/*!
 *	\fn	static enum SW_ACK_ENUM sw_write_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t data)
 *	\brief	writes to a debug port (dp) register
 *
 *	\param	dp_reg_addr	the dp register address to write to
 *	\param	data		the data word to write
 *	\return	the acknowledge value received in the acknowledge phase in
 *		the serial wire protocol (an enumerator value from the SW_ACK_ENUM
 *		enumeration)
 *
 *	\note	this function does not currently support control
 *		over the ctrlsel bit in the dp SELECT register,
 *		if you need such control, you should directly
 *		invoke sw_bitseq_xfer() */
static enum SW_ACK_ENUM sw_write_dp(enum SW_DP_REG_ADDRESS_ENUM dp_reg_addr, uint32_t data)
{
	return sw_bitseq_xfer(false, false, -1, dp_reg_addr, &data);
}

/*!
 *	\fn	static enum SW_ACK_ENUM sw_read_ap(enum SW_MEM_AP_REG_ADDR_ENUM ap_reg_addr, uint32_t * data)
 *	\brief	reads an access port (ap) register
 *
 *	\param	ap_reg_addr	the ap register address to read
 *	\param	data		a pointer to where to store the data read
 *	\return	SW_ACK_OK if the serial wire transaction completed
 *		successfully, another serial wire protocol acknowledge
 *		value on error; see SW_ACK_ENUM for details */
static enum SW_ACK_ENUM sw_read_ap(enum SW_MEM_AP_REG_ADDR_ENUM ap_reg_addr, uint32_t * data)
{
enum SW_ACK_ENUM res;
	/* first, see if the SELECT dp register needs updating */
	if (((ap_reg_addr >> 4) & 0xf) != sw_select_reg.apbanksel)
	{
		/* update the apbanksel field in the dp
		 * SELECT register */
		sw_select_reg.apbanksel = ap_reg_addr >> 4;
		if ((res = sw_write_dp(SW_DP_REG_SELECT, sw_select_reg.select_reg)) != SW_ACK_OK)
			return res;
	}
	res = sw_bitseq_xfer(true, true, -1, (ap_reg_addr >> 2) & 3, data);
	if (res != SW_ACK_OK)
		return res;
	/* for details, consult the arm document:
	 * IHI0031A_ARM_debug_interface_v5.pdf
	 * section 5.4 - 'protocol description'
	 * (and more specifically section 5.4.2 -
	 * 'the ok response'); also have a look at
	 * the description of the dp RDBUFF register
	 * (section 6.2.5) */
	/* obtain the posted result from the ap transaction
	 * just issued */
	/*! \todo	add timeout/error retry counter */
	while (1)
	{
		res = sw_read_dp(SW_DP_REG_RDBUFF, data);
		if (res == SW_ACK_OK || res != SW_ACK_WAIT)
			break;
	}
	return res;
}


/*!
 *	\fn	static enum SW_ACK_ENUM sw_write_ap(enum SW_MEM_AP_REG_ADDR_ENUM ap_reg_addr, uint32_t data)
 *	\brief	writes to an access port (ap) register
 *
 *	\param	ap_reg_addr	the ap register address to write to
 *	\param	data		the data word to write
 *	\return	SW_ACK_OK if the serial wire transaction completed
 *		successfully, another serial wire protocol acknowledge
 *		value on error; see SW_ACK_ENUM for details */
static enum SW_ACK_ENUM sw_write_ap(enum SW_MEM_AP_REG_ADDR_ENUM ap_reg_addr, uint32_t data)
{
enum SW_ACK_ENUM res;

	/* first, see if the SELECT dp register needs updating */
	if (((ap_reg_addr >> 4) & 0xf) != sw_select_reg.apbanksel)
	{
		/* update the apbanksel field in the dp
		 * SELECT register */
		sw_select_reg.apbanksel = ap_reg_addr >> 4;
		if ((res = sw_write_dp(SW_DP_REG_SELECT, sw_select_reg.select_reg)) != SW_ACK_OK)
			return res;
	}
	res = sw_bitseq_xfer(true, false, -1, (ap_reg_addr >> 2) & 3, &data);
	if (res != SW_ACK_OK)
		return res;
	/* for details, consult the arm document:
	 * IHI0031A_ARM_debug_interface_v5.pdf
	 * section 5.4 - 'protocol description'
	 * (and more specifically section 5.4.7 -
	 * 'sw-dp write buffering') v*/
	/* wait for the write buffer to get emptied - perform an access
	 * that the debug port is able to stall - writing the SELECT 
	 * register is one such access */
	/*! \todo	add timeout/error retry counter */
	while (1)
	{
		res = sw_write_dp(SW_DP_REG_SELECT, sw_select_reg.select_reg);
		if (res == SW_ACK_OK || res != SW_ACK_WAIT)
			break;
	}
	return res;
}


/*!
 *	\fn	static bool sw_reset_bus(void)
 *	\brief	performs a reset of the sw bus
 *
 *	\note	it is assumed, that on entry to this function, the swdio hardware
 *		signal is configured as an output, and the swclk hardware signal
 *		is also configured as an output - and it is in a high logic
 *		level state; these assertions are also guaranteed to remain
 *		true on exit from this function
 *
 *	the sw bus reset is achieved by clocking at least
 *	50 cycles on the bus while holding the swdio in a logic
 *	high level, and issuing at least one idle cycle;
 *	to get the sw bus out of reset and into an idle state,
 *	a read of the dp IDCODE register is performed;
 *	for details, refer to the
 *	"DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf"
 *	document available for download on the arm site
 *	(appendix b - 'serial wire protocol')
 *
 *	\param	none
 *	\return	true, if the bus reset was successful (i.e. the
 *		sw transaction succeeded and the value read
 *		from the IDCODE register matches the expected one),
 *		false if some error occurred */
static bool sw_reset_bus(void)
{
int i;
uint32_t x;
enum SW_ACK_ENUM ack;

	/* perform a swd reset sequence */
	/* (1) first - issue
	 * >= 50 clock cycles while holdind
	 * swdio hi */
	for (i = 0; i < 60; i ++)
		sw_clock_out_1();
	/* (2) second - issue
	 * >= 1 clock cycles while holdind
	 * swdio low (i.e. issue at least
	 * one idle cycle) */
	sw_insert_idle_cycles(16);
	/* to bring the serial-wire debug
	 * state machine out of reset and
	 * into an idle state - read the
	 * dp idcode register */

	ack = sw_read_dp(SW_DP_REG_IDCODE, &x);
	if (ack != SW_ACK_OK)
	{
		return false;
	}

	/*! \todo	properly detect and select a memory ap (use the identification
	 *		register - IDR for that purpose) */
	x = sw_select_reg.select_reg = 0;
	ack = sw_bitseq_xfer(false, false, -1, SW_DP_REG_SELECT, &x);

	if (ack != SW_ACK_OK)
	{
		return false;
	}
	if ((ack = sw_set_transfer_addr_reg(0)) != SW_ACK_OK)
	{
		return false;
	}
	else
		return true;
}



/*!
 *	\fn	static bool sw_switch_to_sw(void)
 *	\brief	performs a jtag to serial wire switching sequence on the sw bus
 *
 *	\note	it is assumed, that on entry to this function, the swdio hardware
 *		signal is configured as an output, and the swclk hardware signal
 *		is also configured as an output - and it is in a high logic
 *		level state; these assertions are also guaranteed to remain
 *		true on exit from this function
 *
 *	a jtag to serial wire switching sequence consists of clocking
 *	at least 50 cycles on the bus with the swdio signal held high,
 *	then clock out the 16 bit (magic) sequence 0xe79e (lsb first),
 *	then again issue at least 50 cycles while having the swdio
 *	signal held high, then issue at least one idle cycle, then
 *	read the dp IDCODE register; this sequence will switch the
 *	hardware from jtag to serial-wire debugging (it will work
 *	even if the serial wire mode is already activated), and
 *	will leave the sw bus in an idle state, ready to accept
 *	packet requests
 *	for details, refer to the
 *	"DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf"
 *	document available for download on the arm site
 *	(chapter 6 - 'multiple protocol interoperability')
 *
 *	\param	none
 *	\return	true, if switching to the serial wire protocol (i.e. the
 *		sw transaction succeeded and the value read
 *		from the IDCODE register matches the expected one),
 *		false if some error occurred */
static bool sw_switch_to_sw(void)
{
int i;
uint32_t x;

	/* perform a swd reset sequence */
	/* (1) first - issue
	 * >= 50 clock cycles while holdind
	 * swdio hi */
	for (i = 0; i < 60; i ++)
		sw_clock_out_1();
	/* (2) second - issue the 16 bit jtag-to-serial-wire
	 * sequence (described in the arm adiv5
	 * supplement document, available on the
	 * arm site after registration, document name is:
	 * DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf)
	 *
	 * this sequence is 0xe79e - transmitted lsb first */
	x = 0xe79e;
	for (i = 0; i < 16; i++, x >>= 1)
		if (x & 1)
			sw_clock_out_1();
		else
			sw_clock_out_0();
	/* after this - perform an usual sw bus reset sequence */
	return sw_reset_bus();
}

/*!
 *	\fn	uint32_t sw_read_dp_idcode(void)
 *	\brief	retrieves the 'idcode' dp register
 *
 *	\param	none
 *	\return	the retrieved 'idcode' register
 *
 *	\note	this function currently does not have mechanisms of reporting errors */
uint32_t sw_read_dp_idcode(void)
{
uint32_t x;
	sw_read_dp(SW_DP_REG_IDCODE, &x);
	return x;
}


/*!
 *	\fn	uint32_t sw_read_ap_dbgbase(void)
 *	\brief	retrieves the 'dbgbase' mem ap register
 *
 *	\param	none
 *	\return	the retrieved 'dbgbase' register
 *
 *	\note	this function currently does not have mechanisms of reporting errors */
uint32_t sw_read_ap_dbgbase(void)
{
uint32_t x;
	sw_read_ap(SW_MEM_AP_REG_BASE, &x);
	return x;
}

/*!
 *	\fn	bool sw_read_mem_ap(uint32_t addr, uint32_t * data)
 *	\brief	reads a data word from a memory ap
 *
 *	\param	addr	the memory address to read from
 *	\param	data	a pointer to where to store the data read
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
bool sw_read_mem_ap(uint32_t addr, uint32_t * data)
{
bool res;
int err_retries;

	if (addr & 3)
		return false;

	err_retries = 4;
	do
	{
		/* load the address in the transfer address mem ap register */
		if (sw_set_transfer_addr_reg(addr) != SW_ACK_OK)
			goto retry;
		/* fetch the data from the data read/write mem ap register */
		res = (sw_read_ap(SW_MEM_AP_REG_DRW, data) == SW_ACK_OK);
		if (!res)
		{
			/* error reading the access port - read and clear
			 * debug port ctrl/stat register error bits */
			uint32_t cs;
retry:
			if (read_dp_ctrl_stat_reg(&cs))
			{
				if (cs & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
				{
					sw_reset_bus();
					write_dp_abort_reg(0x1e);
				}
			}
		}
		else
		{
			if (is_tar_reg_reload_needed())
				res = (sw_set_transfer_addr_reg(last_known_tar) == SW_ACK_OK);
			break;
		}
	}
	while (++ err_retries < 4);
	return res;
}


/*!
 *	\fn	bool sw_read_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt)
 *	\brief	reads data words from a memory ap
 *
 *	\param	addr	the memory address to read from
 *	\param	data	a pointer to where to store the data read
 *	\param	wordcnt	the number of words to read
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
bool sw_read_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt)
{
bool res;
enum SW_ACK_ENUM ack;

	/* check address alignment */
	if (addr & 3)
		return false;

	res = true;

reload_tar_register:

	/* load the address in the transfer address mem ap register */
	DBGMSG("reloading tar for read of address "); dprintint(addr); usbprint("\n");
	if (sw_set_transfer_addr_reg(addr) != SW_ACK_OK)
		return false;

	if (wordcnt)
	{
		res = (sw_xfer_read_ap_word(data) == SW_ACK_OK);
		if (!res)
		{
			/* issue a couple of idle cycles to make sure the sw transfers
			 * have completed */
			sw_insert_idle_cycles(10);
			return false;
		}
		if (is_tar_reg_reload_needed())
		{
restart_target_read:
			DBGMSG("tar reload needed last known tar "); dprintint(last_known_tar); usbprint(" addr "); dprintint(addr); usbprint("\n");
			do
				ack = sw_read_dp(SW_DP_REG_RDBUFF, data);
			while (ack != SW_ACK_OK && ack == SW_ACK_WAIT);
			if (ack != SW_ACK_OK)
				return false;
			data ++;
			if (wordcnt)
				wordcnt --;
			addr = last_known_tar;
			goto reload_tar_register;
		}
		while (wordcnt)
		{
			do
				res = ((ack = sw_xfer_read_ap_word(data)) == SW_ACK_OK);
			while (ack != SW_ACK_OK && ack == SW_ACK_WAIT);

			if (!res)
				break;
			data ++;
			wordcnt --;
			if (is_tar_reg_reload_needed())
				goto restart_target_read;
		}

		/* issue a couple of idle cycles to make sure the sw transfers
		 * have completed */
		sw_insert_idle_cycles(10);

		if (res)
			while (1)
			{
				res = ((ack = sw_read_dp(SW_DP_REG_RDBUFF, data)) == SW_ACK_OK);
				if (ack == SW_ACK_OK || ack != SW_ACK_WAIT)
					break;
			}
	}

	DBGMSG("leaving last known tar "); dprintint(last_known_tar); usbprint("\n");
	return res;
}


/*!
 *	\fn	bool sw_write_mem_ap(uint32_t addr, uint32_t data)
 *	\brief	writes a data word to a memory ap
 *
 *	\param	addr	the memory address to write to
 *	\param	data	the data word to write
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
bool sw_write_mem_ap(uint32_t addr, uint32_t data)
{
bool res;
int err_retries;

	if (addr & 3)
		return false;

	err_retries = 4;
	do
	{
		/* load the address in the transfer address mem ap register */
		if (sw_set_transfer_addr_reg(addr) != SW_ACK_OK)
			goto retry;
		/* write the data to the data read/write mem ap register */
		res = (sw_write_ap(SW_MEM_AP_REG_DRW, data) == SW_ACK_OK);
		if (!res)
		{
			/* error reading the access port - read and clear
			 * debug port ctrl/stat register error bits */
			uint32_t cs;
retry:			
			if (read_dp_ctrl_stat_reg(&cs))
			{
				if (cs & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
				{
					sw_reset_bus();
					write_dp_abort_reg(0x1e);
				}
			}
		}
		else
		{
			if (is_tar_reg_reload_needed())
				res = (sw_set_transfer_addr_reg(last_known_tar) == SW_ACK_OK);
			break;
		}
	}
	while (++ err_retries < 4);
	return res;
}


/*!
 *	\fn	bool sw_write_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt)
 *	\brief	writes a data word to a memory ap
 *
 *	\param	addr	the memory address to write to
 *	\param	data	the data word to write
 *	\return	true, if the serial wire (sw) read transaction succeded,
 *		false, if an error occurred */
bool sw_write_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt)
{
enum SW_ACK_ENUM ack;
bool res;

	if (addr & 3)
		return false;

	res = true;

reload_tar_register:

	/* load the address in the transfer address mem ap register */
	if (sw_set_transfer_addr_reg(addr) != SW_ACK_OK)
		return false;

	if (wordcnt)
	{
		res = (sw_xfer_write_ap_word(* data) == SW_ACK_OK);
		if (!res)
		{
			/* issue a couple of idle cycles to make sure the sw transfers
			 * have completed */
			sw_insert_idle_cycles(10);
			return false;
		}

		if (is_tar_reg_reload_needed())
		{
restart_target_write:
			/* for details, consult the arm document:
			 * IHI0031A_ARM_debug_interface_v5.pdf
			 * section 5.4 - 'protocol description'
			 * (and more specifically section 5.4.7 -
			 * 'sw-dp write buffering') */
			/* wait for the write buffer to get emptied - perform an access
			 * that the debug port is able to stall - writing the SELECT 
			 * register is one such access */
			/*! \todo	add timeout/error retry counter */
			do
				res = ((ack = sw_write_dp(SW_DP_REG_SELECT, sw_select_reg.select_reg)) == SW_ACK_OK);
			while (ack != SW_ACK_OK && ack == SW_ACK_WAIT);

			if (ack != SW_ACK_OK)
				return false;
			data ++;
			wordcnt --;
			addr = last_known_tar;
			goto reload_tar_register;
		}

		data ++;
		wordcnt --;
		while (wordcnt)
		{
			do
				res = ((ack = sw_xfer_write_ap_word(* data)) == SW_ACK_OK);
			while (ack != SW_ACK_OK && ack == SW_ACK_WAIT);

			if (!res)
				break;

			if (is_tar_reg_reload_needed())
				goto restart_target_write;
			data ++;
			wordcnt --;
		}

		/* issue a couple of idle cycles to make sure the sw transfers
		 * have completed */
		sw_insert_idle_cycles(10);

		if (res)
		{
			/* for details, consult the arm document:
			 * IHI0031A_ARM_debug_interface_v5.pdf
			 * section 5.4 - 'protocol description'
			 * (and more specifically section 5.4.7 -
			 * 'sw-dp write buffering') */
			/* wait for the write buffer to get emptied - perform an access
			 * that the debug port is able to stall - writing the SELECT 
			 * register is one such access */
			/*! \todo	add timeout/error retry counter */
			do
				res = ((ack = sw_write_dp(SW_DP_REG_SELECT, sw_select_reg.select_reg)) == SW_ACK_OK);
			while (ack != SW_ACK_OK && ack == SW_ACK_WAIT);
		}
	}

	return res;
}


bool init_sw_hardware(void)
{
uint32_t x;
bool res;

	res = true;
	/* configure pins */
	sw_config_swdio_output();
	sw_config_swclk_output();
	swdio_hi();
	swclk_hi();
	/* configure the target reset signal */
	swdelay();
	res &= sw_switch_to_sw();

	if (0) while (sw_debug)
	{
		if (switch_to_sw_flag)
		{
			int i, x;
			for (i = 0; i < 60; i ++)
				sw_clock_out_1();
			/* (2) second - issue the 16 bit jtag-to-serial-wire
			 * sequence (described in the arm adiv5
			 * supplement document, available on the
			 * arm site after registration, document name is:
			 * DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf)
			 *
			 * this sequence is 0xe79e - transmitted lsb first */
			x = 0xe79e;
			for (i = 0; i < 16; i++, x >>= 1)
				if (x & 1)
					sw_clock_out_1();
				else
					sw_clock_out_0();
		}
		if (reset_swd_bus_flag)
			sw_reset_bus();
		if (read_dp_idcode)
			sw_read_dp_idcode();
		if (sw_debug == 3)
		{
			read_dp(0, & x);
			read_ap(0, & x);
			write_dp(0, 0);
			write_ap(0, 0);
		}
	}

	x = -1;
	if (!read_dp_ctrl_stat_reg(&x))
	{
		usbprint("failed to read ctrl/stat register, aborting...\n");
		return false;
	}
	DBGMSG("ctrl/stat after connecting: ");
	dprintint(x);

	if (x & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
	{
		usbprint("ctrl/stat errors detected - trying to clear errors\n");
		if (!write_dp_abort_reg(0x1e))
		{
			usbprint("failed to write ctrl/stat register, aborting...\n");
			return false;
		}
		/* reread ctrl/stat and see if errors have been cleared */
		if (!read_dp_ctrl_stat_reg(&x))
		{
			usbprint("failed to read ctrl/stat register, aborting...\n");
			return false;
		}
		if (x & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
		{
			usbprint("failed to clear ctrl/stat errors, aborting...\n");
			return false;
		}
		usbprint("; dp ctrl/stat register after clearing errors is ");
		dprintint(x);
		usbprint("\n");
	}

	/* power up system and debug blocks, reset debug block */
	res &= sw_write_dp(SW_DP_REG_CTRLSTAT, 0x54000000);
	for (x = 0; x < 1024; x ++)
		swdelay();

	res &= sw_write_dp(SW_DP_REG_CTRLSTAT, 0x50000000);
	for (x = 0; x < 1024; x ++)
		swdelay();

	DBGMSG("ctrl/stat after powering debug and system domains: ");
	x = -1;
	read_dp_ctrl_stat_reg(&x);
	dprintint(x);

	/* configure the cortex ahb-ap csw register
	 * for details, consult the arm document:
	 * DDI0337I_cortexm3_r2p1_trm.pdf
	 * the ahb-ap csw register is described
	 * in section 7.2.2 of this document */
	res &= sw_write_ap(SW_MEM_AP_REG_CSW, 0x22000052);
	res &= sw_read_ap(SW_MEM_AP_REG_CSW, &x);

	DBGMSG("ctrl/stat at end of connecting: ");
	x = -1;
	read_dp_ctrl_stat_reg(&x);
	dprintint(x);
	sw_read_mem_ap(0, & x);
	
	memset(&counters, 0, sizeof counters);

	return true;
}


enum SW_ACK_ENUM read_dp(int address, uint32_t * data)
{
enum SW_ACK_ENUM ack;
	
	while ((ack = sw_bitseq_xfer(false, true, -1, address, data)) != SW_ACK_OK && ack == SW_ACK_WAIT);
	return ack;
}

enum SW_ACK_ENUM read_ap(int address, uint32_t * data)
{
enum SW_ACK_ENUM ack;
	
	/* post read request */
	while ((ack = sw_bitseq_xfer(true, true, -1, address, data)) != SW_ACK_OK && ack == SW_ACK_WAIT);
	if (ack != SW_ACK_OK)
		return ack;
	/* read back data */
	return read_dp(SW_DP_REG_RDBUFF, data);
}
enum SW_ACK_ENUM write_dp(int address, uint32_t data)
{
enum SW_ACK_ENUM ack;
	
	while ((ack = sw_bitseq_xfer(false, false, -1, address, & data)) != SW_ACK_OK && ack == SW_ACK_WAIT);
	if (ack != SW_ACK_OK)
		return ack;
	/* read the read buffer to make sure the dp write buffer is flushed */
	ack = read_dp(SW_DP_REG_RDBUFF, & data);
	if (ack != SW_ACK_OK)
		return ack;
	/* check transaction */
	if ((ack = read_dp(SW_DP_REG_CTRLSTAT, & data)) != SW_ACK_OK)
		return ack;
	if (data & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
	{
		/* write to the abort register to clear any errors */
		sw_bitseq_xfer(false, false, -1, SW_DP_REG_AP_ABORT, (uint32_t [1]) { [0] = 0x1f, });
		return SW_ACK_PROTOCOL_ERROR;
	}
	return ack;
}

enum SW_ACK_ENUM write_ap(int address, uint32_t data)
{
volatile enum SW_ACK_ENUM ack;
	
	counters.write_ap_cnt ++;
	while ((ack = sw_bitseq_xfer(true, false, -1, address, & data)) != SW_ACK_OK && ack == SW_ACK_WAIT)
		counters.wait_cnt ++;
	if (ack != SW_ACK_OK)
	{
		counters.write_ap_err_cnt ++;
		return ack;
	}
	/* read the read buffer to make sure the dp write buffer is flushed */
	ack = read_dp(SW_DP_REG_RDBUFF, & data);
	if (ack != SW_ACK_OK)
	{
		counters.write_ap_err_cnt ++;
		return ack;
	}
	/* check transaction */
	if ((ack = read_dp(SW_DP_REG_CTRLSTAT, & data)) != SW_ACK_OK)
		return ack;
	if (data & ((1 << 7) | (1 << 5) | (1 << 4) | (1 << 1)))
	{
		/* write to the abort register to clear any errors */
		sw_bitseq_xfer(false, false, -1, SW_DP_REG_AP_ABORT, (uint32_t [1]) { [0] = 0x1f, });
		return SW_ACK_PROTOCOL_ERROR;
	}
	return ack;
}

