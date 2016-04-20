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

static void swdelay(void)
{
volatile int i;
	for (i = 0; i < nr_swd_idle_cycles; i ++);
}

static inline void sw_config_swdio_output(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
}

static inline void sw_config_swdio_input(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);
	gpio_set(GPIOA, GPIO15);
	swdelay();
}

static inline void sw_config_swclk_output(void)
{
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

static inline void swdio_hi(void)
{
	gpio_set(GPIOA, GPIO15);
}

static inline void swdio_low(void)
{
	gpio_clear(GPIOA, GPIO15);
}

static inline void swclk_hi(void)
{
	gpio_set(GPIOB, GPIO5);
}

static inline void swclk_low(void)
{
	gpio_clear(GPIOB, GPIO5);
}

static inline void sw_clock_out_0(void)
{
	swdio_low();
	swdelay();
	swclk_low();
	swdelay();
	swclk_hi();
	swdelay();
}

static inline void sw_clock_out_1(void)
{
	swdio_hi();
	swdelay();
	swclk_low();
	swdelay();
	swclk_hi();
	swdelay();
}

static inline bool sw_clock_data_in(void)
{
bool x;
	swclk_low();
	swdelay();
	x = ((gpio_get(GPIOA, GPIO15)) ? true : false);
	swclk_hi();
	swdelay();
	return x;
}


#define SWD_DELAY \
	asm("push	{ r0 }\n"); \
	asm("ldr	r0,	=nr_swd_idle_cycles\n"); \
	asm("ldr	r0,	[r0]\n"); \
	asm("9:\n"); \
	asm("subs	r0,	r0,	#1\n"); \
	asm("bne	9b\n"); \
	asm("pop	{ r0 }\n");

#define LOAD_SWDIO_MASK_TO_R1	asm("mov	r1,	#1\n");
#define LOAD_SWCLK_MASK_TO_R2	asm("mov	r2,	#2\n");
#define TEST_SWDIO_MASK_IN_R1	asm("tst	r1,	#1\n");
#define SWCLK_LOW		asm("str	r2,	[r3, #20]\n");
#define SWCLK_LOW_DELAY		SWCLK_LOW SWD_DELAY
#define SWCLK_HI		asm("str	r2,	[r3, #16]\n");
#define SWCLK_HI_DELAY		SWCLK_HI SWD_DELAY


#define clock_word_and_parity_in_delay clock_word_and_parity_in
#define clock_word_and_parity_out_delay clock_word_and_parity_out
#define clock_header_out_get_ack_delay clock_header_out_get_ack



static uint32_t clock_header_out_get_ack(uint32_t w)
{
int i;
uint32_t ack;

	for (i = 1 << 7; i; i >>= 1, ((w & 1) ? sw_clock_out_1() : sw_clock_out_0()), w >>= 1);
	/* issue a turnaround cycle */
	sw_config_swdio_input();
	swdelay();
	swclk_low();
	swdelay();
	swclk_hi();
	swdelay();
	/* read the acknowledge response */
	ack = (sw_clock_data_in() ? 1 : 0);
	ack |= (sw_clock_data_in() ? 2 : 0);
	return ack | (sw_clock_data_in() ? 4 : 0);
}

static uint64_t clock_word_and_parity_in(void)
{
uint32_t x, i;
	/* clock word in */
	for (x = 0, i = 1; i; x |= (sw_clock_data_in() ? i : 0), i <<= 1);
	/* compute and verify parity */
	i = x ^ (x >> 16);
	i ^= i >> 8;
	i ^= i >> 4;
	i ^= i >> 2;
	i ^= i >> 1;
	i ^= (sw_clock_data_in() ? 1 : 0);
	/* issue a turnaround cycle */
	sw_clock_data_in();
	sw_config_swdio_output();
	sw_clock_out_0();
	return x | ((uint64_t)(i & 1) << 32);
}

static void clock_word_and_parity_out(uint32_t w)
{
uint32_t i;
	/* issue a turnaround cycle */
	sw_clock_data_in();
	sw_config_swdio_output();
	/* clock word out */
	for (i = 1; i; ((w & i) ? sw_clock_out_1 : sw_clock_out_0)(), i <<= 1);
	/* compute and clock out parity */
	w ^= w >> 16;
	w ^= w >> 8;
	w ^= w >> 4;
	w ^= w >> 2;
	w ^= w >> 1;
	((w & 1) ? sw_clock_out_1 : sw_clock_out_0)
		();
}



/*!
 *	\fn	static inline void sw_insert_idle_cycles(int nr_idle_cycles)
 *	\brief	inserts idle cycles on the serial wire
 *
 *	\note	it is assumed, that on entry to this function, the swdio hardware
 *		signal is configured as an output, and the swclk hardware signal
 *		is also configured as an output - and it is in a high logic
 *		level state; these assertions are also guaranteed to remain
 *		true on exit from this function
 *
 *	idle cycles are cycles during which the swdio line is kept
 *	in a logic low state; idle cycles are needed at various
 *	times in the serial wire debug protocol; for details, refer to the
 *	"DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement.pdf"
 *	document available for download on the arm site; amongst
 *	other things, this document states:

8.2 Clocking
The Serial Wire interface clock can be asynchronous to any system clock, including the debug logic clock. The Serial Wire interface clock can be stopped when the debug port is idle.
The host must continue to clock the interface for a number of cycles after the data phase of transactions. This ensures that the transaction can be clocked through the Serial Wire interface. This means that after the data phase of transactions the host must do one of the following:
 immediately start a new transaction
 continue to clock the Serial Wire interface until the host starts a new transaction, inserting idle cycles
 after clocking out the data parity bit, continue to clock the Serial Wire interface inserting idle cycles until it has clocked out at least 8 more clock rising edges, before stopping the clock.
 
8.3.4 Idle cycles
Following transactions, the host must either insert idle cycles or continue immediately with the start bit of a new transaction. The host clocks the Serial Wire interface with the line LOW to insert idle cycles.

 *
 *	\param	nr_idle_cycles	the number of idle cycles to
 *				clock on the sw bus
 *	\return	none */

static void sw_insert_idle_cycles(int nr_idle_cycles)
{
	swdio_low();
	while (nr_idle_cycles-- > 0)
		swclk_low(), swclk_hi();
}

#if 0

static void clock_word_and_parity_out(uint32_t w)
{
	asm("push	{ r4, lr }\n");
	LOAD_SWDIO_MASK_TO_R1
	LOAD_SWCLK_MASK_TO_R2
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");
	/* issue a turnaround cycle */
	SWCLK_LOW
	SWCLK_HI

	/* configure swdio as output */
	asm("ldr	r4,	[r3, #0]\n");
	asm("and	r4,	#0xfffffff0\n");
	asm("orr	r4,	#3\n");
	asm("str	r4,	[r3, #0]\n");


	asm("mov	r4,	#1\n");

	asm("1:\n");
	SWCLK_LOW
	asm("tst	r0,	r4\n");
	asm("ite	ne\n");
	asm("strne	r1,	[r3, #16]\n");
	asm("streq	r1,	[r3, #20]\n");
	SWCLK_HI
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");

	/* compute parity in r0 */
	asm("eor	r0, r0, r0, lsr #16\n");
	asm("eor	r0, r0, r0, lsr #8\n");
	asm("eor	r0, r0, r0, lsr #4\n");
	asm("eor	r0, r0, r0, lsr #2\n");
	asm("eor	r0, r0, r0, lsr #1\n");

	/* shift parity out */
	SWCLK_LOW
	asm("tst	r0,	#1\n");
	asm("ite	ne\n");
	asm("strne	r1,	[r3, #16]\n");
	asm("streq	r1,	[r3, #20]\n");
	SWCLK_HI

	asm("pop	{ r4, pc }\n");

}


static uint32_t clock_header_out_get_ack(uint32_t w)
{
	asm("push	{ r4, lr }\n");
	/* load counter */
	asm("mov	r4,	#(1 << (32 - 8))\n");
	LOAD_SWDIO_MASK_TO_R1
	LOAD_SWCLK_MASK_TO_R2
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");

	asm("1:\n");
	SWCLK_LOW
	asm("lsrs	r0,	#1\n");
	asm("ite	cs\n");
	asm("strcs	r1,	[r3, #16]\n");
	asm("strcc	r1,	[r3, #20]\n");
	SWCLK_HI
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");

	/* configure swdio as input */
	asm("ldr	r0,	[r3, #0]\n");
	asm("and	r0,	#0xfffffff0\n");
	asm("orr	r0,	#8\n");
	asm("str	r0,	[r3, #0]\n");
	//asm("str	r1,	[r3, #16]\n");

	/* issue a turnaround cycle */
	SWCLK_LOW
	asm("mov	r0,	#0\n");
	SWCLK_HI

	/* read the 3-bit ack value */
	SWCLK_LOW
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #1\n");
	SWCLK_HI

	SWCLK_LOW
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #2\n");
	SWCLK_HI


	SWCLK_LOW
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #4\n");
	SWCLK_HI

	asm("pop	{ r4, pc }\n");
}


static uint64_t clock_word_and_parity_in(void)
{
	asm("push	{ r4, lr }\n");
	asm("mov	r0,	#0\n");
	LOAD_SWCLK_MASK_TO_R2
	asm("mov	r4,	#1\n");
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");

	asm("1:\n");
	SWCLK_LOW
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, r4\n");
	SWCLK_HI
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");
	/* compute parity in r1 */
	asm("eor	r1, r0, r0, lsr #16\n");
	asm("eor	r1, r1, r1, lsr #8\n");
	asm("eor	r1, r1, r1, lsr #4\n");
	asm("eor	r1, r1, r1, lsr #2\n");
	asm("eor	r1, r1, r1, lsr #1\n");
	asm("and	r1, r1, #1\n");

	/* read parity bit */

	SWCLK_LOW
	asm("ldr	r4,	[r3, #8]\n");
	asm("tst	r4,	#1\n");
	asm("it		ne\n");
	asm("eorne	r1, r1, #1\n");
	SWCLK_HI

	/* arm document 'ADIv5.1 supplement':
	DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement
	 * section 8.2, mandates:

	8.2 Clocking
	The Serial Wire interface clock can be asynchronous to any system clock, including the debug logic clock. The Serial Wire interface clock can be stopped when the debug port is idle.
	The host must continue to clock the interface for a number of cycles after the data phase of transactions. This ensures that the transaction can be clocked through the Serial Wire interface. This means that after the data phase of transactions the host must do one of the following:
	 immediately start a new transaction
	 continue to clock the Serial Wire interface until the host starts a new transaction, inserting idle cycles
	 after clocking out the data parity bit, continue to clock the Serial Wire interface inserting idle cycles until it has clocked out at least 8 more clock rising edges, before stopping the clock.
	See also Idle cycles on page 43.

	8.3.3 Line turn-round
	To avoid contention, a turnaround period is required when the device driving the wire changes.

	8.3.4 Idle cycles
	Following transactions, the host must either insert idle cycles or continue immediately with the start bit of a new transaction. The host clocks the Serial Wire interface with the line LOW to insert idle cycles.

	* so, insert a turnaround cycle */
	SWCLK_LOW
	SWCLK_HI

	/* configure swdio as output */
	asm("ldr	r2,	[r3, #0]\n");
	asm("and	r2,	#0xfffffff0\n");
	asm("orr	r2,	#3\n");
	asm("str	r2,	[r3, #0]\n");

	asm("pop	{ r4, pc }\n");
}


/* versions of the routines above - in case there is a non-zero swd communication delay requested */


static void clock_word_and_parity_out_delay(uint32_t w)
{
	asm("push	{ r4, lr }\n");
	LOAD_SWDIO_MASK_TO_R1
	LOAD_SWCLK_MASK_TO_R2
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");
	/* issue a turnaround cycle */
	SWCLK_LOW_DELAY
	SWCLK_HI_DELAY

	/* configure swdio as output */
	asm("ldr	r4,	[r3, #0]\n");
	asm("and	r4,	#0xfffffff0\n");
	asm("orr	r4,	#3\n");
	asm("str	r4,	[r3, #0]\n");


	asm("mov	r4,	#1\n");

	asm("1:\n");
	SWCLK_LOW_DELAY
	asm("tst	r0,	r4\n");
	asm("ite	ne\n");
	asm("strne	r1,	[r3, #16]\n");
	asm("streq	r1,	[r3, #20]\n");
	SWCLK_HI_DELAY
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");

	/* compute parity in r0 */
	asm("eor	r0, r0, r0, lsr #16\n");
	asm("eor	r0, r0, r0, lsr #8\n");
	asm("eor	r0, r0, r0, lsr #4\n");
	asm("eor	r0, r0, r0, lsr #2\n");
	asm("eor	r0, r0, r0, lsr #1\n");

	/* shift parity out */
	SWCLK_LOW_DELAY
	asm("tst	r0,	#1\n");
	asm("ite	ne\n");
	asm("strne	r1,	[r3, #16]\n");
	asm("streq	r1,	[r3, #20]\n");
	SWCLK_HI_DELAY

	asm("pop	{ r4, pc }\n");

}


static uint32_t clock_header_out_get_ack_delay(uint32_t w)
{
	asm("push	{ r4, lr }\n");
	/* load counter */
	asm("mov	r4,	#(1 << (32 - 8))\n");
	LOAD_SWDIO_MASK_TO_R1
	LOAD_SWCLK_MASK_TO_R2
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");

	asm("1:\n");
	SWCLK_LOW_DELAY
	asm("lsrs	r0,	#1\n");
	asm("ite	cs\n");
	asm("strcs	r1,	[r3, #16]\n");
	asm("strcc	r1,	[r3, #20]\n");
	SWCLK_HI_DELAY
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");

	/* configure swdio as input */
	asm("ldr	r0,	[r3, #0]\n");
	asm("and	r0,	#0xfffffff0\n");
	asm("orr	r0,	#8\n");
	asm("str	r0,	[r3, #0]\n");
	//asm("str	r1,	[r3, #16]\n");

	/* issue a turnaround cycle */
	SWCLK_LOW_DELAY
	asm("mov	r0,	#0\n");
	SWCLK_HI_DELAY

	/* read the 3-bit ack value */
	SWCLK_LOW_DELAY
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #1\n");
	SWCLK_HI_DELAY

	SWCLK_LOW_DELAY
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #2\n");
	SWCLK_HI_DELAY


	SWCLK_LOW_DELAY
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, #4\n");
	SWCLK_HI_DELAY

	asm("pop	{ r4, pc }\n");
}


static uint64_t clock_word_and_parity_in_delay(void)
{
	asm("push	{ r4, lr }\n");
	asm("mov	r0,	#0\n");
	LOAD_SWCLK_MASK_TO_R2
	asm("mov	r4,	#1\n");
	/* load gpiob-base address */
	asm("ldr	r3,	=0x40010c00\n");

	asm("1:\n");
	SWCLK_LOW_DELAY
	asm("ldr	r1,	[r3, #8]\n");
	TEST_SWDIO_MASK_IN_R1
	asm("it		ne\n");
	asm("orrne	r0, r0, r4\n");
	SWCLK_HI_DELAY
	asm("lsls	r4,	r4,	#1\n");
	asm("bne	1b\n");
	/* compute parity in r1 */
	asm("eor	r1, r0, r0, lsr #16\n");
	asm("eor	r1, r1, r1, lsr #8\n");
	asm("eor	r1, r1, r1, lsr #4\n");
	asm("eor	r1, r1, r1, lsr #2\n");
	asm("eor	r1, r1, r1, lsr #1\n");
	asm("and	r1, r1, #1\n");

	/* read parity bit */

	SWCLK_LOW_DELAY
	asm("ldr	r4,	[r3, #8]\n");
	asm("tst	r4,	#1\n");
	asm("it		ne\n");
	asm("eorne	r1, r1, #1\n");
	SWCLK_HI_DELAY

	/* arm document 'ADIv5.1 supplement':
	DSA09-PRDC-008772-1-0_ARM_debug_interface_v5_supplement
	 * section 8.2, mandates:

	8.2 Clocking
	The Serial Wire interface clock can be asynchronous to any system clock, including the debug logic clock. The Serial Wire interface clock can be stopped when the debug port is idle.
	The host must continue to clock the interface for a number of cycles after the data phase of transactions. This ensures that the transaction can be clocked through the Serial Wire interface. This means that after the data phase of transactions the host must do one of the following:
	 immediately start a new transaction
	 continue to clock the Serial Wire interface until the host starts a new transaction, inserting idle cycles
	 after clocking out the data parity bit, continue to clock the Serial Wire interface inserting idle cycles until it has clocked out at least 8 more clock rising edges, before stopping the clock.
	See also Idle cycles on page 43.

	8.3.3 Line turn-round
	To avoid contention, a turnaround period is required when the device driving the wire changes.

	8.3.4 Idle cycles
	Following transactions, the host must either insert idle cycles or continue immediately with the start bit of a new transaction. The host clocks the Serial Wire interface with the line LOW to insert idle cycles.

	* so, insert a turnaround cycle */
	SWCLK_LOW_DELAY
	SWCLK_HI_DELAY

	/* configure swdio as output */
	asm("ldr	r2,	[r3, #0]\n");
	asm("and	r2,	#0xfffffff0\n");
	asm("orr	r2,	#3\n");
	asm("str	r2,	[r3, #0]\n");

	asm("pop	{ r4, pc }\n");
}

#endif
