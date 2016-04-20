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

#include <stdint.h>
#include <stdbool.h>

/*! an enumeration of the acknowledge responses in the serial wire protocol acknowledge phase */
enum SW_ACK_ENUM
{
	/*! an ok response */
	SW_ACK_OK	= 1,
	/*! a wait response */
	SW_ACK_WAIT	= 2,
	/*! a fault response */
	SW_ACK_FAULT	= 4,
	SW_ACK_PROTOCOL_ERROR	= 7,
};

bool init_sw_hardware(void);
uint32_t sw_read_dp_idcode(void);
uint32_t sw_read_ap_dbgbase(void);
bool sw_read_mem_ap(uint32_t addr, uint32_t * data);
bool sw_read_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt);
bool sw_write_mem_ap(uint32_t addr, uint32_t data);
bool sw_write_mem_ap_words(uint32_t addr, uint32_t * data, uint32_t wordcnt);
enum SW_ACK_ENUM read_dp(int address, uint32_t * data);
enum SW_ACK_ENUM read_ap(int address, uint32_t * data);
enum SW_ACK_ENUM write_dp(int address, uint32_t data);
enum SW_ACK_ENUM write_ap(int address, uint32_t data);

/* number of serial wire idle cycles to perform when communicating over
 * the serial wire debug bus; basically, this determines the rate of
 * the serial wire clock */
extern uint32_t nr_swd_idle_cycles;

