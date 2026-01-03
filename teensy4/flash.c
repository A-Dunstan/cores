/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "imxrt.h"
#include <avr/pgmspace.h>
#include <string.h>

#define LUT0(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define LUT1(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)
#define CMD_SDR         FLEXSPI_LUT_OPCODE_CMD_SDR
#define ADDR_SDR        FLEXSPI_LUT_OPCODE_RADDR_SDR
#define READ_SDR        FLEXSPI_LUT_OPCODE_READ_SDR
#define WRITE_SDR       FLEXSPI_LUT_OPCODE_WRITE_SDR
#define MODE8_SDR       FLEXSPI_LUT_OPCODE_MODE8_SDR
#define DUMMY_SDR       FLEXSPI_LUT_OPCODE_DUMMY_SDR
#define JMP_ON_CS       FLEXSPI_LUT_OPCODE_JMP_ON_CS
#define PINS1           FLEXSPI_LUT_NUM_PADS_1
#define PINS4           FLEXSPI_LUT_NUM_PADS_4

FASTRUN void configure_flexspi() {
	// unlock the LUT
	FLEXSPI_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI_LUTCR = FLEXSPI_LUTCR_UNLOCK;
	asm volatile("dmb");

	// LUT 0: ReadContinuous
	FLEXSPI_LUT0 = LUT0(CMD_SDR, PINS1, 0xEB) | LUT1(ADDR_SDR, PINS4, 24);
	FLEXSPI_LUT1 = LUT0(MODE8_SDR, PINS4, 0x20) | LUT1(DUMMY_SDR, PINS4, 4);
	FLEXSPI_LUT2 = LUT0(READ_SDR, PINS4, 4) | LUT1(JMP_ON_CS, 0, 1);
	FLEXSPI_LUT3 = 0;
	// LUT 1: Read Status Register 1
	FLEXSPI_LUT4 = LUT0(CMD_SDR, PINS1, 0x05) | LUT1(READ_SDR, PINS1, 1);
	FLEXSPI_LUT5 = \
	FLEXSPI_LUT6 = \
	FLEXSPI_LUT7 = 0;
	// LUT 2: Read Status Register 2
	FLEXSPI_LUT8 = LUT0(CMD_SDR, PINS1, 0x35) | LUT1(READ_SDR, PINS1, 1);
	FLEXSPI_LUT9 = \
	FLEXSPI_LUT10 = \
	FLEXSPI_LUT11 = 0;
	// LUT 3: Disable continuous read
	FLEXSPI_LUT12 = LUT0(CMD_SDR, PINS1, 0xFF);
	FLEXSPI_LUT13 = \
	FLEXSPI_LUT14 = \
	FLEXSPI_LUT15 = 0;
	// LUT 4: WriteEnable
	FLEXSPI_LUT16 = LUT0(CMD_SDR, PINS1, 0x06);
	FLEXSPI_LUT17 = \
	FLEXSPI_LUT18 = \
	FLEXSPI_LUT19 = 0;
	// LUT 5: EraseSector (4K)
	FLEXSPI_LUT20 = LUT0(CMD_SDR, PINS1, 0x20) | LUT1(ADDR_SDR, PINS1, 24);
	FLEXSPI_LUT21 = \
	FLEXSPI_LUT22 = \
	FLEXSPI_LUT23 = 0;
	// LUT 6: Erase 32K
	FLEXSPI_LUT24 = LUT0(CMD_SDR, PINS1, 0x52) | LUT1(ADDR_SDR, PINS1, 24);
	FLEXSPI_LUT25 = \
	FLEXSPI_LUT26 = \
	FLEXSPI_LUT27 = 0;
	// LUT 8: Erase 64K
	FLEXSPI_LUT32 = LUT0(CMD_SDR, PINS1, 0xD8) | LUT1(ADDR_SDR, PINS1, 24);
	FLEXSPI_LUT33 = \
	FLEXSPI_LUT34 = \
	FLEXSPI_LUT35 = 0;
	// LUT 9: PageProgram
	FLEXSPI_LUT36 = LUT0(CMD_SDR, PINS1, 0x32) | LUT1(ADDR_SDR, PINS1, 24);
	FLEXSPI_LUT37 = LUT0(WRITE_SDR, PINS4, 1);
	FLEXSPI_LUT38 = \
	FLEXSPI_LUT39 = 0;
	// LUT 12: Erase/Program Suspend
	FLEXSPI_LUT48 = LUT0(CMD_SDR, PINS1, 0x75);
	FLEXSPI_LUT49 = \
	FLEXSPI_LUT50 = \
	FLEXSPI_LUT51 = 0;
	// LUT 13: Read (non-continuous)
	FLEXSPI_LUT52 = LUT0(CMD_SDR, PINS1, 0xEB) | LUT1(ADDR_SDR, PINS4, 24);
	FLEXSPI_LUT53 = LUT0(MODE8_SDR, PINS4, 0xFF) | LUT1(DUMMY_SDR, PINS4, 4);
	FLEXSPI_LUT54 = LUT0(READ_SDR, PINS4, 4);
	FLEXSPI_LUT55 = 0;
	// LUT 14: Erase/Program Resume
	FLEXSPI_LUT56 = LUT0(CMD_SDR, PINS1, 0x7A);
	FLEXSPI_LUT57 = \
	FLEXSPI_LUT58 = \
	FLEXSPI_LUT59 = 0;

	// set AHB read command to ReadContinuous
	FLEXSPI_FLSHA1CR2 = FLEXSPI_FLSHCR2_ARDSEQID(0);

	// re-lock the LUT to prevent changes
	FLEXSPI_LUTKEY = FLEXSPI_LUTKEY_VALUE;
	FLEXSPI_LUTCR = FLEXSPI_LUTCR_LOCK;
	asm volatile("dmb");
}

FASTRUN static void flash_wait()
{
	// tSUS = ~22.5us @ 133MHZ = 3000 cycles
	FLEXSPI_FLSHA1CR1 |= FLEXSPI_FLSHCR1_CSINTERVAL(3000);
	// change AHB read sequence to suspend->read
	FLEXSPI_FLSHA1CR2 = FLEXSPI_FLSHCR2_ARDSEQNUM(1) | FLEXSPI_FLSHCR2_ARDSEQID(12);
	asm volatile("dmb");
	// changing CSINTERVAL won't work without a reset
	FLEXSPI_MCR0 |= FLEXSPI_MCR0_SWRESET;
	while (FLEXSPI_MCR0 & FLEXSPI_MCR0_SWRESET) ; // wait
	__enable_irq();

	FLEXSPI_IPCR0 = 0;
	FLEXSPI_IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF;
	while (1) {
		FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(1);
		FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
		while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE));
		uint8_t status = FLEXSPI_RFDR0;
		FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE|FLEXSPI_INTR_IPRXWA;
		// continue if BUSY is set
		if (status & 1) continue;
		FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(2);
		FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
		while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE));
		status = FLEXSPI_RFDR0;
		FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE|FLEXSPI_INTR_IPRXWA;
		// finish if there's no suspended operation
		if (!(status & 0x80)) break;
		// otherwise manually resume it
		FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(14);
		FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
		while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE));
		FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	}

	__disable_irq();
	// restore old values
	FLEXSPI_FLSHA1CR1 &= ~FLEXSPI_FLSHCR1_CSINTERVAL_MASK;
	FLEXSPI_FLSHA1CR2 = FLEXSPI_FLSHCR2_ARDSEQID(0);

	// purge stale data from FlexSPI's AHB FIFO
	FLEXSPI_MCR0 |= FLEXSPI_MCR0_SWRESET;
	while (FLEXSPI_MCR0 & FLEXSPI_MCR0_SWRESET) ; // wait
	__enable_irq();
}

FASTRUN static void flash_begin()
{
	__disable_irq();
	// disable continuous mode, then enable write commands
	FLEXSPI_IPCR0 = 0;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQNUM(1) | FLEXSPI_IPCR1_ISEQID(3);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
}

// write bytes into flash memory (which is already erased to 0xFF)
FASTRUN void eepromemu_flash_write(void *addr, const void *data, uint32_t len)
{
	flash_begin();
	arm_dcache_delete(addr, len); // purge old data from ARM's cache
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	FLEXSPI_IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF; // clear tx fifo
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FFFFFF;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(9) | FLEXSPI_IPCR1_IDATSZ(len);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	const uint8_t *src = (const uint8_t *)data;
	uint32_t n;
	while (!((n = FLEXSPI_INTR) & FLEXSPI_INTR_IPCMDDONE)) {
		if (len && n & FLEXSPI_INTR_IPTXWE) {
			uint32_t wrlen = len;
			if (wrlen > 8) wrlen = 8;
			if (wrlen > 0) {
				memcpy((void *)&FLEXSPI_TFDR0, src, wrlen);
				src += wrlen;
				len -= wrlen;
			}
			FLEXSPI_INTR = FLEXSPI_INTR_IPTXWE;
		}
	}
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPTXWE;
	flash_wait();
}

// erase a 4K sector
FASTRUN void eepromemu_flash_erase_sector(void *addr)
{
	// don't need flash_begin, all three commands are consecutive LUTs and can be issued as one sequence
	__disable_irq();
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FFF000;
	// execute LUTs 3 + 4 + 5
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQNUM(2) | FLEXSPI_IPCR1_ISEQID(3);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	arm_dcache_delete((void *)((uint32_t)addr & 0xFFFFF000), 4096); // purge data from cache
	flash_wait();
}

FASTRUN void eepromemu_flash_erase_32K_block(void *addr)
{
	flash_begin();
	arm_dcache_delete((void *)((uint32_t)addr & 0xFFFF8000), 32768); // purge data from cache
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FF8000;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(6);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	flash_wait();
}

FASTRUN void eepromemu_flash_erase_64K_block(void *addr)
{
	flash_begin();
	arm_dcache_delete((void *)((uint32_t)addr & 0xFFFF0000), 65536); // purge data from cache
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	FLEXSPI_IPCR0 = (uint32_t)addr & 0x00FF0000;
	FLEXSPI_IPCR1 = FLEXSPI_IPCR1_ISEQID(8);
	FLEXSPI_IPCMD = FLEXSPI_IPCMD_TRG;
	while (!(FLEXSPI_INTR & FLEXSPI_INTR_IPCMDDONE)) ; // wait
	FLEXSPI_INTR = FLEXSPI_INTR_IPCMDDONE;
	flash_wait();
}
