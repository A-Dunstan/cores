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

// To configure the EEPROM size, edit E2END in avr/eeprom.h.
//
// Generally you should avoid editing this code, unless you really
// know what you're doing.

#include "imxrt.h"
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "debug/printf.h"

#if defined(ARDUINO_TEENSY40)
#define FLASH_BASEADDR 0x601F0000
#define FLASH_SECTORS  15
#elif defined(ARDUINO_TEENSY41)
#define FLASH_BASEADDR 0x607C0000
#define FLASH_SECTORS  63
#elif defined(ARDUINO_TEENSY_MICROMOD)
#define FLASH_BASEADDR 0x60FC0000
#define FLASH_SECTORS  63
#endif


#if E2END > (255*FLASH_SECTORS-1)
#error "E2END is set larger than the maximum possible EEPROM size"
#endif

// Conversation about how this code works & what the upper limits are
// https://forum.pjrc.com/threads/57377?p=214566&viewfull=1#post214566

// To be called from LittleFS_Program, any other use at your own risk!
void eepromemu_flash_write(void *addr, const void *data, uint32_t len);
void eepromemu_flash_erase_sector(void *addr);
void eepromemu_flash_erase_32K_block(void *addr);
void eepromemu_flash_erase_64K_block(void *addr);

static uint8_t initialized=0;
static uint16_t sector_index[FLASH_SECTORS];

void eeprom_initialize(void)
{
	uint32_t sector;
	//printf("eeprom init\n");
	for (sector=0; sector < FLASH_SECTORS; sector++) {
		const uint16_t *p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		const uint16_t *end = (uint16_t *)(FLASH_BASEADDR + (sector + 1) * 4096);
		uint16_t index = 0;
		do {
			if (*p++ == 0xFFFF) break;
			index++;
		} while (p < end);
		sector_index[sector] = index;
	}
	initialized = 1;
}

uint8_t eeprom_read_byte(const uint8_t *addr_ptr)
{
	uint32_t addr = (uint32_t)addr_ptr;
	uint32_t sector, offset;
	const uint16_t *p, *end;
	uint8_t data=0xFF;

	if (addr > E2END) return 0xFF;
	if (!initialized) eeprom_initialize();
	sector = (addr >> 2) % FLASH_SECTORS;
	offset = (addr & 3) | (((addr >> 2) / FLASH_SECTORS) << 2);
	//printf("ee_rd, addr=%u, sector=%u, offset=%u, len=%u\n",
		//addr, sector, offset, sector_index[sector]);
	p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
	end = p + sector_index[sector];
	while (p < end) {
		uint32_t val = *p++;
		if ((val & 255) == offset) data = val >> 8;
	}
	return data;
}

void eeprom_write_byte(uint8_t *addr_ptr, uint8_t data)
{
	uint32_t addr = (uint32_t)addr_ptr;
	uint32_t sector, offset, index, i;
	uint16_t *p, *end;
	uint8_t olddata=0xFF;
	uint8_t buf[256];

	if (addr > E2END) return;
	if (!initialized) eeprom_initialize();

	sector = (addr >> 2) % FLASH_SECTORS; 
	offset = (addr & 3) | (((addr >> 2) / FLASH_SECTORS) << 2);
	//printf("ee_wr, addr=%u, sector=%u, offset=%u, len=%u\n",
		//addr, sector, offset, sector_index[sector]);
	p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
	end = p + sector_index[sector];
	while (p < end) {
		uint16_t val = *p++;
		if ((val & 255) == offset) olddata = val >> 8;
	}
	if (data == olddata) return;
	if (sector_index[sector] < 2048) {
		//printf("ee_wr, writing\n");
		uint16_t newdata = offset | (data << 8);
		eepromemu_flash_write(end, &newdata, 2);
		sector_index[sector] = sector_index[sector] + 1;
	} else {
		//printf("ee_wr, erase then write\n");
		memset(buf, 0xFF, sizeof(buf));
		p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		end = p + 2048;
		while (p < end) {
			uint16_t val = *p++;
			buf[val & 255] = val >> 8;
		}
		buf[offset] = data;
		p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		eepromemu_flash_erase_sector(p);
		index = 0;
		for (i=0; i < 256; i++) {
			if (buf[i] != 0xFF) {
				// TODO: combining these to larger write
				// would (probably) be more efficient
				uint16_t newval = i | (buf[i] << 8);
				eepromemu_flash_write(p + index, &newval, 2);
				index = index + 1;
			}
		}
		sector_index[sector] = index;
	}
}

uint16_t eeprom_read_word(const uint16_t *addr)
{
	const uint8_t *p = (const uint8_t *)addr;
	return eeprom_read_byte(p) | (eeprom_read_byte(p+1) << 8);
}

uint32_t eeprom_read_dword(const uint32_t *addr)
{
	const uint8_t *p = (const uint8_t *)addr;
	return eeprom_read_byte(p) | (eeprom_read_byte(p+1) << 8)
		| (eeprom_read_byte(p+2) << 16) | (eeprom_read_byte(p+3) << 24);
}

void eeprom_read_block(void *buf, const void *addr, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)addr;
	uint8_t *dest = (uint8_t *)buf;
	while (len--) {
		*dest++ = eeprom_read_byte(p++);
	}
}

int eeprom_is_ready(void)
{
	return 1;
}

void eeprom_write_word(uint16_t *addr, uint16_t value)
{
	uint8_t *p = (uint8_t *)addr;
	eeprom_write_byte(p++, value);
	eeprom_write_byte(p, value >> 8);
}

void eeprom_write_dword(uint32_t *addr, uint32_t value)
{
	uint8_t *p = (uint8_t *)addr;
	eeprom_write_byte(p++, value);
	eeprom_write_byte(p++, value >> 8);
	eeprom_write_byte(p++, value >> 16);
	eeprom_write_byte(p, value >> 24);
}

void eeprom_write_block(const void *buf, void *addr, uint32_t len)
{
	uint8_t *p = (uint8_t *)addr;
	const uint8_t *src = (const uint8_t *)buf;
	while (len--) {
		eeprom_write_byte(p++, *src++);
	}
}





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
	// don't need flash_begin, all three commands are consecutive and can be issued as one sequence
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
