/*
 * Acorn BBC Micro
 *
 * Copyright (C) 2013 Michael Brown <mbrown@fensystems.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_BBC_H
#define HW_BBC_H

#include "mc6845.h"
#include "mc6850.h"
#include "m6522.h"
#include "wd1770.h"

#define BBC_B_RAM_BASE 0x0000
#define BBC_B_RAM_SIZE 0x8000

#define BBC_PAGED_ROM_COUNT 16
#define BBC_PAGED_ROM_BASE 0x8000
#define BBC_PAGED_ROM_SIZE 0x4000
#define BBC_PAGED_ROM_VIRTUAL_BASE 0x40000

#define BBC_B_MOS_FILENAME "bbc/OS12.ROM"
#define BBC_B_MOS_BASE 0xc000
#define BBC_B_MOS_SIZE 0x4000

#define BBC_B_BASIC_FILENAME "bbc/BASIC2.ROM"

/* FRED */
#define BBC_FRED_BASE 0xfc00
#define BBC_FRED_SIZE 0x0100

/* JIM */
#define BBC_JIM_BASE 0xfd00
#define BBC_JIM_SIZE 0x0100

/* SHEILA */
#define BBC_SHEILA_BASE 0xfe00
#define BBC_SHEILA_SIZE 0x0100
#define BBC_SHEILA_CRTC_BASE 0x00
#define BBC_SHEILA_CRTC_SIZE 0x08
#define BBC_SHEILA_ACIA_BASE 0x08
#define BBC_SHEILA_ACIA_SIZE 0x08
#define BBC_SHEILA_VIDEO_ULA_BASE 0x20
#define BBC_SHEILA_VIDEO_ULA_SIZE 0x10
#define BBC_SHEILA_PAGED_ROM_SELECT_BASE 0x30
#define BBC_SHEILA_PAGED_ROM_SELECT_SIZE 0x10
#define BBC_SHEILA_SYSTEM_VIA_BASE 0x40
#define BBC_SHEILA_SYSTEM_VIA_SIZE 0x10
#define BBC_SHEILA_USER_VIA_BASE 0x60
#define BBC_SHEILA_USER_VIA_SIZE 0x10
#define BBC_SHEILA_FDC_BASE 0x80
#define BBC_SHEILA_FDC_SIZE 0x20
#define BBC_SHEILA_ADC_BASE 0xc0
#define BBC_SHEILA_ADC_SIZE 0x20

/* Video ULA */
#define BBC_VIDEO_ULA_SIZE 0x02
#define BBC_VIDEO_ULA_CONTROL 0x00
#define BBC_VIDEO_ULA_PALETTE 0x01

/* Addressable latch */
#define BBC_LATCH_SOUND_WE 0
#define BBC_LATCH_SPEECH_RS 1
#define BBC_LATCH_SPEECH_WS 2
#define BBC_LATCH_KB_WE 3
#define BBC_LATCH_C0 4
#define BBC_LATCH_C1 5
#define BBC_LATCH_CAPS_LOCK 6
#define BBC_LATCH_SHIFT_LOCK 7

/* Floppy disc controller */
#define BBC_1770_FDC_CONTROL_BASE 0x04
#define BBC_1770_FDC_CONTROL_SIZE 0x04
#define BBC_1770_FDC_BASE 0x04
#define BBC_1770_FDC_SIZE 0x04

/* Floppy disc control register */
#define BBC_1770_FDC_DRIVE_MASK 0x03
#define BBC_1770_FDC_DRIVE_0 0x01
#define BBC_1770_FDC_DRIVE_1 0x02
#define BBC_1770_FDC_SIDE_1 0x04
#define BBC_1770_FDC_SINGLE_DENSITY 0x08
#define BBC_1770_FDC_NOT_MASTER_RESET 0x20

/* Analogue to digital converter */
#define BBC_ADC_SIZE 0x04
#define BBC_ADC_START 0x00
#define BBC_ADC_STATUS 0x00
#define BBC_ADC_DATA_HIGH 0x01
#define BBC_ADC_DATA_LOW 0x02

/* Analogue to digital converter status register */
#define BBC_ADC_NOT_COMPLETE 0x80

/** Number of columns in keyboard grid */
#define BBC_KEYBOARD_COLUMNS 16

/** Rows capable of generating a keyboard interrupt */
#define BBC_KEYBOARD_IRQ_ROW_MASK 0xfe

/** System clock runs at 1MHz */
#define BBC_1MHZ_TICK_NS 1000

typedef struct BBCVideoULAUpdateEntry BBCVideoULAUpdateEntry;

/**
 * Video ULA
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** List of update notification functions */
	QTAILQ_HEAD ( , BBCVideoULAUpdateEntry ) updates;

	/** Cursor segment mask (4 bits) */
	uint8_t cursor_mask;
	/** 6845 CRTC clock (in log2(MHz)) */
	uint8_t crtc_clock_log2;
	/** Pixel clock (in log2(MHz)) */
	uint8_t pixel_clock_log2;
	/** Teletext enabled */
	uint8_t teletext;
	/** Invert flashing colours */
	uint8_t flash;
	/** Palette */
	uint8_t palette[16];
} BBCVideoULA;

/** A Video ULA update notification function */
typedef void ( * BBCVideoULAUpdated ) ( void *opaque );

/** A Video ULA update notification list entry */
struct BBCVideoULAUpdateEntry {
	/** Update notification function */
	BBCVideoULAUpdated updated;
	/** Opaque pointer */
	void *opaque;
	/** Next entry */
	QTAILQ_ENTRY ( BBCVideoULAUpdateEntry ) next;
};

/**
 * Paged ROM
 */
typedef struct {
	/** Name */
	const char *name;
	/** Paged ROM memory region */
	MemoryRegion rom;
	/** Paged ROM bank memory region */
	MemoryRegion roms;
	/** Paged ROM select register memory region */
	MemoryRegion select;
	/** Base target physical address */
	hwaddr targphys;
	/** Paged ROM size */
	hwaddr size;
	/** Number of paged ROMs */
	unsigned int count;

	/** Paged ROM select register */
	uint8_t page;
} BBCPagedROM;

/**
 * System VIA
 */
typedef struct {
	/** Name */
	const char *name;
	/** 6522 VIA */
	M6522VIA *via;
	/** Stored keycode prefix */
	int keycode_prefix;
	/** Keys currently pressed
	 *
	 * Each byte represents one column of the keyboard grid.  We
	 * include columns 10-15 (which don't physically exist) since
	 * the MOS does attempt to read from these columns.
	 */
	uint8_t keys_pressed[BBC_KEYBOARD_COLUMNS];

	/** Slow data bus content */
	uint8_t slow_data;
	/** Addressable latch */
	uint8_t addressable_latch;
} BBCSystemVIA;

/**
 * User VIA
 */
typedef struct {
	/** Name */
	const char *name;
	/** 6522 VIA */
	M6522VIA *via;
} BBCUserVIA;

/**
 * BBC 1770 floppy disc controller
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** WD1770 FDC */
	WD1770FDC *fdc;

	/** Control register */
	uint8_t control;
} BBC1770FDC;

/**
 * Analogue to digital converter
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
} BBCADC;

/**
 * Unimplemented memory region
 */
typedef struct {
	/** Name */
	const char *name;
} BBCUnimplementedMemoryRegion;

/**
 * FRED
 */
typedef struct {
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;	
} BBCFRED;

/**
 * JIM
 */
typedef struct {
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;	
} BBCJIM;

/**
 * SHEILA
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** Unimplemented memory region */
	BBCUnimplementedMemoryRegion unimp;
	/** CRTC */
	MC6845CRTC *crtc;
	/** ACIA */
	MC6850ACIA *acia;
	/** Video ULA */
	BBCVideoULA *video_ula;
	/** Paged ROM */
	BBCPagedROM *paged;
	/** System VIA */
	BBCSystemVIA *system_via;
	/** User VIA */
	BBCUserVIA *user_via;
	/** Floppy disc controller */
	BBC1770FDC *fdc;
	/** Analogue to digital converter */
	BBCADC *adc;
} BBCSHEILA;

/**
 * BBC ROM
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
} BBCROM;

#include "bbc_crt.h"

/** IRQ interrupt sources */
enum bbc_irq_sources {
	BBC_IRQ_SYSTEM_VIA,
	BBC_IRQ_USER_VIA,
	BBC_IRQ_COUNT
};

/** NMI interrupt sources */
enum bbc_nmi_sources {
	BBC_NMI_FDC_DRQ,
	BBC_NMI_FDC_INTRQ,
	BBC_NMI_COUNT
};

/**
 * BBC Micro
 */
typedef struct {
	/** Name */
	const char *name;
	/** CPU */
	CPUM6502State *cpu;
	/** RAM */
	MemoryRegion ram;
	/** MOS ROM */
	BBCROM *mos;
	/** Paged ROM */
	BBCPagedROM *paged;
	/** Maskable interrupts */
	qemu_irq *irq;
	/** Maskable interrupt status */
	bool irq_active[BBC_IRQ_COUNT];
	/** Non-maskable interrupt */
	qemu_irq *nmi;
	/** Non-maskable interrupt status */
	bool nmi_active[BBC_NMI_COUNT];
	/** FRED */
	BBCFRED *fred;
	/** JIM */
	BBCJIM *jim;
	/** SHEILA */
	BBCSHEILA *sheila;
	/** Display */
	BBCDisplay *crt;
} BBCMicro;

extern void bbc_video_ula_update_register ( BBCVideoULA *ula,
					    BBCVideoULAUpdated updated,
					    void *opaque );
extern void bbc_video_ula_update_unregister ( BBCVideoULA *ula,
					      BBCVideoULAUpdated updated,
					      void *opaque );

#endif
