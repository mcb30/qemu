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

/**
 * Video ULA
 */
typedef struct {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;

	/** Cursor segment mask (4 bits) */
	uint8_t cursor_mask;
	/** 6845 CRTC 2MHz/1MHz clock select */
	uint8_t crtc_clock_fast;
	/** Number of columns / pixel rate
	 *
	 * Pixel clock is (2<<pixel_clock) MHz
	 */
	uint8_t pixel_clock_shift;
	/** Teletext enabled */
	uint8_t teletext;
	/** Invert flashing colours */
	uint8_t flash;
	/** Palette */
	uint8_t palette[16];
} BBCVideoULA;

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
	/** Maskable interrupt */
	qemu_irq irq;
	/** Non-maskable interrupt */
	qemu_irq nmi;
	/** FRED */
	BBCFRED *fred;
	/** JIM */
	BBCJIM *jim;
	/** SHEILA */
	BBCSHEILA *sheila;
	/** Display */
	BBCDisplay *crt;
} BBCMicro;

#endif
