/*
 * Acorn BBC Micro CRT
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

#ifndef HW_BBC_CRT_H
#define HW_BBC_CRT_H

/** CRT character size (non-teletext)
 *
 * In non-teletext address translation, bits A2..A0 are taken from the
 * CRTC's RA2..RA0.  Each character therefore occupies eight bytes of
 * memory, even if the CRTC is configured for fewer than eight scan
 * lines.
 */
#define BBC_CRT_CHAR_SIZE 8

/** CRTC address bit 12 selects the adjusted address for translation */
#define BBC_CRTC_ADDR_ADJUST ( 1 << 12 )

/** CRTC address adjustments */
#define BBC_CRTC_ADDR_ADJUSTMENTS { 0x4000, 0x2000, 0x5000, 0x2800 }

/** CRTC address bit 13 selects teletext address translation */
#define BBC_CRTC_ADDR_TELETEXT ( 1 << 13 )

/**
 * A BBC Micro display region
 *
 * This represents a contiguous video RAM region within which address
 * translation is consistent.  The region is guaranteed to start and
 * end on a CRTC character boundary.
 */
typedef struct {
	/** Start address */
	hwaddr start;
	/** Size (in bytes) */
	unsigned int size;
	/** Teletext address translation applies
	 *
	 * Note that teletext address translation (CRTC MA13 bit high)
	 * is entirely separate from teletext character display (video
	 * ULA control register bit 1 high).
	 */
	bool teletext;
} BBCDisplayRegion;

/** BBC Micro display */
typedef struct {
	/** Name */
	const char *name;
	/** Parent memory region */
	MemoryRegion *parent;
	/** 6845 CRTC */
	MC6845CRTC *crtc;
	/** Video ULA */
	BBCVideoULA *ula;
	/** System VIA */
	BBCSystemVIA *via;
	/** Graphical console */
	DisplayState *ds;
	/** Memory region for video RAM
	 *
	 * The BBC display shares the main system RAM.  We create an
	 * aliased region in order to enable dirty logging for only
	 * the display RAM.
	 */
	MemoryRegion mr;

	/** Display region prior to wrap-around */
	BBCDisplayRegion first;
	/** Display region after wrap-around */
	BBCDisplayRegion second;
} BBCDisplay;

extern BBCDisplay * bbc_crt_init ( const char *name, MemoryRegion *parent,
				   MC6845CRTC *crtc, BBCVideoULA *ula,
				   BBCSystemVIA *via );

#endif
