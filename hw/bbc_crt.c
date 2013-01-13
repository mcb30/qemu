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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "ui/console.h"
#include "vga_int.h"
#include "bbc.h"

#define DEBUG_CRT 1

/* Debug messages */
#define LOG_CRT(...) do {						\
		if ( DEBUG_CRT ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/**
 * Bitmap colour index
 *
 * We use a single static structure rather than dynamically allocating
 * one each time we need it, since the reverse lookup array (ent[]) is
 * large.
 */
static pixman_indexed_t bbc_crt_pixman_indexed;

/**
 * Translate a CRTC address into a video RAM address
 *
 * @v crt		BBC display
 * @v crtc_address	Refresh memory address
 * @ret teletext	Teletext address translation applies
 * @ret address		Video RAM address
 */
static hwaddr bbc_crt_translate ( BBCDisplay *crt, unsigned int crtc_address,
				  bool *teletext ) {
	static const unsigned int adjustments[4] = BBC_CRTC_ADDR_ADJUSTMENTS;
	unsigned int adjustment;
	unsigned int adjusted_address;
	hwaddr address;

	/* CRTC address bit 12 causes the address to be adjusted by
	 * subtracting an amount configured using the system VIA's
	 * addressable latch.
	 */
	adjusted_address = ( crtc_address << 3 );
	if ( crtc_address & BBC_CRTC_ADDR_ADJUST ) {
		adjustment = ( ( crt->via->addressable_latch >> BBC_LATCH_C0 )
			       & 0x03 );
		adjusted_address -= adjustments[adjustment];
	}

	/* CRTC address bit 13 selects teletext address translation */
	if ( crtc_address & BBC_CRTC_ADDR_TELETEXT ) {
		/* Teletext address translation:
		 *
		 *   CRTC address:      AA3    All 1   MA9..MA0
		 *   Video RAM address:	A14  A13..A10  A9....A0
		 */
		address = ( ( adjusted_address & 0x4000 ) | 0x3c00 |
			    ( crtc_address & 0x3ff ) );
		*teletext = true;
	} else {
		/* Standard address translation:
		 *
		 *   CRTC address:	AA3..AA0  MA7..MA0  RA2..RA0
		 *   Video RAM address:	A13..A11  A10...A3  A2....A0
		 */
		address = ( ( adjusted_address & 0x7800 ) |
			    ( ( crtc_address & 0x00ff ) << 3 ) );
		*teletext = false;
	}

	/* Addresses wrap around within the available RAM */
	address &= ( memory_region_size ( crt->ram ) - 1 );

	LOG_CRT ( "%s: CRTC address &%04X maps to video RAM address &%04lX\n",
		  crt->name, crtc_address, address );
	return address;
}

/**
 * Update video RAM mappings
 *
 * @v crt		BBC display
 */
static void bbc_crt_vram_update ( BBCDisplay *crt ) {
	MC6845CRTC *crtc = crt->crtc;
	unsigned int crtc_start;
	unsigned int crtc_count;
	unsigned int crtc_transition;
	unsigned int first_count;
	unsigned int second_count;
	hwaddr start;
	hwaddr end;
	unsigned int size;
	void *parent_ptr;

	/* Calcuate screen start CRTC address and size (in characters) */
	crtc_start = crtc->start;
	crtc_count = ( crtc->horiz_displayed * crtc->vert_displayed );

	/* Do nothing more unless CRTC address range has changed */
	if ( ( crt->crtc_start == crtc_start ) &&
	     ( crt->crtc_count == crtc_count ) ) {
		return;
	}

	/* Record updated CRTC address range */
	crt->crtc_start = crtc_start;
	crt->crtc_count = crtc_count;

	/* Calculate transition CRTC address, i.e. the first address
	 * after crtc_start with a different value for MA12.  Note
	 * that (a) this address may exceed the 14-bit address range
	 * of the CRTC, and (b) the CRTC may stop before reaching this
	 * address.
	 */
	crtc_transition = ( ( crtc_start | ( BBC_CRTC_ADDR_ADJUST - 1 ) ) + 1 );

	/* Calculate start address and translation for first display region */
	crt->first.start = bbc_crt_translate ( crt, crtc_start,
					       &crt->first.teletext );
	first_count = ( crtc_transition - crtc_start );
	if ( first_count > crtc_count )
		first_count = crtc_count;
	crt->first.size = ( first_count << ( crt->first.teletext ? 0 :
					     BBC_CRT_CHAR_SIZE_LOG2 ) );
	crt->first.before_count = 0;

	/* Calculate start address and translation for second display region */
	crt->second.before_count = first_count;
	if ( first_count < crtc_count ) {
		/* Second region starts at transition address */
		crt->second.start = bbc_crt_translate ( crt, crtc_transition,
							&crt->second.teletext );
		second_count = ( crtc_count - first_count );
		crt->second.size =
			( second_count << ( crt->second.teletext ? 0 :
					    BBC_CRT_CHAR_SIZE_LOG2 ) );
	} else {
		/* No second region */
		crt->second.start = 0;
		crt->second.size = 0;
		crt->second.teletext = false;
	}

	/* Calculate bounds of regions used by screen memory */
	start = crt->first.start;
	end = ( crt->first.start + crt->first.size );
	if ( crt->second.size ) {
		if ( start > crt->second.start ) {
			start = crt->second.start;
		} else {
			end = ( crt->second.start + crt->second.size );
		}
	}
	start &= ~( BBC_CRT_VIDEO_RAM_ALIGN - 1 );
	end = ( ( end + BBC_CRT_VIDEO_RAM_ALIGN - 1 ) &
		~( BBC_CRT_VIDEO_RAM_ALIGN - 1 ) );
	size = ( end - start );

	/* Debug output */
	LOG_CRT ( "%s: [&%04lx,&%04lx) regions [&%04lX,&%04lX) %s",
		  crt->name, start, ( start + size - 1 ),
		  crt->first.start, ( crt->first.start + crt->first.size - 1 ),
		  ( crt->first.teletext ? "teletext" : "graphic" ) );
	if ( crt->second.size ) {
		LOG_CRT ( " [&%04lX,&%04lX) %s",
			  crt->second.start,
			  ( crt->second.start + crt->second.size - 1 ),
			  ( crt->second.teletext ? "teletext" : "graphic" ) );
	}
	LOG_CRT ( "\n" );

	/* Do nothing more unless screen memory bounds have changed */
	if ( ( start == crt->start ) && ( size == crt->size ) )
		return;

	/* Record new region bounds */
	crt->start = start;
	crt->size = size;

	/* Destroy existing memory region */
	memory_region_del_subregion ( crt->ram, &crt->vram );
	memory_region_destroy ( &crt->vram );

	/* Create new memory region */
	parent_ptr = memory_region_get_ram_ptr ( crt->ram );
	memory_region_init_ram_ptr ( &crt->vram, "vram", size,
				     ( parent_ptr + start ) );
	memory_region_add_subregion ( crt->ram, start, &crt->vram );
	memory_region_set_log ( &crt->vram, true, DIRTY_MEMORY_VGA );
}

/**
 * Resize display
 *
 * @v crt		BBC display
 */
static void bbc_crt_resize ( BBCDisplay *crt ) {
	unsigned int width;
	unsigned int height;

	/* Calculate display geometry */
	width = ( crt->crtc->horiz_displayed <<
		  ( BBC_CRT_PIXEL_CLOCK_LOG2 - crt->ula->crtc_clock_log2 ) );
	height = ( crt->crtc->vert_displayed *
		   ( crt->crtc->max_scan_line + 1 ) );
	if ( ! ( crt->crtc->interlace && crt->crtc->interlace_video ) )
		height <<= 1;

	/* Do nothing more unless display geometry has changed */
	if ( ( width == crt->width ) && ( height == crt->height ) )
		return;

	/* Record new display geometry */
	crt->width = width;
	crt->height = height;

	/* Resize displayed image */
	if ( crt->image ) {
		pixman_image_unref ( crt->image );
		crt->image = NULL;
	}
	if ( crt->width && crt->height ) {
		crt->image = pixman_image_create_bits ( BBC_CRT_IMAGE_FORMAT,
							crt->width, crt->height,
							NULL, 0 );
		assert ( crt->image != NULL );
		bbc_crt_pixman_indexed.color = true;
		pixman_image_set_indexed ( crt->image,
					   &bbc_crt_pixman_indexed );
	}

	/* Resize graphic console */
	qemu_console_resize ( crt->ds, crt->width, crt->height );

	/* Mark display as invalid */
	crt->invalid = 1;
}

/**
 * Update character
 *
 * @v crt		BBC display
 * @v region		Display region
 * @v row		CRTC character row
 * @v column		CRTC character column
 * @v data		Character data
 */
static void bbc_crt_update_character ( BBCDisplay *crt,
				       BBCDisplayRegion *region,
				       unsigned int row, unsigned int column,
				       uint8_t *data ) {
	BBCVideoULA *ula = crt->ula;
	MC6845CRTC *crtc = crt->crtc;
	unsigned int scan_lines;
	unsigned int pixels_log2;
	unsigned int pixels;
	unsigned int pixel_width_log2;
	unsigned int pixel_width;
	unsigned int pixel_height_doubled;
	unsigned int x;
	unsigned int y;
	unsigned int scan_line;
	unsigned int pixel;
	uint8_t byte;
	uint8_t colour;
	void *image_data;
	void *row_dest;
	void *pixel_dest;

	/* Calculate number of scan lines */
	scan_lines = ( crtc->max_scan_line + 1 );
	if ( scan_lines > BBC_CRT_CHAR_MAX_SCAN_LINES )
		scan_lines = BBC_CRT_CHAR_MAX_SCAN_LINES;

	/* Calculate character width in BBC pixels, as the pixel clock
	 * divided by the CRTC clock.  Note that the pixel clock can
	 * never be slower than the CRTC clock, so the left shift can
	 * never be negative (and hence invalid).
	 */
	pixels_log2 = ( ula->pixel_clock_log2 - ula->crtc_clock_log2 );
	pixels = ( 1 << pixels_log2 );

	/* Calculate width of each BBC pixel in host pixels, as the
	 * nominal CRT pixel clock divided by the (BBC) pixel clock.
	 * Note that the pixel clock can never be slower than the
	 * nominal CRT pixel clock, so the left shift can never be
	 * negative (and hence invalid).
	 */
	pixel_width_log2 = ( BBC_CRT_PIXEL_CLOCK_LOG2 - ula->pixel_clock_log2 );
	pixel_width = ( 1 << pixel_width_log2 );

	/* Determine whether or not pixels are double-height
	 * (i.e. interlace disabled, or interlace sync without video).
	 */
	pixel_height_doubled = ( ! ( crt->crtc->interlace &&
				     crt->crtc->interlace_video ) );

	/* Calculate starting address within image */
	x = ( column << ( pixels_log2 + pixel_width_log2 ) );
	y = ( ( row * ( crtc->max_scan_line + 1 ) ) << pixel_height_doubled );
	image_data = pixman_image_get_data ( crt->image );
	row_dest = ( image_data + ( y * crt->width ) + x );

	/* Process each scan line in turn */
	for ( scan_line = 0 ; scan_line < scan_lines ; scan_line++ ) {

		/* Read data */
		byte = data[scan_line];

		/* Process each pixel in turn */
		pixel_dest = row_dest;
		for ( pixel = 0 ; pixel < pixels ; pixel++ ) {

			/* Extract pixel value from bits {7,5,3,1} */
			colour = ( byte & 0xcc );

			/* Fill image */
			memset ( pixel_dest, colour, pixel_width );
			if ( pixel_height_doubled ) {
				memset ( pixel_dest + crt->width, colour,
					 pixel_width );
			}
			pixel_dest += pixel_width;

			/* Left shift, injecting ones */
			byte = ( ( byte << 1 ) | 0x01 );
		}

		/* Move to next row */
		row_dest += ( crt->width << pixel_height_doubled );
	}
}

/**
 * Update display region
 *
 * @v crt		BBC display
 * @v region		Display region
 */
static void bbc_crt_update_region ( BBCDisplay *crt,
				    BBCDisplayRegion *region ) {
	hwaddr addr;
	hwaddr end;
	hwaddr next;
	hwaddr vram_addr;
	uint8_t *vram_ptr;
	unsigned int offset;
	unsigned int size;
	unsigned int step_shift;
	unsigned int step;
	unsigned int horiz_displayed;
	unsigned int row;
	unsigned int column;

	/* Get direct pointer to video RAM */
	vram_ptr = memory_region_get_ram_ptr ( &crt->vram );

	/* Scan through region looking for dirty pages */
	end = ( region->start + region->size );
	offset = ( region->before_count );
	step_shift = ( region->teletext ? 0 : BBC_CRT_CHAR_SIZE_LOG2 );
	step = ( 1 << step_shift );
	horiz_displayed = crt->crtc->horiz_displayed;
	for ( addr = region->start ; addr < end ; addr = next ) {

		/* Find end of this page (or end of region, if sooner) */
		next = TARGET_PAGE_ALIGN ( addr + 1 );
		if ( next > end )
			next = end;
		vram_addr = ( addr - crt->start );
		size = ( next - addr );

		/* Update this subregion, if applicable */
		if ( crt->invalid ||
		     memory_region_get_dirty ( &crt->vram, vram_addr, size,
					       DIRTY_MEMORY_VGA ) ) {

			/* Mark region as clean */
			memory_region_reset_dirty ( &crt->vram, vram_addr, size,
						    DIRTY_MEMORY_VGA );

			/* Calculate starting row and column address */
			row = ( offset / horiz_displayed );
			column = ( offset % horiz_displayed );
			LOG_CRT ( "%s: updating [&%04lX,&%04lX) at (%d,%d)\n",
				  crt->name, addr, ( next - 1 ), column, row );

			/* Redraw each character in subregion */
			while ( addr < next ) {
				bbc_crt_update_character ( crt, region, row,
							   column,
							   ( vram_ptr +
							     vram_addr ) );
				addr += step;
				vram_addr += step;
				offset++;
				if ( ++column == horiz_displayed ) {
					row++;
					column = 0;
				}
			}

		} else {

			/* Skip this sub-region */
			offset += ( size >> step_shift );
		}
	}
}

/**
 * Update display
 *
 * @v opaque		BBC display
 */
static void bbc_crt_update ( void *opaque ) {
	BBCDisplay *crt = opaque;

	/* Do nothing unless we have an image */
	if ( ! crt->image )
		return;

	/* Update both regions */
	bbc_crt_update_region ( crt, &crt->first );
	bbc_crt_update_region ( crt, &crt->second );

	/* Mark display as valid */
	crt->invalid = 0;


	//
	unsigned int i;
	for ( i = 0 ; i < 256 ; i++ ) {
		bbc_crt_pixman_indexed.rgba[i] =
			( ( i & 0x80 ) ? 0xffffff : 0x000000 );
	}

	pixman_image_composite ( PIXMAN_OP_SRC, crt->image, NULL,
				 crt->ds->surface->image, 0, 0, 0, 0,
				 0, 0, crt->width, crt->height );
	dpy_gfx_update ( crt->ds, 0, 0, crt->width, crt->height );
}

/**
 * Invalidate display
 *
 * @v opaque		BBC display
 */
static void bbc_crt_invalidate ( void *opaque ) {
	BBCDisplay *crt = opaque;

	/* Mark display as invalid */
	crt->invalid = 1;
}

/**
 * Create a screen dump
 *
 * @v opaque		BBC display
 * @v filename		Filename
 * @v cswitch		Console is not currently displayed
 * @ret errp		Error pointer
 */
static void bbc_crt_screen_dump ( void *opaque, const char *filename,
				  bool cswitch, Error **errp ) {
	BBCDisplay *crt = opaque;

	/* Invalidate display if currently displaying a different console */
	if ( cswitch )
		bbc_crt_invalidate ( crt );

	/* Update display */
	bbc_crt_update ( crt );

	/* Dump display to PPM file */
	ppm_save ( filename, crt->ds->surface, errp );
}

/**
 * Handle update to CRTC register
 *
 * @v opaque		BBC display
 * @v addr		CRTC register address
 */
static void bbc_crt_crtc_updated ( void *opaque, hwaddr addr ) {
	BBCDisplay *crt = opaque;

	/* Handle updated register */
	switch ( addr ) {
	case MC6845_HORIZ_DISPLAYED:
	case MC6845_VERT_DISPLAYED:
		bbc_crt_vram_update ( crt );
		bbc_crt_resize ( crt );
		break;
	case MC6845_INTERLACE:
	case MC6845_MAX_SCAN_LINE:
		bbc_crt_resize ( crt );
		break;
	case MC6845_START_H:
	case MC6845_START_L:
		bbc_crt_vram_update ( crt );
		break;
	}
}

/**
 * Handle update to Video ULA
 *
 * @v opaque		BBC display
 */
static void bbc_crt_ula_updated ( void *opaque ) {
	BBCDisplay *crt = opaque;

	/* Handle update */
	bbc_crt_resize ( crt );
}

/**
 * Initialise CRT
 *
 * @v name		Name
 * @v ram		System RAM
 * @v crtc		6845 CRTC
 * @v ula		Video ULA
 * @v via		System VIA
 * @ret crt		CRT
 */
BBCDisplay * bbc_crt_init ( const char *name, MemoryRegion *ram,
			    MC6845CRTC *crtc, BBCVideoULA *ula,
			    BBCSystemVIA *via ) {
	BBCDisplay *crt = g_new0 ( BBCDisplay, 1 );

	/* Initialise CRT */
	crt->name = name;
	crt->ram = ram;
	crt->crtc = crtc;
	crt->ula = ula;
	crt->via = via;

	/* Initialise a dummy video RAM memory region */
	memory_region_init ( &crt->vram, "vram", 0 );
	memory_region_add_subregion ( crt->ram, 0, &crt->vram );
	memory_region_set_enabled ( &crt->vram, false );

	/* Initialise graphic console */
	crt->ds = graphic_console_init ( bbc_crt_update,
					 bbc_crt_invalidate,
					 bbc_crt_screen_dump, NULL, crt );

	/* Register for updates to CRTC and Video ULA registers */
	mc6845_update_register ( crt->crtc, bbc_crt_crtc_updated, crt );
	bbc_video_ula_update_register ( crt->ula, bbc_crt_ula_updated, crt );

	return crt;
}
