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
#include "bbc.h"

#define DEBUG_CRT 1

/* Debug messages */
#define LOG_CRT(...) do {						\
		if ( DEBUG_CRT ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )


static void bbc_crt_update ( void *opaque ) {
	BBCDisplay *crt = opaque;

	LOG_CRT ( "%s: update\n", crt->name );
}

static void bbc_crt_invalidate ( void *opaque ) {
	BBCDisplay *crt = opaque;

	LOG_CRT ( "%s: invalidate\n", crt->name );
}

static void bbc_crt_screen_dump ( void *opaque, const char *filename,
				  bool cswitch, Error **errp ) {
	BBCDisplay *crt = opaque;

	LOG_CRT ( "%s: screen dump\n", crt->name );
}

/**
 * Initialise CRT
 *
 * @v name		Name
 * @ret crt		CRT
 */
BBCDisplay * bbc_crt_init ( const char *name ) {
	BBCDisplay *crt = g_new0 ( BBCDisplay, 1 );

	/* Initialise CRT */
	crt->name = name;

	/* Initialise graphic console */
	crt->ds = graphic_console_init ( bbc_crt_update,
					 bbc_crt_invalidate,
					 bbc_crt_screen_dump, NULL, crt );

	return crt;
}
