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

/** BBC Micro display */
typedef struct {
	/** Name */
	const char *name;
	/** Graphical console */
	DisplayState *ds;
} BBCDisplay;

extern BBCDisplay * bbc_crt_init ( const char *name );

#endif
