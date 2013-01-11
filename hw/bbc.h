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

#define BBC_FRED_BASE 0xfc00
#define BBC_FRED_SIZE 0x0100

#define BBC_JIM_BASE 0xfd00
#define BBC_JIM_SIZE 0x0100

#define BBC_SHEILA_BASE 0xfe00
#define BBC_SHEILA_SIZE 0x0100
#define BBC_SHEILA_SERIAL 0x08
#define BBC_SHEILA_PAGED_ROM_SELECT 0x30
#define BBC_SHEILA_SYSTEM_VIA 0x40
#define BBC_SHEILA_USER_VIA 0x60

#define BBC_PAGED_ROM_SELECT_SIZE 0x10

/* Addressable latch */
#define BBC_LATCH_SOUND_WE 0
#define BBC_LATCH_SPEECH_RS 1
#define BBC_LATCH_SPEECH_WS 2
#define BBC_LATCH_KB_WE 3
#define BBC_LATCH_C0 4
#define BBC_LATCH_C1 5
#define BBC_LATCH_CAPS_LOCK 6
#define BBC_LATCH_SHIFT_LOCK 7

#endif
