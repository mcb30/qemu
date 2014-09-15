/*
 * QEMU IEEE1394 FireWire emulation
 *
 * Copyright (C) 2014 Michael Brown <mbrown@fensystems.co.uk>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef _FIREWIRE_H_
#define _FIREWIRE_H_

#include <stdint.h>
#include "qemu/compiler.h"

#define TCODE_WRITE_QUADLET_REQUEST	0x0
#define TCODE_WRITE_BLOCK_REQUEST	0x1
#define TCODE_WRITE_RESPONSE		0x2
#define TCODE_READ_QUADLET_REQUEST	0x4
#define TCODE_READ_BLOCK_REQUEST	0x5
#define TCODE_READ_QUADLET_RESPONSE	0x6
#define TCODE_READ_BLOCK_RESPONSE	0x7
#define TCODE_CYCLE_START		0x8
#define TCODE_LOCK_REQUEST		0x9
#define TCODE_STREAM_DATA		0xa
#define TCODE_LOCK_RESPONSE		0xb
#define TCODE_LINK_INTERNAL		0xe

typedef struct QEMU_PACKED FireWireCommonHeader {
	uint16_t reserved;
	uint16_t control;
} FireWireCommonHeader;

typedef struct QEMU_PACKED FireWireRequestHeader {
	uint16_t destination_id;
	uint16_t control;
	union {
		uint16_t source_id;
		uint64_t destination_offset;
	};
} FireWireRequestHeader;

typedef struct QEMU_PACKED FireWireResponseHeader {
	uint16_t destination_id;
	uint16_t control;
	uint16_t source_id;
	uint16_t rcode;
} FireWireResponseHeader;

typedef struct QEMU_PACKED FireWireNoDataRequestHeader {
	uint16_t destination_id;
	uint16_t control;
	union {
		uint16_t source_id;
		uint64_t destination_offset;
	};
	uint32_t crc32;
} FireWireNoDataRequestHeader;

typedef struct QEMU_PACKED FireWireNoDataResponseHeader {
	uint16_t destination_id;
	uint16_t control;
	uint16_t source_id;
	uint16_t rcode;
	uint32_t reserved;
	uint32_t crc32;
} FireWireNoDataResponseHeader;

typedef struct QEMU_PACKED FireWireQuadletRequestHeader {
	uint16_t destination_id;
	uint16_t control;
	union {
		uint16_t source_id;
		uint64_t destination_offset;
	};
	uint32_t quadlet_data;
	uint32_t crc32;
} FireWireQuadletRequestHeader;

typedef struct QEMU_PACKED FireWireQuadletResponseHeader {
	uint16_t destination_id;
	uint16_t control;
	uint16_t source_id;
	uint16_t rcode;
	uint32_t reserved;
	uint32_t quadlet_data;
	uint32_t crc32;
} FireWireQuadletResponseHeader;

typedef struct QEMU_PACKED FireWireBlockRequestHeader {
	uint16_t destination_id;
	uint16_t control;
	union {
		uint16_t source_id;
		uint64_t destination_offset;
	};
	uint16_t data_length;
	uint16_t reserved;
	uint32_t crc32;
} FireWireBlockRequestHeader;

typedef struct QEMU_PACKED FireWireBlockResponseHeader {
	uint16_t destination_id;
	uint16_t control;
	uint16_t source_id;
	uint16_t rcode;
	uint32_t reserved1;
	uint16_t data_length;
	uint16_t reserved2;
	uint32_t crc32;
} FireWireBlockResponseHeader;

typedef struct QEMU_PACKED FireWireLockRequestHeader {
	uint16_t destination_id;
	uint16_t control;
	union {
		uint16_t source_id;
		uint64_t destination_offset;
	};
	uint16_t data_length;
	uint16_t extended_tcode;
	uint32_t crc32;
} FireWireLockRequestHeader;

typedef struct QEMU_PACKED FireWireLockResponseHeader {
	uint16_t destination_id;
	uint16_t control;
	uint16_t source_id;
	uint16_t rcode;
	uint32_t reserved1;
	uint16_t data_length;
	uint16_t reserved2;
	uint16_t extended_tcode;
	uint32_t crc32;
} FireWireLockResponseHeader;

typedef struct QEMU_PACKED FireWireStreamHeader {
	uint16_t data_length;
	uint16_t control;
	uint32_t crc32;
} FireWireStreamHeader;

typedef struct QEMU_PACKED FireWirePhyHeader {
	uint32_t first;
	uint32_t second;
} FireWirePhyHeader;

typedef union FireWireHeader {
	FireWirePhyHeader phy;
	FireWireCommonHeader common;
	FireWireStreamHeader stream;
	FireWireRequestHeader request;
	FireWireResponseHeader response;
	FireWireNoDataRequestHeader no_data_request;
	FireWireNoDataResponseHeader no_data_response;
	FireWireQuadletRequestHeader quadlet_request;
	FireWireQuadletResponseHeader quadlet_response;
	FireWireBlockRequestHeader block_request;
	FireWireBlockResponseHeader block_response;
	FireWireLockRequestHeader lock_request;
	FireWireLockResponseHeader lock_response;
	uint32_t raw[0];
} FireWireHeader;

#endif /* _FIREWIRE_H_ */
