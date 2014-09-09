/*
 * QEMU IEEE1394 OHCI emulation
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

#ifndef _OHCI1394_REGS_H_
#define _OHCI1394_REGS_H_

/* PCI details */

#define OHCI1394_BAR_SIZE 0x800
#define OHCI1394_BAR_MASK (OHCI1394_BAR_SIZE-1)
#define OHCI1394_PROG_IF 0x10

/* Common offsets used in device registers */

#define OHCI1394_OFFSET_CLEAR 0x04
#define OHCI1394_OFFSET_MASKED 0x04
#define OHCI1394_OFFSET_MASK 0x08

/* Device registers */

#define OHCI1394_BUS_OPTIONS 0x020
#define OHCI1394_BUS_OPTIONS_MAX_REC_DEFAULT		0x0000f000UL
#define OHCI1394_BUS_OPTIONS_LINK_SPD_DEFAULT		0x00000007UL

#define OHCI1394_GUID 0x024

#define OHCI1394_CONFIG_ROM_MAP 0x034

#define OHCI1394_POSTED_WRITE_ADDRESS 0x038

#define OHCI1394_VENDOR_ID 0x040

#define OHCI1394_HC_CONTROL 0x050
#define OHCI1394_HC_CONTROL_SOFT_RESET			0x00010000UL
#define OHCI1394_HC_CONTROL_LINK_ENABLE			0x00020000UL
#define OHCI1394_HC_CONTROL_POSTED_WRITE_ENABLE		0x00040000UL
#define OHCI1394_HC_CONTROL_LPS				0x00080000UL
#define OHCI1394_HC_CONTROL_A_PHY_ENHANCE_ENABLE	0x00400000UL
#define OHCI1394_HC_CONTROL_PROGRAM_PHY_ENABLE		0x00800000UL
#define OHCI1394_HC_CONTROL_ACK_TARDY_ENABLE		0x20000000UL
#define OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA		0x40000000UL
#define OHCI1394_HC_CONTROL_BIB_IMAGE_VALID		0x80000000UL

#define OHCI1394_SELF_ID_BUFFER 0x064

#define OHCI1394_SELF_ID_COUNT 0x068

#define OHCI1394_IR_MULTI_CHAN_MASK 0x070

#define OHCI1394_INTR 0x080

#define OHCI1394_ISO_XMIT_INTR 0x090

#define OHCI1394_ISO_RECV_INTR 0x0a0

#endif /* _OHCI1394_REGS_H_ */
