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

/*
 * PCI details
 *
 */

#define OHCI1394_BAR_SIZE 0x800
#define OHCI1394_BAR_MASK (OHCI1394_BAR_SIZE-1)
#define OHCI1394_PROG_IF 0x10

/*
 * Common offsets used in device registers
 *
 */

#define OHCI1394_OFFSET_CLEAR 0x04
#define OHCI1394_OFFSET_MASKED 0x04
#define OHCI1394_OFFSET_MASK 0x08

/*
 * Device registers
 *
 */

#define OHCI1394_VERSION 0x000
#define OHCI1394_VERSION_1_1				0x00010001U
#define OHCI1394_VERSION_GUID_ROM			0x01000000U

#define OHCI1394_GUID_ROM 0x004

#define OHCI1394_AT_RETRIES 0x008

#define OHCI1394_CSR_DATA 0x00c

#define OHCI1394_CSR_COMPARE_DATA 0x010

#define OHCI1394_CSR_CONTROL 0x014
#define OHCI1394_CSR_CONTROL_CSR_DONE			0x80000000U
#define OHCI1394_CSR_CONTROL_CSR_SEL(x) (((x) >> 0) & 0x3)

#define OHCI1394_CONFIG_ROM_HDR 0x018

#define OHCI1394_BUS_ID 0x01c
#define OHCI1394_BUS_ID_1394				0x31333934U

#define OHCI1394_BUS_OPTIONS 0x020
#define OHCI1394_BUS_OPTIONS_LINK_SPD_DEFAULT		0x00000007U
#define OHCI1394_BUS_OPTIONS_MAX_REC_DEFAULT		0x0000f000U

#define OHCI1394_GUID_HI 0x024

#define OHCI1394_GUID_LO 0x028

#define OHCI1394_CONFIG_ROM_MAP 0x034
#define OHCI1394_CONFIG_ROM_MAP_MASK			0xfffffc00U

#define OHCI1394_POSTED_WRITE_ADDRESS 0x038

#define OHCI1394_VENDOR_ID 0x040

#define OHCI1394_HC_CONTROL 0x050
#define OHCI1394_HC_CONTROL_SOFT_RESET			0x00010000U
#define OHCI1394_HC_CONTROL_LINK_ENABLE			0x00020000U
#define OHCI1394_HC_CONTROL_POSTED_WRITE_ENABLE		0x00040000U
#define OHCI1394_HC_CONTROL_LPS				0x00080000U
#define OHCI1394_HC_CONTROL_A_PHY_ENHANCE_ENABLE	0x00400000U
#define OHCI1394_HC_CONTROL_PROGRAM_PHY_ENABLE		0x00800000U
#define OHCI1394_HC_CONTROL_ACK_TARDY_ENABLE		0x20000000U
#define OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA		0x40000000U
#define OHCI1394_HC_CONTROL_BIB_IMAGE_VALID		0x80000000U

#define OHCI1394_SELF_ID_BUFFER 0x064

#define OHCI1394_SELF_ID_COUNT 0x068
#define OHCI1394_SELF_ID_COUNT_SIZE_MASK		0x000007ffU
#define OHCI1394_SELF_ID_COUNT_GENERATION_INC		0x00010000U
#define OHCI1394_SELF_ID_COUNT_GENERATION_MASK		0x00ff0000U
#define OHCI1394_SELF_ID_COUNT_ERROR			0x80000000U

#define OHCI1394_IR_MULTI_CHAN_MASK 0x070

#define OHCI1394_INTR 0x080
#define OHCI1394_INTR_REQ_TX_COMPLETE			0x00000001U
#define OHCI1394_INTR_RESP_TX_COMPLETE			0x00000002U
#define OHCI1394_INTR_AR_RQ				0x00000004U
#define OHCI1394_INTR_AR_RS				0x00000008U
#define OHCI1394_INTR_RQ_PKT				0x00000010U
#define OHCI1394_INTR_RS_PKT				0x00000020U
#define OHCI1394_INTR_ISOCH_TX				0x00000040U
#define OHCI1394_INTR_ISOCH_RX				0x00000080U
#define OHCI1394_INTR_POSTED_WRITE_ERR			0x00000100U
#define OHCI1394_INTR_LOCK_RESP_ERR			0x00000200U
#define OHCI1394_INTR_SELF_ID_COMPLETE_2		0x00008000U
#define OHCI1394_INTR_SELF_ID_COMPLETE			0x00010000U
#define OHCI1394_INTR_BUS_RESET				0x00020000U
#define OHCI1394_INTR_REG_ACCESS_FAIL			0x00040000U
#define OHCI1394_INTR_PHY				0x00080000U
#define OHCI1394_INTR_CYCLE_SYNCH			0x00100000U
#define OHCI1394_INTR_CYCLE_64_SECONDS			0x00200000U
#define OHCI1394_INTR_CYCLE_LOST			0x00400000U
#define OHCI1394_INTR_CYCLE_INCONSISTENT		0x00800000U
#define OHCI1394_INTR_UNRECOVERABLE_ERROR		0x01000000U
#define OHCI1394_INTR_CYCLE_TOO_LONG			0x02000000U
#define OHCI1394_INTR_PHY_REG_RCVD			0x04000000U
#define OHCI1394_INTR_ACK_TARDY				0x08000000U
#define OHCI1394_INTR_SOFT_INTERRUPT			0x02000000U
#define OHCI1394_INTR_VENDOR_SPECIFIC			0x04000000U
#define OHCI1394_INTR_MASTER_ENABLE			0x80000000U

#define OHCI1394_ISO_XMIT_INTR 0x090

#define OHCI1394_ISO_RECV_INTR 0x0a0

#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE 0x0b0
#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_DEFAULT	0x00001333U

#define OHCI1394_INITIAL_CHANNELS_AVAILABLE 0x0b4
#define OHCI1394_INITIAL_CHANNELS_AVAILABLE_DEFAULT	0xffffffffffffffffULL

#define OHCI1394_FAIRNESS_CONTROL 0x0dc

#define OHCI1394_LINK_CONTROL 0x0e0

#define OHCI1394_NODE_ID 0x0e8
#define OHCI1394_NODE_ID_NODE_NUMBER(x) ((x) << 0)
#define OHCI1394_NODE_ID_BUS_NUMBER_DEFAULT		0x0000ffc0U
#define OHCI1394_NODE_ID_CPS				0x08000000U
#define OHCI1394_NODE_ID_ROOT				0x40000000U
#define OHCI1394_NODE_ID_ID_VALID			0x80000000U

#define OHCI1394_PHY_CONTROL 0x0ec
#define OHCI1394_PHY_CONTROL_WR_DATA(x) (((x) >> 0 ) &0xff)
#define OHCI1394_PHY_CONTROL_REG_ADDR(x) (((x) >> 8) & 0x0f)
#define OHCI1394_PHY_CONTROL_WR_REG			0x00004000U
#define OHCI1394_PHY_CONTROL_RD_REG			0x00008000U
#define OHCI1394_PHY_CONTROL_RD_DATA_MASK		0x00ff0000U
#define OHCI1394_PHY_CONTROL_RD_DATA(x) ((x) << 16)
#define OHCI1394_PHY_CONTROL_RD_ADDR_MASK		0x0f000000U
#define OHCI1394_PHY_CONTROL_RD_ADDR(x) ((x) << 24)
#define OHCI1394_PHY_CONTROL_RD_DONE			0x80000000U

#define OHCI1394_ISOCHRONOUS_CYCLE_TIMER 0x0f0

#define OHCI1394_ASYNCHRONOUS_REQUEST_FILTER 0x100

#define OHCI1394_PHYSICAL_REQUEST_FILTER 0x110

#define OHCI1394_PHYSICAL_UPPER_BOUND 0x120

#define OHCI1394_DMA_CONTEXT 0x180

#define OHCI1394_ASYNC_REQUEST_TX 0x180

#define OHCI1394_ASYNC_RESPONSE_TX 0x1a0

#define OHCI1394_ASYNC_REQUEST_RX 0x1c0

#define OHCI1394_ASYNC_RESPONSE_RX 0x1e0

#define OHCI1394_ISOCH_TX 0x200

#define OHCI1394_ISOCH_RX 0x400

/*
 * Bus management resource registers
 *
 */

#define OHCI1394_BUS_MANAGER_ID_DEFAULT			0x0000003fU

/*
 * PHY registers
 *
 */

#define OHCI1394_PHY_RESET 0x01
#define OHCI1394_PHY_RESET_GAP_COUNT_DEFAULT		0x3f
#define OHCI1394_PHY_RESET_IBR				0x40
#define OHCI1394_PHY_RESET_RHB				0x80

#define OHCI1394_PHY_LINK 0x04
#define OHCI1394_PHY_LINK_C				0x40
#define OHCI1394_PHY_LINK_LCTRL				0x80

#define OHCI1394_PHY_MISC 0x05
#define OHCI1394_PHY_MISC_PORT_EVENT			0x04
#define OHCI1394_PHY_MISC_TIMEOUT			0x08
#define OHCI1394_PHY_MISC_PWR_FAIL			0x10
#define OHCI1394_PHY_MISC_ISBR				0x40

#define OHCI1394_PHY_SELECT 0x07
#define OHCI1394_PHY_SELECT_PAGE(x) (((x) >> 5) & 0x07)
#define OHCI1394_PHY_SELECT_PORT(x) (((x) >> 0) & 0x0f)

/*
 * Self ID packets
 *
 */

#define SELF_ID_MORE_PACKETS				0x00000001U
#define SELF_ID_PHY_INITIATOR				0x00000002U
#define SELF_ID_CONTENDER				0x00000800U
#define SELF_ID_PHY_SPEED(x) ((x) << 14)
#define SELF_ID_GAP_COUNT(x) ((x) << 16)
#define SELF_ID_PHY_ID(x) ((x) << 24)
#define SELF_ID_LINK_ON					0x00400000U
#define SELF_ID_EXTENDED				0x00800000U

#endif /* _OHCI1394_REGS_H_ */
