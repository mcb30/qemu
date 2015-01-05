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

#ifndef _OHCI1394_HW_H_
#define _OHCI1394_HW_H_

#include "hw/firewire.h"

/*
 * PCI details
 *
 */

#define OHCI1394_BAR_SIZE 0x800
#define OHCI1394_BAR_MASK (OHCI1394_BAR_SIZE-1)
#define OHCI1394_PROG_IF 0x10

#define OHCI1394_MASK_READONLY				0x00000000U
#define OHCI1394_MASK_32BIT				0xffffffffU
#define OHCI1394_MASK_64BIT				0xffffffffffffffffULL
#define OHCI1394_MASK_ISOCH_TX				\
	(0xffffffffU >> (32 - OHCI1394_ISOCH_TX_CONTEXTS))
#define OHCI1394_MASK_ISOCH_RX				\
	(0xffffffffU >> (32 - OHCI1394_ISOCH_RX_CONTEXTS))

/*
 * Device registers
 *
 */

#define OHCI1394_VERSION 0x000
#define OHCI1394_VERSION_REVISION_MASK			0x000000ffU
#define OHCI1394_VERSION_VERSION_MASK			0x00ff0000U
#define OHCI1394_VERSION_VERSION_1_1			0x00010001U
#define OHCI1394_VERSION_GUID_ROM			0x01000000U
#define OHCI1394_VERSION_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_VERSION_CLR_MASK			OHCI1394_MASK_READONLY

#define OHCI1394_GUID_ROM 0x004
#define OHCI1394_GUID_ROM_MINIROM_MASK			0x000000ffU
#define OHCI1394_GUID_ROM_RD_DATA_MASK			0x00ff0000U
#define OHCI1394_GUID_ROM_RD_START			0x01000000U
#define OHCI1394_GUID_ROM_ADDR_RESET			0x80000000U
#define OHCI1394_GUID_ROM_SET_MASK			\
	(OHCI1394_GUID_ROM_ADDR_RESET |			\
	 OHCI1394_GUID_ROM_RD_START)
#define OHCI1394_GUID_ROM_CLR_MASK 0

#define OHCI1394_AT_RETRIES 0x008
#define OHCI1394_AT_RETRIES_MAX_AT_REQ_RETRIES_MASK	0x0000000fU
#define OHCI1394_AT_RETRIES_MAX_AT_RESP_RETRIES_MASK	0x000000f0U
#define OHCI1394_AT_RETRIES_MAX_PHYS_RESP_RETRIES_MASK	0x00000f00U
#define OHCI1394_AT_RETRIES_CYCLE_LIMIT_MASK		0x1fff0000U
#define OHCI1394_AT_RETRIES_SECOND_LIMIT_MASK		0xe0000000U
#define OHCI1394_AT_RETRIES_SET_MASK			\
	(OHCI1394_AT_RETRIES_MAX_AT_REQ_RETRIES_MASK |	\
	 OHCI1394_AT_RETRIES_MAX_AT_RESP_RETRIES_MASK |	\
	 OHCI1394_AT_RETRIES_MAX_PHYS_RESP_RETRIES_MASK)
#define OHCI1394_AT_RETRIES_CLR_MASK			\
	OHCI1394_AT_RETRIES_SET_MASK

#define OHCI1394_CSR_DATA 0x00c
#define OHCI1394_CSR_DATA_SET_MASK			OHCI1394_MASK_32BIT
#define OHCI1394_CSR_DATA_CLR_MASK			OHCI1394_MASK_32BIT

#define OHCI1394_CSR_COMPARE_DATA 0x010
#define OHCI1394_CSR_COMPARE_DATA_SET_MASK		OHCI1394_MASK_32BIT
#define OHCI1394_CSR_COMPARE_DATA_CLR_MASK		OHCI1394_MASK_32BIT

#define OHCI1394_CSR_CONTROL 0x014
#define OHCI1394_CSR_CONTROL_CSR_SEL_SET(x)		\
	((x) << 0)
#define OHCI1394_CSR_CONTROL_CSR_SEL_GET(x)		\
	(((x) >> 0) & 0x3)
#define OHCI1394_CSR_CONTROL_CSR_SEL_MASK		\
	OHCI1394_CSR_CONTROL_CSR_SEL_SET(0x3)
#define OHCI1394_CSR_CONTROL_CSR_DONE			0x80000000U
#define OHCI1394_CSR_CONTROL_SET_MASK			\
	(OHCI1394_CSR_CONTROL_CSR_SEL_MASK)
#define OHCI1394_CSR_CONTROL_CLR_MASK			\
	OHCI1394_CSR_CONTROL_SET_MASK

#define OHCI1394_CONFIG_ROM_HDR 0x018
#define OHCI1394_CONFIG_ROM_HDR_SET_MASK		OHCI1394_MASK_32BIT
#define OHCI1394_CONFIG_ROM_HDR_CLR_MASK		OHCI1394_MASK_32BIT

#define OHCI1394_BUS_ID 0x01c
#define OHCI1394_BUS_ID_1394				0x31333934U
#define OHCI1394_BUS_ID_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_BUS_ID_CLR_MASK			OHCI1394_MASK_READONLY

#define OHCI1394_BUS_OPTIONS 0x020
#define OHCI1394_BUS_OPTIONS_LINK_SPD_SET(x)		\
	((x) << 0)
#define OHCI1394_BUS_OPTIONS_LINK_SPD_DEFAULT		\
	OHCI1394_BUS_OPTIONS_LINK_SPD_SET(0x7)
#define OHCI1394_BUS_OPTIONS_LINK_SPD_MASK		\
	OHCI1394_BUS_OPTIONS_LINK_SPD_SET(0x7)
#define OHCI1394_BUS_OPTIONS_MAX_REC_SET(x)		\
	((x) << 12)
#define OHCI1394_BUS_OPTIONS_MAX_REC_MASK		\
	OHCI1394_BUS_OPTIONS_MAX_REC_SET(0xf)
#define OHCI1394_BUS_OPTIONS_SET_MASK			OHCI1394_MASK_32BIT
#define OHCI1394_BUS_OPTIONS_CLR_MASK			OHCI1394_MASK_32BIT

#define OHCI1394_GUID_HI 0x024
#define OHCI1394_GUID_HI_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_GUID_HI_CLR_MASK			OHCI1394_MASK_READONLY

#define OHCI1394_GUID_LO 0x028
#define OHCI1394_GUID_LO_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_GUID_LO_CLR_MASK			OHCI1394_MASK_READONLY

#define OHCI1394_CONFIG_ROM_MAP 0x034
#define OHCI1394_CONFIG_ROM_MAP_CONFIG_ROM_ADDR_MASK	0xfffffc00U
#define OHCI1394_CONFIG_ROM_MAP_SET_MASK		\
	(OHCI1394_CONFIG_ROM_MAP_CONFIG_ROM_ADDR_MASK)
#define OHCI1394_CONFIG_ROM_MAP_CLR_MASK		\
	OHCI1394_CONFIG_ROM_MAP_SET_MASK

#define OHCI1394_POSTED_WRITE_ADDRESS 0x038
#define OHCI1394_POSTED_WRITE_ADDRESS_SET_MASK		OHCI1394_MASK_READONLY
#define OHCI1394_POSTED_WRITE_ADDRESS_CLR_MASK		OHCI1394_MASK_READONLY

#define OHCI1394_VENDOR_ID 0x040
#define OHCI1394_VENDOR_ID_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_VENDOR_ID_CLR_MASK			OHCI1394_MASK_READONLY

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
#define OHCI1394_HC_CONTROL_SET_MASK			\
	(OHCI1394_HC_CONTROL_SOFT_RESET |		\
	 OHCI1394_HC_CONTROL_LINK_ENABLE |		\
	 OHCI1394_HC_CONTROL_POSTED_WRITE_ENABLE |	\
	 OHCI1394_HC_CONTROL_LPS |			\
	 OHCI1394_HC_CONTROL_A_PHY_ENHANCE_ENABLE |	\
	 OHCI1394_HC_CONTROL_ACK_TARDY_ENABLE |		\
	 OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA |	\
	 OHCI1394_HC_CONTROL_BIB_IMAGE_VALID)
#define OHCI1394_HC_CONTROL_CLR_MASK			\
	(OHCI1394_HC_CONTROL_POSTED_WRITE_ENABLE |	\
	 OHCI1394_HC_CONTROL_A_PHY_ENHANCE_ENABLE |	\
	 OHCI1394_HC_CONTROL_PROGRAM_PHY_ENABLE |	\
	 OHCI1394_HC_CONTROL_ACK_TARDY_ENABLE |		\
	 OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA)

#define OHCI1394_SELF_ID_BUFFER 0x064
#define OHCI1394_SELF_ID_BUFFER_SELF_ID_BUFFER_PTR_MASK	0xfffff800U
#define OHCI1394_SELF_ID_BUFFER_SET_MASK		\
	(OHCI1394_SELF_ID_BUFFER_SELF_ID_BUFFER_PTR_MASK)
#define OHCI1394_SELF_ID_BUFFER_CLR_MASK		\
	OHCI1394_SELF_ID_BUFFER_SET_MASK

#define OHCI1394_SELF_ID_COUNT 0x068
#define OHCI1394_SELF_ID_COUNT_SIZE_MASK		0x000007ffU
#define OHCI1394_SELF_ID_COUNT_GENERATION_INC		0x00010000U
#define OHCI1394_SELF_ID_COUNT_GENERATION_MASK		0x00ff0000U
#define OHCI1394_SELF_ID_COUNT_ERROR			0x80000000U
#define OHCI1394_SELF_ID_COUNT_SET_MASK			OHCI1394_MASK_READONLY
#define OHCI1394_SELF_ID_COUNT_CLR_MASK			OHCI1394_MASK_READONLY

#define OHCI1394_IR_MULTI_CHAN_MASK 0x070
#define OHCI1394_IR_MULTI_CHAN_MASK_SET_MASK		OHCI1394_MASK_64BIT
#define OHCI1394_IR_MULTI_CHAN_MASK_CLR_MASK		OHCI1394_MASK_64BIT

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
#define OHCI1394_INTR_EVENT_SET_MASK			\
	(OHCI1394_INTR_REQ_TX_COMPLETE |		\
	 OHCI1394_INTR_RESP_TX_COMPLETE |		\
	 OHCI1394_INTR_AR_RQ |				\
	 OHCI1394_INTR_AR_RS |				\
	 OHCI1394_INTR_RQ_PKT |				\
	 OHCI1394_INTR_RS_PKT |				\
	 OHCI1394_INTR_POSTED_WRITE_ERR |		\
	 OHCI1394_INTR_LOCK_RESP_ERR |			\
	 OHCI1394_INTR_SELF_ID_COMPLETE_2 |		\
	 OHCI1394_INTR_SELF_ID_COMPLETE |		\
	 OHCI1394_INTR_BUS_RESET |			\
	 OHCI1394_INTR_REG_ACCESS_FAIL |		\
	 OHCI1394_INTR_PHY |				\
	 OHCI1394_INTR_CYCLE_SYNCH |			\
	 OHCI1394_INTR_CYCLE_64_SECONDS |		\
	 OHCI1394_INTR_CYCLE_LOST |			\
	 OHCI1394_INTR_CYCLE_INCONSISTENT |		\
	 OHCI1394_INTR_UNRECOVERABLE_ERROR |		\
	 OHCI1394_INTR_CYCLE_TOO_LONG |			\
	 OHCI1394_INTR_PHY_REG_RCVD |			\
	 OHCI1394_INTR_ACK_TARDY |			\
	 OHCI1394_INTR_SOFT_INTERRUPT)
#define OHCI1394_INTR_EVENT_CLR_MASK			\
	OHCI1394_INTR_EVENT_SET_MASK
#define OHCI1394_INTR_MASK_SET_MASK			\
	(OHCI1394_INTR_EVENT_SET_MASK |			\
	 OHCI1394_INTR_MASTER_ENABLE)
#define OHCI1394_INTR_MASK_CLR_MASK			\
	OHCI1394_INTR_MASK_SET_MASK
#define OHCI1394_INTR_SET_MASK				\
	OHCI1394_EVENTMASK(OHCI1394_INTR_EVENT_SET_MASK,\
			   OHCI1394_INTR_MASK_SET_MASK)
#define OHCI1394_INTR_CLR_MASK				\
	OHCI1394_EVENTMASK(OHCI1394_INTR_EVENT_CLR_MASK,\
			   OHCI1394_INTR_MASK_CLR_MASK)

#define OHCI1394_ISO_XMIT_INTR 0x090
#define OHCI1394_ISO_XMIT_INTR_EVENT_SET_MASK		OHCI1394_MASK_ISOCH_TX
#define OHCI1394_ISO_XMIT_INTR_EVENT_CLR_MASK		OHCI1394_MASK_ISOCH_TX
#define OHCI1394_ISO_XMIT_INTR_MASK_SET_MASK		OHCI1394_MASK_ISOCH_TX
#define OHCI1394_ISO_XMIT_INTR_MASK_CLR_MASK		OHCI1394_MASK_ISOCH_TX
#define OHCI1394_ISO_XMIT_INTR_SET_MASK					\
	OHCI1394_EVENTMASK(OHCI1394_ISO_XMIT_INTR_EVENT_SET_MASK,	\
			   OHCI1394_ISO_XMIT_INTR_MASK_SET_MASK)
#define OHCI1394_ISO_XMIT_INTR_CLR_MASK					\
	OHCI1394_EVENTMASK(OHCI1394_ISO_XMIT_INTR_EVENT_CLR_MASK,	\
			   OHCI1394_ISO_XMIT_INTR_MASK_CLR_MASK)

#define OHCI1394_ISO_RECV_INTR 0x0a0
#define OHCI1394_ISO_RECV_INTR_EVENT_SET_MASK		OHCI1394_MASK_ISOCH_RX
#define OHCI1394_ISO_RECV_INTR_EVENT_CLR_MASK		OHCI1394_MASK_ISOCH_RX
#define OHCI1394_ISO_RECV_INTR_MASK_SET_MASK		OHCI1394_MASK_ISOCH_RX
#define OHCI1394_ISO_RECV_INTR_MASK_CLR_MASK		OHCI1394_MASK_ISOCH_RX
#define OHCI1394_ISO_RECV_INTR_SET_MASK					\
	OHCI1394_EVENTMASK(OHCI1394_ISO_RECV_INTR_EVENT_SET_MASK,	\
			   OHCI1394_ISO_RECV_INTR_MASK_SET_MASK)
#define OHCI1394_ISO_RECV_INTR_CLR_MASK					\
	OHCI1394_EVENTMASK(OHCI1394_ISO_RECV_INTR_EVENT_CLR_MASK,	\
			   OHCI1394_ISO_RECV_INTR_MASK_CLR_MASK)

#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE 0x0b0
#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_DEFAULT	0x00001333U
#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_MASK	0x00001fffU
#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_SET_MASK	\
	(OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_MASK)
#define OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_CLR_MASK	\
	OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_SET_MASK

#define OHCI1394_INITIAL_CHANNELS_AVAILABLE 0x0b4
#define OHCI1394_INITIAL_CHANNELS_AVAILABLE_DEFAULT	0xffffffffffffffffULL
#define OHCI1394_INITIAL_CHANNELS_AVAILABLE_SET_MASK	OHCI1394_MASK_64BIT
#define OHCI1394_INITIAL_CHANNELS_AVAILABLE_CLR_MASK	OHCI1394_MASK_64BIT

#define OHCI1394_FAIRNESS_CONTROL 0x0dc
#define OHCI1394_FAIRNESS_CONTROL_PRI_REQ_MASK		0x000000ffU
#define OHCI1394_FAIRNESS_CONTROL_SET_MASK		\
	(OHCI1394_FAIRNESS_CONTROL_PRI_REQ_MASK)
#define OHCI1394_FAIRNESS_CONTROL_CLR_MASK		\
	OHCI1394_FAIRNESS_CONTROL_SET_MASK

#define OHCI1394_LINK_CONTROL 0x0e0
#define OHCI1394_LINK_CONTROL_TAG1_SYNC_FILTER_LOCK	0x00000040U
#define OHCI1394_LINK_CONTROL_RCV_SELF_ID		0x00000200U
#define OHCI1394_LINK_CONTROL_RCV_PHY_PKT		0x00000400U
#define OHCI1394_LINK_CONTROL_CYCLE_TIMER_ENABLE	0x00100000U
#define OHCI1394_LINK_CONTROL_CYCLE_MASTER		0x00200000U
#define OHCI1394_LINK_CONTROL_CYCLE_SOURCE		0x00400000U
#define OHCI1394_LINK_CONTROL_SET_MASK			\
	(OHCI1394_LINK_CONTROL_TAG1_SYNC_FILTER_LOCK |	\
	 OHCI1394_LINK_CONTROL_RCV_SELF_ID |		\
	 OHCI1394_LINK_CONTROL_RCV_PHY_PKT |		\
	 OHCI1394_LINK_CONTROL_CYCLE_TIMER_ENABLE |	\
	 OHCI1394_LINK_CONTROL_CYCLE_MASTER |		\
	 OHCI1394_LINK_CONTROL_CYCLE_SOURCE)
#define OHCI1394_LINK_CONTROL_CLR_MASK			\
	(OHCI1394_LINK_CONTROL_RCV_SELF_ID |		\
	 OHCI1394_LINK_CONTROL_RCV_PHY_PKT |		\
	 OHCI1394_LINK_CONTROL_CYCLE_TIMER_ENABLE |	\
	 OHCI1394_LINK_CONTROL_CYCLE_MASTER |		\
	 OHCI1394_LINK_CONTROL_CYCLE_SOURCE)

#define OHCI1394_NODE_ID 0x0e8
#define OHCI1394_NODE_ID_NODE_NUMBER_SET(x)		\
	((x) << 0)
#define OHCI1394_NODE_ID_NODE_NUMBER_MASK		\
	OHCI1394_NODE_ID_NODE_NUMBER_SET(0x3f)
#define OHCI1394_NODE_ID_BUS_NUMBER_SET(x)		\
	((x) << 6)
#define OHCI1394_NODE_ID_BUS_NUMBER_DEFAULT		\
	OHCI1394_NODE_ID_BUS_NUMBER_SET(0x3ff)
#define OHCI1394_NODE_ID_BUS_NUMBER_MASK		\
	OHCI1394_NODE_ID_BUS_NUMBER_SET(0x3ff)
#define OHCI1394_NODE_ID_GET(x)				\
	(((x) >> 0) & 0xffff)
#define OHCI1394_NODE_ID_CPS				0x08000000U
#define OHCI1394_NODE_ID_ROOT				0x40000000U
#define OHCI1394_NODE_ID_ID_VALID			0x80000000U
#define OHCI1394_NODE_ID_SET_MASK			\
	(OHCI1394_NODE_ID_BUS_NUMBER_MASK)
#define OHCI1394_NODE_ID_CLR_MASK			\
	OHCI1394_NODE_ID_SET_MASK

#define OHCI1394_PHY_CONTROL 0x0ec
#define OHCI1394_PHY_CONTROL_WR_DATA_SET(x)		\
	((x) << 0)
#define OHCI1394_PHY_CONTROL_WR_DATA_GET(x)		\
	(((x) >> 0 ) &0xff)
#define OHCI1394_PHY_CONTROL_WR_DATA_MASK		\
	OHCI1394_PHY_CONTROL_WR_DATA_SET(0xff)
#define OHCI1394_PHY_CONTROL_REG_ADDR_SET(x)		\
	((x) << 8)
#define OHCI1394_PHY_CONTROL_REG_ADDR_GET(x)		\
	(((x) >> 8) & 0xf)
#define OHCI1394_PHY_CONTROL_REG_ADDR_MASK		\
	OHCI1394_PHY_CONTROL_REG_ADDR_SET(0xf)
#define OHCI1394_PHY_CONTROL_WR_REG			0x00004000U
#define OHCI1394_PHY_CONTROL_RD_REG			0x00008000U
#define OHCI1394_PHY_CONTROL_RD_DATA_SET(x)		\
	((x) << 16)
#define OHCI1394_PHY_CONTROL_RD_DATA_MASK		\
	OHCI1394_PHY_CONTROL_RD_DATA_SET(0xff)
#define OHCI1394_PHY_CONTROL_RD_ADDR_SET(x)		\
	((x) << 24)
#define OHCI1394_PHY_CONTROL_RD_ADDR_MASK		\
	OHCI1394_PHY_CONTROL_RD_ADDR_SET(0xf)
#define OHCI1394_PHY_CONTROL_RD_DONE			0x80000000U
#define OHCI1394_PHY_CONTROL_SET_MASK			\
	(OHCI1394_PHY_CONTROL_WR_DATA_MASK |		\
	 OHCI1394_PHY_CONTROL_REG_ADDR_MASK |		\
	 OHCI1394_PHY_CONTROL_WR_REG |			\
	 OHCI1394_PHY_CONTROL_RD_REG)
#define OHCI1394_PHY_CONTROL_CLR_MASK			\
	OHCI1394_PHY_CONTROL_SET_MASK

#define OHCI1394_ISOCHRONOUS_CYCLE_TIMER 0x0f0
#define OHCI1394_ISOCHRONOUS_CYCLE_TIMER_SET_MASK	OHCI1394_MASK_32BIT
#define OHCI1394_ISOCHRONOUS_CYCLE_TIMER_CLR_MASK	OHCI1394_MASK_32BIT

#define OHCI1394_ASYNCHRONOUS_REQUEST_FILTER 0x100
#define OHCI1394_ASYNCHRONOUS_REQUEST_FILTER_SET_MASK	OHCI1394_MASK_64BIT
#define OHCI1394_ASYNCHRONOUS_REQUEST_FILTER_CLR_MASK	OHCI1394_MASK_64BIT

#define OHCI1394_PHYSICAL_REQUEST_FILTER 0x110
#define OHCI1394_PHYSICAL_REQUEST_FILTER_SET_MASK	OHCI1394_MASK_64BIT
#define OHCI1394_PHYSICAL_REQUEST_FILTER_CLR_MASK	OHCI1394_MASK_64BIT

#define OHCI1394_PHYSICAL_UPPER_BOUND 0x120
#define OHCI1394_PHYSICAL_UPPER_BOUND_SET_MASK		OHCI1394_MASK_32BIT
#define OHCI1394_PHYSICAL_UPPER_BOUND_CLR_MASK		OHCI1394_MASK_32BIT

/*
 * DMA context register sets
 *
 */

#define OHCI1394_ASYNC(x) (0x180 + ((x) * 0x20))

#define OHCI1394_ISOCH_TX(x) (0x200 + ((x) * 0x10))

#define OHCI1394_ISOCH_RX(x) (0x400 + ((x) * 0x20))

/*
 * DMA context registers
 *
 */

#define OHCI1394_CONTEXT_CONTROL 0x00
#define OHCI1394_CONTEXT_CONTROL_EVENT_CODE_SET(x)	\
	((x) << 0)
#define OHCI1394_CONTEXT_CONTROL_EVENT_CODE_MASK	\
	OHCI1394_CONTEXT_CONTROL_EVENT_CODE_SET(0x1f)
#define OHCI1394_CONTEXT_CONTROL_SPD_SET(x)		\
	((x) << 5)
#define OHCI1394_CONTEXT_CONTROL_SPD_MASK		\
	OHCI1394_CONTEXT_CONTROL_SPD_SET(0x7)
#define OHCI1394_CONTEXT_CONTROL_ACTIVE			0x00000400U
#define OHCI1394_CONTEXT_CONTROL_DEAD			0x00000800U
#define OHCI1394_CONTEXT_CONTROL_WAKE			0x00001000U
#define OHCI1394_CONTEXT_CONTROL_RUN			0x00008000U
#define OHCI1394_CONTEXT_CONTROL_XFER_STATUS_GET(x)	\
	(((x) >> 0) & 0xffff)
#define OHCI1394_CONTEXT_CONTROL_SET_MASK		\
	(OHCI1394_CONTEXT_CONTROL_WAKE |		\
	 OHCI1394_CONTEXT_CONTROL_RUN)
#define OHCI1394_CONTEXT_CONTROL_CLR_MASK		\
	(OHCI1394_CONTEXT_CONTROL_RUN)

#define OHCI1394_IT_CONTEXT_CONTROL			\
	OHCI1394_CONTEXT_CONTROL
#define OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_GET(x)	\
	(((x) >> 16) & 0x3fff)
#define OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_SET(x)	\
	((x) << 16)
#define OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_MASK	\
	OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_SET(0x3fff)
#define OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_ENABLE	0x80000000U
#define OHCI1394_IT_CONTEXT_CONTROL_SET_MASK		\
	(OHCI1394_CONTEXT_CONTROL_SET_MASK |		\
	 OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_MASK | \
	 OHCI1394_IT_CONTEXT_CONTROL_CYCLE_MATCH_ENABLE)
#define OHCI1394_IT_CONTEXT_CONTROL_CLR_MASK		\
	OHCI1394_IT_CONTEXT_CONTROL_SET_MASK

#define OHCI1394_IR_CONTEXT_CONTROL			\
	OHCI1394_CONTEXT_CONTROL
#define OHCI1394_IR_CONTEXT_CONTROL_DUAL_BUFFER_MODE	0x08000000U
#define OHCI1394_IR_CONTEXT_CONTROL_MULTI_CHAN_MODE	0x10000000U
#define OHCI1394_IR_CONTEXT_CONTROL_CYCLE_MATCH_ENABLE	0x20000000U
#define OHCI1394_IR_CONTEXT_CONTROL_ISOCH_HEADER	0x40000000U
#define OHCI1394_IR_CONTEXT_CONTROL_BUFFER_FILL		0x80000000U
#define OHCI1394_IR_CONTEXT_CONTROL_SET_MASK		\
	(OHCI1394_CONTEXT_CONTROL_SET_MASK |		\
	 OHCI1394_IR_CONTEXT_CONTROL_DUAL_BUFFER_MODE |	\
	 OHCI1394_IR_CONTEXT_CONTROL_MULTI_CHAN_MODE |	\
	 OHCI1394_IR_CONTEXT_CONTROL_CYCLE_MATCH_ENABLE|\
	 OHCI1394_IR_CONTEXT_CONTROL_ISOCH_HEADER |	\
	 OHCI1394_IR_CONTEXT_CONTROL_BUFFER_FILL)
#define OHCI1394_IR_CONTEXT_CONTROL_CLR_MASK		\
	OHCI1394_IR_CONTEXT_CONTROL_SET_MASK

#define OHCI1394_COMMAND_PTR 0x0c
#define OHCI1394_COMMAND_PTR_Z_SET(x)			\
	((x) << 0)
#define OHCI1394_COMMAND_PTR_Z_GET(x)			\
	(((x) >> 0) & 0xf)
#define OHCI1394_COMMAND_PTR_Z_MASK			\
	OHCI1394_COMMAND_PTR_Z_SET(0xf)
#define OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_SET(x)	\
	((x) & 0xfffffff0U)
#define OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_GET(x)	\
	((x) & 0xfffffff0U)
#define OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_MASK	\
	OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_SET(0xfffffff0U)
#define OHCI1394_COMMAND_PTR_SET_MASK			\
	(OHCI1394_COMMAND_PTR_Z_MASK |			\
	 OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_MASK)
#define OHCI1394_COMMAND_PTR_CLR_MASK			\
	OHCI1394_COMMAND_PTR_SET_MASK

#define OHCI1394_CONTEXT_MATCH 0x10
#define OHCI1394_CONTEXT_MATCH_SET_MASK			OHCI1394_MASK_32BIT
#define OHCI1394_CONTEXT_MATCH_CLR_MASK			OHCI1394_MASK_32BIT

#define OHCI1394_EVT_NO_STATUS				0x00
#define OHCI1394_EVT_LONG_PACKET			0x02
#define OHCI1394_EVT_MISSING_ACK			0x03
#define OHCI1394_EVT_UNDERRUN				0x04
#define OHCI1394_EVT_OVERRUN				0x05
#define OHCI1394_EVT_DESCRIPTOR_READ			0x06
#define OHCI1394_EVT_DATA_READ				0x07
#define OHCI1394_EVT_DATA_WRITE				0x08
#define OHCI1394_EVT_BUS_RESET				0x09
#define OHCI1394_EVT_TIMEOUT				0x0a
#define OHCI1394_EVT_TCODE_ERR				0x0b
#define OHCI1394_EVT_UNKNOWN				0x0e
#define OHCI1394_EVT_FLUSHED				0x0f

/*
 * DMA descriptors
 *
 */

typedef struct QEMU_PACKED OHCI1394DmaProgramCommon {
	uint16_t reserved1;
	uint16_t insn;
	uint32_t reserved2;
	uint32_t branch_address;
	uint32_t reserved3;
} OHCI1394DmaProgramCommon;

typedef struct QEMU_PACKED OHCI1394DmaProgramOutput {
	uint16_t req_count;
	uint16_t insn;
	uint32_t data_address;
	uint32_t branch_address;
	uint16_t timestamp;
	uint16_t xfer_status;
} OHCI1394DmaProgramOutput;

typedef struct QEMU_PACKED OHCI1394DmaProgramInput {
	uint16_t req_count;
	uint16_t insn;
	uint32_t data_address;
	uint32_t branch_address;
	uint16_t res_count;
	uint16_t xfer_status;
} OHCI1394DmaProgramInput;

typedef struct QEMU_PACKED OHCI1394DmaProgramStore {
	uint16_t store_doublet;
	uint16_t insn;
	uint32_t data_address;
	uint32_t skip_address;
	uint32_t reserved;
} OHCI1394DmaProgramStore;

typedef struct QEMU_PACKED OHCI1394DmaProgramDualBuffer {
	uint16_t first_size;
	uint16_t insn;
	uint16_t second_req_count;
	uint16_t first_req_count;
	uint32_t branch_address;
	uint16_t second_res_count;
	uint16_t first_res_count;
} OHCI1394DmaProgramDualBuffer;

typedef struct QEMU_PACKED OHCI1394DmaProgramDualBufferPayload {
	uint32_t reserved1;
	uint32_t first_buffer;
	uint32_t second_buffer;
	uint32_t reserved2;
} OHCI1394DmaProgramDualBufferPayload;

typedef union QEMU_PACKED OHCI1394MangledHeader {
	uint16_t control;
	uint32_t quadlet[4];
} OHCI1394MangledHeader;

typedef union QEMU_PACKED OHCI1394DemangledHeader {
	FireWireHeader fw;
	struct QEMU_PACKED {
		uint16_t garbage;
		uint16_t control;
		uint16_t first;
	} before;
	struct QEMU_PACKED {
		uint16_t first;
		uint16_t control;
		uint16_t source_id;
	} after;
	uint32_t quadlet[4];
} OHCI1394DemangledHeader;

typedef union QEMU_PACKED OHCI1394DmaProgram {
	OHCI1394DmaProgramCommon common;
	OHCI1394DmaProgramOutput output;
	OHCI1394DmaProgramInput input;
	OHCI1394DmaProgramStore store;
	OHCI1394DmaProgramDualBuffer dual_buffer;
	OHCI1394DmaProgramDualBufferPayload dual_buffer_payload;
	OHCI1394MangledHeader mangled;
	uint32_t raw[4];
} OHCI1394DmaProgram;

#define OHCI1394_INSN_WAIT_MASK				0x0003
#define OHCI1394_INSN_BRANCH_MASK			0x000c
#define OHCI1394_INSN_INTR_ERROR			0x0010
#define OHCI1394_INSN_INTR_ALWAYS			0x0030
#define OHCI1394_INSN_INTR_MASK				0x0030
#define OHCI1394_INSN_PING				0x0080
#define OHCI1394_INSN_KEY_IMMEDIATE			0x0200
#define OHCI1394_INSN_KEY_STORE_VALUE			0x0600
#define OHCI1394_INSN_KEY_MASK				0x0700
#define OHCI1394_INSN_STATUS				0x0800
#define OHCI1394_INSN_CMD_OUTPUT_MORE			0x0000
#define OHCI1394_INSN_CMD_OUTPUT_LAST			0x1000
#define OHCI1394_INSN_CMD_INPUT_MORE			0x2000
#define OHCI1394_INSN_CMD_INPUT_LAST			0x3000
#define OHCI1394_INSN_CMD_STORE_VALUE			0x8000
#define OHCI1394_INSN_CMD_MASK				0xf000

#define OHCI1394_INSN_MASK				\
	(OHCI1394_INSN_CMD_MASK |			\
	 OHCI1394_INSN_KEY_MASK)
#define OHCI1394_INSN_OUTPUT_MORE			\
	(OHCI1394_INSN_CMD_OUTPUT_MORE)
#define OHCI1394_INSN_OUTPUT_LAST			\
	(OHCI1394_INSN_CMD_OUTPUT_LAST)
#define OHCI1394_INSN_OUTPUT_MORE_immediate		\
	(OHCI1394_INSN_CMD_OUTPUT_MORE |		\
	 OHCI1394_INSN_KEY_IMMEDIATE)
#define OHCI1394_INSN_OUTPUT_LAST_immediate		\
	(OHCI1394_INSN_CMD_OUTPUT_LAST |		\
	 OHCI1394_INSN_KEY_IMMEDIATE)
#define OHCI1394_INSN_INPUT_MORE			\
	(OHCI1394_INSN_CMD_INPUT_MORE)
#define OHCI1394_INSN_INPUT_LAST			\
	(OHCI1394_INSN_CMD_INPUT_LAST)
#define OHCI1394_INSN_STORE_VALUE			\
	(OHCI1394_INSN_CMD_STORE_VALUE |		\
	 OHCI1394_INSN_KEY_STORE_VALUE)

#define OHCI1394_TCODE_PHY				0x0e

#define OHCI1394_TCODE_GET(x)				\
	(((x) >> 4) & 0xf)

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

#endif /* _OHCI1394_HW_H_ */
