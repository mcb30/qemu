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

#ifndef _OHCI1394_H_
#define _OHCI1394_H_

#include <stdint.h>
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"

/*
 * Device state
 *
 */

#define OHCI1394_ISOCH_TX_CONTEXTS 8

#define OHCI1394_ISOCH_RX_CONTEXTS 8

#define TYPE_OHCI1394 "ohci1394"

#define OHCI1394(obj) \
     OBJECT_CHECK(OHCI1394State, (obj), TYPE_OHCI1394)

typedef struct OHCI1394Shadowed {
    uint32_t active;
    uint32_t shadow;
} OHCI1394Shadowed;

typedef struct OHCI1394EventMask {
    uint32_t event;
    uint32_t mask;
} OHCI1394EventMask;

typedef struct OHCI1394DmaContext {
    uint32_t context_control;
    uint32_t command_ptr;
    uint32_t context_match;	/* Isochronous receive contexts only */
} OHCI1394DmaContext;

typedef struct OHCI1394State {
    /*< private >*/
    PCIDevice pci;
    /*< public >*/

    NICState *nic;
    NICConf conf;
    MemoryRegion bar_mem;

    /* PHY registers, in (page,address) order */
    uint8_t phy_reset;
    uint8_t phy_link;
    uint8_t phy_misc;
    uint8_t phy_select;

    /* Device registers, in address order */
    uint32_t version;
    uint32_t at_retries;
    uint32_t csr_data;
    uint32_t csr_compare_data;
    uint32_t csr_control;
    union {
	uint32_t numbered[5];
	struct {
	    uint32_t config_rom_hdr;
	    uint32_t bus_id;
	    uint32_t bus_options;
	    uint32_t guid_hi;
	    uint32_t guid_lo;
	};
    } config_rom;
    OHCI1394Shadowed config_rom_map;
    uint32_t vendor_id;
    uint32_t hc_control;
    uint32_t self_id_buffer;
    uint32_t self_id_count;
    uint64_t ir_multi_chan_mask;
    OHCI1394EventMask intr;
    OHCI1394EventMask iso_xmit_intr;
    OHCI1394EventMask iso_recv_intr;
    uint32_t initial_bandwidth_available;
    uint64_t initial_channels_available;
    uint32_t fairness_control;
    uint32_t link_control;
    uint32_t node_id;
    uint32_t phy_control;
    uint32_t isochronous_cycle_timer;
    uint64_t asynchronous_request_filter;
    uint64_t physical_request_filter;
    uint32_t physical_upper_bound;
    union {
	OHCI1394DmaContext numbered[4];
	struct {
	    OHCI1394DmaContext request_tx;
	    OHCI1394DmaContext response_tx;
	    OHCI1394DmaContext request_rx;
	    OHCI1394DmaContext response_rx;
	};
    } async;
    OHCI1394DmaContext isoch_tx[OHCI1394_ISOCH_TX_CONTEXTS];
    OHCI1394DmaContext isoch_rx[OHCI1394_ISOCH_RX_CONTEXTS];

    /* Bus management resource registers (compare-and-swap access only) */
    union {
	uint32_t numbered[4];
	struct {
	    uint32_t bus_manager_id;
	    uint32_t bandwidth_available;
	    uint32_t channels_available_hi;
	    uint32_t channels_available_lo;
	};
    } bus_management;

} OHCI1394State;

#define OHCI1394_CONFIG_ROM_OFFSET(_field) \
    offsetof(typeof(((OHCI1394State *)NULL)->config_rom), _field)

/*
 * PHY registers
 *
 */

typedef struct OHCI1394PhyRegisterOp OHCI1394PhyRegisterOp;

typedef struct OHCI1394PhyRegister {
    /* Name */
    const char *name;
    /* Offset within OHCI1394State */
    unsigned int offset;
    /* Register read/write operations */
    const OHCI1394PhyRegisterOp *op;
    /* Handle register updates */
    void (*notify) (OHCI1394State *s);
} OHCI1394PhyRegister;

struct OHCI1394PhyRegisterOp {
    void (*write) (OHCI1394State *s, const OHCI1394PhyRegister *r,
		   unsigned int port, uint8_t val);
    uint8_t (*read) (OHCI1394State *s, const OHCI1394PhyRegister *r,
		     unsigned int port);
};

#define OHCI1394_PHY_REG(_field, _op, _notify) {			\
	.name = #_field,						\
	.offset = offsetof(OHCI1394State, _field),			\
	.op = _op,							\
	.notify = _notify,						\
    }

static inline uint8_t *
ohci1394_phy_reg(OHCI1394State *s, const OHCI1394PhyRegister *r)
{
    return (((uint8_t *)s) + r->offset);
}

#define OHCI1394_PHY_REG_INDEX(_page, _addr) \
    (((_addr) < 0x08) ? (_addr) : ((_page) * 0x08) + (_addr))

#define OHCI1394_PHY_MAP(_page, _addr, _register) \
    [OHCI1394_PHY_REG_INDEX( (_page), (_addr) )] = _register

#endif /* _OHCI1394_H_ */

/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 8
 * End:
 */
