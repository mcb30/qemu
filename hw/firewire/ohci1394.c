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

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "qemu/iov.h"
#include "ohci1394_regs.h"

/*
 * Debugging
 *
 */

#define OHCI1394_DEBUG 1

#define DBG(fmt, ...) do {					\
    if (OHCI1394_DEBUG)						\
	fprintf(stderr, "ohci1394: " fmt, ## __VA_ARGS__);	\
    } while (0)

/*
 * Device state
 *
 */

#define TYPE_OHCI1394 "ohci1394"

#define OHCI1394(obj) \
     OBJECT_CHECK(OHCI1394State, (obj), TYPE_OHCI1394)

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
    uint32_t config_rom_map;
    uint32_t config_rom_map_next; /* Shadow register */
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
    OHCI1394DmaContext async_request_tx;
    OHCI1394DmaContext async_response_tx;
    OHCI1394DmaContext async_request_rx;
    OHCI1394DmaContext async_response_rx;
    OHCI1394DmaContext isoch_tx[32];
    OHCI1394DmaContext isoch_rx[32];

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

/*
 * Interrupts
 *
 */

static void
ohci1394_set_irq(OHCI1394State *s)
{
    int intr;

    /* Update interrupt status */
    s->intr.event &= ~(OHCI1394_INTR_ISOCH_TX | OHCI1394_INTR_ISOCH_RX);
    if (s->iso_xmit_intr.event & s->iso_xmit_intr.mask)
	s->intr.event |= OHCI1394_INTR_ISOCH_TX;
    if (s->iso_recv_intr.event & s->iso_recv_intr.mask)
	s->intr.event |= OHCI1394_INTR_ISOCH_RX;
    intr = ((s->intr.mask & OHCI1394_INTR_MASTER_ENABLE) ?
	    (s->intr.event & s->intr.mask) : 0);
    DBG("intr %s (%08x:%08x,%08x:%08x,%08x:%08x)\n",
	(intr ? "asserted" : "deasserted"), s->intr.event, s->intr.mask,
	s->iso_xmit_intr.event, s->iso_xmit_intr.mask,
	s->iso_recv_intr.event, s->iso_recv_intr.mask);

    /* Report interrupt status */
    pci_set_irq(&s->pci, !!intr);
}

/*
 * Reset
 *
 */

#define OHCI1394_CONFIG_ROM_OFFSET(_field) \
    offsetof(typeof(((OHCI1394State *)NULL)->config_rom), _field)

static void
ohci1394_bus_reset(OHCI1394State *s)
{
    dma_addr_t config_rom;
    unsigned int config_rom_hdr_offset;
    unsigned int bus_options_offset;

    /* Set bus-reset initial values */
    s->self_id_count =
	((s->self_id_count + OHCI1394_SELF_ID_COUNT_GENERATION_INC) &
	 OHCI1394_SELF_ID_COUNT_GENERATION_MASK);
    s->intr.event &= ~OHCI1394_INTR_SELF_ID_COMPLETE;
    s->intr.event |= OHCI1394_INTR_BUS_RESET;

    /* Update configuration ROM if applicable */
    if (s->hc_control & OHCI1394_HC_CONTROL_BIB_IMAGE_VALID) {
	s->config_rom_map = s->config_rom_map_next;
	config_rom = (s->config_rom_map & OHCI1394_CONFIG_ROM_MAP_MASK);
	config_rom_hdr_offset = OHCI1394_CONFIG_ROM_OFFSET(config_rom_hdr);
	bus_options_offset = OHCI1394_CONFIG_ROM_OFFSET(bus_options);
	s->config_rom.config_rom_hdr =
	    ldl_le_pci_dma(&s->pci, (config_rom + config_rom_hdr_offset));
	s->config_rom.bus_options =
	    ldl_le_pci_dma(&s->pci, (config_rom + bus_options_offset));
	DBG("config ROM at 0x%08x header 0x%08x options 0x%08x\n",
	    s->config_rom_map, s->config_rom.config_rom_hdr,
	    s->config_rom.bus_options);
    }

    /* Load bus management resource registers */
    s->bus_management.bus_manager_id = OHCI1394_BUS_MANAGER_ID_DEFAULT;
    s->bus_management.bandwidth_available = s->initial_bandwidth_available;
    s->bus_management.channels_available_hi =
	( s->initial_channels_available >> 32 );
    s->bus_management.channels_available_lo =
	( s->initial_channels_available >> 0 );

    // hack self-ID
    uint32_t self_ids[] = {
	(s->self_id_count & OHCI1394_SELF_ID_COUNT_GENERATION_MASK),
	((1 << 31) | SELF_ID_PHY_ID(0) | SELF_ID_LINK_ON | SELF_ID_CONTENDER |
	 (2 << 6)),
	~self_ids[1],
	((1 << 31) | SELF_ID_PHY_ID(1) | SELF_ID_LINK_ON | SELF_ID_CONTENDER |
	 (3 << 6)),
	~self_ids[3],
    };
    s->node_id |= (OHCI1394_NODE_ID_NODE_NUMBER(1) |
		   OHCI1394_NODE_ID_ROOT |
		   OHCI1394_NODE_ID_ID_VALID);
    pci_dma_write(&s->pci, s->self_id_buffer, self_ids, sizeof(self_ids));
    s->self_id_count |= sizeof(self_ids);
    s->intr.event |= (OHCI1394_INTR_SELF_ID_COMPLETE |
		      OHCI1394_INTR_SELF_ID_COMPLETE_2);

    /* Generate interrupt */
    ohci1394_set_irq(s);
}

static void
ohci1394_soft_reset(OHCI1394State *s)
{

    /* Set soft-reset initial values */
    s->at_retries = 0;
    s->csr_control = OHCI1394_CSR_CONTROL_CSR_DONE;
    s->config_rom.config_rom_hdr = 0;
    s->hc_control &= (OHCI1394_HC_CONTROL_PROGRAM_PHY_ENABLE |
		      OHCI1394_HC_CONTROL_A_PHY_ENHANCE_ENABLE);
    s->intr.mask &= ~OHCI1394_INTR_MASTER_ENABLE;
    s->initial_bandwidth_available =
	OHCI1394_INITIAL_BANDWIDTH_AVAILABLE_DEFAULT;
    s->initial_channels_available = OHCI1394_INITIAL_CHANNELS_AVAILABLE_DEFAULT;
    s->node_id = OHCI1394_NODE_ID_BUS_NUMBER_DEFAULT;
    s->phy_control = 0;
    s->asynchronous_request_filter = 0;
    s->physical_request_filter = 0;
    s->phy_reset = OHCI1394_PHY_RESET_GAP_COUNT_DEFAULT;
    s->phy_link = OHCI1394_PHY_LINK_LCTRL;
    s->phy_misc = 0;
    s->phy_select = 0;

    /* Include effects of bus reset */
    ohci1394_bus_reset(s);
}

static void
ohci1394_hard_reset(OHCI1394State *s)
{
    union {
	uint32_t be32;
	uint8_t bytes[4];
    } guid_hi;
    union {
	uint32_t be32;
	uint8_t bytes[4];
    } guid_lo;

    /* Construct GUID from MAC address */
    memcpy(&guid_hi.bytes[0], &s->conf.macaddr.a[0], 3);
    guid_hi.bytes[3] = 0;
    guid_lo.bytes[0] = 0;
    memcpy(&guid_lo.bytes[1], &s->conf.macaddr.a[3], 3);

    /* Set hard-reset initial values */
    s->version = OHCI1394_VERSION_1_1;
    s->config_rom.config_rom_hdr = 0;
    s->config_rom.bus_id = OHCI1394_BUS_ID_1394;
    s->config_rom.bus_options = (OHCI1394_BUS_OPTIONS_MAX_REC_DEFAULT |
				 OHCI1394_BUS_OPTIONS_LINK_SPD_DEFAULT);
    s->config_rom.guid_hi = be32_to_cpu(guid_hi.be32);
    s->config_rom.guid_lo = be32_to_cpu(guid_lo.be32);
    s->config_rom_map = 0;
    s->vendor_id = 0;
    s->hc_control = 0;
    s->link_control = 0;

    /* Include effects of soft reset */
    ohci1394_soft_reset(s);
}

/*
 * Network device (FireWire-over-Ethernet) interface
 *
 */

static int
ohci1394_can_receive(NetClientState *nc)
{
    OHCI1394State *s = qemu_get_nic_opaque(nc);

    return ((s->hc_control & OHCI1394_HC_CONTROL_LINK_ENABLE) &&
	    (s->hc_control & OHCI1394_HC_CONTROL_LPS));
}

static ssize_t
ohci1394_receive_iov(NetClientState *nc, const struct iovec *iov, int iovcnt)
{

    //
    return -1;
}

static ssize_t
ohci1394_receive(NetClientState *nc, const uint8_t *buf, size_t len)
{
    const struct iovec iov = {
	.iov_base = (uint8_t *)buf,
	.iov_len = len,
    };

    return ohci1394_receive_iov(nc, &iov, 1);
}

static void
ohci1394_set_link_status(NetClientState *nc)
{
    OHCI1394State *s = qemu_get_nic_opaque(nc);

    DBG("link is %s\n", (nc->link_down ? "down" : "up"));
    s->node_id &= ~OHCI1394_NODE_ID_CPS;
    if (!nc->link_down)
	s->node_id |= OHCI1394_NODE_ID_CPS;
    ohci1394_bus_reset(s);
}

static void
ohci1394_cleanup(NetClientState *nc)
{
    OHCI1394State *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static NetClientInfo ohci1394_net_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .can_receive = ohci1394_can_receive,
    .receive_iov = ohci1394_receive_iov,
    .receive = ohci1394_receive,
    .link_status_changed = ohci1394_set_link_status,
    .cleanup = ohci1394_cleanup,
};

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

#define OHCI1394_PHY_REG(_field, _op, _notify) {		\
	.name = #_field,					\
	.offset = offsetof(OHCI1394State, _field),		\
	.op = _op,						\
	.notify = _notify,					\
    }

/*
 * Access to 8-bit PHY register
 *
 */

static inline uint8_t *
ohci1394_phy_reg(OHCI1394State *s, const OHCI1394PhyRegister *r)
{
    return (((uint8_t *)s) + r->offset);
}

/*
 * 8-bit PHY register operations 
 *
 */

static void
ohci1394_phy_reg_write(OHCI1394State *s, const OHCI1394PhyRegister *r,
		       unsigned int port, uint8_t val)
{
    uint8_t *reg = ohci1394_phy_reg(s, r);
    *reg = val;
}

static uint8_t
ohci1394_phy_reg_read(OHCI1394State *s, const OHCI1394PhyRegister *r,
		      unsigned int port)
{
    uint8_t *reg = ohci1394_phy_reg(s, r);
    return *reg;
}

static const OHCI1394PhyRegisterOp ohci1394_op_phy = {
    .write = ohci1394_phy_reg_write,
    .read = ohci1394_phy_reg_read,
};

static const OHCI1394PhyRegisterOp ohci1394_op_phy_readonly = {
    .read = ohci1394_phy_reg_read,
};

/*
 * PHY reset register
 *
 */

static void
ohci1394_phy_reset_notify(OHCI1394State *s)
{
    if (s->phy_reset & OHCI1394_PHY_RESET_IBR) {
	DBG("initiate bus reset\n");
	s->phy_reset &= ~OHCI1394_PHY_RESET_IBR;
	ohci1394_bus_reset(s);
    }
}

static const OHCI1394PhyRegister ohci1394_phy_reset =
    OHCI1394_PHY_REG(phy_reset, &ohci1394_op_phy, ohci1394_phy_reset_notify);

/*
 * PHY link register
 *
 */

static const OHCI1394PhyRegister ohci1394_phy_link =
    OHCI1394_PHY_REG(phy_link, &ohci1394_op_phy, NULL);

/*
 * PHY misc register
 *
 */

static void
ohci1394_phy_misc_notify(OHCI1394State *s)
{
    if (s->phy_misc & OHCI1394_PHY_MISC_ISBR) {
	DBG("initiate short bus reset\n");
	s->phy_misc &= ~OHCI1394_PHY_MISC_ISBR;
	ohci1394_bus_reset(s);
    }
    s->phy_misc &= ~(OHCI1394_PHY_MISC_PWR_FAIL |
		     OHCI1394_PHY_MISC_TIMEOUT |
		     OHCI1394_PHY_MISC_PORT_EVENT);
}

static const OHCI1394PhyRegister ohci1394_phy_misc =
    OHCI1394_PHY_REG(phy_misc, &ohci1394_op_phy, ohci1394_phy_misc_notify);

/*
 * PHY select register
 *
 */

static const OHCI1394PhyRegister ohci1394_phy_select =
    OHCI1394_PHY_REG(phy_select, &ohci1394_op_phy, NULL);

/*
 * PHY register map
 *
 */

#define OHCI1394_PHY_REG_INDEX(_page, _addr) \
    (((_addr) < 0x08) ? (_addr) : ((_page) * 0x08) + (_addr))

#define OHCI1394_PHY_MAP(_page, _addr, _register) \
    [OHCI1394_PHY_REG_INDEX( (_page), (_addr) )] = _register

static const OHCI1394PhyRegister *ohci1394_phy_regs[] = {
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_RESET, &ohci1394_phy_reset),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_LINK, &ohci1394_phy_link),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_MISC, &ohci1394_phy_misc),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_SELECT, &ohci1394_phy_select),
};

/*
 * Device control registers
 *
 */

typedef struct OHCI1394RegisterOp OHCI1394RegisterOp;

typedef struct OHCI1394Register {
    /* Name */
    const char *name;
    /* Base address within BAR */
    unsigned int base;
    /* Offset within OHCI1394State */
    unsigned int offset;
    /* Register read/write operations */
    const OHCI1394RegisterOp *op;
    /* Handle register updates */
    void (*notify) (OHCI1394State *s);
} OHCI1394Register;

struct OHCI1394RegisterOp {
    const char **write_names;
    void (*write) (OHCI1394State *s, const OHCI1394Register *r,
		   unsigned int offset, uint32_t val);
    const char **read_names;
    uint32_t (*read) (OHCI1394State *s, const OHCI1394Register *r,
		      unsigned int offset);
};

#define OHCI1394_REG(_base, _field, _op, _notify) {		\
	.name = #_field,					\
	.base = _base,						\
	.offset = offsetof(OHCI1394State, _field),		\
	.op = _op,						\
	.notify = _notify,					\
    }

#define OHCI1394_REG_NAME(_offset) ((_offset) >> 2)

/*
 * Access to 32-bit portion of (possibly 64-bit) device control register
 *
 */

#ifdef HOST_WORDS_BIGENDIAN
#define OHCI1394_OFFSET_HI32 0
#define OHCI1394_OFFSET_LO32 4
#else
#define OHCI1394_OFFSET_HI32 4
#define OHCI1394_OFFSET_LO32 0
#endif

static inline uint32_t *
ohci1394_reg32(OHCI1394State *s, const OHCI1394Register *r, bool high)
{
    return ((uint32_t *)(((uint8_t *)s) + r->offset +
			 (high ? OHCI1394_OFFSET_HI32 : OHCI1394_OFFSET_LO32)));
}

/*
 * 32-bit device control register operations 
 *
 */

static void
ohci1394_reg32_write(OHCI1394State *s, const OHCI1394Register *r,
		     unsigned int offset, uint32_t val)
{
    uint32_t *reg = ohci1394_reg32(s, r, 0);
    *reg = val;
}

static uint32_t
ohci1394_reg32_read(OHCI1394State *s, const OHCI1394Register *r,
		    unsigned int offset)
{
    uint32_t *reg = ohci1394_reg32(s, r, 0);
    return *reg;
}

static const char *ohci1394_reg32_names[] = { "" };

static const OHCI1394RegisterOp ohci1394_op_reg32 = {
    .write_names = ohci1394_reg32_names,
    .write = ohci1394_reg32_write,
    .read_names = ohci1394_reg32_names,
    .read = ohci1394_reg32_read,
};

static const OHCI1394RegisterOp ohci1394_op_reg32_readonly = {
    .read_names = ohci1394_reg32_names,
    .read = ohci1394_reg32_read,
};

/*
 * 64-bit high/low device control register operations
 *
 */

static void
ohci1394_hilo_write(OHCI1394State *s, const OHCI1394Register *r,
		    unsigned int offset, uint32_t val)
{
    uint32_t *reg = ohci1394_reg32(s, r, !offset);
    *reg = val;
}

static uint32_t
ohci1394_hilo_read(OHCI1394State *s, const OHCI1394Register *r,
		   unsigned int offset)
{
    uint32_t *reg = ohci1394_reg32(s, r, !offset);
    return *reg;
}

static const char *ohci1394_hilo_names[] = { ".hi", ".lo" };

static const OHCI1394RegisterOp ohci1394_op_hilo = {
    .write_names = ohci1394_hilo_names,
    .write = ohci1394_hilo_write,
    .read_names = ohci1394_hilo_names,
    .read = ohci1394_hilo_read,
};

static const OHCI1394RegisterOp ohci1394_op_hilo_readonly = {
    .read_names = ohci1394_hilo_names,
    .read = ohci1394_hilo_read,
};

/*
 * Set/clear device control register operations
 *
 */

static void
ohci1394_setclear_write(OHCI1394State *s, const OHCI1394Register *r,
			unsigned int offset, uint32_t val)
{
    uint32_t *reg = ohci1394_reg32(s, r, 0);

    if (offset & OHCI1394_OFFSET_CLEAR) {
	*reg &= ~val;
    } else {
	*reg |= val;
    }
}

static uint32_t
ohci1394_setclear_read(OHCI1394State *s, const OHCI1394Register *r,
		       unsigned int offset)
{
    uint32_t *reg = ohci1394_reg32(s, r, 0);
    return *reg;
}

static const char *ohci1394_setclear_write_names[] = { ".set", ".clear" };

static const char *ohci1394_setclear_read_names[] = { "", "" };

static const OHCI1394RegisterOp ohci1394_op_setclear = {
    .write_names = ohci1394_setclear_write_names,
    .write = ohci1394_setclear_write,
    .read_names = ohci1394_setclear_read_names,
    .read = ohci1394_setclear_read,
};

/*
 * 64-bit high/low set/clear device control register operations
 *
 */

static void
ohci1394_hilo_setclear_write(OHCI1394State *s, const OHCI1394Register *r,
			     unsigned int offset, uint32_t val)
{
    uint32_t *reg = ohci1394_reg32(s, r, !(offset & ~OHCI1394_OFFSET_CLEAR));

    if (offset & OHCI1394_OFFSET_CLEAR) {
	*reg &= ~val;
    } else {
	*reg |= val;
    }
}

static uint32_t
ohci1394_hilo_setclear_read(OHCI1394State *s, const OHCI1394Register *r,
			    unsigned int offset)
{
    uint32_t *reg = ohci1394_reg32(s, r, !(offset & ~OHCI1394_OFFSET_CLEAR));
    return *reg;
}

static const char *ohci1394_hilo_setclear_write_names[] =
    { ".hi.set", ".hi.clear", ".lo.set", ".lo.clear" };

static const char *ohci1394_hilo_setclear_read_names[] =
    { ".hi", ".hi", ".lo", ".lo" };

static const OHCI1394RegisterOp ohci1394_op_hilo_setclear = {
    .write_names = ohci1394_hilo_setclear_write_names,
    .write = ohci1394_hilo_setclear_write,
    .read_names = ohci1394_hilo_setclear_read_names,
    .read = ohci1394_hilo_setclear_read,
};

/*
 * Event/mask device control register operations
 *
 */

static inline OHCI1394EventMask *
ohci1394_eventmask(OHCI1394State *s, const OHCI1394Register *r) {
    return ((OHCI1394EventMask *)(((uint8_t *)s) + r->offset));
}

static void
ohci1394_eventmask_write(OHCI1394State *s, const OHCI1394Register *r,
			 unsigned int offset, uint32_t val)
{
    OHCI1394EventMask *reg = ohci1394_eventmask(s, r);

    if (offset & OHCI1394_OFFSET_MASK) {
	if (offset & OHCI1394_OFFSET_CLEAR) {
	    reg->mask &= ~val;
	} else {
	    reg->mask |= val;
	}
    } else {
	if (offset & OHCI1394_OFFSET_CLEAR) {
	    reg->event &= ~val;
	} else {
	    reg->event |= val;
	}
    }
}

static uint32_t
ohci1394_eventmask_read(OHCI1394State *s, const OHCI1394Register *r,
			unsigned int offset)
{
    OHCI1394EventMask *reg = ohci1394_eventmask(s, r);

    if (offset & OHCI1394_OFFSET_MASK) {
	return reg->mask;
    } else {
	if (offset & OHCI1394_OFFSET_MASKED) {
	    return (reg->event & reg->mask);
	} else {
	    return reg->event;
	}
    }
}

static const char *ohci1394_eventmask_write_names[] =
    { ".event.set", ".event.clear", ".mask.set", ".mask.clear" };

static const char *ohci1394_eventmask_read_names[] =
    { ".event", ".event.masked", ".mask", ".mask" };

static const OHCI1394RegisterOp ohci1394_op_eventmask = {
    .write_names = ohci1394_eventmask_write_names,
    .write = ohci1394_eventmask_write,
    .read_names = ohci1394_eventmask_read_names,
    .read = ohci1394_eventmask_read,
};

/*
 * Version register
 *
 */

static const OHCI1394Register ohci1394_version =
    OHCI1394_REG(OHCI1394_VERSION, version, &ohci1394_op_reg32_readonly, NULL);

/*
 * ATRetries register
 *
 */

static const OHCI1394Register ohci1394_at_retries =
    OHCI1394_REG(OHCI1394_AT_RETRIES, at_retries, &ohci1394_op_reg32, NULL);

/*
 * CSRReadData / CSRWriteData / CSRCompareData / CSRControl registers
 *
 */

static void
ohci1394_csr_control_notify(OHCI1394State *s)
{
    unsigned int csr_sel = OHCI1394_CSR_CONTROL_CSR_SEL(s->csr_control);
    uint32_t *bus_reg = &s->bus_management.numbered[csr_sel];

    s->csr_data = atomic_cmpxchg(bus_reg, s->csr_compare_data, s->csr_data);
    s->csr_control |= OHCI1394_CSR_CONTROL_CSR_DONE;
}

static const OHCI1394Register ohci1394_csr_data =
    OHCI1394_REG(OHCI1394_CSR_DATA, csr_data,
		 &ohci1394_op_reg32, NULL);

static const OHCI1394Register ohci1394_csr_compare_data =
    OHCI1394_REG(OHCI1394_CSR_COMPARE_DATA, csr_compare_data,
		 &ohci1394_op_reg32, NULL);

static const OHCI1394Register ohci1394_csr_control =
    OHCI1394_REG(OHCI1394_CSR_CONTROL, csr_control,
		 &ohci1394_op_reg32, ohci1394_csr_control_notify);

/*
 * ConfigROMhdr register
 *
 */

static const OHCI1394Register ohci1394_config_rom_hdr =
    OHCI1394_REG(OHCI1394_CONFIG_ROM_HDR, config_rom.config_rom_hdr,
		 &ohci1394_op_reg32, NULL);

/*
 * BusID register
 *
 */

static const OHCI1394Register ohci1394_bus_id =
    OHCI1394_REG(OHCI1394_BUS_ID, config_rom.bus_id,
		 &ohci1394_op_reg32_readonly, NULL);

/*
 * BusOptions register
 *
 */

static const OHCI1394Register ohci1394_bus_options =
    OHCI1394_REG(OHCI1394_BUS_OPTIONS, config_rom.bus_options,
		 &ohci1394_op_reg32, NULL);

/*
 * GUID register
 *
 */

static const OHCI1394Register ohci1394_guid_hi =
    OHCI1394_REG(OHCI1394_GUID_HI, config_rom.guid_hi,
		 &ohci1394_op_reg32_readonly, NULL);

static const OHCI1394Register ohci1394_guid_lo =
    OHCI1394_REG(OHCI1394_GUID_LO, config_rom.guid_lo,
		 &ohci1394_op_reg32_readonly, NULL);

/*
 * ConfigROMmap register
 *
 */

static void
ohci1394_config_rom_map_write(OHCI1394State *s, const OHCI1394Register *r,
			      unsigned int offset, uint32_t val)
{
    /* Store in shadow register as per spec */
    s->config_rom_map_next = val;
}

static const OHCI1394RegisterOp ohci1394_op_config_rom_map = {
    .write_names = ohci1394_reg32_names,
    .write = ohci1394_config_rom_map_write,
    .read_names = ohci1394_reg32_names,
    .read = ohci1394_reg32_read,
};

static const OHCI1394Register ohci1394_config_rom_map =
    OHCI1394_REG(OHCI1394_CONFIG_ROM_MAP, config_rom_map,
		 &ohci1394_op_config_rom_map, NULL);

/*
 * VendorID register
 *
 */

static const OHCI1394Register ohci1394_vendor_id =
    OHCI1394_REG(OHCI1394_VENDOR_ID, vendor_id,
		 &ohci1394_op_reg32_readonly, NULL);

/*
 * HCControl register
 *
 */

static void
ohci1394_hc_control_notify(OHCI1394State *s)
{
    if (s->hc_control & OHCI1394_HC_CONTROL_SOFT_RESET) {
	DBG("soft reset\n");
	ohci1394_soft_reset(s);
    }
}

static const OHCI1394Register ohci1394_hc_control =
    OHCI1394_REG(OHCI1394_HC_CONTROL, hc_control,
		 &ohci1394_op_setclear, ohci1394_hc_control_notify);

/*
 * SelfIDBuffer / SelfIDCount registers
 *
 */

static const OHCI1394Register ohci1394_self_id_buffer =
    OHCI1394_REG(OHCI1394_SELF_ID_BUFFER, self_id_buffer,
		 &ohci1394_op_reg32, NULL);

static const OHCI1394Register ohci1394_self_id_count =
    OHCI1394_REG(OHCI1394_SELF_ID_COUNT, self_id_count,
		 &ohci1394_op_reg32_readonly, NULL);

/*
 * IRMultiChanMask register
 *
 */

static const OHCI1394Register ohci1394_ir_multi_chan_mask =
    OHCI1394_REG(OHCI1394_IR_MULTI_CHAN_MASK, ir_multi_chan_mask,
		 &ohci1394_op_hilo_setclear, NULL);

/*
 * {Int,IsoXmitInt,IsoRecvInt}{Event,Mask} registers
 *
 */

static const OHCI1394Register ohci1394_intr =
    OHCI1394_REG(OHCI1394_INTR, intr,
		 &ohci1394_op_eventmask, ohci1394_set_irq);

static const OHCI1394Register ohci1394_iso_xmit_intr =
    OHCI1394_REG(OHCI1394_ISO_XMIT_INTR, iso_xmit_intr,
		 &ohci1394_op_eventmask, ohci1394_set_irq);

static const OHCI1394Register ohci1394_iso_recv_intr =
    OHCI1394_REG(OHCI1394_ISO_RECV_INTR, iso_recv_intr,
		 &ohci1394_op_eventmask, ohci1394_set_irq);

/*
 * Initial availability registers
 *
 */

static const OHCI1394Register ohci1394_initial_bandwidth_available =
    OHCI1394_REG(OHCI1394_INITIAL_BANDWIDTH_AVAILABLE,
		 initial_bandwidth_available,
		 &ohci1394_op_reg32, NULL);

static const OHCI1394Register ohci1394_initial_channels_available =
    OHCI1394_REG(OHCI1394_INITIAL_CHANNELS_AVAILABLE,
		 initial_channels_available,
		 &ohci1394_op_hilo, NULL);

/*
 * FairnessControl register
 *
 */

static const OHCI1394Register ohci1394_fairness_control =
    OHCI1394_REG(OHCI1394_FAIRNESS_CONTROL, fairness_control,
		 &ohci1394_op_reg32, NULL);

/*
 * LinkControl register
 *
 */

static const OHCI1394Register ohci1394_link_control =
    OHCI1394_REG(OHCI1394_LINK_CONTROL, link_control,
		 &ohci1394_op_setclear, NULL);

/*
 * NodeID register
 *
 */

static const OHCI1394Register ohci1394_node_id =
    OHCI1394_REG(OHCI1394_NODE_ID, node_id, &ohci1394_op_reg32, NULL);

/*
 * PhyControl register
 *
 */

static void
ohci1394_phy_write(OHCI1394State *s, unsigned int addr, uint8_t val)
{
    unsigned int page = OHCI1394_PHY_SELECT_PAGE(s->phy_select);
    unsigned int port = OHCI1394_PHY_SELECT_PORT(s->phy_select);
    unsigned int index = OHCI1394_PHY_REG_INDEX(page, addr);
    const OHCI1394PhyRegister *r;

    if ((index < ARRAY_SIZE(ohci1394_phy_regs)) &&
	(r = ohci1394_phy_regs[index])) {
	if (r->op->write) {
	    DBG("P0x%x(%x/%x/%x) <= 0x%02x %s\n",
		addr, page, port, index, val, r->name);
	    r->op->write(s, r, port, val);
	    if (r->notify)
		r->notify(s);
	} else {
	    DBG("P0x%x(%x/%x/%x) <= 0x%02x %s READ-ONLY\n",
		addr, page, port, index, val, r->name);
	}
    } else {
	DBG("P0x%x(%x/%x/%x) <= 0x%02x UNKNOWN\n",
	    addr, page, port, index, val);
    }
}

static uint8_t
ohci1394_phy_read(OHCI1394State *s, unsigned int addr)
{
    unsigned int page = OHCI1394_PHY_SELECT_PAGE(s->phy_select);
    unsigned int port = OHCI1394_PHY_SELECT_PORT(s->phy_select);
    unsigned int index = OHCI1394_PHY_REG_INDEX(page, addr);
    const OHCI1394PhyRegister *r;
    uint8_t val;

    if ((index < ARRAY_SIZE(ohci1394_phy_regs)) &&
	(r = ohci1394_phy_regs[index])) {
	val = r->op->read(s, r, port);
	DBG("P0x%x(%x/%x/%x) => 0x%02x %s\n",
	    addr, page, port, index, val, r->name);
    } else {
	val = 0;
	DBG("P0x%x(%x/%x/%x) => 0x%02x UNKNOWN\n",
	    addr, page, port, index, val);
    }
    return val;
}

static void
ohci1394_phy_control_notify(OHCI1394State *s)
{
    unsigned int addr = OHCI1394_PHY_CONTROL_REG_ADDR(s->phy_control);
    unsigned int val = OHCI1394_PHY_CONTROL_WR_DATA(s->phy_control);

    if (s->phy_control & OHCI1394_PHY_CONTROL_WR_REG) {
	ohci1394_phy_write(s, addr, val);
	s->phy_control &= ~OHCI1394_PHY_CONTROL_WR_REG;
    }
    if (s->phy_control & OHCI1394_PHY_CONTROL_RD_REG) {
	val = ohci1394_phy_read(s, addr);
	s->phy_control &= ~(OHCI1394_PHY_CONTROL_RD_REG |
			    OHCI1394_PHY_CONTROL_RD_DATA_MASK |
			    OHCI1394_PHY_CONTROL_RD_ADDR_MASK);
	s->phy_control |= (OHCI1394_PHY_CONTROL_RD_DATA(val) |
			   OHCI1394_PHY_CONTROL_RD_ADDR(addr) |
			   OHCI1394_PHY_CONTROL_RD_DONE);
    }
}

static const OHCI1394Register ohci1394_phy_control =
    OHCI1394_REG(OHCI1394_PHY_CONTROL, phy_control,
		 &ohci1394_op_reg32, ohci1394_phy_control_notify);

/*
 * Isochronous Cycle Timer register
 *
 */

static const OHCI1394Register ohci1394_isochronous_cycle_timer =
    OHCI1394_REG(OHCI1394_ISOCHRONOUS_CYCLE_TIMER, isochronous_cycle_timer,
		 &ohci1394_op_reg32, NULL);

/*
 * AsynchronousRequestFilter / PhysicalRequestFilter registers
 *
 */

static const OHCI1394Register ohci1394_asynchronous_request_filter =
    OHCI1394_REG(OHCI1394_ASYNCHRONOUS_REQUEST_FILTER,
		 asynchronous_request_filter,
		 &ohci1394_op_hilo_setclear, NULL);

static const OHCI1394Register ohci1394_physical_request_filter =
    OHCI1394_REG(OHCI1394_PHYSICAL_REQUEST_FILTER,
		 physical_request_filter,
		 &ohci1394_op_hilo_setclear, NULL);

/*
 * PhysicalUpperBound register
 *
 */

static const OHCI1394Register ohci1394_physical_upper_bound =
    OHCI1394_REG(OHCI1394_PHYSICAL_UPPER_BOUND, physical_upper_bound,
		 &ohci1394_op_reg32, NULL);

/*
 * Device control register map
 *
 */

#define OHCI1394_REG_INDEX(_bar_offset) ((_bar_offset) >> 2)

#define OHCI1394_MAP1(_bar_offset, _register)			\
    [OHCI1394_REG_INDEX(_bar_offset) + 0] = _register

#define OHCI1394_MAP2(_bar_offset, _register)			\
    OHCI1394_MAP1(((_bar_offset) + 0x0), _register),		\
    OHCI1394_MAP1(((_bar_offset) + 0x4), _register)

#define OHCI1394_MAP4(_bar_offset, _register)			\
    OHCI1394_MAP2(((_bar_offset) + 0x0), _register),		\
    OHCI1394_MAP2(((_bar_offset) + 0x8), _register)

static const OHCI1394Register *ohci1394_regs[] = {
    OHCI1394_MAP1(OHCI1394_VERSION, &ohci1394_version),
    OHCI1394_MAP1(OHCI1394_AT_RETRIES, &ohci1394_at_retries),
    OHCI1394_MAP1(OHCI1394_CSR_DATA, &ohci1394_csr_data),
    OHCI1394_MAP1(OHCI1394_CSR_COMPARE_DATA, &ohci1394_csr_compare_data),
    OHCI1394_MAP1(OHCI1394_CSR_CONTROL, &ohci1394_csr_control),
    OHCI1394_MAP1(OHCI1394_CONFIG_ROM_HDR, &ohci1394_config_rom_hdr),
    OHCI1394_MAP1(OHCI1394_BUS_ID, &ohci1394_bus_id),
    OHCI1394_MAP1(OHCI1394_BUS_OPTIONS, &ohci1394_bus_options),
    OHCI1394_MAP1(OHCI1394_GUID_HI, &ohci1394_guid_hi),
    OHCI1394_MAP1(OHCI1394_GUID_LO, &ohci1394_guid_lo),
    OHCI1394_MAP1(OHCI1394_CONFIG_ROM_MAP, &ohci1394_config_rom_map),
    OHCI1394_MAP1(OHCI1394_VENDOR_ID, &ohci1394_vendor_id),
    OHCI1394_MAP2(OHCI1394_HC_CONTROL, &ohci1394_hc_control),
    OHCI1394_MAP1(OHCI1394_SELF_ID_BUFFER, &ohci1394_self_id_buffer),
    OHCI1394_MAP1(OHCI1394_SELF_ID_COUNT, &ohci1394_self_id_count),
    OHCI1394_MAP4(OHCI1394_IR_MULTI_CHAN_MASK, &ohci1394_ir_multi_chan_mask),
    OHCI1394_MAP4(OHCI1394_INTR, &ohci1394_intr),
    OHCI1394_MAP4(OHCI1394_ISO_XMIT_INTR, &ohci1394_iso_xmit_intr),
    OHCI1394_MAP4(OHCI1394_ISO_RECV_INTR, &ohci1394_iso_recv_intr),
    OHCI1394_MAP1(OHCI1394_INITIAL_BANDWIDTH_AVAILABLE,
		  &ohci1394_initial_bandwidth_available),
    OHCI1394_MAP2(OHCI1394_INITIAL_CHANNELS_AVAILABLE,
		  &ohci1394_initial_channels_available),
    OHCI1394_MAP1(OHCI1394_FAIRNESS_CONTROL, &ohci1394_fairness_control),
    OHCI1394_MAP2(OHCI1394_LINK_CONTROL, &ohci1394_link_control),
    OHCI1394_MAP1(OHCI1394_NODE_ID, &ohci1394_node_id),
    OHCI1394_MAP1(OHCI1394_PHY_CONTROL, &ohci1394_phy_control),
    OHCI1394_MAP1(OHCI1394_ISOCHRONOUS_CYCLE_TIMER,
		  &ohci1394_isochronous_cycle_timer),
    OHCI1394_MAP4(OHCI1394_ASYNCHRONOUS_REQUEST_FILTER,
		  &ohci1394_asynchronous_request_filter),
    OHCI1394_MAP4(OHCI1394_PHYSICAL_REQUEST_FILTER,
		  &ohci1394_physical_request_filter),
    OHCI1394_MAP1(OHCI1394_PHYSICAL_UPPER_BOUND,
		  &ohci1394_physical_upper_bound),
};

/*
 * Device DMA context registers
 *
 */

typedef struct OHCI1394DmaContextRegister {
    /* Name */
    const char *name;
    /* Base address within BAR */
    unsigned int base;
    /* Instance shift */
    unsigned int shift;
    /* Instance mask */
    unsigned int mask;
    /* Offset within OHCI1394State */
    unsigned int offset;
} OHCI1394DmaContextRegister;

#define OHCI1394_DMA_REG(_base, _field) {			\
	.name = #_field,					\
	.base = _base,						\
	.shift = 4,						\
	.mask = 1,						\
	.offset = offsetof(OHCI1394State, _field),		\
    }

#define OHCI1394_DMA_ARRAY_REG(_base, _shift, _field) {		\
	.name = #_field,					\
	.base = _base,						\
	.shift = _shift,					\
	.mask = (ARRAY_SIZE(((OHCI1394State *)NULL)->_field)-1),\
	.offset = offsetof(OHCI1394State, _field),		\
    }

/*
 * Asynchronous DMA context registers
 *
 */

static const OHCI1394DmaContextRegister ohci1394_async_request_tx =
    OHCI1394_DMA_REG(OHCI1394_ASYNC_REQUEST_TX, async_request_tx);

static const OHCI1394DmaContextRegister ohci1394_async_response_tx =
    OHCI1394_DMA_REG(OHCI1394_ASYNC_RESPONSE_TX, async_response_tx);

static const OHCI1394DmaContextRegister ohci1394_async_request_rx =
    OHCI1394_DMA_REG(OHCI1394_ASYNC_REQUEST_RX, async_request_rx);

static const OHCI1394DmaContextRegister ohci1394_async_response_rx =
    OHCI1394_DMA_REG(OHCI1394_ASYNC_RESPONSE_RX, async_response_rx);

/*
 * Isochronous transmit DMA context registers
 *
 */

static const OHCI1394DmaContextRegister ohci1394_isoch_tx =
    OHCI1394_DMA_ARRAY_REG(OHCI1394_ISOCH_TX, 4, isoch_tx);

/*
 * Isochronous receive DMA context registers
 *
 */

static const OHCI1394DmaContextRegister ohci1394_isoch_rx =
    OHCI1394_DMA_ARRAY_REG(OHCI1394_ISOCH_RX, 5, isoch_rx);

/*
 * Device DMA context register map
 *
 */

#define OHCI1394_DMA_REG_INDEX(_bar_offset)			\
    (((_bar_offset) - OHCI1394_DMA_CONTEXT) >> 4)

#define OHCI1394_DMA_MAP1(_bar_offset, _register)		\
    [OHCI1394_DMA_REG_INDEX(_bar_offset) + 0] = _register

#define OHCI1394_DMA_MAP2(_bar_offset, _register)		\
    OHCI1394_DMA_MAP1(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP1(((_bar_offset) + 0x010), _register)

#define OHCI1394_DMA_MAP4(_bar_offset, _register)		\
    OHCI1394_DMA_MAP2(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP2(((_bar_offset) + 0x020), _register)

#define OHCI1394_DMA_MAP8(_bar_offset, _register)		\
    OHCI1394_DMA_MAP4(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP4(((_bar_offset) + 0x040), _register)

#define OHCI1394_DMA_MAP16(_bar_offset, _register)		\
    OHCI1394_DMA_MAP8(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP8(((_bar_offset) + 0x080), _register)

#define OHCI1394_DMA_MAP32(_bar_offset, _register)		\
    OHCI1394_DMA_MAP16(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP16(((_bar_offset) + 0x100), _register)

#define OHCI1394_DMA_MAP64(_bar_offset, _register)		\
    OHCI1394_DMA_MAP32(((_bar_offset) + 0x000), _register),	\
    OHCI1394_DMA_MAP32(((_bar_offset) + 0x200), _register)

static const OHCI1394DmaContextRegister *ohci1394_dma_regs[] = {
    OHCI1394_DMA_MAP1(OHCI1394_ASYNC_REQUEST_TX, &ohci1394_async_request_tx),
    OHCI1394_DMA_MAP1(OHCI1394_ASYNC_RESPONSE_TX, &ohci1394_async_response_tx),
    OHCI1394_DMA_MAP1(OHCI1394_ASYNC_REQUEST_RX, &ohci1394_async_request_rx),
    OHCI1394_DMA_MAP1(OHCI1394_ASYNC_RESPONSE_RX, &ohci1394_async_response_rx),
    OHCI1394_DMA_MAP32(OHCI1394_ISOCH_TX, &ohci1394_isoch_tx),
    OHCI1394_DMA_MAP64(OHCI1394_ISOCH_RX, &ohci1394_isoch_rx),
};

/*
 * MMIO operations
 *
 */

static void
ohci1394_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size)
{
    OHCI1394State *s = opaque;
    unsigned int bar_offset = (addr & OHCI1394_BAR_MASK);
    unsigned int index;
    unsigned int offset;
    unsigned int instance;
    unsigned int name;
    const OHCI1394Register *r;
    const OHCI1394DmaContextRegister *cr;

    if (bar_offset < OHCI1394_DMA_CONTEXT) {
	index = OHCI1394_REG_INDEX(bar_offset);
	if ((index < ARRAY_SIZE(ohci1394_regs)) && (r = ohci1394_regs[index])) {
	    offset = (bar_offset - r->base);
	    name = OHCI1394_REG_NAME(offset);
	    if (r->op->write) {
		DBG("0x%03x <= 0x%08"PRIx64" %s%s\n",
		    bar_offset, val, r->name, r->op->write_names[name]);
		r->op->write(s, r, offset, val);
		if ( r->notify )
		    r->notify(s);
	    } else {
		DBG("0x%03x <= 0x%08"PRIx64" %s%s READ-ONLY\n",
		    bar_offset, val, r->name, r->op->read_names[name]);
	    }
	} else {
	    DBG("0x%03x <= 0x%08"PRIx64" UNKNOWN\n", offset, val);
	}
    } else {
	index = OHCI1394_DMA_REG_INDEX(bar_offset);
	if ((index < ARRAY_SIZE(ohci1394_dma_regs)) &&
	    (cr = ohci1394_dma_regs[index])) {
	    offset = (bar_offset - cr->base);
	    instance = ((offset >> cr->shift) & cr->mask);
	    offset &= ~(-1UL << cr->shift);
	    DBG("0x%03x <= 0x%08"PRIx64" %s[%x].%x\n",
		bar_offset, val, cr->name, instance, offset);
	} else {
	    DBG("0x%03x <= 0x%08"PRIx64" DMA UNKNOWN\n", offset, val);
	}
    }
}

static uint64_t
ohci1394_mmio_read(void *opaque, hwaddr addr, unsigned int size)
{
    OHCI1394State *s = opaque;
    unsigned int bar_offset = (addr & OHCI1394_BAR_MASK);
    unsigned int index;
    unsigned int offset;
    unsigned int instance;
    unsigned int name;
    const OHCI1394Register *r;
    const OHCI1394DmaContextRegister *cr;
    uint32_t val;

    if (bar_offset < OHCI1394_DMA_CONTEXT) {
	index = OHCI1394_REG_INDEX(bar_offset);
	if ((index < ARRAY_SIZE(ohci1394_regs)) && (r = ohci1394_regs[index])) {
	    offset = (bar_offset - r->base);
	    name = OHCI1394_REG_NAME(offset);
	    val = r->op->read(s, r, offset);
	    DBG("0x%03x => 0x%08x %s%s\n",
		bar_offset, val, r->name, r->op->read_names[name]);
	} else {
	    val = -1U;
	    DBG("0x%03x => 0x%08x UNKNOWN\n", bar_offset, val);
	}
    } else {
	index = OHCI1394_DMA_REG_INDEX(bar_offset);
	if ((index < ARRAY_SIZE(ohci1394_dma_regs)) &&
	    (cr = ohci1394_dma_regs[index])) {
	    offset = (bar_offset - cr->base);
	    instance = ((offset >> cr->shift) & cr->mask);
	    offset &= ~(-1UL << cr->shift);
	    
	    //
	    val = 0;

	    DBG("0x%03x => 0x%08x %s[%x].%x\n",
		bar_offset, val, cr->name, instance, offset);
	} else {
	    val = -1U;
	    DBG("0x%03x => 0x%08x DMA UNKNOWN\n", offset, val);
	}
    }
    return val;
}

static const MemoryRegionOps ohci1394_mmio_ops = {
    .read = ohci1394_mmio_read,
    .write = ohci1394_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Initialisation
 *
 */

static int
pci_ohci1394_init(PCIDevice *dev)
{
    OHCI1394State *s = OHCI1394(dev);
    DeviceState *d = DEVICE(dev);
    uint8_t *pci_conf;

    /* Fill in PCI configuration space */
    pci_conf = dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */
    pci_conf[PCI_CLASS_PROG] = OHCI1394_PROG_IF; /* OHCI */
    memory_region_init_io(&s->bar_mem, OBJECT(s), &ohci1394_mmio_ops, s,
			  "ohci1394", OHCI1394_BAR_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar_mem);

    /* Register network device */
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ohci1394_net_info, &s->conf,
			  object_get_typename(OBJECT(s)), d->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    return 0;
}

static void
pci_ohci1394_uninit(PCIDevice *dev)
{
    OHCI1394State *s = OHCI1394(dev);

    printf("*** OHCI1394 uninit %p\n", s);
}

static void
ohci1394_reset(DeviceState *d)
{
    OHCI1394State *s = OHCI1394(d);
    ohci1394_hard_reset(s);
    ohci1394_set_link_status(qemu_get_queue(s->nic));
};

static const VMStateDescription vmstate_ohci1394_eventmask = {
    .name = "ohci1394_eventmask",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
	VMSTATE_UINT32(event, OHCI1394EventMask),
	VMSTATE_UINT32(mask, OHCI1394EventMask),
	VMSTATE_END_OF_LIST()
    },
};

#define VMSTATE_OHCI1394_EVENTMASK(_field, _state)			\
    VMSTATE_STRUCT(_field, _state, 1, vmstate_ohci1394_eventmask,	\
		   OHCI1394EventMask)

static const VMStateDescription vmstate_ohci1394_dmacontext = {
    .name = "ohci1394_dmacontext",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
	VMSTATE_UINT32(context_control, OHCI1394DmaContext),
	VMSTATE_UINT32(command_ptr, OHCI1394DmaContext),
	VMSTATE_UINT32(context_match, OHCI1394DmaContext),
	VMSTATE_END_OF_LIST()
    },
};

#define VMSTATE_OHCI1394_DMACONTEXT(_field, _state)			\
    VMSTATE_STRUCT(_field, _state, 1, vmstate_ohci1394_dmacontext,	\
		   OHCI1394DmaContext)

#define VMSTATE_OHCI1394_DMACONTEXT_ARRAY(_field, _state, _num)		\
    VMSTATE_STRUCT_ARRAY(_field, _state, _num, 1,			\
			 vmstate_ohci1394_dmacontext, OHCI1394DmaContext)

static const VMStateDescription vmstate_ohci1394 = {
    .name = "ohci1394",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
	VMSTATE_PCI_DEVICE(pci, OHCI1394State),
	VMSTATE_MACADDR(conf.macaddr, OHCI1394State),
	VMSTATE_UINT8(phy_reset, OHCI1394State),
	VMSTATE_UINT8(phy_link, OHCI1394State),
	VMSTATE_UINT8(phy_misc, OHCI1394State),
	VMSTATE_UINT8(phy_select, OHCI1394State),
	VMSTATE_UINT32(version, OHCI1394State),
	VMSTATE_UINT32(at_retries, OHCI1394State),
	VMSTATE_UINT32(csr_data, OHCI1394State),
	VMSTATE_UINT32(csr_compare_data, OHCI1394State),
	VMSTATE_UINT32(csr_control, OHCI1394State),
	VMSTATE_UINT32(config_rom.config_rom_hdr, OHCI1394State),
	VMSTATE_UINT32(config_rom.bus_id, OHCI1394State),
	VMSTATE_UINT32(config_rom.bus_options, OHCI1394State),
	VMSTATE_UINT32(config_rom.guid_hi, OHCI1394State),
	VMSTATE_UINT32(config_rom.guid_lo, OHCI1394State),
	VMSTATE_UINT32(config_rom_map, OHCI1394State),
	VMSTATE_UINT32(config_rom_map_next, OHCI1394State),
	VMSTATE_UINT32(vendor_id, OHCI1394State),
	VMSTATE_UINT32(hc_control, OHCI1394State),
	VMSTATE_UINT32(self_id_buffer, OHCI1394State),
	VMSTATE_UINT32(self_id_count, OHCI1394State),
	VMSTATE_UINT64(ir_multi_chan_mask, OHCI1394State),
	VMSTATE_OHCI1394_EVENTMASK(intr, OHCI1394State),
	VMSTATE_OHCI1394_EVENTMASK(iso_xmit_intr, OHCI1394State),
	VMSTATE_OHCI1394_EVENTMASK(iso_recv_intr, OHCI1394State),
	VMSTATE_UINT32(initial_bandwidth_available, OHCI1394State),
	VMSTATE_UINT64(initial_channels_available, OHCI1394State),
	VMSTATE_UINT32(fairness_control, OHCI1394State),
	VMSTATE_UINT32(link_control, OHCI1394State),
	VMSTATE_UINT32(node_id, OHCI1394State),
	VMSTATE_UINT32(phy_control, OHCI1394State),
	VMSTATE_UINT32(isochronous_cycle_timer, OHCI1394State),
	VMSTATE_UINT64(asynchronous_request_filter, OHCI1394State),
	VMSTATE_UINT64(physical_request_filter, OHCI1394State),
	VMSTATE_UINT32(physical_upper_bound, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async_request_tx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async_response_tx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async_request_rx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async_response_rx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT_ARRAY(isoch_tx, OHCI1394State, 32),
	VMSTATE_OHCI1394_DMACONTEXT_ARRAY(isoch_rx, OHCI1394State, 32),
	VMSTATE_UINT32(bus_management.bus_manager_id, OHCI1394State),
	VMSTATE_UINT32(bus_management.bandwidth_available, OHCI1394State),
	VMSTATE_UINT32(bus_management.channels_available_hi, OHCI1394State),
	VMSTATE_UINT32(bus_management.channels_available_lo, OHCI1394State),
	VMSTATE_END_OF_LIST()
    },
};

static Property ohci1394_properties[] = {
    DEFINE_NIC_PROPERTIES(OHCI1394State, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void
ohci1394_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = pci_ohci1394_init;
    k->exit = pci_ohci1394_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = PCI_DEVICE_ID_QEMU_OHCI1394;
    k->class_id = PCI_CLASS_SERIAL_FIREWIRE;
    dc->reset = ohci1394_reset;
    dc->vmsd = &vmstate_ohci1394;
    dc->props = ohci1394_properties;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo ohci1394_info = {
    .name          = TYPE_OHCI1394,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(OHCI1394State),
    .class_init    = ohci1394_class_init,
};

static void
ohci1394_register_types(void)
{
    type_register_static(&ohci1394_info);
}

type_init(ohci1394_register_types)

/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 8
 * End:
 */
