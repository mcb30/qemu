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
#include "qemu/iov.h"
#include "ohci1394_hw.h"
#include "ohci1394.h"
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
	config_rom = s->config_rom_map.active = s->config_rom_map.shadow;
	config_rom_hdr_offset = OHCI1394_CONFIG_ROM_OFFSET(config_rom_hdr);
	bus_options_offset = OHCI1394_CONFIG_ROM_OFFSET(bus_options);
	s->config_rom.config_rom_hdr =
	    ldl_le_pci_dma(&s->pci, (config_rom + config_rom_hdr_offset));
	s->config_rom.bus_options =
	    ldl_le_pci_dma(&s->pci, (config_rom + bus_options_offset));
	DBG("config ROM at 0x%08x header 0x%08x options 0x%08x\n",
	    s->config_rom_map.active, s->config_rom.config_rom_hdr,
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
    s->node_id |= (OHCI1394_NODE_ID_NODE_NUMBER_SET(1) |
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
    unsigned int i;

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
    for (i = 0; i < ARRAY_SIZE(s->async.numbered); i++)
	s->async.numbered[i].context_control = 0;
    for (i = 0; i < ARRAY_SIZE(s->isoch_tx); i++)
	s->isoch_tx[i].context_control = 0;
    for (i = 0; i < ARRAY_SIZE(s->isoch_rx); i++)
	s->isoch_rx[i].context_control = 0;

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
    s->version = OHCI1394_VERSION_VERSION_1_1;
    s->config_rom.config_rom_hdr = 0;
    s->config_rom.bus_id = OHCI1394_BUS_ID_1394;
    s->config_rom.bus_options =
	(OHCI1394_BUS_OPTIONS_MAX_REC_SET(OHCI1394_MAX_REC) |
	 OHCI1394_BUS_OPTIONS_LINK_SPD_DEFAULT);
    s->config_rom.guid_hi = be32_to_cpu(guid_hi.be32);
    s->config_rom.guid_lo = be32_to_cpu(guid_lo.be32);
    s->config_rom_map.active = 0;
    s->config_rom_map.shadow = 0;
    s->vendor_id = 0;
    s->hc_control = 0;
    s->link_control = 0;

    /* Include effects of soft reset */
    ohci1394_soft_reset(s);
}

/*
 * DMA contexts
 *
 */

static const char *
ohci1394_dma_insn_name(unsigned int insn)
{
    static char buf[14 /* "UNKNOWN(xxxx)" + NUL */];

    switch (insn & OHCI1394_INSN_MASK) {
    case OHCI1394_INSN_OUTPUT_MORE:
	return "OUTPUT_MORE";
    case OHCI1394_INSN_OUTPUT_LAST:
	return "OUTPUT_LAST";
    case OHCI1394_INSN_OUTPUT_MORE_immediate:
	return "OUTPUT_MORE_immediate";
    case OHCI1394_INSN_OUTPUT_LAST_immediate:
	return "OUTPUT_LAST_immediate";
    case OHCI1394_INSN_INPUT_MORE:
	return "INPUT_MORE";
    case OHCI1394_INSN_INPUT_LAST:
	return "INPUT_LAST";
    case OHCI1394_INSN_STORE_VALUE:
	return "STORE_VALUE";
    default:
	snprintf(buf, sizeof(buf), "UNKNOWN(%04x)", insn);
	return buf;
    }
}

static const char *
ohci1394_dma_context_name(OHCI1394State *s, OHCI1394DmaContext *c)
{
    static char buf[18 /* "async.response_xx" + NUL */];
    static const char *async_names[] =
	{ "request_tx", "response_tx", "request_rx", "response_rx" };
    unsigned int instance;

    if ((c >= s->async.numbered) &&
	(c < &s->async.numbered[ARRAY_SIZE(s->async.numbered)])) {
	instance = (c - s->async.numbered);
	snprintf(buf, sizeof(buf), "async.%s", async_names[instance]);
    } else if ((c >= s->isoch_tx) &&
	       (c < &s->isoch_tx[ARRAY_SIZE(s->isoch_tx)])) {
	instance = (c - s->isoch_tx);
	snprintf(buf, sizeof(buf), "isoch_tx[%02x]", instance);
    } else if ((c >= s->isoch_rx) &&
	       (c < &s->isoch_rx[ARRAY_SIZE(s->isoch_rx)])) {
	instance = (c - s->isoch_rx);
	snprintf(buf, sizeof(buf), "isoch_rx[%02x]", instance);
    } else {
	snprintf(buf, sizeof(buf), "<INVALID>");
    }
    return buf;
}

static void
ohci1394_dma_die(OHCI1394State *s, OHCI1394DmaContext *c,
		 unsigned int event_code)
{
    DBG("%s died (0x%02x)\n", ohci1394_dma_context_name(s, c), event_code);
    c->context_control &= ~(OHCI1394_CONTEXT_CONTROL_ACTIVE |
			    OHCI1394_CONTEXT_CONTROL_EVENT_CODE_MASK);
    c->context_control |= (OHCI1394_CONTEXT_CONTROL_DEAD |
			   OHCI1394_CONTEXT_CONTROL_EVENT_CODE_SET(event_code));
}

static size_t
ohci1394_demangle_header(OHCI1394State *s, const OHCI1394MangledHeader *mangled,
			 OHCI1394DemangledHeader *demangled, size_t len)
{
    unsigned int count = (len / sizeof(uint32_t));
    unsigned int tcode;
    unsigned int i;

    for (i = 0; i < count; i++)
	demangled->quadlet[i] = cpu_to_be32(le32_to_cpu(mangled->quadlet[i]));

    tcode = OHCI1394_TCODE_GET(le16_to_cpu(mangled->control));
    if (tcode == TCODE_LINK_INTERNAL) {
	demangled->quadlet[0] = demangled->quadlet[1];
	demangled->quadlet[1] = demangled->quadlet[2];
	return (2 * sizeof(uint32_t));
    }

    demangled->after.first = demangled->before.first;
    demangled->after.source_id = cpu_to_be16(OHCI1394_NODE_ID_GET(s->node_id));
    if (tcode == TCODE_STREAM_DATA) {
	return (1 * sizeof(uint32_t));
    };

    if ((s->hc_control & OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA) &&
	((tcode == TCODE_WRITE_QUADLET_REQUEST) ||
	 (tcode == TCODE_READ_QUADLET_RESPONSE))) {
	bswap32s(&demangled->quadlet[3]);
    }

    return count;
}

static void
ohci1394_transmit(OHCI1394State *s, FireWireHeader *header, size_t header_len,
		  void *data, size_t len)
{
    unsigned int i;
    uint8_t *bytes = data;

    for (i = 0; i < (header_len/4); i++)
	DBG("TXH: %08x\n", be32_to_cpu(header->raw[i]));
    for (i = 0; i < len; i += 4)
	DBG("TXD: %02x%02x%02x%02x\n", bytes[i + 0], bytes[i + 1],
	    bytes[i + 2], bytes[i + 3]);
}

static void
ohci1394_dma_run(OHCI1394State *s, OHCI1394DmaContext *c)
{
    uint32_t addr = OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_GET(c->command_ptr);
    unsigned int z = OHCI1394_COMMAND_PTR_Z_GET(c->command_ptr);
    OHCI1394DmaProgram program[z];
    OHCI1394DmaProgram *p;
    OHCI1394DmaProgram *n;
    unsigned int i;
    unsigned int insn;
    uint32_t data_address;
    uint16_t req_count;
    uint16_t store_doublet;
    uint32_t branch_address;
    size_t header_len = 0;
    size_t len = 0;
    bool branch;

    c->context_control &= ~OHCI1394_CONTEXT_CONTROL_WAKE;
    pci_dma_read(&s->pci, addr, program, sizeof(program));

    for (i = 0; i < z; i++, addr += sizeof(*p)) {
	c->command_ptr = OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_SET(addr);
	p = &program[i];
	n = (((i + 1) < z) ? (p + 1) : NULL);
	insn = le16_to_cpu(p->common.insn);
	branch = (insn & OHCI1394_INSN_BRANCH_MASK);
	switch (insn & OHCI1394_INSN_MASK) {

	case OHCI1394_INSN_OUTPUT_LAST:
	    branch = true;
	case OHCI1394_INSN_OUTPUT_MORE:
	    req_count = le16_to_cpu(p->output.req_count);
	    data_address = le32_to_cpu(p->output.data_address);
	    if ((len + req_count) > sizeof(s->tx.buf)) {
		DBG("%s 0x%08x[%x/%x] %s 0x%08x+0x%04x data overrun\n",
		    ohci1394_dma_context_name(s, c), addr, i, z,
		    ohci1394_dma_insn_name(insn), data_address, req_count);
		ohci1394_dma_die(s, c, OHCI1394_EVT_DESCRIPTOR_READ);
		return;
	    }
	    DBG("%s 0x%08x[%x/%x] %s 0x%08x+0x%04x\n",
		ohci1394_dma_context_name(s, c), addr, i, z,
		ohci1394_dma_insn_name(insn), data_address, req_count);
	    pci_dma_read(&s->pci, data_address, (s->tx.buf.bytes + len),
			 req_count);
	    len += req_count;
	    break;

	case OHCI1394_INSN_OUTPUT_LAST_immediate:
	    branch = true;
	case OHCI1394_INSN_OUTPUT_MORE_immediate:
	    req_count = le16_to_cpu(p->output.req_count);
	    if ((n == NULL) || (req_count > sizeof(*n))) {
		DBG("%s 0x%08x[%x/%x] %s +0x%04x program overrun\n",
		    ohci1394_dma_context_name(s, c), addr, i, z,
		    ohci1394_dma_insn_name(insn), req_count);
		ohci1394_dma_die(s, c, OHCI1394_EVT_DESCRIPTOR_READ);
		return;
	    }
	    DBG("%s 0x%08x[%x/%x] %s +0x%04x\n",
		ohci1394_dma_context_name(s, c), addr, i, z,
		ohci1394_dma_insn_name(insn), req_count);
	    header_len = ohci1394_demangle_header(s, &n->mangled,
						  &s->tx.header, req_count);
	    i++;
	    break;

	case OHCI1394_INSN_STORE_VALUE:
	    store_doublet = le16_to_cpu(p->store.store_doublet);
	    data_address = le32_to_cpu(p->store.data_address);
	    DBG("%s 0x%08x[%x/%x] %s 0x%08x <= 0x%04x\n",
		ohci1394_dma_context_name(s, c), addr, i, z,
		ohci1394_dma_insn_name(insn), data_address, store_doublet);
	    stw_le_pci_dma(&s->pci, data_address, store_doublet);
	    break;

	default:
	    DBG("%s 0x%08x[%x/%x] %s %08x:%08x:%08x:%08x\n",
		ohci1394_dma_context_name(s, c), addr, i, z,
		ohci1394_dma_insn_name(insn), le32_to_cpu(p->raw[0]),
		le32_to_cpu(p->raw[1]), le32_to_cpu(p->raw[2]),
		le32_to_cpu(p->raw[3]));
	    ohci1394_dma_die(s, c, OHCI1394_EVT_DESCRIPTOR_READ);
	    return;
	}

	if (branch) {
	    branch_address = le32_to_cpu(p->common.branch_address);
	    DBG("%s 0x%08x[%x/%x] %s branch 0x%08x[%x]\n",
		ohci1394_dma_context_name(s, c), addr, i, z,
		ohci1394_dma_insn_name(insn),
		OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_GET(branch_address),
		OHCI1394_COMMAND_PTR_Z_GET(branch_address));
	    if (branch_address & OHCI1394_COMMAND_PTR_Z_MASK)
		c->command_ptr = branch_address;
	    break;
	}
    }

    while (len & 0x3)
	s->tx.buf.bytes[len++] = 0;

    if (!(s->hc_control & OHCI1394_HC_CONTROL_NO_BYTE_SWAP_DATA)) {
	for (i = 0; i < (len/sizeof(s->tx.buf.quadlets[0])); i++)
	    bswap32s(&s->tx.buf.quadlets[i]);
    }

    ohci1394_transmit(s, &s->tx.header.fw, header_len, s->tx.buf.bytes, len);
}

static void
ohci1394_dma_wake(OHCI1394State *s, OHCI1394DmaContext *c)
{
    uint32_t addr = OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_GET(c->command_ptr);
    OHCI1394DmaProgram program;
    OHCI1394DmaProgram *p = &program;
    uint32_t branch_address;

    pci_dma_read(&s->pci, addr, &program, sizeof(program));
    branch_address = le32_to_cpu(p->common.branch_address);
    DBG("%s 0x%08x[-/-] wake 0x%08x[%x]\n",
	ohci1394_dma_context_name(s, c), addr,
	OHCI1394_COMMAND_PTR_DESCRIPTOR_ADDRESS_GET(branch_address),
	OHCI1394_COMMAND_PTR_Z_GET(branch_address));
    if (branch_address & OHCI1394_COMMAND_PTR_Z_MASK)
	c->command_ptr = branch_address;
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
 * PHY register operations 
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
 * PHY registers
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

static const OHCI1394PhyRegister ohci1394_phy_reset =
    OHCI1394_PHY_REG(phy_reset, &ohci1394_op_phy, ohci1394_phy_reset_notify);

static const OHCI1394PhyRegister ohci1394_phy_link =
    OHCI1394_PHY_REG(phy_link, &ohci1394_op_phy, NULL);

static const OHCI1394PhyRegister ohci1394_phy_misc =
    OHCI1394_PHY_REG(phy_misc, &ohci1394_op_phy, ohci1394_phy_misc_notify);

static const OHCI1394PhyRegister ohci1394_phy_select =
    OHCI1394_PHY_REG(phy_select, &ohci1394_op_phy, NULL);

static const OHCI1394PhyRegister *ohci1394_phy_regs[] = {
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_RESET, &ohci1394_phy_reset),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_LINK, &ohci1394_phy_link),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_MISC, &ohci1394_phy_misc),
    OHCI1394_PHY_MAP(0, OHCI1394_PHY_SELECT, &ohci1394_phy_select),
};

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
	    DBG("P0x%x(%x/%x/%x) <= 0x%02x %s *** READ-ONLY ***\n",
		addr, page, port, index, val, r->name);
	}
    } else {
	DBG("P0x%x(%x/%x/%x) <= 0x%02x *** UNKNOWN ***\n",
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
	DBG("P0x%x(%x/%x/%x) => 0x%02x *** UNKNOWN ***\n",
	    addr, page, port, index, val);
    }
    return val;
}

/*
 * Device control registers
 *
 */

static void
ohci1394_csr_control_notify(OHCI1394State *s)
{
    unsigned int csr_sel = OHCI1394_CSR_CONTROL_CSR_SEL_GET(s->csr_control);
    uint32_t *bus_reg = &s->bus_management.numbered[csr_sel];

    s->csr_data = atomic_cmpxchg(bus_reg, s->csr_compare_data, s->csr_data);
    s->csr_control |= OHCI1394_CSR_CONTROL_CSR_DONE;
}

static void
ohci1394_hc_control_notify(OHCI1394State *s)
{
    if (s->hc_control & OHCI1394_HC_CONTROL_SOFT_RESET) {
	DBG("soft reset\n");
	ohci1394_soft_reset(s);
    }
}

static void
ohci1394_phy_control_notify(OHCI1394State *s)
{
    unsigned int addr = OHCI1394_PHY_CONTROL_REG_ADDR_GET(s->phy_control);
    unsigned int val = OHCI1394_PHY_CONTROL_WR_DATA_GET(s->phy_control);

    if (s->phy_control & OHCI1394_PHY_CONTROL_WR_REG) {
	ohci1394_phy_write(s, addr, val);
	s->phy_control &= ~OHCI1394_PHY_CONTROL_WR_REG;
    }
    if (s->phy_control & OHCI1394_PHY_CONTROL_RD_REG) {
	val = ohci1394_phy_read(s, addr);
	s->phy_control &= ~(OHCI1394_PHY_CONTROL_RD_REG |
			    OHCI1394_PHY_CONTROL_RD_DATA_MASK |
			    OHCI1394_PHY_CONTROL_RD_ADDR_MASK);
	s->phy_control |= (OHCI1394_PHY_CONTROL_RD_DATA_SET(val) |
			   OHCI1394_PHY_CONTROL_RD_ADDR_SET(addr) |
			   OHCI1394_PHY_CONTROL_RD_DONE);
    }
}

/*
 * DMA context registers
 *
 */

static void
ohci1394_context_control_notify(OHCI1394State *s, OHCI1394DmaContext *c)
{
    if (c->context_control & OHCI1394_CONTEXT_CONTROL_RUN) {
	if ((c->context_control & OHCI1394_CONTEXT_CONTROL_WAKE) &&
	    !(c->command_ptr & OHCI1394_COMMAND_PTR_Z_MASK)) {
	    ohci1394_dma_wake(s, c);
	}
	while (c->command_ptr & OHCI1394_COMMAND_PTR_Z_MASK)
	    ohci1394_dma_run(s, c);
    } else {
	c->context_control &= ~OHCI1394_CONTEXT_CONTROL_DEAD;
    }
}

/*
 * Register maps
 *
 */

const OHCI1394ControlRegister ohci1394_ctrl_registers[] = {
    OHCI1394_CTRL_REG(VERSION, version, reg32, NULL),
    OHCI1394_CTRL_REG(AT_RETRIES, at_retries, reg32, NULL),
    OHCI1394_CTRL_REG(CSR_DATA, csr_data, reg32, NULL),
    OHCI1394_CTRL_REG(CSR_COMPARE_DATA, csr_compare_data, reg32, NULL),
    OHCI1394_CTRL_REG(CSR_CONTROL, csr_control,
		      reg32, ohci1394_csr_control_notify),
    OHCI1394_CTRL_REG(CONFIG_ROM_HDR, config_rom.config_rom_hdr, reg32, NULL),
    OHCI1394_CTRL_REG(BUS_ID, config_rom.bus_id, reg32, NULL),
    OHCI1394_CTRL_REG(BUS_OPTIONS, config_rom.bus_options, reg32, NULL),
    OHCI1394_CTRL_REG(GUID_HI, config_rom.guid_hi, reg32, NULL),
    OHCI1394_CTRL_REG(GUID_LO, config_rom.guid_lo, reg32, NULL),
    OHCI1394_CTRL_REG(CONFIG_ROM_MAP, config_rom_map, shadowed, NULL),
    OHCI1394_CTRL_REG(VENDOR_ID, vendor_id, reg32, NULL),
    OHCI1394_CTRL_REG(HC_CONTROL, hc_control,
		      setclear, ohci1394_hc_control_notify),
    OHCI1394_CTRL_REG(SELF_ID_BUFFER, self_id_buffer, reg32, NULL),
    OHCI1394_CTRL_REG(SELF_ID_COUNT, self_id_count, reg32, NULL),
    OHCI1394_CTRL_REG(IR_MULTI_CHAN_MASK, ir_multi_chan_mask,
		      hilo_setclear, NULL),
    OHCI1394_CTRL_REG(INTR, intr, eventmask, ohci1394_set_irq),
    OHCI1394_CTRL_REG(ISO_XMIT_INTR, iso_xmit_intr,
		      eventmask, ohci1394_set_irq),
    OHCI1394_CTRL_REG(ISO_RECV_INTR, iso_recv_intr,
		      eventmask, ohci1394_set_irq),
    OHCI1394_CTRL_REG(INITIAL_BANDWIDTH_AVAILABLE, initial_bandwidth_available,
		      reg32, NULL),
    OHCI1394_CTRL_REG(INITIAL_CHANNELS_AVAILABLE, initial_channels_available,
		      hilo, NULL),
    OHCI1394_CTRL_REG(FAIRNESS_CONTROL, fairness_control, reg32, NULL),
    OHCI1394_CTRL_REG(LINK_CONTROL, link_control, setclear, NULL),
    OHCI1394_CTRL_REG(NODE_ID, node_id, reg32, NULL),
    OHCI1394_CTRL_REG(PHY_CONTROL, phy_control,
		      reg32, ohci1394_phy_control_notify),
    OHCI1394_CTRL_REG(ISOCHRONOUS_CYCLE_TIMER, isochronous_cycle_timer,
		      reg32, NULL),
    OHCI1394_CTRL_REG(ASYNCHRONOUS_REQUEST_FILTER, asynchronous_request_filter,
		      hilo_setclear, NULL),
    OHCI1394_CTRL_REG(PHYSICAL_REQUEST_FILTER, physical_request_filter,
		      hilo_setclear, NULL),
    OHCI1394_CTRL_REG(PHYSICAL_UPPER_BOUND, physical_upper_bound, reg32, NULL),
    OHCI1394_CTRL_REG_END
};

const OHCI1394DmaRegister ohci1394_async_registers[] = {
    OHCI1394_DMA_REG(CONTEXT_CONTROL, context_control,
		     setclear, ohci1394_context_control_notify),
    OHCI1394_DMA_REG(COMMAND_PTR, command_ptr, reg32, NULL),
    OHCI1394_DMA_REG_END
};

const OHCI1394DmaRegister ohci1394_isoch_tx_registers[] = {
    OHCI1394_DMA_REG(IT_CONTEXT_CONTROL, context_control, setclear, NULL),
    OHCI1394_DMA_REG(COMMAND_PTR, command_ptr, reg32, NULL),
    OHCI1394_DMA_REG_END
};

const OHCI1394DmaRegister ohci1394_isoch_rx_registers[] = {
    OHCI1394_DMA_REG(IR_CONTEXT_CONTROL, context_control, setclear, NULL),
    OHCI1394_DMA_REG(COMMAND_PTR, command_ptr, reg32, NULL),
    OHCI1394_DMA_REG(CONTEXT_MATCH, context_match, reg32, NULL),
    OHCI1394_DMA_REG_END
};

/*
 * MMIO operations
 *
 */

static void
ohci1394_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size)
{
    OHCI1394State *s = opaque;
    ohci1394_write(s, addr, val);
}

static uint64_t
ohci1394_mmio_read(void *opaque, hwaddr addr, unsigned int size)
{
    OHCI1394State *s = opaque;
    return ohci1394_read(s, addr);
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

static const VMStateDescription vmstate_ohci1394_shadowed = {
    .name = "ohci1394_shadowed",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
	VMSTATE_UINT32(active, OHCI1394Shadowed),
	VMSTATE_UINT32(shadow, OHCI1394Shadowed),
	VMSTATE_END_OF_LIST()
    },
};

#define VMSTATE_OHCI1394_SHADOWED(_field, _state)			\
    VMSTATE_STRUCT(_field, _state, 1, vmstate_ohci1394_shadowed,	\
		   OHCI1394Shadowed)

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
	VMSTATE_OHCI1394_SHADOWED(config_rom_map, OHCI1394State),
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
	VMSTATE_OHCI1394_DMACONTEXT(async.request_tx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async.response_tx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async.request_rx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT(async.response_rx, OHCI1394State),
	VMSTATE_OHCI1394_DMACONTEXT_ARRAY(isoch_tx, OHCI1394State,
					  OHCI1394_ISOCH_TX_CONTEXTS),
	VMSTATE_OHCI1394_DMACONTEXT_ARRAY(isoch_rx, OHCI1394State,
					  OHCI1394_ISOCH_RX_CONTEXTS),
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

    /* Initialise class */
    k->init = pci_ohci1394_init;
    k->exit = pci_ohci1394_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = PCI_DEVICE_ID_QEMU_OHCI1394;
    k->class_id = PCI_CLASS_SERIAL_FIREWIRE;
    dc->reset = ohci1394_reset;
    dc->vmsd = &vmstate_ohci1394;
    dc->props = ohci1394_properties;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    /* Generate register map */
    ohci1394_map_registers();
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
