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

#include "ohci1394_hw.h"
#include "ohci1394.h"
#include "ohci1394_regs.h"

#define OHCI1394_REG_INDEX(_offset) ((_offset) >> 2 )

#define OHCI1394_REG_COUNT OHCI1394_REG_INDEX(OHCI1394_BAR_SIZE)

#define OHCI1394_MASK64_LO 0x00000000ffffffffULL
#define OHCI1394_MASK64_HI 0xffffffff00000000ULL

struct OHCI1394ControlRegisterOp {
    unsigned int count;
    const char **write_names;
    void (*write) (OHCI1394State *s, const OHCI1394ControlRegister *r,
		   unsigned int index, uint32_t val);
    const char **read_names;
    uint32_t (*read) (OHCI1394State *s, const OHCI1394ControlRegister *r,
		      unsigned int index);
};

struct OHCI1394DmaRegisterOp {
    unsigned int count;
    const char **write_names;
    void (*write) (OHCI1394State *s, OHCI1394DmaContext *c,
		   const OHCI1394DmaRegister *r, unsigned int index,
		   uint32_t val);
    const char **read_names;
    uint32_t (*read) (OHCI1394State *s, OHCI1394DmaContext *c,
		      const OHCI1394DmaRegister *r, unsigned int index);
};

typedef struct QEMU_PACKED OHCI1394RegisterMap {
    uint16_t group : 3;
    uint16_t instance : 5;
    uint16_t reg : 5;
    uint16_t index : 2;
} OHCI1394RegisterMap;

typedef struct OHCI1394RegisterGroup {
    void (*write) (OHCI1394State *s, OHCI1394RegisterMap map,
		   unsigned int hwaddr, uint32_t val);
    uint32_t (*read) (OHCI1394State *s, OHCI1394RegisterMap map,
		      unsigned int hwaddr);
    void (*map) (unsigned int group);
} OHCI1394RegisterGroup;

static void
ohci1394_map(unsigned int base, unsigned int count, unsigned int group,
	     unsigned int instance, unsigned int reg);

/*
 * Debugging
 *
 */

#define OHCI1394_REGS_DEBUG 1

#define DBG(fmt, ...) do {					\
    if (OHCI1394_REGS_DEBUG)					\
	fprintf(stderr, "ohci1394: " fmt, ## __VA_ARGS__);	\
    } while (0)

/*
 * Device control register operations
 *
 */

#define OHCI1394_INDEX_CLEAR 0x1
#define OHCI1394_INDEX_MASKED 0x1
#define OHCI1394_INDEX_MASK 0x2

static inline uint32_t *
ohci1394_ctrl_reg32(OHCI1394State *s, const OHCI1394ControlRegister *r)
{
    return ((uint32_t *)(((uint8_t *)s) + r->offset));
}

static inline uint64_t *
ohci1394_ctrl_reg64(OHCI1394State *s, const OHCI1394ControlRegister *r)
{
    return ((uint64_t *)(((uint8_t *)s) + r->offset));
}

static inline OHCI1394Shadowed *
ohci1394_ctrl_shadowed(OHCI1394State *s, const OHCI1394ControlRegister *r) {
    return ((OHCI1394Shadowed *)(((uint8_t *)s) + r->offset));
}

static inline OHCI1394EventMask *
ohci1394_ctrl_eventmask(OHCI1394State *s, const OHCI1394ControlRegister *r) {
    return ((OHCI1394EventMask *)(((uint8_t *)s) + r->offset));
}

static void
ohci1394_ctrl_reg32_write(OHCI1394State *s, const OHCI1394ControlRegister *r,
			  unsigned int index, uint32_t val)
{
    uint32_t *reg = ohci1394_ctrl_reg32(s, r);
    uint32_t set = r->set;
    uint32_t clear = r->clear;

    *reg = ((*reg & val) | (*reg & ~clear) | (val & set));
}

static uint32_t
ohci1394_ctrl_reg32_read(OHCI1394State *s, const OHCI1394ControlRegister *r,
			 unsigned int index)
{
    uint32_t *reg = ohci1394_ctrl_reg32(s, r);

    return *reg;
}

static void
ohci1394_ctrl_hilo_write(OHCI1394State *s, const OHCI1394ControlRegister *r,
			 unsigned int index, uint32_t val)
{
    uint64_t *reg = ohci1394_ctrl_reg64(s, r);
    uint64_t mask = (index ? OHCI1394_MASK64_LO : OHCI1394_MASK64_HI);
    uint64_t set = (r->set & mask);
    uint64_t clear = (r->clear & mask);
    uint64_t valdup = ((((uint64_t) val) << 32) | val);

    *reg = ((*reg & valdup) | (*reg & ~clear) | (valdup & set));
}

static uint32_t
ohci1394_ctrl_hilo_read(OHCI1394State *s, const OHCI1394ControlRegister *r,
			unsigned int index)
{
    uint64_t *reg = ohci1394_ctrl_reg64(s, r);
    unsigned int shift = (index ? 0 : 32);

    return (*reg >> shift);
}

static void
ohci1394_ctrl_setclear_write(OHCI1394State *s, const OHCI1394ControlRegister *r,
			     unsigned int index, uint32_t val)
{
    uint32_t *reg = ohci1394_ctrl_reg32(s, r);
    uint32_t set = r->set;
    uint32_t clear = r->clear;

    if (index & OHCI1394_INDEX_CLEAR) {
	*reg &= ~(val & clear);
    } else {
	*reg |= (val & set);
    }
}

static uint32_t
ohci1394_ctrl_setclear_read(OHCI1394State *s, const OHCI1394ControlRegister *r,
			    unsigned int index)
{
    uint32_t *reg = ohci1394_ctrl_reg32(s, r);

    return *reg;
}

static void
ohci1394_ctrl_hilo_setclear_write(OHCI1394State *s,
				  const OHCI1394ControlRegister *r,
				  unsigned int index, uint32_t val)
{
    uint64_t *reg = ohci1394_ctrl_reg64(s, r);
    uint64_t mask = ((index & ~OHCI1394_INDEX_CLEAR) ?
		     OHCI1394_MASK64_LO : OHCI1394_MASK64_HI);
    uint64_t set = (r->set & mask);
    uint64_t clear = (r->clear & mask);

    if (index & OHCI1394_INDEX_CLEAR) {
	*reg &= ~(val & clear);
    } else {
	*reg |= (val & set);
    }
}

static uint32_t
ohci1394_ctrl_hilo_setclear_read(OHCI1394State *s,
				 const OHCI1394ControlRegister *r,
				 unsigned int index)
{
    uint64_t *reg = ohci1394_ctrl_reg64(s, r);
    unsigned int shift = ((index & ~OHCI1394_INDEX_CLEAR) ? 0 : 32);

    return (*reg >> shift);
}

static void
ohci1394_ctrl_eventmask_write(OHCI1394State *s,
			      const OHCI1394ControlRegister *r,
			      unsigned int index, uint32_t val)
{
    OHCI1394EventMask *reg = ohci1394_ctrl_eventmask(s, r);
    uint32_t mask_set = OHCI1394_EVENTMASK_MASK(r->set);
    uint32_t mask_clear = OHCI1394_EVENTMASK_MASK(r->clear);
    uint32_t event_set = OHCI1394_EVENTMASK_EVENT(r->set);
    uint32_t event_clear = OHCI1394_EVENTMASK_EVENT(r->clear);

    if (index & OHCI1394_INDEX_MASK) {
	if (index & OHCI1394_INDEX_CLEAR) {
	    reg->mask &= ~(val & mask_clear);
	} else {
	    reg->mask |= (val & mask_set);
	}
    } else {
	if (index & OHCI1394_INDEX_CLEAR) {
	    reg->event &= ~(val & event_clear);
	} else {
	    reg->event |= (val & event_set);
	}
    }
}

static uint32_t
ohci1394_ctrl_eventmask_read(OHCI1394State *s,
			     const OHCI1394ControlRegister *r,
			     unsigned int index)
{
    OHCI1394EventMask *reg = ohci1394_ctrl_eventmask(s, r);

    if (index & OHCI1394_INDEX_MASK) {
	return reg->mask;
    } else {
	if (index & OHCI1394_INDEX_MASKED) {
	    return (reg->event & reg->mask);
	} else {
	    return reg->event;
	}
    }
}

static void
ohci1394_ctrl_shadowed_write(OHCI1394State *s, const OHCI1394ControlRegister *r,
			     unsigned int index, uint32_t val)
{
    OHCI1394Shadowed *reg = ohci1394_ctrl_shadowed(s, r);
    uint32_t set = r->set;
    uint32_t clear = r->clear;

    reg->shadow = ((reg->shadow & val) | (reg->shadow & ~clear) | (val & set));
}

static uint32_t
ohci1394_ctrl_shadowed_read(OHCI1394State *s, const OHCI1394ControlRegister *r,
			    unsigned int index)
{
    OHCI1394Shadowed *reg = ohci1394_ctrl_shadowed(s, r);

    return reg->active;
}

static const char *ohci1394_ctrl_reg32_names[] =
    { "" };

static const char *ohci1394_ctrl_shadowed_names[] =
    { "" };

static const char *ohci1394_ctrl_hilo_names[] =
    { ".hi", ".lo" };

static const char *ohci1394_ctrl_setclear_write_names[] =
    { ".set", ".clear" };

static const char *ohci1394_ctrl_setclear_read_names[] =
    { "", "" };

static const char *ohci1394_ctrl_hilo_setclear_write_names[] =
    { ".hi.set", ".hi.clear", ".lo.set", ".lo.clear" };

static const char *ohci1394_ctrl_hilo_setclear_read_names[] =
    { ".hi", ".hi", ".lo", ".lo" };

static const char *ohci1394_ctrl_eventmask_write_names[] =
    { ".event.set", ".event.clear", ".mask.set", ".mask.clear" };

static const char *ohci1394_ctrl_eventmask_read_names[] =
    { ".event", ".event.masked", ".mask", ".mask" };

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_reg32 = {
    .count = 1,
    .write_names = ohci1394_ctrl_reg32_names,
    .write = ohci1394_ctrl_reg32_write,
    .read_names = ohci1394_ctrl_reg32_names,
    .read = ohci1394_ctrl_reg32_read,
};

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_hilo = {
    .count = 2,
    .write_names = ohci1394_ctrl_hilo_names,
    .write = ohci1394_ctrl_hilo_write,
    .read_names = ohci1394_ctrl_hilo_names,
    .read = ohci1394_ctrl_hilo_read,
};

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_setclear = {
    .count = 2,
    .write_names = ohci1394_ctrl_setclear_write_names,
    .write = ohci1394_ctrl_setclear_write,
    .read_names = ohci1394_ctrl_setclear_read_names,
    .read = ohci1394_ctrl_setclear_read,
};

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_hilo_setclear = {
    .count = 4,
    .write_names = ohci1394_ctrl_hilo_setclear_write_names,
    .write = ohci1394_ctrl_hilo_setclear_write,
    .read_names = ohci1394_ctrl_hilo_setclear_read_names,
    .read = ohci1394_ctrl_hilo_setclear_read,
};

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_eventmask = {
    .count = 4,
    .write_names = ohci1394_ctrl_eventmask_write_names,
    .write = ohci1394_ctrl_eventmask_write,
    .read_names = ohci1394_ctrl_eventmask_read_names,
    .read = ohci1394_ctrl_eventmask_read,
};

const OHCI1394ControlRegisterOp ohci1394_ctrl_op_shadowed = {
    .count = 1,
    .write_names = ohci1394_ctrl_shadowed_names,
    .write = ohci1394_ctrl_shadowed_write,
    .read_names = ohci1394_ctrl_shadowed_names,
    .read = ohci1394_ctrl_shadowed_read,
};

/*
 * DMA context register operations
 *
 */

static inline uint32_t *
ohci1394_dma_reg32(OHCI1394DmaContext *c, const OHCI1394DmaRegister *r)
{
    return ((uint32_t *)(((uint8_t *)c) + r->offset));
}

static void
ohci1394_dma_reg32_write(OHCI1394State *s, OHCI1394DmaContext *c,
			 const OHCI1394DmaRegister *r, unsigned int index,
			 uint32_t val)
{
    uint32_t *reg = ohci1394_dma_reg32(c, r);
    uint32_t set = r->set;
    uint32_t clear = r->clear;

    *reg = ((*reg & val) | (*reg & ~clear) | (val & set));
}

static uint32_t
ohci1394_dma_reg32_read(OHCI1394State *s, OHCI1394DmaContext *c,
			const OHCI1394DmaRegister *r, unsigned int index)
{
    uint32_t *reg = ohci1394_dma_reg32(c, r);

    return *reg;
}

static void
ohci1394_dma_setclear_write(OHCI1394State *s, OHCI1394DmaContext *c,
			    const OHCI1394DmaRegister *r, unsigned int index,
			    uint32_t val)
{
    uint32_t *reg = ohci1394_dma_reg32(c, r);
    uint32_t set = r->set;
    uint32_t clear = r->clear;

    if (index & OHCI1394_INDEX_CLEAR) {
	*reg &= ~(val & clear);
    } else {
	*reg |= (val & set);
    }
}

static uint32_t
ohci1394_dma_setclear_read(OHCI1394State *s, OHCI1394DmaContext *c,
			   const OHCI1394DmaRegister *r, unsigned int index)
{
    uint32_t *reg = ohci1394_dma_reg32(c, r);

    return *reg;
}

static const char *ohci1394_dma_reg32_names[] =
    { "" };

static const char *ohci1394_dma_setclear_write_names[] =
    { ".set", ".clear" };

static const char *ohci1394_dma_setclear_read_names[] =
    { "", "" };

const OHCI1394DmaRegisterOp ohci1394_dma_op_reg32 = {
    .count = 1,
    .write_names = ohci1394_dma_reg32_names,
    .write = ohci1394_dma_reg32_write,
    .read_names = ohci1394_dma_reg32_names,
    .read = ohci1394_dma_reg32_read,
};

const OHCI1394DmaRegisterOp ohci1394_dma_op_setclear = {
    .count = 2,
    .write_names = ohci1394_dma_setclear_write_names,
    .write = ohci1394_dma_setclear_write,
    .read_names = ohci1394_dma_setclear_read_names,
    .read = ohci1394_dma_setclear_read,
};

/*
 * Device control register group
 *
 */

static void
ohci1394_ctrl_write(OHCI1394State *s, OHCI1394RegisterMap map,
		    unsigned int addr, uint32_t val)
{
    const OHCI1394ControlRegister *r = &ohci1394_ctrl_registers[map.reg];

    DBG("0x%03x <= 0x%08x %s%s\n",
	addr, val, r->name, r->op->write_names[map.index]);
    r->op->write(s, r, map.index, val);
    if (r->notify)
	r->notify(s);
}

static uint32_t
ohci1394_ctrl_read(OHCI1394State *s, OHCI1394RegisterMap map,
		   unsigned int addr)
{
    const OHCI1394ControlRegister *r = &ohci1394_ctrl_registers[map.reg];
    uint32_t val;

    val = r->op->read(s, r, map.index);
    DBG("0x%03x => 0x%08x %s%s\n",
	addr, val,r->name, r->op->read_names[map.index]);
    return val;
}

static void
ohci1394_ctrl_map(unsigned int group)
{
    const OHCI1394ControlRegister *r;

    for (r = ohci1394_ctrl_registers; r->op; r++) {
	ohci1394_map(r->base, r->op->count, group, 0,
		     (r - ohci1394_ctrl_registers));
    }
}

static OHCI1394RegisterGroup ohci1394_ctrl = {
    .write = ohci1394_ctrl_write,
    .read = ohci1394_ctrl_read,
    .map = ohci1394_ctrl_map,
};

/*
 * Asynchronous DMA context register group
 *
 */

static const char *ohci1394_async_names[] =
    { "request_tx", "response_tx", "request_rx", "response_rx" };

static void
ohci1394_async_write(OHCI1394State *s, OHCI1394RegisterMap map,
		     unsigned int addr, uint32_t val)
{
    const OHCI1394DmaRegister *r = &ohci1394_async_registers[map.reg];
    OHCI1394DmaContext *c = &s->async.numbered[map.instance];

    DBG("0x%03x <= 0x%08x async.%s.%s%s\n", addr, val,
	ohci1394_async_names[map.instance], r->name,
	r->op->write_names[map.index]);
    r->op->write(s, c, r, map.index, val);
    if (r->notify)
	r->notify(s, c);
}

static uint32_t
ohci1394_async_read(OHCI1394State *s, OHCI1394RegisterMap map,
		    unsigned int addr)
{
    const OHCI1394DmaRegister *r = &ohci1394_async_registers[map.reg];
    OHCI1394DmaContext *c = &s->async.numbered[map.instance];
    uint32_t val;

    val = r->op->read(s, c, r, map.index);
    DBG("0x%03x => 0x%08x async.%s.%s%s\n", addr, val,
	ohci1394_async_names[map.instance], r->name,
	r->op->read_names[map.index]);
    return val;
}

static void
ohci1394_async_map(unsigned int group)
{
    const OHCI1394DmaRegister *r;
    OHCI1394State *s;
    unsigned int instance;

    for (instance = 0; instance < ARRAY_SIZE(s->async.numbered); instance++) {
	for (r = ohci1394_async_registers; r->op; r++) {
	    ohci1394_map((OHCI1394_ASYNC(instance) + r->base), r->op->count,
			 group, instance, (r - ohci1394_async_registers));
	}
    }
}

static OHCI1394RegisterGroup ohci1394_async = {
    .write = ohci1394_async_write,
    .read = ohci1394_async_read,
    .map = ohci1394_async_map,
};

/*
 * Isochronous transmit context register group
 *
 */

static void
ohci1394_isoch_tx_write(OHCI1394State *s, OHCI1394RegisterMap map,
			unsigned int addr, uint32_t val)
{
    const OHCI1394DmaRegister *r = &ohci1394_isoch_tx_registers[map.reg];
    OHCI1394DmaContext *c = &s->isoch_tx[map.instance];

    DBG("0x%03x <= 0x%08x isoch_tx[%02x].%s%s\n",
	addr, val, map.instance, r->name, r->op->write_names[map.index]);
    r->op->write(s, c, r, map.index, val);
    if (r->notify)
	r->notify(s, c);
}

static uint32_t
ohci1394_isoch_tx_read(OHCI1394State *s, OHCI1394RegisterMap map,
		       unsigned int addr)
{
    const OHCI1394DmaRegister *r = &ohci1394_isoch_tx_registers[map.reg];
    OHCI1394DmaContext *c = &s->isoch_tx[map.instance];
    uint32_t val;

    val = r->op->read(s, c, r, map.index);
    DBG("0x%03x => 0x%08x isoch_tx[%02x].%s%s\n",
	addr, val, map.instance, r->name, r->op->read_names[map.index]);
    return val;
}

static void
ohci1394_isoch_tx_map(unsigned int group)
{
    const OHCI1394DmaRegister *r;
    OHCI1394State *s;
    unsigned int instance;

    for (r = ohci1394_isoch_tx_registers; r->op; r++) {
	for (instance = 0; instance < ARRAY_SIZE(s->isoch_tx); instance++) {
	    ohci1394_map((OHCI1394_ISOCH_TX(instance) + r->base), r->op->count,
			 group, instance, (r - ohci1394_isoch_tx_registers));
	}
    }
}

static OHCI1394RegisterGroup ohci1394_isoch_tx = {
    .write = ohci1394_isoch_tx_write,
    .read = ohci1394_isoch_tx_read,
    .map = ohci1394_isoch_tx_map,
};

/*
 * Isochronous receive context register group
 *
 */

static void
ohci1394_isoch_rx_write(OHCI1394State *s, OHCI1394RegisterMap map,
			unsigned int addr, uint32_t val)
{
    const OHCI1394DmaRegister *r = &ohci1394_isoch_rx_registers[map.reg];
    OHCI1394DmaContext *c = &s->isoch_rx[map.instance];

    DBG("0x%03x <= 0x%08x isoch_rx[%02x].%s%s\n",
	addr, val, map.instance, r->name, r->op->write_names[map.index]);
    r->op->write(s, c, r, map.index, val);
    if (r->notify)
	r->notify(s, c);
}

static uint32_t
ohci1394_isoch_rx_read(OHCI1394State *s, OHCI1394RegisterMap map,
		       unsigned int addr)
{
    const OHCI1394DmaRegister *r = &ohci1394_isoch_rx_registers[map.reg];
    OHCI1394DmaContext *c = &s->isoch_rx[map.instance];
    uint32_t val;

    val = r->op->read(s, c, r, map.index);
    DBG("0x%03x => 0x%08x isoch_rx[%02x].%s%s\n",
	addr, val, map.instance, r->name, r->op->read_names[map.index]);
    return val;
}

static void
ohci1394_isoch_rx_map(unsigned int group)
{
    const OHCI1394DmaRegister *r;
    OHCI1394State *s;
    unsigned int instance;

    for (r = ohci1394_isoch_rx_registers; r->op; r++) {
	for (instance = 0; instance < ARRAY_SIZE(s->isoch_rx); instance++) {
	    ohci1394_map((OHCI1394_ISOCH_RX(instance) + r->base), r->op->count,
			 group, instance, (r - ohci1394_isoch_rx_registers));
	}
    }
}

static OHCI1394RegisterGroup ohci1394_isoch_rx = {
    .write = ohci1394_isoch_rx_write,
    .read = ohci1394_isoch_rx_read,
    .map = ohci1394_isoch_rx_map,
};

/*
 * Register map
 *
 */

static const OHCI1394RegisterGroup *ohci1394_register_groups[] = {
    NULL, /* Must be first */
    &ohci1394_ctrl,
    &ohci1394_async,
    &ohci1394_isoch_tx,
    &ohci1394_isoch_rx,
};

static OHCI1394RegisterMap ohci1394_register_map[OHCI1394_REG_COUNT];

static void
ohci1394_map(unsigned int base, unsigned int count, unsigned int group,
	     unsigned int instance, unsigned int reg) {
    OHCI1394RegisterMap *map;
    unsigned int index;

    for (index = 0; index < count; index++) {
	map = &ohci1394_register_map[OHCI1394_REG_INDEX(base) + index];
	map->group = group;
	map->instance = instance;
	map->reg = reg;
	map->index = index;
    }
}

void
ohci1394_map_registers(void)
{
    const OHCI1394RegisterGroup *rg;
    unsigned int group;

    for (group = 0; group < ARRAY_SIZE(ohci1394_register_groups); group++) {
	rg = ohci1394_register_groups[group];
	if (rg)
	    rg->map(group);
    }
}

/*
 * Register access
 *
 */

void
ohci1394_write(OHCI1394State *s, unsigned int addr, uint32_t val)
{
    const OHCI1394RegisterGroup *rg;
    OHCI1394RegisterMap map;
    unsigned int index;

    index = OHCI1394_REG_INDEX(addr);
    map = ohci1394_register_map[index];
    rg = ohci1394_register_groups[map.group];
    if (likely(rg)) {
	rg->write(s, map, addr, val);
    } else {
	DBG("0x%03x <= 0x%08x *** UNKNOWN ***\n", addr, val);
    }
}

uint32_t
ohci1394_read(OHCI1394State *s, unsigned int addr)
{
    const OHCI1394RegisterGroup *rg;
    OHCI1394RegisterMap map;
    unsigned int index;
    uint32_t val;

    index = OHCI1394_REG_INDEX(addr);
    map = ohci1394_register_map[index];
    rg = ohci1394_register_groups[map.group];
    if (likely(rg)) {
	val = rg->read(s, map, addr);
    } else {
	val = 0;
	DBG("0x%03x => 0x%08x *** UNKNOWN ***\n", addr, val);
    }
    return val;
}

/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 8
 * End:
 */
