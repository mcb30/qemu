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

#include "ohci1394.h"

typedef struct OHCI1394ControlRegisterOp OHCI1394ControlRegisterOp;

typedef struct OHCI1394ControlRegister {
    /* Name */
    const char *name;
    /* Base address within BAR */
    unsigned int base;
    /* Offset within OHCI1394State */
    unsigned int offset;
    /* Register read/write operations */
    const OHCI1394ControlRegisterOp *op;
    /* Handle register updates */
    void (*notify) (OHCI1394State *s);
} OHCI1394ControlRegister;

#define OHCI1394_CTRL_REG(_base, _field, _op, _notify) {		\
	.name = #_field,						\
	.base = OHCI1394_ ## _base,					\
	.offset = offsetof(OHCI1394State, _field),			\
	.op = &ohci1394_ctrl_op_ ## _op,				\
	.notify = _notify,						\
    }

#define OHCI1394_CTRL_REG_END { .op = NULL }

typedef struct OHCI1394DmaRegisterOp OHCI1394DmaRegisterOp;

typedef struct OHCI1394DmaRegister {
    /* Name */
    const char *name;
    /* Base address within register set */
    unsigned int base;
    /* Offset within OHCI1394Dma */
    unsigned int offset;
    /* Register read/write operations */
    const OHCI1394DmaRegisterOp *op;
    /* Handle register updates */
    void (*notify) (OHCI1394State *s, OHCI1394DmaContext *c);
} OHCI1394DmaRegister;

#define OHCI1394_DMA_REG(_base, _field, _op, _notify) {			\
	.name = #_field,						\
	.base = OHCI1394_DMA_ ## _base,					\
	.offset = offsetof(OHCI1394DmaContext, _field),			\
	.op = &ohci1394_dma_op_ ## _op,					\
	.notify = _notify,						\
    }

#define OHCI1394_DMA_REG_END { .op = NULL }

extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_reg32;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_reg32_readonly;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_hilo;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_setclear;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_hilo_setclear;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_eventmask;
extern const OHCI1394ControlRegisterOp ohci1394_ctrl_op_shadowed;
extern const OHCI1394DmaRegisterOp ohci1394_dma_op_reg32;
extern const OHCI1394DmaRegisterOp ohci1394_dma_op_setclear;

extern const OHCI1394ControlRegister ohci1394_ctrl_registers[];
extern const OHCI1394DmaRegister ohci1394_async_registers[];
extern const OHCI1394DmaRegister ohci1394_isoch_tx_registers[];
extern const OHCI1394DmaRegister ohci1394_isoch_rx_registers[];

extern void
ohci1394_map_registers(void);
extern void
ohci1394_write(OHCI1394State *s, unsigned int addr, uint32_t val);
extern uint32_t
ohci1394_read(OHCI1394State *s, unsigned int addr);

#endif /* _OHCI1394_REGS_H_ */
