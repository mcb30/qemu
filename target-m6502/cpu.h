/*
 * MOS Technologies 6502 virtual CPU
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

#ifndef CPU_M6502_H
#define CPU_M6502_H

#define TARGET_LONG_BITS 32

struct m6502;
typedef struct m6502 CPUArchState;

#include "config.h"
#include "qemu-common.h"
#include "exec/cpu-defs.h"
#include "qom/cpu.h"
#include "cpu.h"

#define TARGET_HAS_ICE 1

#define NB_MMU_MODES 1
#define TARGET_PAGE_BITS 8
static inline int m6502_mmu_index ( struct m6502 *m6502 ) {
	return 0;
}
#define cpu_mmu_index m6502_mmu_index

/* The 6502 has a flat 16-bit address space */
#define TARGET_PHYS_ADDR_SPACE_BITS 16
#define TARGET_VIRT_ADDR_SPACE_BITS 16

/** Status register bits */
enum {
	/** Carry flag */
	P_C = 0x01,
	/** Zero flag */
	P_Z = 0x02,
	/** Interrupt disable */
	P_I = 0x04,
	/** Decimal mode */
	P_D = 0x08,
	/** Break flag */
	P_B = 0x10,

	/** Overflow flag */
	P_V = 0x40,
	/** Negative flag */
	P_N = 0x80,
};

/** CPU feature bits */
enum {
	/** 65C02 (CMOS) model */
	M6502_CMOS = 0x0001,
};

/** CPU state */
struct m6502 {
	/** Parent object */
	CPUState parent;

	/** Accumulator (A) */
	uint8_t a;
	/** X index register (X) */
	uint8_t x;
	/** Y index register (Y) */
	uint8_t y;
	/** Processor status register (P) */
	uint8_t p;
	/** Stack pointer (S) */
	uint8_t s;
	/** Program counter */
	uint16_t pc;

	/** Features */
	unsigned int features;

	/** Common fields */
	CPU_COMMON
};

/** CPU class */
struct m6502_class {
    CPUClass parent;
    void ( * parent_reset ) ( CPUState *cpu );
};

/** qemu object model type name */
#define TYPE_M6502_CPU "m6502-cpu"

#define M6502_CLASS_CHECK( class ) \
    OBJECT_CLASS_CHECK ( struct m6502_class, (class), TYPE_M6502_CPU )

#define M6502_CHECK( object ) \
    OBJECT_CHECK ( struct m6502, (object), TYPE_M6502_CPU )

#define M6502_CLASS( object ) \
    OBJECT_GET_CLASS ( struct m6502_class, (object), TYPE_M6502_CPU )

#define ENV_GET_CPU(e) CPU(e)

/**
 * Check if interrupts are enabled
 *
 * @v m6502		CPU state
 * @ret enabled		Interrupts are enabled
 */
static inline int m6502_interrupts_enabled ( struct m6502 *m6502 ) {
	return ( ! ( m6502->p & P_I ) );
}
#define cpu_interrupts_enabled m6502_interrupts_enabled

extern void m6502_list ( FILE *f, fprintf_function cpu_fprintf );
#define cpu_list m6502_list

extern struct m6502 * m6502_init ( const char *cpu_model );
#define cpu_init m6502_init

#define cpu_dump_state m6502_dump_state
#define cpu_save m6502_save
#define cpu_load m6502_load
#define tlb_fill m6502_tlb_fill
#define do_interrupt m6502_interrupt


int cpu_m6502_exec(struct m6502 *m6502);
void cpu_m6502_close(struct m6502 *m6502);
void do_interrupt(struct m6502 *m6502);
/* you can call this signal handler from your SIGBUS and SIGSEGV
   signal handlers to inform the virtual CPU of exceptions. non zero
   is returned if the signal was handled by the virtual CPU.  */
int cpu_m6502_signal_handler(int host_signum, void *pinfo,
                          void *puc);
void m6502_translate_init(void);


#define cpu_exec cpu_m6502_exec
#define cpu_gen_code cpu_m6502_gen_code
#define cpu_signal_handler cpu_m6502_signal_handler


#define CPU_SAVE_VERSION 1

int cpu_m6502_handle_mmu_fault(struct m6502 *env, target_ulong address, int rw,
                              int mmu_idx);
#define cpu_handle_mmu_fault cpu_m6502_handle_mmu_fault

static inline void m6502_set_tls ( struct m6502 *m6502, target_ulong newtls ) {
}
#define cpu_set_tls m6502_set_tls

#include "exec/cpu-all.h"


static inline target_ulong m6502_get_pc ( struct m6502 *m6502 ) {
	return m6502->pc;
}
#define cpu_get_pc m6502_get_pc

static inline void m6502_get_tb_cpu_state ( struct m6502 *m6502,
					    target_ulong *pc,
					    target_ulong *cs_base,
					    int *flags ) {
	*pc = m6502->pc;
	*cs_base = 0;
	*flags = 0;
}
#define cpu_get_tb_cpu_state m6502_get_tb_cpu_state

static inline bool m6502_has_work ( CPUState *cpu ) {
	return 1;
}
#define cpu_has_work m6502_has_work

#include "exec/exec-all.h"

static inline void m6502_pc_from_tb ( struct m6502 *m6502,
				      TranslationBlock *tb ) {
	m6502->pc = tb->pc;
}
#define cpu_pc_from_tb m6502_pc_from_tb

#endif
