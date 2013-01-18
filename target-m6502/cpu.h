/*
 * MOS Technology 6502 virtual CPU
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

#define CPUArchState struct CPUM6502State

#include "config.h"
#include "qemu-common.h"
#include "exec/cpu-defs.h"

#define TARGET_HAS_ICE 1

/* The 6502 allows for arbitrary self-modifying code */
#define TARGET_HAS_SMC
#define TARGET_HAS_PRECISE_SMC

#define NB_MMU_MODES 1
#define TARGET_PAGE_BITS 8

#define CPU_INTERRUPT_NMI CPU_INTERRUPT_TGT_EXT_3

/* The 6502 has a flat 16-bit address space */
#define TARGET_PHYS_ADDR_SPACE_BITS 16
#define TARGET_VIRT_ADDR_SPACE_BITS 16

/** Exceptions */
enum {
	EXCP_IRQ,
	EXCP_NMI,
};

/** Status register bits */
enum {
	/** Carry flag */
	P_C = 0,
	/** Zero flag */
	P_Z = 1,
	/** Interrupt disable */
	P_I = 2,
	/** Decimal mode */
	P_D = 3,
	/** Break flag */
	P_B = 4,
	/** Unused flag */
	P_U = 5,
	/** Overflow flag */
	P_V = 6,
	/** Negative flag */
	P_N = 7,
};

/** CPU feature bits */
enum {
	/** 65C02 (CMOS) model */
	M6502_CMOS = 0x0001,
};

/** CPU state
 *
 * TCG can operate only on 32-bit and 64-bit quantities.  We therefore
 * use 32-bit fields for the 8-bit and 16-bit registers.
 */
typedef struct CPUM6502State {
	/** Accumulator (A) */
	uint32_t a;
	/** X index register (X) */
	uint32_t x;
	/** Y index register (Y) */
	uint32_t y;
	/** Carry flag
	 *
	 * Valid values are zero and one.
	 */
	uint32_t p_c;
	/** Not zero flag
	 *
	 * Zero flag is set iff this value is zero.  Defining the flag
	 * this way allows us to eliminate some conditional logic; we
	 * can simply store the value being tested directly into p_nz.
	 */
	uint32_t p_nz;
	/** Interrupt disable
	 *
	 * Valid values are zero and one.
	 */
	uint32_t p_i;
	/** Decimal mode
	 *
	 * Valid values are zero and one.
	 */
	uint32_t p_d;
	/** Overflow flag
	 *
	 * Overflow flag is set iff this value is non-zero.  Defining
	 * the flag this way allows us to eliminate some shifts.
	 */
	uint32_t p_v;
	/** Negative flag
	 *
	 * Valid values are zero and (1<<P_N).  Defining the flag this
	 * way allows us to eliminate some conditional logic; we can
	 * simply AND the value being tested with (1<<P_N) and store
	 * the result in p_n.
	 */
	uint32_t p_n;
	/** Stack pointer (S) */
	uint32_t s;
	/** Program counter */
	uint32_t pc;

	/** NMI executing flag
	 *
	 * On a real 6502, NMI is an edge-sensitive interrupt which
	 * cannot be blocked by anything.  It is typically used for
	 * latency-sensitive applications such as transferring each
	 * data byte during a disk I/O on systems that do not support
	 * DMA.
	 *
	 * This creates potential issues in a VM: for example, the
	 * disk controller might have to use a high-frequency timer to
	 * generate NMIs for each data byte.  If the frequency is too
	 * high then the virtual CPU will lose data, if the frequency
	 * is too low then performance suffers.
	 *
	 * We can get better performance by deferring NMIs (and IRQs)
	 * while an NMI handler is currently executing.  The virtual
	 * hardware can then deassert and reassert the interrupt
	 * immediately, without causing the CPU to reenter the NMI
	 * handler.  Upon reaching an RTI instruction, the "NMI
	 * executing" flag will be cleared, allowing the deferred NMI
	 * to happen.
	 *
	 * This scheme assumes that an NMI handler must exit with an
	 * RTI, that an NMI handler does not reenable interrupts
	 * before exiting, and that an NMI handler does not use RTI
	 * except to exit.  These should be reasonable assumptions to
	 * make.
	 */
	bool in_nmi;

	/** Features */
	unsigned int features;

	/** Common fields */
	CPU_COMMON
} CPUM6502State;

#include "cpu-qom.h"

extern unsigned int m6502_get_p ( CPUM6502State *env );
extern void m6502_set_p ( CPUM6502State *env, unsigned int p );

extern void m6502_rti ( CPUM6502State *env );

/**
 * Check if interrupts are enabled
 *
 * @v env		CPU state
 * @ret enabled		Interrupts are enabled
 */
static inline int m6502_interrupts_enabled ( CPUM6502State *env ) {
	return ( ! env->p_i );
}
#define cpu_interrupts_enabled m6502_interrupts_enabled

static inline int m6502_mmu_index ( CPUM6502State *env ) {
	return 0;
}
#define cpu_mmu_index m6502_mmu_index

extern void m6502_list ( FILE *f, fprintf_function cpu_fprintf );
#define cpu_list m6502_list

extern CPUM6502State * m6502_init ( const char *cpu_model );
#define cpu_init m6502_init

extern void m6502_dump_stack ( CPUM6502State *env, FILE *f,
			       fprintf_function fprintf );

#define cpu_dump_state m6502_dump_state
#define cpu_save m6502_save
#define cpu_load m6502_load
#define tlb_fill m6502_tlb_fill
#define do_interrupt m6502_interrupt
#define cpu_get_phys_page_debug m6502_get_phys_page_debug

int m6502_exec(CPUM6502State *env);
void m6502_close(CPUM6502State *env);
void do_interrupt(CPUM6502State *env);
/* you can call this signal handler from your SIGBUS and SIGSEGV
   signal handlers to inform the virtual CPU of exceptions. non zero
   is returned if the signal was handled by the virtual CPU.  */
int cpu_m6502_signal_handler(int host_signum, void *pinfo,
                          void *puc);
void m6502_translate_init(void);


#define cpu_exec m6502_exec
#define cpu_gen_code m6502_gen_code
#define cpu_signal_handler m6502_signal_handler
#define gen_intermediate_code m6502_gen_intermediate_code
#define gen_intermediate_code_pc m6502_gen_intermediate_code_pc
#define restore_state_to_opc m6502_restore_state_to_opc


#define CPU_SAVE_VERSION 1

int cpu_m6502_handle_mmu_fault(CPUM6502State *env, target_ulong address, int rw,
                              int mmu_idx);
#define cpu_handle_mmu_fault cpu_m6502_handle_mmu_fault

static inline void m6502_set_tls ( CPUM6502State *env, target_ulong newtls ) {
}
#define cpu_set_tls m6502_set_tls

#include "exec/cpu-all.h"


static inline target_ulong m6502_get_pc ( CPUM6502State *env ) {
	return env->pc;
}
#define cpu_get_pc m6502_get_pc

static inline void m6502_get_tb_cpu_state ( CPUM6502State *env,
					    target_ulong *pc,
					    target_ulong *cs_base,
					    int *flags ) {
	*pc = env->pc;
	*cs_base = 0;
	*flags = 0;
}
#define cpu_get_tb_cpu_state m6502_get_tb_cpu_state

static inline bool m6502_has_work ( CPUState *cpu ) {
	return 0;
}
#define cpu_has_work m6502_has_work

#include "exec/exec-all.h"

static inline void m6502_pc_from_tb ( CPUM6502State *env,
				      TranslationBlock *tb ) {
	env->pc = tb->pc;
}
#define cpu_pc_from_tb m6502_pc_from_tb

#define M6502_NMI_VECTOR 0xfffa
#define M6502_RESET_VECTOR 0xfffc
#define M6502_IRQ_VECTOR 0xfffe
#define M6502_ADDRESS_MASK 0xffff
#define M6502_ZERO_PAGE_MASK 0xff
#define M6502_STACK_BASE 0x0100
#define M6502_STACK_TOP 0x01ff

#endif
