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

#include "cpu.h"
#include "disas/disas.h"
#include "helper.h"
#include "tcg-op.h"

#define GEN_HELPER 1
#include "helper.h"

#define MEM_INDEX 0

static TCGv_ptr cpu_env;
static TCGv_i32 cpu_a;
static TCGv_i32 cpu_x;
static TCGv_i32 cpu_y;
static TCGv_i32 cpu_p;
static TCGv_i32 cpu_s;
static TCGv_i32 cpu_pc;

#include "exec/gen-icount.h"

typedef struct {
	CPUM6502State *env;
	uint32_t pc;

	uint8_t opcode;
	TCGv_i32 dest;
	TCGv_i32 src;
	TCGv_i32 address;
} DisasContext;

/* Absolute addressing mode */
static void m6502_abs ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );
}

/* Absolute,X addressing mode */
static void m6502_abs_x ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );
}

/* Absolute,Y addressing mode */
static void m6502_abs_y ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );
}

/* Zero-page addressing mode */
static void m6502_zero ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );
}

/* Zero-page,X addressing mode */
static void m6502_zero_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );
}

/* Zero-page,Y addressing mode */
static void m6502_zero_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );
}

/* (Indirect,X) addressing mode */
static void m6502_ind_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );
}

/* (Indirect),Y addressing mode */
static void m6502_ind_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );
	tcg_gen_add_i32 ( dc->address, dc->address, cpu_y );
}

/* LDA, LDX, LDY (immediate) */
static void m6502_gen_load_imm ( DisasContext *dc ) {
	uint8_t value;

	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_movi_i32 ( dc->dest, value );
}

/* LDA, LDX, LDY (memory) */
static void m6502_gen_load ( DisasContext *dc ) {
	tcg_gen_qemu_ld8u ( dc->dest, dc->address, MEM_INDEX );
}

/* STA, STX, STY */
static void m6502_gen_store ( DisasContext *dc ) {
	tcg_gen_qemu_st8 ( dc->src, dc->address, MEM_INDEX );
}

/* TAX, TAY, TSX, TXA, TXS, TYA */
static void m6502_gen_transfer ( DisasContext *dc ) {
	tcg_gen_mov_i32 ( dc->dest, dc->src );
}

/** An instruction definition */
typedef struct {
	/** Generate instruction */
	void ( * gen ) ( DisasContext *dc );
	/** Destination register (if any) */
	TCGv_i32 *dest;
	/** Source register (if any) */
	TCGv_i32 *src;
	/** Memory addressing mode (if any) */
	void ( * mem ) ( DisasContext *dc );
	/** Length of instruction */
	size_t len;
} M6502Instruction;

/** Instruction table */
static const M6502Instruction m6502_instructions[256] = {
	/* LDA */
	[0xa9] = { m6502_gen_load_imm,	&cpu_a,	NULL,	NULL,		2 },
	[0xa5] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_zero,	2 },
	[0xb5] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_zero_x,	2 },
	[0xad] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs,	3 },
	[0xbd] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs_x,	3 },
	[0xb9] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_abs_y,	3 },
	[0xa1] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_ind_x,	2 },
	[0xb1] = { m6502_gen_load,	&cpu_a,	NULL,	m6502_ind_y,	2 },
	/* LDX */
	[0xa2] = { m6502_gen_load_imm,	&cpu_x,	NULL,	NULL,		2 },
	[0xa6] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_zero,	2 },
	[0xb6] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_zero_y,	2 },
	[0xae] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_abs,	3 },
	[0xbe] = { m6502_gen_load,	&cpu_x,	NULL,	m6502_abs_y,	3 },
	/* LDY */
	[0xa0] = { m6502_gen_load_imm,	&cpu_y,	NULL,	NULL,		2 },
	[0xa4] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_zero,	2 },
	[0xb4] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_zero_x,	2 },
	[0xac] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_abs,	3 },
	[0xbc] = { m6502_gen_load,	&cpu_y,	NULL,	m6502_abs_x,	3 },
	/* STA */
	[0x85] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_zero,	2 },
	[0x95] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_zero_x,	2 },
	[0x8d] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs,	3 },
	[0x9d] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs_x,	3 },
	[0x99] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_abs_y,	3 },
	[0x81] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_ind_x,	2 },
	[0x91] = { m6502_gen_store,	NULL,	&cpu_a,	m6502_ind_y,	2 },
	/* STX */
	[0x86] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_zero,	2 },
	[0x96] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_zero_y,	2 },
	[0x8e] = { m6502_gen_store,	NULL,	&cpu_x,	m6502_abs,	3 },
	/* STY */
	[0x84] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_zero,	2 },
	[0x94] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_zero_x,	2 },
	[0x8c] = { m6502_gen_store,	NULL,	&cpu_y,	m6502_abs,	3 },
	/* TAX */
	[0xaa] = { m6502_gen_transfer,	&cpu_x,	&cpu_a,	NULL,		1 },
	/* TAY */
	[0xa8] = { m6502_gen_transfer,	&cpu_y,	&cpu_a,	NULL,		1 },
	/* TSX */
	[0xba] = { m6502_gen_transfer,	&cpu_x,	&cpu_s,	NULL,		1 },
	/* TXA */
	[0x88] = { m6502_gen_transfer,	&cpu_a,	&cpu_x,	NULL,		1 },
	/* TXS */
	[0x9a] = { m6502_gen_transfer,	&cpu_s,	&cpu_x,	NULL,		1 },
	/* TYA */
	[0x98] = { m6502_gen_transfer,	&cpu_a,	&cpu_y,	NULL,		1 },
};

static size_t m6502_gen_instruction ( DisasContext *dc ) {
	const M6502Instruction *insn;

	/* Fetch and validate opcode */
	dc->opcode = cpu_ldub_code ( dc->env, dc->pc );
	insn = &m6502_instructions[dc->opcode];
	if ( ! insn->gen ) {
		cpu_abort ( dc->env, "Unknown opcode %02x at %04x\n",
			    dc->opcode, dc->pc );
		return 0;
	}

	/* Identify registers, if applicable */
	if ( insn->src )
		dc->src = *(insn->src);
	if ( insn->dest )
		dc->dest = *(insn->dest);

	/* Generate memory address, if applicable */
	if ( insn->mem )
		insn->mem ( dc );

	/* Generate instruction */
	insn->gen ( dc );

	/* Free address, if one was generated */
	if ( insn->mem )
		tcg_temp_free_i32 ( dc->address );

	return insn->len;
}

void m6502_gen_intermediate_code ( CPUM6502State *env,
				   struct TranslationBlock *tb ) {
	DisasContext ctx;
	DisasContext *dc = &ctx;
	hwaddr pc_start = tb->pc;
	unsigned int num_insns = 0;
	size_t len;

	printf ( "gen_intermediate_code() pc=%04x\n", tb->pc );

	dc->env = env;
	dc->pc = pc_start;

	gen_icount_start();
	do {
		len = m6502_gen_instruction ( dc );
		dc->pc += len;
		num_insns++;
	} while ( len );

	tcg_gen_movi_i32 ( cpu_pc, dc->pc );
	gen_helper_hlt ( cpu_env );
	tcg_gen_exit_tb ( 0 );

	gen_icount_end ( tb, num_insns );
	*tcg_ctx.gen_opc_ptr = INDEX_op_end;
	tb->size = ( dc->pc - pc_start );
	tb->icount = num_insns;
}

void m6502_gen_intermediate_code_pc ( CPUM6502State *env,
				      struct TranslationBlock *tb ) {

	printf ( "gen_intermediate_code_pc()\n" );
}

void m6502_restore_state_to_opc ( CPUM6502State *env,
				  TranslationBlock *tb, int pc_pos ) {

	printf ( "restore_state_to_opc\n" );

	env->pc = tcg_ctx.gen_opc_pc[pc_pos];
}

void m6502_translate_init ( void ) {

	printf ( "translate_init()\n" );

	cpu_env = tcg_global_reg_new_ptr ( TCG_AREG0, "env" );
	cpu_a = tcg_global_mem_new ( TCG_AREG0,
				     offsetof ( CPUM6502State, a ), "a" );
	cpu_x = tcg_global_mem_new ( TCG_AREG0,
				     offsetof ( CPUM6502State, x ), "x" );
	cpu_y = tcg_global_mem_new ( TCG_AREG0,
				     offsetof ( CPUM6502State, y ), "y" );
	cpu_p = tcg_global_mem_new ( TCG_AREG0,
				     offsetof ( CPUM6502State, p ), "p" );
	cpu_s = tcg_global_mem_new ( TCG_AREG0,
				     offsetof ( CPUM6502State, s ), "s" );
	cpu_pc = tcg_global_mem_new ( TCG_AREG0,
				      offsetof ( CPUM6502State, pc ), "pc" );
}
