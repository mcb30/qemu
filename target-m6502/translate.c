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
	TCGv_i32 reg;
	TCGv_i32 address;
} DisasContext;

typedef struct {
	/** Generate instruction */
	void ( * gen ) ( DisasContext *dc );
	/** Register (if any) */
	TCGv_i32 *reg;
	/** Addressing mode (if any) */
	void ( * gen_address ) ( DisasContext *dc );
	/** Length of instruction */
	size_t len;
} M6502Instruction;

static void m6502_absolute ( DisasContext *dc ) {

	dc->address = tcg_const_i32 ( cpu_lduw_code ( dc->env,
						      ( dc->pc + 1 ) ) );
}

static void m6502_gen_load_immediate ( DisasContext *dc ) {
	uint8_t value;

	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_movi_i32 ( dc->reg, value );
}

static void m6502_gen_store ( DisasContext *dc ) {
	
	tcg_gen_qemu_st8 ( dc->reg, dc->address, MEM_INDEX );
}

#if 0
static void m6502_gen_inc_zeropage_x ( CPUM6502State *env, hwaddr pc ) {
	TCGv_i32 address = tcg_temp_new_i32();
	TCGv_i32 value = tcg_temp_new_i32();

	/* Calculate address within zero page */
	tcg_gen_addi_i32 ( address, cpu_x, cpu_ldub_code ( env, ( pc + 1 ) ) );
	tcg_gen_andi_i32 ( address, address, 0xff );

	tcg_gen_qemu_ld8u ( value, address, MEM_INDEX );
	tcg_gen_addi_i32 ( value, value, 1 );
	tcg_gen_qemu_st8 ( value, address, MEM_INDEX );

	tcg_temp_free_i32 ( value );
	tcg_temp_free_i32 ( address );
}
#endif

static const M6502Instruction m6502_instructions[256] = {
	[0x8d] = { m6502_gen_store, &cpu_a, m6502_absolute, 3 },
	[0xa9] = { m6502_gen_load_immediate, &cpu_a, NULL, 2 },
};

static size_t m6502_gen_instruction ( DisasContext *dc ) {
	const M6502Instruction *insn;

	/* Fetch and validate opcode */
	dc->opcode = cpu_ldub_code ( dc->env, dc->pc );
	insn = &m6502_instructions[dc->opcode];
	if ( ! insn->gen ) {

		//
		return 0;

		cpu_abort ( dc->env, "Unknown opcode %02x at %04x\n",
			    dc->opcode, dc->pc );
		return 0;
	}

	/* Identify register, if applicable */
	if ( insn->reg )
		dc->reg = *(insn->reg);

	/* Generate address, if applicable */
	if ( insn->gen_address )
		insn->gen_address ( dc );

	/* Generate instruction */
	insn->gen ( dc );

	/* Free address, if one was generated */
	if ( insn->gen_address )
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
