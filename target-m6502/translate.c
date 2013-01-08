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

static TCGv_ptr cpu_env;
static TCGv cpu_a;
static TCGv cpu_x;
static TCGv cpu_y;
static TCGv cpu_p;
static TCGv cpu_s;
static TCGv cpu_pc;

typedef struct {
	/** Length of instruction (including opcode) */
	size_t len;
	/** Read */
	unsigned int ( * read ) ( CPUM6502State *env, hwaddr pc );
	/** Write */
	void ( * write ) ( CPUM6502State *env, hwaddr pc, unsigned int value );
} M6502AddressingMode;

static unsigned int m6502_immediate_read ( CPUM6502State *env, hwaddr pc ) {
	return cpu_ldub_code ( env, ( pc + 1 ) );
}

static const M6502AddressingMode m6502_immediate = {
	.len = 2,
	.read = m6502_immediate_read,
};

typedef struct {
	/** Generate instruction */
	void ( * gen ) ( CPUM6502State *env, hwaddr pc,
			 const M6502AddressingMode *mode );
	/** Addressing mode */
	const M6502AddressingMode *mode;
} M6502Instruction;

static void m6502_lda ( CPUM6502State *env, hwaddr pc,
			const M6502AddressingMode *mode ) {
	fprintf ( stderr, "Load accumulator with %02x\n",
		  mode->read ( env, pc ) );

	tcg_gen_movi_i32 ( cpu_a, mode->read ( env, pc ) );
}

static const M6502Instruction m6502_instructions[256] = {
	[0xa9] = { m6502_lda, &m6502_immediate },
};

static size_t m6502_gen_instruction ( CPUM6502State *env, hwaddr pc ) {
	uint8_t opcode;
	const M6502Instruction *insn;

	/* Decode opcode */
	opcode = cpu_ldub_code ( env, pc );
	insn = &m6502_instructions[opcode];
	if ( ! insn ) {
		cpu_abort ( env, "Unknown opcode %02x\n", opcode );
		return 0;
	}

	/* Generate instruction */
	insn->gen ( env, pc, insn->mode );

	return insn->mode->len;
}

void m6502_gen_intermediate_code ( CPUM6502State *env,
				   struct TranslationBlock *tb ) {
	uint8_t foo;

	printf ( "gen_intermediate_code()\n" );

	foo = cpu_ldub_code ( env, tb->pc );
	printf ( "Byte at %04x = %02x\n", tb->pc, foo );

	m6502_gen_instruction ( env, tb->pc );

	tcg_gen_exit_tb ( 0 );

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
