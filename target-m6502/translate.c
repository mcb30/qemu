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

#define LOG_DIS( ... ) qemu_log_mask ( CPU_LOG_TB_IN_ASM, ## __VA_ARGS__ )

#define MEM_INDEX 0

typedef struct {
	/** TCG variable */
	TCGv_i32 var;
	/** Name (as used in disassembly) */
	char name;
} M6502Register;

static TCGv_ptr cpu_env;
static M6502Register cpu_a = { .name = 'A' };
static M6502Register cpu_x = { .name = 'X' };
static M6502Register cpu_y = { .name = 'Y' };
static M6502Register cpu_p_c = { .name = 'C' };
static M6502Register cpu_p_z = { .name = 'Z' };
static M6502Register cpu_p_i = { .name = 'I' };
static M6502Register cpu_p_d = { .name = 'D' };
static M6502Register cpu_p_b = { .name = 'B' };
static M6502Register cpu_p_u = { .name = 'U' };
static M6502Register cpu_p_v = { .name = 'V' };
static M6502Register cpu_p_n = { .name = 'N' };
static M6502Register cpu_s = { .name = 'S' };
static TCGv_i32 cpu_pc;

#include "exec/gen-icount.h"

typedef struct DisasContext DisasContext;

/** An instruction definition */
typedef struct {
	/** Generate instruction */
	void ( * gen ) ( DisasContext *dc );
	/** Destination register (if any) */
	M6502Register *dest;
	/** Source register (if any) */
	M6502Register *src;
	/** Memory addressing mode (if any) */
	void ( * mem ) ( DisasContext *dc );
	/** Length of instruction */
	size_t len;
} M6502Instruction;

struct DisasContext {
	CPUM6502State *env;
	uint32_t pc;

	const M6502Instruction *insn;
	TCGv_i32 address;
	char address_desc[16];
};

static void m6502_address_desc ( DisasContext *dc, const char *fmt, ... ) {
	va_list args;

	if ( qemu_loglevel_mask ( CPU_LOG_TB_IN_ASM ) ) {
		va_start ( args, fmt );
		vsnprintf ( dc->address_desc, sizeof ( dc->address_desc ),
			    fmt, args );
		va_end ( args );
	}
}

/* Absolute addressing mode */
static void m6502_abs ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );

	m6502_address_desc ( dc, "&%04X", base );
}

/* Absolute,X addressing mode */
static void m6502_abs_x ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	m6502_address_desc ( dc, "&%04X,X", base );
}

/* Absolute,Y addressing mode */
static void m6502_abs_y ( DisasContext *dc ) {
	uint16_t base = cpu_lduw_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ADDRESS_MASK );

	m6502_address_desc ( dc, "&%04X,Y", base );
}

/* Zero-page addressing mode */
static void m6502_zero ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );

	m6502_address_desc ( dc, "&%02X", base );
}

/* Zero-page,X addressing mode */
static void m6502_zero_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	m6502_address_desc ( dc, "&%02X,X", base );
}

/* Zero-page,Y addressing mode */
static void m6502_zero_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_y.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );

	m6502_address_desc ( dc, "&%02X,Y", base );
}

/* (Indirect,X) addressing mode */
static void m6502_ind_x ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_temp_new_i32();
	tcg_gen_addi_i32 ( dc->address, cpu_x.var, base );
	tcg_gen_andi_i32 ( dc->address, dc->address, M6502_ZERO_PAGE_MASK );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );

	m6502_address_desc ( dc, "(&%02X,X)", base );
}

/* (Indirect),Y addressing mode */
static void m6502_ind_y ( DisasContext *dc ) {
	uint8_t base = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );

	dc->address = tcg_const_i32 ( base );
	tcg_gen_qemu_ld16u ( dc->address, dc->address, MEM_INDEX );
	tcg_gen_add_i32 ( dc->address, dc->address, cpu_y.var );

	m6502_address_desc ( dc, "(&%02X),Y", base );
}

/* LDA, LDX, LDY (immediate) */
static void m6502_gen_load_imm ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	uint8_t value;

	/* Load register */
	value = cpu_ldub_code ( dc->env, ( dc->pc + 1 ) );
	tcg_gen_movi_i32 ( dest->var, value );

	/* Update flags */
	tcg_gen_movi_i32 ( cpu_p_z.var, ( ( value == 0 ) ? 1 : 0 ) );
	tcg_gen_movi_i32 ( cpu_p_n.var, ( ( value & 0x80 ) ? 1 : 0 ) );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%c #&%02X\n", dc->pc, dest->name, value );
}

/* LDA, LDX, LDY (memory) */
static void m6502_gen_load ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Load register */
	tcg_gen_qemu_ld8u ( dest->var, dc->address, MEM_INDEX );

	/* Update flags */
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, dest->var, 0 );
	tcg_gen_setcondi_i32 ( TCG_COND_GE, cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : LD%c %s\n", dc->pc, dest->name, dc->address_desc );
}

/* STA, STX, STY */
static void m6502_gen_store ( DisasContext *dc ) {
	M6502Register *src = dc->insn->src;

	/* Store register */
	tcg_gen_qemu_st8 ( src->var, dc->address, MEM_INDEX );

	/* Flags are not affected */

	/* Generate disassembly */
	LOG_DIS ( "&%04X : ST%c %s\n", dc->pc, src->name, dc->address_desc );
}

/* TAX, TAY, TSX, TXA, TXS, TYA */
static void m6502_gen_transfer ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;
	M6502Register *src = dc->insn->src;

	/* Transfer register */
	tcg_gen_mov_i32 ( dest->var, src->var );

	/* Update flags */
	tcg_gen_setcondi_i32 ( TCG_COND_EQ, cpu_p_z.var, dest->var, 0 );
	tcg_gen_setcondi_i32 ( TCG_COND_GE, cpu_p_n.var, dest->var, 0x80 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : T%c%c\n", dc->pc, src->name, dest->name );
}

/* SEC, SED, SEI */
static void m6502_set_flag ( DisasContext *dc ) {
	M6502Register *dest = dc->insn->dest;

	/* Set flag */
	tcg_gen_movi_i32 ( dest->var, 1 );

	/* Generate disassembly */
	LOG_DIS ( "&%04X : SE%c\n", dc->pc, dest->name );
}

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
	/* SEC */
	[0x38] = { m6502_set_flag,	&cpu_p_c, NULL,	NULL,		1 },
	/* SED */
	[0xf8] = { m6502_set_flag,	&cpu_p_d, NULL,	NULL,		1 },
	/* SEI */
	[0x78] = { m6502_set_flag,	&cpu_p_i, NULL,	NULL,		1 },
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
	uint8_t opcode;

	/* Fetch and validate opcode */
	opcode = cpu_ldub_code ( dc->env, dc->pc );
	dc->insn = insn = &m6502_instructions[opcode];
	if ( ! insn->gen ) {

		//
		return 0;

		cpu_abort ( dc->env, "Unknown opcode %02x at %04x\n",
			    opcode, dc->pc );
		return 0;
	}

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
	cpu_a.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, a ), "a" );
	cpu_x.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, x ), "x" );
	cpu_y.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, y ), "y" );
	cpu_p_c.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_C] ),
					   "p.c" );
	cpu_p_z.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_Z] ),
					   "p.c" );
	cpu_p_i.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_I] ),
					   "p.c" );
	cpu_p_d.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_D] ),
					   "p.c" );
	cpu_p_b.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_B] ),
					   "p.c" );
	cpu_p_u.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_U] ),
					   "p.c" );
	cpu_p_v.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_V] ),
					   "p.c" );
	cpu_p_n.var = tcg_global_mem_new ( TCG_AREG0,
					   offsetof ( CPUM6502State, p[P_N] ),
					   "p.c" );
	cpu_s.var = tcg_global_mem_new ( TCG_AREG0,
					 offsetof ( CPUM6502State, s ), "s" );
	cpu_pc = tcg_global_mem_new ( TCG_AREG0,
				      offsetof ( CPUM6502State, pc ), "pc" );
}
