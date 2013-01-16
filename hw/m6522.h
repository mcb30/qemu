/*
 * MOS Technology 6522 virtual VIA
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

#ifndef HW_M6522_H
#define HW_M6522_H

typedef struct M6522VIA M6522VIA;

/** 6522 VIA port operations */
typedef struct {
	/**
	 * Input from port
	 *
	 * @v opaque		Opaque pointer
	 * @ret data		Input data
	 */
	uint8_t ( * input ) ( void *opaque );
	/**
	 * Output to port
	 *
	 * @v opaque		Opaque pointer
	 * @v data		Output data
	 */
	void ( * output ) ( void *opaque, uint8_t data );
} M6522VIAPortOps;

/** 6522 VIA operations */
typedef struct {
	/** Port A operations */
	M6522VIAPortOps a;
	/** Port B operations */
	M6522VIAPortOps b;
} M6522VIAOps;

/** A 6522 VIA control line */
typedef struct {
	/** IRQ when used as an input */
	qemu_irq in;
	/** IRQ when used as an output */
	qemu_irq out;
	/** Interrupt flag bit */
	uint8_t ifr;

	/** Previous state of control line input */
	int32_t previous;
} M6522ViaControl;

/** A 6522 VIA port */
typedef struct {
	/** Containing VIA */
	M6522VIA *via;
	/** Name */
	const char *name;
	/** Operations */
	const M6522VIAPortOps *ops;
	/** PCR shift */
	unsigned int pcr_shift;
	/** Control line 1 */
	M6522ViaControl c1;
	/** Control line 2 */
	M6522ViaControl c2;

	/** Output register */
	uint8_t or;
	/** Data direction register */
	uint8_t ddr;
} M6522VIAPort;

/** A 6522 VIA timer */
typedef struct {
	/** Containing VIA */
	M6522VIA *via;
	/** Name */
	const char *name;
	/** Interrupt flag bit */
	uint8_t ifr;
	/** Timer */
	QEMUTimer *timer;

	/** Latches */
	uint16_t latch;
	/** Counter */
	uint16_t counter;
} M6522VIATimer;

/** A 6522 VIA */
struct M6522VIA {
	/** Name */
	const char *name;
	/** Memory region */
	MemoryRegion mr;
	/** IRQ */
	qemu_irq irq;
	/** Opaque pointer */
	void *opaque;

	/** Clock tick duration (in nanoseconds) */
	unsigned long tick_ns;
	/** Port A */
	M6522VIAPort a;
	/** Port B */
	M6522VIAPort b;
	/** Timer 1 */
	M6522VIATimer t1;
	/** Timer 2 */
	M6522VIATimer t2;
	/** Shift register */
	uint8_t sr;
	/** Auxiliary control register */
	uint8_t acr;
	/** Peripheral control register */
	uint8_t pcr;
	/** Interrupt flag register */
	uint8_t ifr;
	/** Interrupt enable register */
	uint8_t ier;
};

/** Size of memory region */
#define M6522_SIZE 0x10

/* Register addresses */
#define M6522_ORB 0x0
#define M6522_IRB 0x0
#define M6522_ORA 0x1
#define M6522_IRA 0x1
#define M6522_DDRB 0x2
#define M6522_DDRA 0x3
#define M6522_T1C_L 0x4
#define M6522_T1C_H 0x5
#define M6522_T1L_L 0x6
#define M6522_T1L_H 0x7
#define M6522_T2C_L 0x8
#define M6522_T2C_H 0x9
#define M6522_SR 0xa
#define M6522_ACR 0xb
#define M6522_PCR 0xc
#define M6522_IFR 0xd
#define M6522_IER 0xe
#define M6522_ORA_NO_HS 0xf
#define M6522_IRA_NO_HS 0xf

/* Interrupts */
#define M6522_INT_CA2 0x01
#define M6522_INT_CA1 0x02
#define M6522_INT_SR 0x04
#define M6522_INT_CB2 0x08
#define M6522_INT_CB1 0x10
#define M6522_INT_T2 0x20
#define M6522_INT_T1 0x40
#define M6522_INT_IRQ 0x80

/* Peripheral control register */
#define M6522_PCR_CA_SHIFT 0
#define M6522_PCR_CB_SHIFT 4
#define M6522_PCR_C1_MASK 0x01
#define M6522_PCR_C1_POSITIVE 0x01
#define M6522_PCR_C2_MASK 0x0e
#define M6522_PCR_C2_INPUT_INDEPENDENT 0x02
#define M6522_PCR_C2_OUTPUT_PULSE 0x02
#define M6522_PCR_C2_OUTPUT_FIXED_LEVEL 0x02
#define M6522_PCR_C2_INPUT_POSITIVE 0x04
#define M6522_PCR_C2_OUTPUT_FIXED 0x04
#define M6522_PCR_C2_OUTPUT 0x08

/* Auxiliary control register */
#define M6522_ACR_T2_COUNT 0x20
#define M6522_ACR_T1_CONTINUOUS 0x40

extern M6522VIA * m6522_init ( MemoryRegion *parent, hwaddr offset,
			       const char *name, void *opaque,
			       const M6522VIAOps *ops, qemu_irq irq,
			       unsigned long tick_ns );

#endif
