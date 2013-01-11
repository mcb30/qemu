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

#include "hw.h"
#include "boards.h"
#include "exec/address-spaces.h"
#include "m6522.h"

#define DEBUG_M6522 1

typedef uint8_t ( * M6522VIARegRead ) ( M6522VIA *via );
typedef void ( * M6522VIARegWrite ) ( M6522VIA *via, uint8_t data );

/* Debug messages */
#define LOG_M6522(...) do {						\
		if ( DEBUG_M6522 ) {					\
			qemu_log_mask ( CPU_LOG_IOPORT, __VA_ARGS__ );	\
		}							\
	} while ( 0 )

/******************************************************************************
 *
 * Interrupts
 *
 */

/**
 * Update 6522 VIA interrupt request
 *
 * @v via		6522 VIA
 */
static void m6522_update_irq ( M6522VIA *via ) {
	int irq;

	/* Recalculate IRQ */
	via->ifr &= ~M6522_INT_IRQ;
	irq = ( !! ( via->ifr & via->ier ) );
	if ( irq )
		via->ifr |= M6522_INT_IRQ;

	/* Report IRQ to CPU */
	qemu_set_irq ( via->irq, irq );
}

/**
 * Set 6522 VIA interrupt flags
 *
 * @v via		6522 VIA
 * @v mask		Mask of interrupts to set
 */
static inline void m6522_set_interrupts ( M6522VIA *via, uint8_t mask ) {

	/* Set interrupts in IFR */
	via->ifr |= ( mask & ~M6522_INT_IRQ );

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/**
 * Clear 6522 VIA interrupt flags
 *
 * @v via		6522 VIA
 * @v mask		Mask of interrupts to clear
 */
static inline void m6522_clear_interrupts ( M6522VIA *via, uint8_t mask ) {

	/* Clear interrupts in IFR */
	via->ifr &= ~( mask & ~M6522_INT_IRQ );

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/**
 * Read from 6522 VIA IFR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ifr_read ( M6522VIA *via ) {

	/* Read IFR */
	return via->ifr;
}

/**
 * Write to 6522 VIA IFR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ifr_write ( M6522VIA *via, uint8_t data ) {
	uint8_t mask;

	/* Decode data */
	mask = ( data & ~M6522_INT_IRQ );
	LOG_M6522 ( "%s: IFR clearing 0x%02x\n", via->name, mask );

	/* Clear interrupts */
	m6522_clear_interrupts ( via, mask );
}

/**
 * Read from 6522 VIA IER
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ier_read ( M6522VIA *via ) {

	/* Read IER.  Documentation conflicts on whether bit 7
	 * (M6522_INT_IRQ) reads as zero or one; we arbitrarily leave
	 * it as zero.
	 */
	return via->ier;
}

/**
 * Write to 6522 VIA IER
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ier_write ( M6522VIA *via, uint8_t data ) {
	int enable;
	uint8_t mask;

	/* Decode data */
	enable = ( data & M6522_INT_IRQ );
	mask = ( data & ~M6522_INT_IRQ );
	LOG_M6522 ( "%s: IER %s 0x%02x\n", via->name,
		    ( enable ? "enabling" : "disabling" ), mask );

	/* Enable or disable interrupts as specified */
	if ( enable ) {
		via->ier |= mask;
	} else {
		via->ier &= ~mask;
	}

	/* Update interrupt request */
	m6522_update_irq ( via );
}

/******************************************************************************
 *
 * Port I/O
 *
 */

/**
 * Input from 6522 VIA port
 *
 * @v via		6522 VIA
 * @v port		Port
 * @ret data		Output data
 */
static uint8_t m6522_input ( M6522VIA *via, M6522VIAPort *port ) {
	uint8_t data;

	/* Input from port */
	data = ( port->ops->input ? port->ops->input ( via, port ) : 0 );

	/* For IRB (but not IRA), pins programmed as outputs will
	 * always read back the programmed output value.
	 */
	if ( port == &via->b ) {
		data = ( ( data & ~port->ddr ) | ( port->or & port->ddr ) );
	}

	return data;
}

/**
 * Output to 6522 VIA port
 *
 * @v via		6522 VIA
 * @v port		Port
 * @v data		Output data
 */
static void m6522_output ( M6522VIA *via, M6522VIAPort *port, uint8_t data ) {

	/* Update stored OR */
	port->or = data;

	/* Output to port */
	if ( port->ops->output )
		port->ops->output ( via, port, data );
}

/**
 * Read from 6522 VIA IRB
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_irb_read ( M6522VIA *via ) {
	uint8_t data;

	/* Read from port and clear interrupts */
	data = m6522_input ( via, &via->b );
	m6522_clear_interrupts ( via, M6522_INT_CB_BOTH );

	LOG_M6522 ( "%s: IRB=0x%02x\n", via->name, data );
	return data;
}

/**
 * Read from 6522 VIA IRA
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ira_read ( M6522VIA *via ) {
	uint8_t data;

	/* Read from port and clear interrupts */
	data = m6522_input ( via, &via->a );
	m6522_clear_interrupts ( via, M6522_INT_CA_BOTH );

	LOG_M6522 ( "%s: IRA=0x%02x\n", via->name, data );
	return data;
}

/**
 * Read from 6522 VIA IRA_NO_HS
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ira_no_hs_read ( M6522VIA *via ) {
	uint8_t data;

	/* Read from port but do not clear interrupts */
	data = m6522_input ( via, &via->a );

	LOG_M6522 ( "%s: IRA_NO_HS=0x%02x\n", via->name, data );
	return data;
}

/**
 * Write to 6522 VIA ORB
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_orb_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: ORB=0x%02x\n", via->name, data );

	/* Write to port and clear interrupts */
	m6522_output ( via, &via->b, data );
	m6522_clear_interrupts ( via, M6522_INT_CB_BOTH );
}

/**
 * Write to 6522 VIA ORA
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ora_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: ORA=0x%02x\n", via->name, data );

	/* Write to port and clear interrupts */
	m6522_output ( via, &via->a, data );
	m6522_clear_interrupts ( via, M6522_INT_CA_BOTH );
}

/**
 * Write to 6522 VIA ORA_NO_HS
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ora_no_hs_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: ORA=0x%02x\n", via->name, data );

	/* Write to port but do not clear interrupts */
	m6522_output ( via, &via->a, data );
}

/**
 * Read from 6522 VIA DDRB
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ddrb_read ( M6522VIA *via ) {

	/* Read data direction register */
	return via->b.ddr;
}

/**
 * Read from 6522 VIA DDRA
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_ddra_read ( M6522VIA *via ) {

	/* Read data direction register */
	return via->a.ddr;
}

/**
 * Write to 6522 VIA DDRB
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ddrb_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: DDRB=0x%02x\n", via->name, data );

	/* Write data direction register */
	via->b.ddr = data;
}

/**
 * Write to 6522 VIA DDRA
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_ddra_write ( M6522VIA *via, uint8_t data ) {

	LOG_M6522 ( "%s: DDRA=0x%02x\n", via->name, data );

	/* Write data direction register */
	via->a.ddr = data;
}

/******************************************************************************
 *
 * Timers
 *
 */

/**
 * Start timer
 *
 * @v via		6522 VIA
 * @v timer		Timer
 */
static void m6522_timer_start ( M6522VIA *via, M6522VIATimer *timer ) {

	/* Load counter low byte from latches */
	timer->c &= ~0xff;
	timer->c |= ( timer->l & 0xff );

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented timer start 0x%04x\n",
			via->name, timer->c );
}

/**
 * Read from 6522 VIA timer latch low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_latch_read_low ( M6522VIA *via, M6522VIATimer *timer ) {

	return ( ( timer->l >> 0 ) & 0xff );
}

/**
 * Read from 6522 VIA timer latch high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_latch_read_high ( M6522VIA *via, M6522VIATimer *timer ) {

	return ( ( timer->l >> 8 ) & 0xff );
}

/**
 * Read from 6522 VIA timer counter low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_counter_read_low ( M6522VIA *via, M6522VIATimer *timer ) {

	return ( ( timer->c >> 0 ) & 0xff );
}

/**
 * Read from 6522 VIA timer counter high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @ret data		Data
 */
static uint8_t m6522_counter_read_high ( M6522VIA *via, M6522VIATimer *timer ) {

	return ( ( timer->c >> 8 ) & 0xff );
}

/**
 * Write to 6522 VIA timer latch low byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_latch_write_low ( M6522VIA *via, M6522VIATimer *timer,
				    uint8_t data ) {

	/* Update latch value */
	timer->l &= ~0xff;
	timer->l |= data;
}

/**
 * Write to 6522 VIA timer latch high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_latch_write_high ( M6522VIA *via, M6522VIATimer *timer,
				     uint8_t data ) {

	/* Update latch value */
	timer->l &= ~0xff00;
	timer->l |= ( data << 8 );
}

/**
 * Write to 6522 VIA timer counter high byte
 *
 * @v via		6522 VIA
 * @v timer		Timer
 * @v data		Data
 */
static void m6522_counter_write_high ( M6522VIA *via, M6522VIATimer *timer,
				       uint8_t data ) {

	/* Update counter value */
	timer->c &= ~0xff00;
	timer->c |= ( data << 8 );
}

/**
 * Read from 6522 VIA T1C_L
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t1c_l_read ( M6522VIA *via ) {

	/* Read counter and clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_T1 );
	return m6522_counter_read_low ( via, &via->t1 );
}

/**
 * Read from 6522 VIA T1C_H
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t1c_h_read ( M6522VIA *via ) {

	/* Read counter */
	return m6522_counter_read_high ( via, &via->t1 );
}

/**
 * Read from 6522 VIA T1L_L
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t1l_l_read ( M6522VIA *via ) {

	/* Read latches */
	return m6522_latch_read_low ( via, &via->t1 );
}

/**
 * Read from 6522 VIA T1L_H
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t1l_h_read ( M6522VIA *via ) {

	/* Read latches */
	return m6522_latch_read_high ( via, &via->t1 );
}

/**
 * Read from 6522 VIA T2C_L
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t2c_l_read ( M6522VIA *via ) {

	/* Read counter and clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_T2 );
	return m6522_counter_read_low ( via, &via->t2 );
}

/**
 * Read from 6522 VIA T2C_H
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_t2c_h_read ( M6522VIA *via ) {

	/* Read counter */
	return m6522_counter_read_high ( via, &via->t2 );
}

/**
 * Write to 6522 VIA T1C_L
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t1c_l_write ( M6522VIA *via, uint8_t data ) {

	/* Write latches (not counter) */
	LOG_M6522 ( "%s: T1C_L=0x%02x\n", via->name, data );
	m6522_latch_write_low ( via, &via->t1, data );
}

/**
 * Write to 6522 VIA T1C_H
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t1c_h_write ( M6522VIA *via, uint8_t data ) {

	/* Write latches, and counter, clear interrupts, and start timer */
	LOG_M6522 ( "%s: T1C_H=0x%02x\n", via->name, data );
	m6522_latch_write_high ( via, &via->t1, data );
	m6522_counter_write_high ( via, &via->t1, data );
	m6522_clear_interrupts ( via, M6522_INT_T1 );
	m6522_timer_start ( via, &via->t1 );
}

/**
 * Write to 6522 VIA T1L_L
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t1l_l_write ( M6522VIA *via, uint8_t data ) {

	/* Write latches */
	LOG_M6522 ( "%s: T1L_L=0x%02x\n", via->name, data );
	m6522_latch_write_low ( via, &via->t1, data );
}

/**
 * Write to 6522 VIA T1L_H
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t1l_h_write ( M6522VIA *via, uint8_t data ) {

	/* Write latches, clear interrupts, do not start timer */
	LOG_M6522 ( "%s: T1L_H=0x%02x\n", via->name, data );
	m6522_latch_write_high ( via, &via->t1, data );
	m6522_clear_interrupts ( via, M6522_INT_T1 );
}

/**
 * Write to 6522 VIA T2C_L
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t2c_l_write ( M6522VIA *via, uint8_t data ) {

	/* Write latches (not counter) */
	LOG_M6522 ( "%s: T2C_L=0x%02x\n", via->name, data );
	m6522_latch_write_low ( via, &via->t2, data );
}

/**
 * Write to 6522 VIA T2C_H
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_t2c_h_write ( M6522VIA *via, uint8_t data ) {

	/* Write counter, clear interrupts, and start timer */
	LOG_M6522 ( "%s: T2C_H=0x%02x\n", via->name, data );
	m6522_counter_write_high ( via, &via->t2, data );
	m6522_clear_interrupts ( via, M6522_INT_T2 );
	m6522_timer_start ( via, &via->t2 );
}

/******************************************************************************
 *
 * Shift register
 *
 */

/**
 * Read from 6522 VIA SR
 *
 * @v via		6522 VIA
 * @ret data		Data
 */
static uint8_t m6522_sr_read ( M6522VIA *via ) {

	/* Clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_SR );

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from shift "
			"register\n", via->name );
	return 0;
}

/**
 * Write to 6522 VIA SR
 *
 * @v via		6522 VIA
 * @v data		Data
 */
static void m6522_sr_write ( M6522VIA *via, uint8_t data ) {

	/* Clear interrupts */
	m6522_clear_interrupts ( via, M6522_INT_SR );

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to shift "
			"register\n", via->name, data );
}

/******************************************************************************
 *
 * VIA
 *
 */

/**
 * Read from 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t m6522_read ( void *opaque, hwaddr addr, unsigned int size ) {
	static const M6522VIARegRead read[M6522_SIZE] = {
		[M6522_IRB] = m6522_irb_read,
		[M6522_IRA] = m6522_ira_read,
		[M6522_DDRB] = m6522_ddrb_read,
		[M6522_DDRA] = m6522_ddra_read,
		[M6522_T1C_L] = m6522_t1c_l_read,
		[M6522_T1C_H] = m6522_t1c_h_read,
		[M6522_T1L_L] = m6522_t1l_l_read,
		[M6522_T1L_H] = m6522_t1l_h_read,
		[M6522_T2C_L] = m6522_t2c_l_read,
		[M6522_T2C_H] = m6522_t2c_h_read,
		[M6522_SR] = m6522_sr_read,
		[M6522_IFR] = m6522_ifr_read,
		[M6522_IER] = m6522_ier_read,
		[M6522_IRA_NO_HS] = m6522_ira_no_hs_read,
	};
	M6522VIA *via = opaque;

	/* Read from specified register */
	if ( read[addr] ) {
		return read[addr] ( via );
	} else {
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", via->name, addr );
		return 0;
	}
}

/**
 * Write to 6522 VIA
 *
 * @v opaque		6522 VIA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void m6522_write ( void *opaque, hwaddr addr, uint64_t data64,
			  unsigned int size ) {
	static const M6522VIARegWrite write[M6522_SIZE] = {
		[M6522_ORB] = m6522_orb_write,
		[M6522_ORA] = m6522_ora_write,
		[M6522_DDRB] = m6522_ddrb_write,
		[M6522_DDRA] = m6522_ddra_write,
		[M6522_T1C_L] = m6522_t1c_l_write,
		[M6522_T1C_H] = m6522_t1c_h_write,
		[M6522_T1L_L] = m6522_t1l_l_write,
		[M6522_T1L_H] = m6522_t1l_h_write,
		[M6522_T2C_L] = m6522_t2c_l_write,
		[M6522_T2C_H] = m6522_t2c_h_write,
		[M6522_SR] = m6522_sr_write,
		[M6522_IFR] = m6522_ifr_write,
		[M6522_IER] = m6522_ier_write,
		[M6522_ORA_NO_HS] = m6522_ora_no_hs_write,
	};
	M6522VIA *via = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	if ( write[addr] ) {
		write[addr] ( via, data );
	} else {
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write 0x%02x to "
				"0x%02lx\n", via->name, data, addr );
	}
}

/** 6522 VIA operations */
static const MemoryRegionOps m6522_ops = {
	.read = m6522_read,
	.write = m6522_write,
};

/** Virtual machine state description */
static const VMStateDescription vmstate_m6522 = {
	.name = "m6522",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( b.or, M6522VIA ),
		VMSTATE_UINT8 ( b.ddr, M6522VIA ),
		VMSTATE_UINT8 ( a.or, M6522VIA ),
		VMSTATE_UINT8 ( a.ddr, M6522VIA ),
		VMSTATE_UINT16 ( t1.l, M6522VIA ),
		VMSTATE_UINT16 ( t1.c, M6522VIA ),
		VMSTATE_UINT16 ( t2.l, M6522VIA ),
		VMSTATE_UINT16 ( t2.c, M6522VIA ),
		VMSTATE_UINT8 ( sr, M6522VIA ),
		VMSTATE_UINT8 ( acr, M6522VIA ),
		VMSTATE_UINT8 ( pcr, M6522VIA ),
		VMSTATE_UINT8 ( ifr, M6522VIA ),
		VMSTATE_UINT8 ( ier, M6522VIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 6522 VIA
 *
 * @v parent		Parent memory region
 * @v offset		Offset within memory region
 * @v name		Device name
 * @v ops		VIA operations
 * @v irq		Interrupt request line
 */
void m6522_init ( MemoryRegion *parent, hwaddr offset, const char *name,
		  const M6522VIAOps *ops, qemu_irq irq ) {
	M6522VIA *via = g_new0 ( M6522VIA, 1 );

	/* Initialise VIA */
	via->name = name;
	via->b.ops = &ops->b;
	via->a.ops = &ops->a;
	via->irq = irq;

	/* Register memory region */
	memory_region_init_io ( &via->mr, &m6522_ops, via, via->name,
				M6522_SIZE );
	memory_region_add_subregion ( parent, offset, &via->mr );

	/* Register virtual machine state */
	vmstate_register ( NULL, offset, &vmstate_m6522, via );
}
