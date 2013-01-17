/*
 * Acorn BBC Micro
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
#include "sysemu/sysemu.h"
#include "ui/console.h"
#include "char/char.h"
#include "loader.h"
#include "bbc.h"

/******************************************************************************
 *
 * Unimplemented I/O region
 *
 */

/**
 * Read from unimplemented I/O region
 *
 * @v opaque		Unimplemented I/O region
 * @v addr		Address within region
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_unimplemented_read ( void *opaque, hwaddr addr,
					 unsigned int size ) {
	BBCUnimplemented *unimp = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from &%02lX\n",
			unimp->name, addr );
	return 0;
}

/**
 * Write to unimplemented I/O region
 *
 * @v opaque		Unimplemented I/O region
 * @v addr		Address within region
 * @v data		Data to write
 * @v size		Size of write
 */
static void bbc_unimplemented_write ( void *opaque, hwaddr addr, uint64_t data,
				      unsigned int size ) {
	BBCUnimplemented *unimp = opaque;

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write to &%02lX\n",
			unimp->name, addr );
}

/** Unimplemented I/O region operations */
static const MemoryRegionOps bbc_unimplemented_ops = {
	.read = bbc_unimplemented_read,
	.write = bbc_unimplemented_write,
};

/**
 * Initialise unimplemented I/O region
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret unimp		Unimplemented I/O region
 */
static BBCUnimplemented * bbc_unimplemented_init ( hwaddr addr, uint64_t size,
						   const char *name ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCUnimplemented *unimp = g_new0 ( BBCUnimplemented, 1 );

	/* Initialise unimplemented I/O region */
	unimp->name = name;

	/* Initialise memory region */
	memory_region_init_io ( &unimp->mr, &bbc_unimplemented_ops, unimp,
				name, size );
	memory_region_add_subregion ( address_space_mem, addr, &unimp->mr );

	return unimp;
}

/******************************************************************************
 *
 * I/O aliases
 *
 */

/**
 * Create I/O aliases
 *
 * @v mr		I/O peripheral memory region
 * @v addr		Starting address of region to be aliased
 * @v size		Total size of region to be aliased
 * @v offset		Offset of peripheral within region
 * @v stride		Distance between each alias
 */
static void bbc_io_alias_offset ( MemoryRegion *mr, hwaddr addr, uint64_t size,
				  hwaddr offset, uint64_t stride ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *alias;
	const char *alias_name;
	hwaddr end;
	unsigned int i;

	/* Calculate end address for aliased region */
	end = ( addr + size );

	/* Skip first alias (which will be the original peripheral */
	addr += stride;

	/* Add aliases to fill out the memory space */
	for ( i = 1 ; addr < end ; addr += stride, i++ ) {
		alias = g_new0 ( MemoryRegion, 1 );
		alias_name = g_strdup_printf ( "%s.%d",
					       memory_region_name ( mr ), i );
		memory_region_init_alias ( alias, alias_name, mr, 0,
					   memory_region_size ( mr ) );
		memory_region_add_subregion ( address_space_mem,
					      ( addr + offset ), alias );
	}
}

/**
 * Create I/O aliases
 *
 * @v mr		I/O peripheral memory region
 * @v addr		Starting address of region to be aliased
 * @v size		Total size of region to be aliased
 */
static void bbc_io_alias ( MemoryRegion *mr, hwaddr addr, uint64_t size ) {

	bbc_io_alias_offset ( mr, addr, size, 0, memory_region_size ( mr ) );
}

/******************************************************************************
 *
 * ROM loading
 *
 */

/**
 * Load ROM
 *
 * @v roms		ROM bank memory region
 * @v offset		Offset within ROM bank memory region
 * @v targphys		Target physical address for load_image_targphys()
 * @v size		Size
 * @v name		Name
 * @v filename		Filename
 * @ret rom		ROM
 */
static BBCROM * bbc_load_rom ( MemoryRegion *roms, hwaddr offset,
			       hwaddr targphys, uint64_t size,
			       const char *name, const char *filename ) {
	BBCROM *rom = g_new0 ( BBCROM, 1 );
	const char *actual_filename;
	int actual_size;

	/* Initialise ROM */
	rom->name = name;

	/* Initialise memory region */
	memory_region_init_ram ( &rom->mr, name, size );
	vmstate_register_ram_global ( &rom->mr );
	memory_region_set_readonly ( &rom->mr, true );
	memory_region_add_subregion ( roms, offset, &rom->mr );

	/* Locate ROM file */
	actual_filename = qemu_find_file ( QEMU_FILE_TYPE_BIOS, filename );
	if ( ! actual_filename ) {
		fprintf ( stderr, "qemu: could not find ROM '%s'\n",
			  filename );
		exit ( 1 );
	}

	/* Check size */
	actual_size = load_image_targphys ( actual_filename, targphys, size );
	if ( actual_size < 0 ) {
		fprintf ( stderr, "qemu: could not load (or bad size) ROM "
			  "'%s'\n", actual_filename );
		exit ( 1 );
	}

	return rom;
}

/******************************************************************************
 *
 * MOS ROM
 *
 */

/**
 * Initialise MOS
 *
 * @v addr		Address
 * @v size		Size
 * @v low_size		Size of low portion of MOS
 * @v high_size		Size of high portion of MOS
 * @v name		Name
 * @v raw_addr		Address of raw image of MOS
 */
static BBCMOS * bbc_mos_init ( hwaddr addr, uint64_t size, uint64_t low_size,
			       uint64_t high_size, const char *name,
			       hwaddr raw_addr ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCMOS *mos = g_new0 ( BBCMOS, 1 );
	const char *low_name;
	const char *high_name;

	/* Initialise MOS */
	mos->name = name;
	mos->addr = addr;
	mos->size = size;

	/* Initialise raw image memory region */
	memory_region_init ( &mos->raw, name, size );
	memory_region_set_readonly ( &mos->raw, true );
	memory_region_add_subregion ( address_space_mem, raw_addr, &mos->raw );

	/* Initialise low section memory region */
	low_name = g_strdup_printf ( "%s.low", name );
	memory_region_init_alias ( &mos->low, low_name, &mos->raw, 0,
				   low_size );
	memory_region_set_readonly ( &mos->low, true );
	memory_region_add_subregion ( address_space_mem, addr, &mos->low );

	/* Initialise high section memory region */
	high_name = g_strdup_printf ( "%s.high", name );
	memory_region_init_alias ( &mos->high, high_name, &mos->raw,
				   ( size - high_size ), high_size );
	memory_region_set_readonly ( &mos->high, true );
	memory_region_add_subregion ( address_space_mem,
				      ( addr + size - high_size ), &mos->high );

	return mos;
}

/**
 * Load MOS ROM
 *
 * @v mos		MOS
 * @v default_filename	Default filename to use if none specified
 */
static void bbc_mos_load ( BBCMOS *mos, const char *default_filename ) {
	const char *name;

	/* Use default filename if no 'BIOS' name is specified */
	if ( bios_name == NULL )
		bios_name = default_filename;

	/* Load MOS ROM */
	name = g_strdup_printf ( "mos %s", bios_name );
	bbc_load_rom ( &mos->raw, 0, mos->addr, mos->size, name, bios_name );
}

/******************************************************************************
 *
 * Paged ROMs
 *
 */

/**
 * Calculate paged ROM bank offset
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret offset		Offset within page ROM bank
 */
static inline hwaddr bbc_paged_rom_offset ( BBCPagedROM *paged,
					    unsigned int page ) {

	return ( page * paged->size );
}

/**
 * Calculate paged ROM bank physical address for load_image_targphys()
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @ret targphys	Target physical address
 */
static inline hwaddr bbc_paged_rom_targphys ( BBCPagedROM *paged,
					      unsigned int page ) {

	return ( paged->roms_addr + bbc_paged_rom_offset ( paged, page ) );
}

/**
 * Update paged ROM memory alias
 *
 * @v paged		Paged ROM
 */
static void bbc_paged_rom_update_alias ( BBCPagedROM *paged ) {
	hwaddr offset;

	/* Change offset address into paged ROM virtual memory region */
	offset = bbc_paged_rom_offset ( paged, paged->page );
	memory_region_set_alias_offset ( &paged->rom, offset );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: ROM %d activated\n",
			paged->name, paged->page );
}

/**
 * Read from paged ROM select register
 *
 * @v opaque		Paged ROM
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_paged_rom_select_read ( void *opaque, hwaddr addr,
					    unsigned int size ) {

	/* This is a write-only register */
	return 0xff;
}

/**
 * Write to paged ROM select register
 *
 * @v opaque		Paged ROM
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_paged_rom_select_write ( void *opaque, hwaddr addr,
					 uint64_t data64, unsigned int size ) {
	BBCPagedROM *paged = opaque;
	uint8_t data = data64;

	/* Store page and update memory alias */
	paged->page = ( data & ( paged->count - 1 ) );
	bbc_paged_rom_update_alias ( paged );
}

/** Paged ROM select register operations */
static const MemoryRegionOps bbc_paged_rom_select_ops = {
	.read = bbc_paged_rom_select_read,
	.write = bbc_paged_rom_select_write,
};

/**
 * Update paged ROM state after loading from snapshot
 *
 * @v opaque		Paged ROM
 * @v version_id	State description version ID
 */
static int bbc_paged_rom_post_load ( void *opaque, int version_id ) {
	BBCPagedROM *paged = opaque;

	/* Update memory alias */
	bbc_paged_rom_update_alias ( paged );

	return 0;
}

/** Paged ROM state description */
static const VMStateDescription vmstate_bbc_paged_rom = {
	.name = "bbc_paged_rom",
	.version_id = 1,
	.minimum_version_id = 1,
	.post_load = bbc_paged_rom_post_load,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( page, BBCPagedROM ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise paged ROM
 *
 * @v addr		Address of paged ROM memory region
 * @v size		Size of memory region
 * @v name		Device name
 * @v roms_addr		Address of ROM bank memory region
 * @v count		Number of paged ROMs
 * @v select_addr	Address of paged ROM select register memory region
 * @v select_size	Size of paged ROM select register memory region
 * @ret paged		Paged ROM
 */
static BBCPagedROM * bbc_paged_rom_init ( hwaddr addr, uint64_t size,
					  const char *name, hwaddr roms_addr,
					  unsigned int count,
					  hwaddr select_addr,
					  uint64_t select_size ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCPagedROM *paged = g_new0 ( BBCPagedROM, 1 );
	const char *roms_name;
	const char *select_name;

	/* Initialise paged ROM */
	paged->name = name;
	paged->roms_addr = roms_addr;
	paged->size = size;
	paged->count = count;

	/* Initialise ROM bank memory region.  There is no way for the
	 * CPU to access this region directly, but it gives us an
	 * address to pass to load_image_targphys().
	 */
	roms_name = g_strdup_printf ( "%s.roms", name );
	memory_region_init ( &paged->roms, roms_name,
			     ( paged->count * paged->size ) );
	memory_region_set_readonly ( &paged->roms, true );
	memory_region_add_subregion ( address_space_mem, roms_addr,
				      &paged->roms );

	/* Initialise paged ROM memory region */
	memory_region_init_alias ( &paged->rom, name, &paged->roms,
				   0, paged->size );
	memory_region_set_readonly ( &paged->rom, true );
	memory_region_add_subregion ( address_space_mem, addr, &paged->rom );

	/* Initialise select register memory region */
	select_name = g_strdup_printf ( "%s.select", name );
	memory_region_init_io ( &paged->select, &bbc_paged_rom_select_ops,
				paged, select_name, BBC_PAGED_ROM_SELECT_SIZE );
	memory_region_add_subregion ( address_space_mem, select_addr,
				      &paged->select );	
	bbc_io_alias ( &paged->select, select_addr, select_size );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_paged_rom, paged );

	return paged;
}

/**
 * Load paged ROM
 *
 * @v paged		Paged ROM
 * @v page		Page
 * @v filename		Filename
 */
static void bbc_paged_rom_load ( BBCPagedROM *paged, unsigned int page,
				 const char *filename ) {
	const char *name = g_strdup_printf ( "rom%d %s", page, filename );

	/* Load ROM within paged ROM bank */
	bbc_load_rom ( &paged->roms, bbc_paged_rom_offset ( paged, page ),
		       bbc_paged_rom_targphys ( paged, page ), paged->size,
		       name, filename );
}

/******************************************************************************
 *
 * Interrupts
 *
 */

/**
 * Interrupt handler
 *
 * @v bbc		BBC Micro
 * @v active		Interrupt set status
 * @v count		Interrupt set counter
 * @v cpu_irq_type	CPU interrupt type
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_interrupt ( BBCMicro *bbc, bool *active, uint16_t *count,
			    int cpu_irq_type, int n, int level ) {

	/* Do nothing unless interrupt has changed */
	level = ( !! level );
	if ( level == active[n] )
		return;

	/* Record state of this interrupt */
	active[n] = level;

	/* Assert or deassert CPU interrupt */
	if ( level ) {
		if ( ! *count )
			cpu_interrupt ( bbc->cpu, cpu_irq_type );
		(*count)++;
	} else {
		(*count)--;
		if ( ! *count )
			cpu_reset_interrupt ( bbc->cpu, cpu_irq_type );
	}
}

/**
 * IRQ handler
 *
 * @v opaque		BBC Micro
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_irq_handler ( void *opaque, int n, int level ) {
	BBCMicro *bbc = opaque;

	/* Control CPU IRQ pin */
	bbc_interrupt ( bbc, bbc->irq_active, &bbc->irq_count,
			CPU_INTERRUPT_HARD, n, level );
}

/**
 * NMI handler
 *
 * @v opaque		BBC Micro
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_nmi_handler ( void *opaque, int n, int level ) {
	BBCMicro *bbc = opaque;

	/* Control CPU NMI pin */
	bbc_interrupt ( bbc, bbc->nmi_active, &bbc->nmi_count,
			CPU_INTERRUPT_NMI, n, level );
}

/**
 * Initialise interrupts
 *
 * @v bbc		BBC Micro
 */
static void bbc_interrupts_init ( BBCMicro *bbc ) {

	/* Allocate IRQ and NMI interrupts and set inactive (high) */
	bbc->irq = qemu_allocate_irqs ( bbc_irq_handler, bbc, BBC_IRQ_COUNT );
	bbc->nmi = qemu_allocate_irqs ( bbc_nmi_handler, bbc, BBC_NMI_COUNT );
}

/******************************************************************************
 *
 * 6845 CRTC (SHEILA &00-&07)
 *
 */

/**
 * Initialise CRTC
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret crtc		CRTC
 */
static MC6845CRTC * bbc_crtc_init ( hwaddr addr, uint64_t size,
				    const char *name ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MC6845CRTC *crtc;

	/* Initialise CRTC */
	crtc = mc6845_init ( address_space_mem, addr, name );
	bbc_io_alias ( &crtc->mr, addr, size );

	return crtc;
}

/******************************************************************************
 *
 * 6850 ACIA (SHEILA &08-&0F)
 *
 */

/**
 * Initialise ACIA
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret acia		ACIA
 */
static MC6850ACIA * bbc_acia_init ( hwaddr addr, uint64_t size,
				    const char *name ) {
	MemoryRegion *address_space_mem = get_system_memory();
	MC6850ACIA *acia;

	/* Initialise ACIA */
	acia = mc6850_init ( address_space_mem, addr, name, serial_hds[0] );
	bbc_io_alias ( &acia->mr, addr, size );

	return acia;
}

/******************************************************************************
 *
 * Serial ULA (SHEILA &10-&1F)
 *
 */

/**
 * Initialise serial ULA
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret ula		Serial ULA
 */
static BBCSerialULA * bbc_serial_ula_init ( hwaddr addr, uint64_t size,
					    const char *name ) {
	BBCSerialULA *ula = g_new0 ( BBCSerialULA, 1 );

	/* Initialise ULA */
	ula->name = name;

	/* Initialise as an unimplemented I/O region */
	ula->unimp = bbc_unimplemented_init ( addr, size, name );

	return ula;
}

/******************************************************************************
 *
 * Video ULA (SHEILA &20-&2F)
 *
 */

/**
 * Register update notification function
 *
 * @v ula		Video ULA
 * @v updated		Update notification function
 * @v opaque		Opaque pointer
 */
void bbc_video_ula_update_register ( BBCVideoULA *ula,
				     BBCVideoULAUpdated updated,
				     void *opaque ) {
	BBCVideoULAUpdateEntry *entry = g_new0 ( BBCVideoULAUpdateEntry, 1 );

	entry->updated = updated;
	entry->opaque = opaque;
	QTAILQ_INSERT_TAIL ( &ula->updates, entry, next );
}

/**
 * Unregister update notification function
 *
 * @v ula		Video ULA
 * @v updated		Update notification function
 * @v opaque		Opaque pointer
 */
void bbc_video_ula_update_unregister ( BBCVideoULA *ula,
				       BBCVideoULAUpdated updated,
				       void *opaque ) {
	BBCVideoULAUpdateEntry *entry;

	QTAILQ_FOREACH ( entry, &ula->updates, next ) {
		if ( ( entry->updated == updated ) &&
		     ( entry->opaque == opaque ) ) {
			QTAILQ_REMOVE ( &ula->updates, entry, next );
			g_free ( entry );
			return;
		}
	}
}

/**
 * Call update notification functions
 *
 * @v ula		Video ULA
 */
static void bbc_video_ula_updated ( BBCVideoULA *ula ) {
	BBCVideoULAUpdateEntry *entry;

	QTAILQ_FOREACH ( entry, &ula->updates, next ) {
		entry->updated ( entry->opaque );
	}
}

/**
 * Read from video ULA register
 *
 * @v opaque		Video ULA
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_video_ula_read ( void *opaque, hwaddr addr,
				     unsigned int size ) {

	/* These are write-only registers */
	return 0xff;
}

/**
 * Write to video ULA control register
 *
 * @v ula		Video ULA
 * @v data		Data to write
 */
static void bbc_video_ula_control_write ( BBCVideoULA *ula, uint8_t data ) {

	/* Decode register */
	ula->cursor_mask = ( ( ( ( data >> 7 ) & 0x01 ) << 0 ) |
			     ( ( ( data >> 6 ) & 0x01 ) << 1 ) |
			     ( ( ( data >> 5 ) & 0x01 ) << 2 ) |
			     ( ( ( data >> 5 /* sic */ ) & 1 ) << 3 ) );
	ula->crtc_clock_log2 = ( ( data >> 4 ) & 0x01 );
	ula->pixel_clock_log2 = ( ( ( data >> 2 ) & 0x03 ) + 1 );
	ula->teletext = ( ( data >> 1 ) & 0x01 );
	ula->flash = ( ( data >> 0 ) & 0x01 );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: cursor mask %c%c%c%c CRTC %dMHz "
			"pixel %dMHz%s%s\n", ula->name,
			( ( ula->cursor_mask & 0x01 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x02 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x04 ) ? 'X' : '-' ),
			( ( ula->cursor_mask & 0x08 ) ? 'X' : '-' ),
			( 1 << ula->crtc_clock_log2 ),
			( 1 << ula->pixel_clock_log2 ),
			( ula->teletext ? " teletext" : "" ),
			( ula->flash ? " flash" : "" ) );

	/* Call update notification functions */
	bbc_video_ula_updated ( ula );
}

/**
 * Write to video ULA palette register
 *
 * @v ula		Video ULA
 * @v data		Data to write
 */
static void bbc_video_ula_palette_write ( BBCVideoULA *ula, uint8_t data ) {
	unsigned int logical;
	unsigned int actual;

	/* Decode register and update palette */
	logical = ( ( data >> 4 ) & 0x0f );
	actual = ( ( data >> 0 ) & 0x0f );
	ula->palette[logical] = actual;
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: logical colour &%x maps to actual "
			"colour &%x\n", ula->name, logical, actual );
}

/**
 * Write to video ULA register
 *
 * @v opaque		Video ULA
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_video_ula_write ( void *opaque, hwaddr addr,
				  uint64_t data64, unsigned int size ) {
	BBCVideoULA *ula = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr ) {
	case BBC_VIDEO_ULA_CONTROL:
		bbc_video_ula_control_write ( ula, data );
		break;
	case BBC_VIDEO_ULA_PALETTE:
		bbc_video_ula_palette_write ( ula, data );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write &%02x to "
				"&%02lx\n", ula->name, data, addr );
		break;

	}
}

/** Video ULA operations */
static const MemoryRegionOps bbc_video_ula_ops = {
	.read = bbc_video_ula_read,
	.write = bbc_video_ula_write,
};

/** Video ULA state description */
static const VMStateDescription vmstate_bbc_video_ula = {
	.name = "bbc_video_ula",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( cursor_mask, BBCVideoULA ),
		VMSTATE_UINT8 ( crtc_clock_log2, BBCVideoULA ),
		VMSTATE_UINT8 ( pixel_clock_log2, BBCVideoULA ),
		VMSTATE_UINT8 ( teletext, BBCVideoULA ),
		VMSTATE_UINT8 ( flash, BBCVideoULA ),
		VMSTATE_UINT8_ARRAY ( palette, BBCVideoULA, 16 ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise video ULA
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Device name
 * @ret ula		Video ULA
 */
static BBCVideoULA * bbc_video_ula_init ( hwaddr addr, uint64_t size,
					  const char *name ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCVideoULA *ula = g_new0 ( BBCVideoULA, 1 );

	/* Initialise ULA */
	ula->name = name;
	QTAILQ_INIT ( &ula->updates );

	/* Register memory region */
	memory_region_init_io ( &ula->mr, &bbc_video_ula_ops, ula, name,
				BBC_VIDEO_ULA_SIZE );
	memory_region_add_subregion ( address_space_mem, addr, &ula->mr );
	bbc_io_alias ( &ula->mr, addr, size );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_video_ula, ula );

	return ula;
}

/******************************************************************************
 *
 * Keyboard
 *
 */

/**
 * Check if CAPS LOCK is enabled
 *
 * @v via		System VIA
 * @ret caps_lock	CAPS LOCK is enabled
 */
static inline int bbc_caps_lock ( BBCSystemVIA *via ) {

	/* CAPS LOCK is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_CAPS_LOCK ) );
}

/**
 * Check if SHIFT LOCK is enabled
 *
 * @v via		System VIA
 * @ret shift_lock	SHIFT LOCK is enabled
 */
static inline int bbc_shift_lock ( BBCSystemVIA *via ) {

	/* SHIFT LOCK is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_SHIFT_LOCK ) );
}

/**
 * Check if keyboard autoscan is enabled
 *
 * @v via		System VIA
 * @ret autoscan	Autoscan is enabled
 */
static inline int bbc_keyboard_autoscan ( BBCSystemVIA *via ) {

	/* Autoscan is controlled via the addressable latch */
	return ( via->addressable_latch & ( 1 << BBC_LATCH_KB_WE ) );
}

/**
 * Get keyboard row address (valid only if autoscan is disabled)
 *
 * @v via		System VIA
 * @ret row		Row address
 */
static inline unsigned int bbc_keyboard_row ( BBCSystemVIA *via ) {

	/* Row address is in PA6:4 */
	return ( ( via->slow_data >> 4 ) & 0x07 );
}

/**
 * Get keyboard column address (valid only if autoscan is disabled)
 *
 * @v via		System VIA
 * @ret column		Column address
 */
static inline unsigned int bbc_keyboard_column ( BBCSystemVIA *via ) {

	/* Column address is in PA3:0 */
	return ( ( via->slow_data >> 0 ) & 0x0f );
}

/** Keyboard DIP switches SW1-SW8 are mapped into row 0 */
#define BBC_KB_SW_ROW 0

/** Keyboard DIP switch SW<n> (i.e. bit 8-<n>) is mapped to row 0
 * column <n>+1
 */
#define BBC_KB_SW_COLUMN(bit) ( 8 - (bit) + 1 )

/** A BBC key */
typedef struct {
	/** Scancode (possibly extended) */
	uint16_t scancode;
	/** Column */
	uint8_t column;
	/** Row */
	uint8_t row;
	/** Name */
	const char *name;
} BBCKey;

/** BBC keyboard map
 *
 * The keyboard circuit diagram present in the BBC Advanced User Guide
 * appears to be incorrect; rows 1-6 are in the reverse order to that
 * in the diagram.
 */
static BBCKey bbc_keys[] = {

	/* Keyboard grid row 7 */
	{ 0x0001, 0, 7, "ESC" },		{ 0x003b, 1, 7, "F1" },
	{ 0x003c, 2, 7, "F2" },			{ 0x003d, 3, 7, "F3" },
	{ 0x003f, 4, 7, "F5" },			{ 0x0040, 5, 7, "F6" },
	{ 0x0042, 6, 7, "F8" },			{ 0x0043, 7, 7, "F9" },
	{ 0x002b, 8, 7, "\\|" },		{ 0xe04d, 9, 7, "Right" },

	/* Keyboard grid row 6 (row 1 in circuit diagram) */
	{ 0x000f, 0, 6, "Tab" },		{ 0x002c, 1, 6, "Z" },
	{ 0x0039, 2, 6, "Space" },		{ 0x002f, 3, 6, "V" },
	{ 0x0030, 4, 6, "B" },			{ 0x0032, 5, 6, "M" },
	{ 0x0033, 6, 6, ",<" },			{ 0x0034, 7, 6, ".>" },
	{ 0x0035, 8, 6, "/?" },		 { 0xe052, 9, 6, "Copy" /* Insert */ },

	/* Keyboard grid row 5 (row 2 in circuit diagram) */
	{ 0x0045, 0, 5, "ShiftLock" /* NumLock */ }, { 0x001f, 1, 5, "S" },
	{ 0x002e, 2, 5, "C" },			{ 0x0022, 3, 5, "G" },
	{ 0x0023, 4, 5, "H" },			{ 0x0031, 5, 5, "N" },
	{ 0x0026, 6, 5, "L" },			{ 0x0027, 7, 5, ";+" /* ;: */ },
	{ 0x001b, 8, 5, "]}" },			{ 0xe053, 9, 5, "Delete" },

	/* Keyboard grid row 4 (row 3 in circuit diagram) */
	{ 0x003a, 0, 4, "CapsLock" },		{ 0x001e, 1, 4, "A" },
	{ 0x002d, 2, 4, "X" },			{ 0x0021, 3, 4, "F" },
	{ 0x0015, 4, 4, "Y" },			{ 0x0024, 5, 4, "J" },
	{ 0x0025, 6, 4, "K" },			{ 0x0058, 7, 4, "@" /* F12 */ },
	{ 0x0028, 8, 4, ":*" /* '" */ },	{ 0x001c, 9, 4, "Return" },

	/* Keyboard grid row 3 (row 4 in circuit diagram) */
	{ 0x0002, 0, 3, "1" },			{ 0x0003, 1, 3, "2" },
	{ 0x0020, 2, 3, "D" },			{ 0x0013, 3, 3, "R" },
	{ 0x0007, 4, 3, "6" },			{ 0x0016, 5, 3, "U" },
	{ 0x0018, 6, 3, "O" },			{ 0x0019, 7, 3, "P" },
	{ 0x001a, 8, 3, "[{" },			{ 0xe048, 9, 3, "Up" },

	/* Keyboard grid row 2 (row 5 in circuit diagram) */
	{ 0x0044, 0, 2, "F0" /* F10 */ },	{ 0x0011, 1, 2, "W" },
	{ 0x0012, 2, 2, "E" },			{ 0x0014, 3, 2, "T" },
	{ 0x0008, 4, 2, "7" },			{ 0x0017, 5, 2, "I" },
	{ 0x000a, 6, 2, "9" },			{ 0x000b, 7, 2, "0" },
	{ 0x0029, 8, 2, "_£" /* `~ */ },	{ 0xe050, 9, 2, "Down" },

	/* Keyboard grid row 1 (row 6 in circuit diagram) */
	{ 0x0010, 0, 1, "Q" },			{ 0x0004, 1, 1, "3" },
	{ 0x0005, 2, 1, "4" },			{ 0x0006, 3, 1, "5" },
	{ 0x003e, 4, 1, "F4" },			{ 0x0009, 5, 1, "8" },
	{ 0x0041, 6, 1, "F7" },			{ 0x000c, 7, 1, "-=" /* -_ */ },
	{ 0x000d, 8, 1, "^~" /* =+ */ },	{ 0xe04b, 9, 1, "Left" },

	/* Keyboard grid row 0: modifier keys and DIP switches */
	{ 0x002a /* LShift */, 0, 0, "Shift" },
	{ 0x001d /* LCtrl */, 1, 0, "Ctrl" },
	{ 0, 2, 0, "SW1" },			{ 0, 3, 0, "SW2" },
	{ 0, 4, 0, "SW3" },			{ 0, 5, 0, "SW4" },
	{ 0, 6, 0, "SW5" },			{ 0, 7, 0, "SW6" },
	{ 0, 8, 0, "SW7" },			{ 0, 9, 0, "SW8" },

	/* Allow left/right modifier keys to function as equivalents */
	{ 0x0036, 0, 0, "Shift (RShift)" },
	{ 0xe01d, 1, 0, "Ctrl (RCtrl)" },

	/* Allow Backspace to function as equivalent to Delete */
	{ 0x000e, 9, 5, "Delete (Backspace)" },

	/* Allow numeric keypad to function as nearest equivalent keys */
	{ 0x0052, 7, 2, "0 (Keypad0)" },
	{ 0x004f, 0, 3, "1 (Keypad1)" },
	{ 0x0050, 1, 3, "2 (Keypad2)" },
	{ 0x0051, 1, 1, "3 (Keypad3)" },
	{ 0x004b, 2, 1, "4 (Keypad4)" },
	{ 0x004c, 3, 1, "5 (Keypad5)" },
	{ 0x004d, 4, 3, "6 (Keypad6)" },
	{ 0x0047, 4, 2, "7 (Keypad7)" },
	{ 0x0048, 5, 1, "8 (Keypad8)" },
	{ 0x0049, 5, 2, "9 (Keypad9)" },
	{ 0x0053, 7, 6, ".> (Keypad.)" },
	{ 0x004e, 7, 5, ";+ (Keypad+)" },
	{ 0x004a, 7, 1, "-= (Keypad-)" },
	{ 0x0037, 8, 4, ":* (Keypad*)" },
	{ 0xe035, 8, 6, "/? (Keypad/)" },
	{ 0xe01c, 9, 4, "Return (KeypadEnter)" },

	/* Allow "\|" key on UK keyboards ("<>" on many international
	 * keyboards) to function as equivalent to US "\|".
	 */
	{ 0x0056, 8, 7, "\\| (UK)" },

	/* Allow left Windows key to function as equivalent to
	 * CapsLock, and right Windows key as equivalent to ShiftLock.
	 */
	{ 0xe05b, 0, 4, "CapsLock (LeftWindow)" },
	{ 0xe05c, 0, 5, "ShiftLock (RightWindow)" },
};

/** Break key is a hardwired reset line on the BBC */
#define BBC_KEY_BREAK 0xe11d

/** Extended keypresses are prefixed by 0xeX */
#define BBC_KEY_IS_PREFIX(keycode) ( ( (keycode) & 0xf0) == 0xe0 )

/** Key releases are indicated by bit 7 */
#define BBC_KEY_PRESSED(keycode) ( ! ( (keycode) & 0x80 ) )

/** Scancode is constructed from prefix and keycode (ignoring bit 7) */
#define BBC_KEY_SCANCODE( prefix, scancode ) \
	( ( (prefix) << 8 ) | ( (scancode) & 0x7f ) )

/**
 * Update keyboard interrupt line
 *
 * @v via		System VIA
 */
static void bbc_keyboard_update_irq ( BBCSystemVIA *via ) {
	uint8_t keys_pressed;
	unsigned int column;

	/* Get bitmask of contributing keypresses */
	if ( bbc_keyboard_autoscan ( via ) ) {
		keys_pressed = 0;
		for ( column = 0 ; column < BBC_KEYBOARD_COLUMNS ; column++ )
			keys_pressed |= via->keys_pressed[column];
	} else {
		column = bbc_keyboard_column ( via );
		keys_pressed = via->keys_pressed[column];
	}

	/* Only rows 1-7 contribute towards the interrupt */
	keys_pressed &= BBC_KEYBOARD_IRQ_ROW_MASK;

	/* Update VIA's CA2 interrupt line */
	qemu_set_irq ( via->via->a.c2.in, keys_pressed );
}

/**
 * Handle keyboard event
 *
 * @v opaque		System VIA
 * @v keycode		Keycode
 */
static void bbc_keyboard_event ( void *opaque, int keycode ) {
	BBCSystemVIA *via = opaque;
	BBCKey *key = NULL;
	bool pressed;
	uint16_t scancode;
	unsigned int i;

	/* Handle extended keypresses */
	if ( BBC_KEY_IS_PREFIX ( keycode ) ) {
		via->keycode_prefix = keycode;
		return;
	}

	/* Separate out key press/release indicator */
	pressed = BBC_KEY_PRESSED ( keycode );

	/* Construct extended scancode */
	scancode = BBC_KEY_SCANCODE ( via->keycode_prefix, keycode );
	via->keycode_prefix = 0;

	/* BREAK isn't part of the keyboard; it's a hardware reset switch */
	if ( scancode == BBC_KEY_BREAK ) {
		qemu_system_reset_request();
		return;
	}

	/* Identify key */
	for ( i = 0 ; i < ARRAY_SIZE ( bbc_keys ) ; i++ ) {
		if ( bbc_keys[i].scancode == scancode ) {
			key = &bbc_keys[i];
			break;
		}
	}

	/* Ignore unknown keys */
	if ( ! key ) {
		qemu_log_mask ( LOG_UNIMP, "%s: unknown scancode 0x%04x\n",
				via->name, scancode );
		return;
	}

	/* Ignore duplicate press/release events */
	if ( pressed == ( !! ( via->keys_pressed[key->column] &
			       ( 1 << key->row ) ) ) ) {
		return;
	}

	/* Record key as pressed/released */
	via->keys_pressed[key->column] &= ~( 1 << key->row );
	if ( pressed )
		via->keys_pressed[key->column] |= ( 1 << key->row );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: key %s %s\n", via->name,
			key->name, ( pressed ? "pressed" : "released" ) );

	/* Update keyboard interrupt line */
	bbc_keyboard_update_irq ( via );
}

/**
 * Check if currently-selected key is pressed
 *
 * @v via		System VIA
 * @ret pressed		Key is pressed
 */
static int bbc_keyboard_pressed ( BBCSystemVIA *via ) {
	unsigned int row;
	unsigned int column;
	int pressed;

	/* Check pressed-key bitmap */
	row = bbc_keyboard_row ( via );
	column = bbc_keyboard_column ( via );
	pressed = ( via->keys_pressed[column] & ( 1 << row ) );

	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: keyboard column %d row %d %s\n",
			column, row, ( pressed ? "pressed" : "not pressed" ) );
	return pressed;
}

/**
 * Update keyboard LEDs
 *
 * @v via		System VIA
 */
static void bbc_keyboard_leds ( BBCSystemVIA *via ) {
	int caps_lock = bbc_caps_lock ( via );
	int shift_lock = bbc_shift_lock ( via );
	int ledstate;

	/* Update LEDs based on control bits in addressable latch */
	ledstate = ( ( caps_lock ? QEMU_CAPS_LOCK_LED : 0 ) |
		     ( shift_lock ? QEMU_NUM_LOCK_LED : 0 ) );
	kbd_put_ledstate ( ledstate );
	qemu_log_mask ( CPU_LOG_IOPORT, "BBC: keyboard leds %s %s\n",
			( caps_lock ? "CAPSLOCK" : "capslock" ),
			( shift_lock ? "SHIFTLOCK" : "shiftlock" ) );
}

/**
 * Set state of keyboard DIP switches
 *
 * @v via		System VIA
 * @v dip		DIP switch settings
 */
static void bbc_keyboard_press_dip ( BBCSystemVIA *via, uint8_t dip ) {
	unsigned int i;
	unsigned int row;
	unsigned int column;

	/* DIP switches are mapped to row 0 columns 2-9.  Mark these
	 * non-interrupting "keys" as being permanently pressed.
	 */
	for ( i = 0 ; i < 8 ; i++ ) {
		row = BBC_KB_SW_ROW;
		column = BBC_KB_SW_COLUMN ( i );
		if ( dip & ( 1 << i ) )
			via->keys_pressed[column] |= ( 1 << row );
	}
}

/******************************************************************************
 *
 * System VIA (SHEILA &40-&4F)
 *
 */

/**
 * Get contents of slow data bus (system VIA port A)
 *
 * @v opaque		System VIA
 * @ret data		Slow data bus contents
 */
static uint8_t bbc_slow_data_read ( void *opaque ) {
	BBCSystemVIA *via = opaque;
	M6522VIAPort *port = &via->via->a;
	uint8_t data;

	/* Set data equal to outputs for all pins configured as outputs */
	data = ( via->slow_data & port->ddr );

	/* Read from keyboard into PA7 if keyboard is enabled */
	if ( ! bbc_keyboard_autoscan ( via ) ) {
		data &= ~( 1 << 7 );
		if ( bbc_keyboard_pressed ( via ) )
			data |= ( 1 << 7 );
	}

	return data;
}

/**
 * Write to slow data bus (system VIA port A)
 *
 * @v opaque		System VIA
 * @v data		Data
 */
static void bbc_slow_data_write ( void *opaque, uint8_t data ) {
	BBCSystemVIA *via = opaque;

	/* Record slow data bus contents */
	via->slow_data = data;

	/* Update keyboard interrupt line */
	bbc_keyboard_update_irq ( via );
}

/**
 * Write to 76489 sound chip
 *
 * @v via		System VIA
 * @v port		Port
 */
static void bbc_sound_write ( BBCSystemVIA *via ) {

	qemu_log_mask ( LOG_UNIMP, "%s: unimplemented sound write &%02x\n",
			via->name, via->slow_data );
}

/**
 * Write to addressable latch (system VIA port B)
 *
 * @v opaque		System VIA
 * @v data		Output data
 */
static void bbc_addressable_latch_write ( void *opaque, uint8_t data ) {
	BBCSystemVIA *via = opaque;
	static const char * names[8] = {
		[BBC_LATCH_SOUND_WE] = "SOUND_WE",
		[BBC_LATCH_SPEECH_RS] = "SPEECH_RS",
		[BBC_LATCH_SPEECH_WS] = "SPEECH_WS",
		[BBC_LATCH_KB_WE] = "KB_WE",
		[BBC_LATCH_C0] = "C0",
		[BBC_LATCH_C1] = "C1",
		[BBC_LATCH_CAPS_LOCK] = "CAPS_LOCK",
		[BBC_LATCH_SHIFT_LOCK] = "SHIFT_LOCK",
	};
	unsigned int latch_address;
	unsigned int latch_data;

	/* Update addressable latch stored value */
	latch_address = ( ( data >> 0 ) & 0x07 );
	latch_data = ( ( data >> 3 ) & 0x01 );
	via->addressable_latch &= ~( 1 << latch_address );
	via->addressable_latch |= ( latch_data << latch_address );
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: addressable latch now &%02X "
			"(bit %d %s %s)\n", via->name, via->addressable_latch,
			latch_address, names[latch_address],
			( latch_data ? "high" : "low" ) );

	/* Handle write events */
	switch ( latch_address ) {
	case BBC_LATCH_SOUND_WE:
		if ( ! latch_data )
			bbc_sound_write ( via );
		break;
	case BBC_LATCH_KB_WE:
		bbc_keyboard_update_irq ( via );
		break;
	case BBC_LATCH_CAPS_LOCK:
	case BBC_LATCH_SHIFT_LOCK:
		bbc_keyboard_leds ( via );
		break;
	}
}

/** System VIA operations */
static M6522VIAOps bbc_system_via_ops = {
	.b = {
		.output = bbc_addressable_latch_write,
	},
	.a = {
		.input = bbc_slow_data_read,
		.output = bbc_slow_data_write,
	},
};

/** System VIA state description */
static const VMStateDescription vmstate_bbc_system_via = {
	.name = "bbc_system_via",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( slow_data, BBCSystemVIA ),
		VMSTATE_UINT8 ( addressable_latch, BBCSystemVIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise system VIA
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Device name
 * @v irq		Interrupt request line
 * @v dip		DIP switch settings
 * @ret via		System VIA
 */
static BBCSystemVIA * bbc_system_via_init ( hwaddr addr, uint64_t size,
					    const char *name, qemu_irq irq,
					    uint8_t dip ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCSystemVIA *via = g_new0 ( BBCSystemVIA, 1 );

	/* Initialise system VIA */
	via->name = name;
	via->via = m6522_init ( address_space_mem, addr, name, via,
				&bbc_system_via_ops, irq, BBC_1MHZ_TICK_NS );
	bbc_io_alias ( &via->via->mr, addr, size );

	/* Initialise keyboard */
	bbc_keyboard_press_dip ( via, dip );
	qemu_add_kbd_event_handler ( bbc_keyboard_event, via );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_system_via, via );

	return via;
}

/******************************************************************************
 *
 * User VIA (SHEILA &60-&6F)
 *
 */

/**
 * Write to parallel port (user VIA port A)
 *
 * @v opaque		User VIA
 * @v data		Output data
 */
static void bbc_parallel_write ( void *opaque, uint8_t data ) {
	BBCUserVIA *via = opaque;
	BBCParallel *parallel = &via->parallel;

	parallel->data = data;
}

/**
 * Strobe parallel port (user VIA port A line C2)
 *
 * @v opaque		User VIA
 * @v n			Interrupt number
 * @v level		Interrupt level
 */
static void bbc_parallel_strobe ( void *opaque, int n, int level ) {
	BBCUserVIA *via = opaque;
	BBCParallel *parallel = &via->parallel;
	uint8_t data = via->parallel.data;

	/* Do nothing unless this is a falling edge */
	level = ( !! level );
	if ( level == parallel->previous )
		return;
	parallel->previous = level;
	if ( ! level )
		return;
	
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: print character &%02x '%c'\n",
			via->name, data, ( isprint ( data ) ? data : '.' ) );

	/* Send data to parallel port, if connected */
	if ( parallel->chr ) {
		qemu_chr_fe_write ( parallel->chr, &data,
				    sizeof ( data ) );
	}

	/* Acknowledge data via CA1 */
	qemu_set_irq ( via->via->a.c1.in, 0 );
	qemu_set_irq ( via->via->a.c1.in, 1 );
}

/** User VIA operations */
static M6522VIAOps bbc_user_via_ops = {
	.a = {
		.output = bbc_parallel_write,
	},
};

/** User VIA state description */
static const VMStateDescription vmstate_bbc_user_via = {
	.name = "bbc_user_via",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_INT32 ( parallel.previous, BBCUserVIA ),
		VMSTATE_UINT8 ( parallel.data, BBCUserVIA ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise user VIA
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Device name
 * @v irq		Interrupt request line
 * @v chr		Character device, if any
 * @ret via		User VIA
 */
static BBCUserVIA * bbc_user_via_init ( hwaddr addr, uint64_t size,
					const char *name, qemu_irq irq,
					CharDriverState *chr ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCUserVIA *via = g_new0 ( BBCUserVIA, 1 );
	BBCParallel *parallel = &via->parallel;

	/* Initialise user VIA */
	via->name = name;
	via->via = m6522_init ( address_space_mem, addr, name, via,
				&bbc_user_via_ops, irq, BBC_1MHZ_TICK_NS );
	bbc_io_alias ( &via->via->mr, addr, size );

	/* Initialise parallel port */
	parallel->chr = chr;
	parallel->strobe =
		qemu_allocate_irqs ( bbc_parallel_strobe, via, 1 )[0];
	parallel->previous = 1;
	via->via->a.c2.out = parallel->strobe;
	parallel->ack = via->via->a.c1.in;
	qemu_set_irq ( parallel->ack, 1 );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_user_via, via );

	return via;
}

/******************************************************************************
 *
 * 1770 floppy disc controller (SHEILA &80-&9F)
 *
 */

/**
 * Read from 1770 FDC control register
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_1770_fdc_read ( void *opaque, hwaddr addr,
				    unsigned int size ) {

	/* This is a write-only register */
	return 0xff;
}

/**
 * Write to 1770 FDC control register
 *
 * @v opaque		1770 FDC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_1770_fdc_write ( void *opaque, hwaddr addr, uint64_t data64,
				 unsigned int size ) {
	BBC1770FDC *fdc = opaque;
	uint8_t data = data64;
	int drive;
	unsigned int side;
	bool single_density;
	bool master_reset;

	/* Write to control register */
	fdc->control = data;
	qemu_log_mask ( CPU_LOG_IOPORT, "%s: control register (&%02lX) set to "
			"&%02X\n", fdc->name, addr, data );

	/* Select drive number */
	switch ( data & BBC_1770_FDC_DRIVE_MASK ) {
	case 0:
		/* No drive selected */
		drive = WD1770_NO_DRIVE;
		break;
	case BBC_1770_FDC_DRIVE_0:
		drive = 0;
		break;
	case BBC_1770_FDC_DRIVE_1:
		drive = 1;
		break;
	default:
		/* Invalid combination */
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented simultaneous "
				"operation of both drives\n", fdc->name );
		drive = WD1770_NO_DRIVE;
		break;
	}
	wd1770_set_drive ( fdc->fdc, drive );

	/* Decode side number */
	side = ( ( data & BBC_1770_FDC_SIDE_1 ) ? 1 : 0 );
	wd1770_set_side ( fdc->fdc, side );

	/* Decode density */
	single_density = ( !! ( data & BBC_1770_FDC_SINGLE_DENSITY ) );
	wd1770_set_single_density ( fdc->fdc, single_density );

	/* Decode master reset.  This line is supposed to be
	 * level-sensitive; the real hardware would be held in reset
	 * while the line remained active.
	 */
	master_reset = ( ! ( data & BBC_1770_FDC_NOT_MASTER_RESET ) );
	if ( master_reset )
		wd1770_reset ( fdc->fdc );
}

/** 1770 FDC operations */
static const MemoryRegionOps bbc_1770_fdc_ops = {
	.read = bbc_1770_fdc_read,
	.write = bbc_1770_fdc_write,
};

/** 1770 FDC state description */
static const VMStateDescription vmstate_bbc_1770_fdc = {
	.name = "bbc_1770_fdc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_UINT8 ( control, BBC1770FDC ),
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise 1770 FDC
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Device name
 * @v drq		Data request non-maskable interrupt request line
 * @v intrq		Completion non-maskable interrupt request line
 * @ret fdc		1770 FDC
 */
static BBC1770FDC * bbc_1770_fdc_init ( hwaddr addr, uint64_t size,
					const char *name, qemu_irq drq,
					qemu_irq intrq ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBC1770FDC *fdc = g_new0 ( BBC1770FDC, 1 );
	DriveInfo *fds[WD1770_DRIVE_COUNT];
	unsigned int i;

	/* The memory map for this peripheral is a little strange.
	 * The BBC was originally designed to take an Intel 8271 FDC,
	 * occupying eight bytes of address space.  The WD1770 FDC was
	 * mapped in to the latter four byte of this address space,
	 * with the first four bytes being occupied by a 74LS174 hex
	 * D-type which provides additional signals to the WD1770.
	 */

	/* Initialise 1770 FDC */
	fdc->name = name;
	for ( i = 0 ; i < ARRAY_SIZE ( fds ) ; i++ )
		fds[i] = drive_get ( IF_FLOPPY, 0, i );
	fdc->fdc = wd1770_init ( ( addr + BBC_1770_FDC_WD1770_OFFSET ),
				 drq, intrq, fds );

	/* Register memory region */
	memory_region_init_io ( &fdc->mr, &bbc_1770_fdc_ops, fdc, fdc->name,
				BBC_1770_FDC_CONTROL_SIZE );
	memory_region_add_subregion ( address_space_mem, addr, &fdc->mr );
	bbc_io_alias_offset ( &fdc->mr, addr, size, 0, BBC_1770_FDC_SIZE );
	bbc_io_alias_offset ( &fdc->fdc->mr, addr, size,
			      BBC_1770_FDC_WD1770_OFFSET, BBC_1770_FDC_SIZE );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_1770_fdc, fdc );

	return fdc;
}

/******************************************************************************
 *
 * Econet controller (SHEILA &A0-&BF)
 *
 */

/**
 * Initialise Econet controller
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret econet		Econet controller
 */
static BBCEconet * bbc_econet_init ( hwaddr addr, uint64_t size,
				     const char *name ) {
	BBCEconet *econet = g_new0 ( BBCEconet, 1 );

	/* Initialise Econet controller */
	econet->name = name;

	/* Initialise as an unimplemented I/O region */
	econet->unimp = bbc_unimplemented_init ( addr, size, name );

	return econet;
}

/******************************************************************************
 *
 * Analogue to digital converter (UPD7002) (SHEILA &C0-&DF)
 *
 */

/**
 * Read from analogue to digital converter status register
 *
 * @v adc		ADC
 * @ret data		Data
 */
static uint8_t bbc_adc_status_read ( BBCADC *adc ) {
	uint8_t data;

	/* While we don't implement the ADC, we must implement the
	 * status register since the MOS's interrupt service routine
	 * will check to see if the ADC was the source of an
	 * interrupt.
	 */
	data = BBC_ADC_NOT_COMPLETE;

	return data;
}

/**
 * Read from analogue to digital converter
 *
 * @v opaque		ADC
 * @v addr		Register address
 * @v size		Size of read
 * @ret data		Read data
 */
static uint64_t bbc_adc_read ( void *opaque, hwaddr addr, unsigned int size ) {
	BBCADC *adc = opaque;
	uint8_t data;

	/* Read from specified register */
	switch ( addr ) {
	case BBC_ADC_STATUS:
		data = bbc_adc_status_read ( adc );
		break;
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented read from "
				"0x%02lx\n", adc->name, addr );
		data = 0;
		break;
	}

	return data;
}

/**
 * Write to analogue to digital converter
 *
 * @v opaque		ADC
 * @v addr		Register address
 * @v data64		Data to write
 * @v size		Size of write
 */
static void bbc_adc_write ( void *opaque, hwaddr addr,
			    uint64_t data64, unsigned int size ) {
	BBCADC *adc = opaque;
	uint8_t data = data64;

	/* Write to specified register */
	switch ( addr ) {
	default:
		qemu_log_mask ( LOG_UNIMP, "%s: unimplemented write &%02x to "
				"&%02lx\n", adc->name, data, addr );
		break;
	}
}

/** ADC operations */
static const MemoryRegionOps bbc_adc_ops = {
	.read = bbc_adc_read,
	.write = bbc_adc_write,
};

/** ADC state description */
static const VMStateDescription vmstate_bbc_adc = {
	.name = "bbc_adc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_END_OF_LIST()
	},
};

/**
 * Initialise analogue to digital converter
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Device name
 * @ret adc		ADC
 */
static BBCADC * bbc_adc_init ( hwaddr addr, uint64_t size, const char *name ) {
	MemoryRegion *address_space_mem = get_system_memory();
	BBCADC *adc = g_new0 ( BBCADC, 1 );

	/* Initialise ADC */
	adc->name = name;

	/* Register memory region */
	memory_region_init_io ( &adc->mr, &bbc_adc_ops, adc, name,
				BBC_ADC_SIZE );
	memory_region_add_subregion ( address_space_mem, addr, &adc->mr );
	bbc_io_alias ( &adc->mr, addr, size );

	/* Register virtual machine state */
	vmstate_register ( NULL, addr, &vmstate_bbc_adc, adc );

	return adc;
}

/******************************************************************************
 *
 * Tube (SHEILA &E0-&FF)
 *
 */

/**
 * Initialise Tube
 *
 * @v addr		Address
 * @v size		Size
 * @v name		Name
 * @ret tube		Tube
 */
static BBCTube * bbc_tube_init ( hwaddr addr, uint64_t size,
				 const char *name ) {
	BBCTube *tube = g_new0 ( BBCTube, 1 );

	/* Initialise tube */
	tube->name = name;

	/* Initialise as an unimplemented I/O region */
	tube->unimp = bbc_unimplemented_init ( addr, size, name );

	return tube;
}

/******************************************************************************
 *
 * Machine initialisation
 *
 */

/**
 * Initialise RAM
 *
 * @v bbc		BBC micro
 * @v size		RAM size
 */
static void bbc_ram_init ( BBCMicro *bbc, uint64_t size ) {
	MemoryRegion *address_space_mem = get_system_memory();

	/* Initialise RAM */
	memory_region_init_ram ( &bbc->ram, "ram", size );
	vmstate_register_ram_global ( &bbc->ram );
	memory_region_add_subregion ( address_space_mem, 0, &bbc->ram );
}

/**
 * Initialise ROM
 *
 * @v bbc		BBC micro
 * @v default_filename	Default MOS filename if none provided
 * @v basic_filename	Filename for BASIC ROM (always loaded)
 */
static void bbc_rom_init ( BBCMicro *bbc, const char *default_filename,
			   const char *basic_filename ) {
	unsigned int i;
	unsigned int page;
	const char *filename;

	/* Initialise MOS ROM */
	bbc->mos = bbc_mos_init ( BBC_MOS_BASE, BBC_MOS_SIZE,
				  BBC_MOS_LOW_SIZE, BBC_MOS_HIGH_SIZE,
				  "mos", BBC_MOS_VIRTUAL_BASE );

	/* Initialise paged ROM */
	bbc->paged = bbc_paged_rom_init ( BBC_PAGED_ROM_BASE,
					  BBC_PAGED_ROM_SIZE, "paged_rom",
					  BBC_PAGED_ROM_VIRTUAL_BASE,
					  BBC_PAGED_ROM_COUNT,
					  BBC_SHEILA_PAGED_ROM_SELECT_BASE,
					  BBC_SHEILA_PAGED_ROM_SELECT_SIZE );

	/* Load MOS ROM */
	bbc_mos_load ( bbc->mos, default_filename );

	/* Load any specified option ROMs plus the BASIC ROM */
	for ( i = 0 ; ( ( i < bbc->paged->count ) &&
			( i < ( nb_option_roms + 1 ) ) ) ; i++ ) {
		page = ( bbc->paged->count - i - 1 );
		filename = ( ( i == nb_option_roms ) ? basic_filename :
			     option_rom[i].name );
		bbc_paged_rom_load ( bbc->paged, page, filename );
	}
}

/**
 * Initialise CPU
 *
 * @v bbc		BBC micro
 * @v cpu_model		CPU model specified by user
 * @v default_model	Default CPU model
 */
static void bbc_cpu_init ( BBCMicro *bbc, const char *cpu_model,
			   const char *default_model ) {

	/* Initialise CPU */
	bbc->cpu = m6502_init ( cpu_model ? cpu_model : default_model );
}

/**
 * Initialise I/O regions
 *
 * @v bbc		BBC micro
 */
static void bbc_io_init ( BBCMicro *bbc ) {

	/* Initialise FRED as an unimplemented I/O region */
	bbc->fred = bbc_unimplemented_init ( BBC_FRED_BASE, BBC_FRED_SIZE,
					     "fred" );

	/* Initialise JIM as an unimplemented I/O region */
	bbc->jim = bbc_unimplemented_init ( BBC_JIM_BASE, BBC_JIM_SIZE, "jim" );

	/* Initialise CRTC */
	bbc->crtc = bbc_crtc_init ( BBC_SHEILA_CRTC_BASE, BBC_SHEILA_CRTC_SIZE,
				    "crtc" );

	/* Initialise ACIA */
	bbc->acia = bbc_acia_init ( BBC_SHEILA_ACIA_BASE, BBC_SHEILA_ACIA_SIZE,
				    "acia" );

	/* Initialise serial ULR */
	bbc->serial_ula = bbc_serial_ula_init ( BBC_SHEILA_SERIAL_ULA_BASE,
						BBC_SHEILA_SERIAL_ULA_SIZE,
						"serial_ula" );

	/* Initialise video ULA */
	bbc->video_ula = bbc_video_ula_init ( BBC_SHEILA_VIDEO_ULA_BASE,
					      BBC_SHEILA_VIDEO_ULA_SIZE,
					      "video_ula" );

	/* Initialise system VIA */
	bbc->system_via = bbc_system_via_init ( BBC_SHEILA_SYSTEM_VIA_BASE,
						BBC_SHEILA_SYSTEM_VIA_SIZE,
						"system_via",
						bbc->irq[BBC_IRQ_SYSTEM_VIA],
						bbc->dip );

	/* Initialise user VIA */
	bbc->user_via = bbc_user_via_init ( BBC_SHEILA_USER_VIA_BASE,
					    BBC_SHEILA_USER_VIA_SIZE,
					    "user_via",
					    bbc->irq[BBC_IRQ_USER_VIA],
					    parallel_hds[0] );

	/* Initialise floppy disc controller */
	bbc->fdc = bbc_1770_fdc_init ( BBC_SHEILA_FDC_BASE,
				       BBC_SHEILA_FDC_SIZE, "fdc",
				       bbc->nmi[BBC_NMI_FDC_DRQ],
				       bbc->nmi[BBC_NMI_FDC_INTRQ] );

	/* Initialise Econet controller */
	bbc->econet = bbc_econet_init ( BBC_SHEILA_ECONET_BASE,
					BBC_SHEILA_ECONET_SIZE, "econet" );

	/* Initialise analogue to digital converter */
	bbc->adc = bbc_adc_init ( BBC_SHEILA_ADC_BASE, BBC_SHEILA_ADC_SIZE,
				  "adc" );

	/* Initialise Tube */
	bbc->tube = bbc_tube_init ( BBC_SHEILA_TUBE_BASE, BBC_SHEILA_TUBE_SIZE,
				    "tube" );
}

/**
 * Initialise display
 *
 * @v bbc		BBC micro
 */
static void bbc_display_init ( BBCMicro *bbc ) {

	/* Initialise CRT */
	bbc->crt = bbc_crt_init ( "crt", &bbc->ram, bbc->crtc, bbc->video_ula,
				  bbc->system_via );
}

/**
 * Initialise BBC Model B
 *
 * @v args		Machine arguments
 */
static void bbcb_init ( QEMUMachineInitArgs *args ) {
	DeviceState *qdev;
	BBCMicro *bbc;

	/* Initialise machine */
	qdev = qdev_create ( NULL, "bbc" );
	bbc = DO_UPCAST ( BBCMicro, qdev, qdev );
	qdev_init_nofail ( qdev );

	/* Initialise RAM */
	bbc_ram_init ( bbc, BBC_B_RAM_SIZE );

	/* Initialise and load ROMs */
	bbc_rom_init ( bbc, BBC_B_MOS_FILENAME, BBC_B_BASIC_FILENAME );

	/* Initialise CPU */
	bbc_cpu_init ( bbc, args->cpu_model, BBC_B_DEFAULT_CPU_MODEL );

	/* Initialise interrupts */
	bbc_interrupts_init ( bbc );

	/* Initialise I/O regions */
	bbc_io_init ( bbc );

	/* Initialise display */
	bbc_display_init ( bbc );
}

/** BBC Model B */
static QEMUMachine bbc_model_b = {
	.name = "bbcb",
	.desc = "Acorn BBC Micro Model B",
	.init = bbcb_init,
	.is_default = 1,
};

/**
 * Register BBC machines
 */
static void bbc_machine_init ( void ) {
	qemu_register_machine ( &bbc_model_b );
}

machine_init ( bbc_machine_init );

/******************************************************************************
 *
 * Machine properties
 *
 */

/** QEMU device initialiser */
static int bbc_qdev_init ( DeviceState *qdev ) {
	return 0;
}

/** BBC Micro state description */
static const VMStateDescription vmstate_bbc = {
	.name = "bbc",
	.version_id = 1,
	.minimum_version_id = 1,
	.fields = ( VMStateField[] ) {
		VMSTATE_BOOL_ARRAY ( irq_active, BBCMicro, BBC_IRQ_COUNT ),
		VMSTATE_UINT16 ( irq_count, BBCMicro ),
		VMSTATE_BOOL_ARRAY ( nmi_active, BBCMicro, BBC_NMI_COUNT ),
		VMSTATE_UINT16 ( nmi_count, BBCMicro ),
		VMSTATE_UINT8 ( dip, BBCMicro ),
		VMSTATE_END_OF_LIST()
	},
};

/** Properties */
static Property bbc_properties[] = {
	DEFINE_PROP_UINT8 ( "dip", BBCMicro, dip, 0 ),
	DEFINE_PROP_END_OF_LIST(),
};

/** Class initialiser */
static void bbc_class_init ( ObjectClass *class, void *data ) {
	DeviceClass *dc = DEVICE_CLASS ( class );

	dc->init = bbc_qdev_init;
	dc->vmsd = &vmstate_bbc;
	dc->props = bbc_properties;
}

/** Type information */
static TypeInfo bbc_info = {
	.name = "bbc",
	.parent = TYPE_DEVICE,
	.instance_size = sizeof ( BBCMicro ),
	.class_init = bbc_class_init,
};

/** Type registrar */
static void bbc_register_types ( void ) {
	type_register_static ( &bbc_info );
}

/** Type initialiser */
type_init ( bbc_register_types );
