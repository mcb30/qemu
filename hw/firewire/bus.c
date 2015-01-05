/*
 * QEMU IEEE1394 emulation
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

#include "hw/hw.h"
#include "hw/firewire.h"
#include "monitor/monitor.h"

void
firewire_transmit(FireWireBus *bus, FireWirePort *port, const void *header,
		  size_t header_len, const void *data, size_t data_len)
{
    FireWirePort *dest_port;
    unsigned int i;
    const uint8_t *bytes;

    /* Broadcast to all ports except the originating port */
    QTAILQ_FOREACH(dest_port, &bus->used, next) {
	if (dest_port != port)
	    port->dev->receive(port->dev, header, header_len, data, data_len);
    }

    /* Broadcast to host controller (unless originating from the controller) */
    if (port)
	bus->receive(bus, header, header_len, data, data_len);

    bytes = header;
    for (i = 0; i < header_len; i += 4)
	DBG("TXH: %02x%02x%02x%02x\n", bytes[i + 0], bytes[i + 1],
	    bytes[i + 2], bytes[i + 3]);
    bytes = data;
    for (i = 0; i < len; i += 4)
	DBG("TXD: %02x%02x%02x%02x\n", bytes[i + 0], bytes[i + 1],
	    bytes[i + 2], bytes[i + 3]);
}

void
firewire_bus_new(FireWireBus *bus, DeviceState *host)
{
    qbus_create_inplace(bus, sizeof(*bus), TYPE_FIREWIRE_BUS, host, NULL);
    bus->qbus.allow_hotplug = 1;
}

void
firewire_register_port(FireWireBus *bus, FireWirePort *port, unsigned int index)
{
    port->index = index;
    QTAILQ_INSERT_TAIL(&bus->free, port, next);
    bus->nfree++;
}

static Property firewire_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static void
firewire_bus_print_dev(Monitor *mon, DeviceState *qdev, int indent)
{
    FireWireDevice *dev = FIREWIRE_DEVICE(qdev);
    FireWireBus *bus = firewire_bus_from_device(dev);

    monitor_printf(mon, "%*sport %d, addr %d.%d\n",
		   indent, "", dev->port->index, bus->bus_id, dev->phy_id);
}

static char *
firewire_get_dev_path(DeviceState *qdev)
{
    FireWireDevice *dev = FIREWIRE_DEVICE(qdev);
    DeviceState *hcd = qdev->parent_bus->parent;
    char *id;
    char *path;

    id = qdev_get_dev_path(hcd);
    if (id) {
	path = g_strdup_printf("%s/%d", id, dev->port->index);
    } else {
	path = g_strdup_printf("%d", dev->port->index);
    }
    g_free(id);
    return path;
}

static char *
firewire_get_fw_dev_path(DeviceState *qdev)
{
    return NULL;
}

static void
firewire_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->print_dev = firewire_bus_print_dev;
    k->get_dev_path = firewire_get_dev_path;
    k->get_fw_dev_path = firewire_get_fw_dev_path;
}

static const TypeInfo firewire_bus_info = {
    .name = TYPE_FIREWIRE_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(FireWireBus),
    .class_init = firewire_bus_class_init,
};

static int
firewire_qdev_init(DeviceState *qdev)
{
    return 0;
}

static int
firewire_qdev_exit(DeviceState *qdev)
{
    return 0;
}

static void
firewire_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->bus_type = TYPE_FIREWIRE_BUS;
    k->init     = firewire_qdev_init;
    k->unplug   = qdev_simple_unplug_cb;
    k->exit     = firewire_qdev_exit;
    k->props    = firewire_props;
}

static const TypeInfo firewire_device_type_info = {
    .name = TYPE_FIREWIRE_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(FireWireDevice),
    .abstract = true,
    .class_size = sizeof(FireWireDeviceClass),
    .class_init = firewire_device_class_init,
};

static void
firewire_register_types(void)
{
    type_register_static(&firewire_bus_info);
    type_register_static(&firewire_device_type_info);
}

type_init(firewire_register_types)

/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 8
 * End:
 */
