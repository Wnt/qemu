/*
 * Gallery low-latency input device.
 *
 * Copyright (c) 2026 osgallery contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "chardev/char-fe.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/gallery-hid.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "qemu/atomic.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_GALLERY_HID_PCI "gallery-hid-pci"
OBJECT_DECLARE_SIMPLE_TYPE(GalleryHIDState, GALLERY_HID_PCI)

struct GalleryHIDState {
    PCIDevice parent_obj;
    MemoryRegion bar0;
    MemoryRegion bar2;
    CharFrontend chr;
    uint8_t *ram;

    uint32_t irq_status;
    uint32_t irq_mask;
    uint32_t epoch;
    uint32_t producer;
    uint32_t protocol_errors;
    uint32_t ring_stalls;
    uint32_t saved_consumer;
    uint16_t next_sequence;
    bool backend_connected;
    bool driver_ready;
    bool reset_required;
    bool stalled;
    bool hello_done;
    bool irq_level;
    bool rearm_required;

    uint8_t input[GHID_FRAME_SIZE];
    unsigned input_have;
    uint8_t staged[GHID_RECORD_SIZE];
    bool staged_valid;
};

static uint32_t gallery_status(GalleryHIDState *s)
{
    uint32_t status = 0;

    status |= s->backend_connected ? GHID_STATUS_CONNECTED : 0;
    status |= s->driver_ready ? GHID_STATUS_DRIVER_READY : 0;
    status |= (s->reset_required || s->rearm_required) ?
              GHID_STATUS_RESET_REQ : 0;
    status |= s->stalled ? GHID_STATUS_STALLED : 0;
    return status;
}

static void gallery_update_irq(GalleryHIDState *s)
{
    s->irq_level = (s->irq_status & s->irq_mask) != 0;
    pci_set_irq(&s->parent_obj, s->irq_level);
}

static uint32_t gallery_consumer(GalleryHIDState *s)
{
    uint32_t *p = (uint32_t *)(s->ram + GHID_RING_OFF_CONSUMER);

    return le32_to_cpu(qatomic_load_acquire(p));
}

static void gallery_store_producer(GalleryHIDState *s)
{
    uint32_t *p = (uint32_t *)(s->ram + GHID_RING_OFF_PRODUCER);

    qatomic_store_release(p, cpu_to_le32(s->producer));
}

static void gallery_write_header(GalleryHIDState *s)
{
    stl_le_p(s->ram + GHID_RING_OFF_MAGIC, GHID_RING_MAGIC);
    stw_le_p(s->ram + GHID_RING_OFF_MAJOR, 1);
    stw_le_p(s->ram + GHID_RING_OFF_MINOR, 0);
    stw_le_p(s->ram + GHID_RING_OFF_HDR_BYTES, GHID_HEADER_SIZE);
    stw_le_p(s->ram + GHID_RING_OFF_REC_BYTES, GHID_RECORD_SIZE);
    stw_le_p(s->ram + GHID_RING_OFF_ENTRIES, GHID_RING_ENTRIES);
    stl_le_p(s->ram + GHID_RING_OFF_FEATURES, GHID_FEATURES);
    stl_le_p(s->ram + GHID_RING_OFF_EPOCH, s->epoch);
    gallery_store_producer(s);
    stw_le_p(s->ram + GHID_RING_OFF_NEXT_SEQ, s->next_sequence);
    stl_le_p(s->ram + GHID_RING_OFF_PROTO_ERRS, s->protocol_errors);
    stl_le_p(s->ram + GHID_RING_OFF_STALLS, s->ring_stalls);
}

static void gallery_ring_reset(GalleryHIDState *s, bool clear_counters)
{
    memset(s->ram, 0, GHID_BAR2_SIZE);
    s->epoch++;
    if (s->epoch == 0) {
        s->epoch = 1;
    }
    s->producer = 0;
    s->next_sequence = 0;
    s->driver_ready = false;
    s->reset_required = true;
    s->stalled = false;
    s->staged_valid = false;
    if (clear_counters) {
        s->protocol_errors = 0;
        s->ring_stalls = 0;
    }
    gallery_write_header(s);
    s->irq_status = GHID_IRQ_RESET;
    gallery_update_irq(s);
}

static void gallery_protocol_reset(GalleryHIDState *s)
{
    s->protocol_errors++;
    gallery_ring_reset(s, false);
}

static bool gallery_consumer_valid(GalleryHIDState *s, uint32_t consumer)
{
    if (s->producer - consumer <= GHID_RING_ENTRIES) {
        return true;
    }
    gallery_protocol_reset(s);
    return false;
}

static bool gallery_record_valid(const uint8_t *r)
{
    uint16_t key;

    if (lduw_le_p(r + 2) != 0) {
        return false;
    }
    switch (r[0]) {
    case GHID_EVENT_POINTER_ABS:
        return r[1] == 0 && lduw_le_p(r + 4) <= 32767 &&
               lduw_le_p(r + 6) <= 32767 &&
               (lduw_le_p(r + 8) & ~0x1fU) == 0;
    case GHID_EVENT_KEY:
        key = lduw_le_p(r + 4);
        return (r[1] & ~0x03U) == 0 &&
               (key <= 0x007f ||
                ((key & 0xff00) == 0xe000 && (key & 0xff) <= 0x7f) ||
                key == 0xe145) &&
               lduw_le_p(r + 6) <= 0x00ff &&
               ldl_le_p(r + 8) == 0;
    case GHID_EVENT_RELEASE_ALL:
        return (r[1] & ~0x07U) == 0 &&
               ldl_le_p(r + 4) == 0 && ldl_le_p(r + 8) == 0;
    default:
        return false;
    }
}

static bool gallery_publish(GalleryHIDState *s, const uint8_t *host_record)
{
    uint8_t record[GHID_RECORD_SIZE];
    uint32_t consumer = gallery_consumer(s);
    uint32_t occupancy;
    uint8_t *slot;

    if (!gallery_consumer_valid(s, consumer)) {
        return false;
    }
    occupancy = s->producer - consumer;
    if (occupancy == GHID_RING_ENTRIES) {
        if (!s->staged_valid) {
            memcpy(s->staged, host_record, GHID_RECORD_SIZE);
            s->staged_valid = true;
            s->ring_stalls++;
            stl_le_p(s->ram + GHID_RING_OFF_STALLS, s->ring_stalls);
        }
        s->stalled = true;
        return false;
    }

    memcpy(record, host_record, sizeof(record));
    stw_le_p(record + 2, s->next_sequence++);
    slot = s->ram + GHID_RING_OFF_RECORDS +
           ((s->producer & GHID_RING_MASK) * GHID_RECORD_SIZE);
    memcpy(slot, record, sizeof(record));
    smp_wmb();
    s->producer++;
    gallery_store_producer(s);
    stw_le_p(s->ram + GHID_RING_OFF_NEXT_SEQ, s->next_sequence);
    s->irq_status |= GHID_IRQ_RING;
    s->stalled = (s->producer - consumer == GHID_RING_ENTRIES);
    gallery_update_irq(s);
    return true;
}

static void gallery_retry_staged(GalleryHIDState *s)
{
    uint8_t record[GHID_RECORD_SIZE];

    if (!s->staged_valid) {
        return;
    }
    memcpy(record, s->staged, sizeof(record));
    s->staged_valid = false;
    if (!gallery_publish(s, record) && !s->staged_valid) {
        memcpy(s->staged, record, sizeof(record));
        s->staged_valid = true;
    }
}

static void gallery_recheck(GalleryHIDState *s)
{
    uint32_t consumer = gallery_consumer(s);

    if (!gallery_consumer_valid(s, consumer)) {
        return;
    }
    gallery_retry_staged(s);
    consumer = gallery_consumer(s);
    if (!gallery_consumer_valid(s, consumer)) {
        return;
    }
    if (s->producer != consumer) {
        s->irq_status |= GHID_IRQ_RING;
    } else {
        s->irq_status &= ~GHID_IRQ_RING;
    }
    s->stalled = s->staged_valid ||
                 (s->producer - consumer == GHID_RING_ENTRIES);
    gallery_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static uint64_t gallery_bar0_read(void *opaque, hwaddr addr, unsigned size)
{
    GalleryHIDState *s = opaque;

    if (size != 4 || (addr & 3)) {
        s->protocol_errors++;
        stl_le_p(s->ram + GHID_RING_OFF_PROTO_ERRS, s->protocol_errors);
        return 0;
    }
    switch (addr) {
    case GHID_REG_DEVICE_MAGIC:
        return GHID_DEVICE_MAGIC;
    case GHID_REG_ABI_VERSION:
        return GHID_ABI_VERSION;
    case GHID_REG_FEATURES:
        return GHID_FEATURES;
    case GHID_REG_STATUS:
        return gallery_status(s);
    case GHID_REG_IRQ_STATUS:
        return s->irq_status;
    case GHID_REG_IRQ_MASK:
        return s->irq_mask;
    default:
        return 0;
    }
}

static void gallery_bar0_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    GalleryHIDState *s = opaque;
    uint32_t val = value;

    if (size != 4 || (addr & 3)) {
        s->protocol_errors++;
        stl_le_p(s->ram + GHID_RING_OFF_PROTO_ERRS, s->protocol_errors);
        return;
    }
    switch (addr) {
    case GHID_REG_IRQ_MASK:
        s->irq_mask = val & GHID_IRQ_ALL;
        gallery_update_irq(s);
        break;
    case GHID_REG_IRQ_ACK:
        if (!gallery_consumer_valid(s, gallery_consumer(s))) {
            break;
        }
        s->irq_status &= ~(val & GHID_IRQ_ALL);
        if ((val & GHID_IRQ_RING) &&
            s->producer != gallery_consumer(s)) {
            s->irq_status |= GHID_IRQ_RING;
        }
        gallery_update_irq(s);
        break;
    case GHID_REG_DRIVER_READY:
        if (val == s->epoch &&
            ldl_le_p(s->ram + GHID_RING_OFF_MAGIC) == GHID_RING_MAGIC &&
            gallery_consumer_valid(s, gallery_consumer(s))) {
            s->driver_ready = true;
            s->reset_required = false;
            s->rearm_required = false;
            gallery_recheck(s);
        }
        break;
    case GHID_REG_GUEST_KICK:
        gallery_recheck(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps gallery_bar0_ops = {
    .read = gallery_bar0_read,
    .write = gallery_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void gallery_bad_backend_input(GalleryHIDState *s)
{
    s->protocol_errors++;
    stl_le_p(s->ram + GHID_RING_OFF_PROTO_ERRS, s->protocol_errors);
}

static bool gallery_hello_valid(const uint8_t *h)
{
    return memcmp(h, GHID_HELLO_MAGIC, 4) == 0 &&
           lduw_le_p(h + 4) == GHID_BACKEND_MAJOR &&
           lduw_le_p(h + 6) == GHID_BACKEND_MINOR &&
           lduw_le_p(h + 8) == GHID_RECORD_SIZE &&
           lduw_le_p(h + 10) == 0 &&
           ldl_le_p(h + 12) == 0;
}

static void gallery_finish_hello(GalleryHIDState *s)
{
    uint8_t reply[GHID_FRAME_SIZE] = { 0 };

    memcpy(reply, GHID_OK_MAGIC, 4);
    stw_le_p(reply + 4, GHID_BACKEND_MAJOR);
    stw_le_p(reply + 6, GHID_BACKEND_MINOR);
    s->hello_done = true;
    s->backend_connected = true;
    s->irq_status |= GHID_IRQ_LINK;
    gallery_update_irq(s);
    stl_le_p(reply + 8, s->epoch);
    stl_le_p(reply + 12, gallery_status(s));
    if (qemu_chr_fe_write_all(&s->chr, reply, sizeof(reply)) != sizeof(reply)) {
        qemu_chr_fe_disconnect(&s->chr);
    }
}

static int gallery_can_receive(void *opaque)
{
    GalleryHIDState *s = opaque;

    if (s->hello_done && (!s->driver_ready || s->rearm_required)) {
        return 0;
    }
    if (s->hello_done && s->staged_valid) {
        return 0;
    }
    return GHID_FRAME_SIZE - s->input_have;
}

static void gallery_receive(void *opaque, const uint8_t *buf, int size)
{
    GalleryHIDState *s = opaque;

    while (size-- > 0) {
        s->input[s->input_have++] = *buf++;
        if (s->input_have != GHID_FRAME_SIZE) {
            continue;
        }
        s->input_have = 0;
        if (!s->hello_done) {
            if (!gallery_hello_valid(s->input)) {
                gallery_bad_backend_input(s);
                qemu_chr_fe_disconnect(&s->chr);
                return;
            }
            gallery_finish_hello(s);
        } else if (!gallery_record_valid(s->input)) {
            gallery_bad_backend_input(s);
        } else {
            gallery_publish(s, s->input);
        }
    }
}

static void gallery_chr_event(void *opaque, QEMUChrEvent event)
{
    GalleryHIDState *s = opaque;
    bool was_connected = s->backend_connected;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->hello_done = false;
        s->backend_connected = false;
        s->input_have = 0;
        if (was_connected) {
            s->irq_status |= GHID_IRQ_LINK;
            gallery_update_irq(s);
        }
        break;
    case CHR_EVENT_CLOSED:
        s->hello_done = false;
        s->backend_connected = false;
        s->input_have = 0;
        if (was_connected) {
            s->irq_status |= GHID_IRQ_LINK;
            gallery_update_irq(s);
        }
        break;
    default:
        break;
    }
}

static void gallery_reset(DeviceState *dev)
{
    GalleryHIDState *s = GALLERY_HID_PCI(dev);

    pci_set_irq(&s->parent_obj, 0);
    s->irq_level = false;
    s->irq_mask = 0;
    s->hello_done = false;
    s->backend_connected = false;
    s->input_have = 0;
    s->rearm_required = false;
    gallery_ring_reset(s, true);
}

static int gallery_pre_save(void *opaque)
{
    GalleryHIDState *s = opaque;

    s->saved_consumer = gallery_consumer(s);
    s->irq_level = (s->irq_status & s->irq_mask) != 0;
    return 0;
}

static int gallery_post_load(void *opaque, int version_id)
{
    GalleryHIDState *s = opaque;
    uint32_t ram_epoch = ldl_le_p(s->ram + GHID_RING_OFF_EPOCH);
    uint32_t ram_producer = ldl_le_p(s->ram + GHID_RING_OFF_PRODUCER);
    uint32_t ram_consumer = ldl_le_p(s->ram + GHID_RING_OFF_CONSUMER);
    uint16_t ram_sequence = lduw_le_p(s->ram + GHID_RING_OFF_NEXT_SEQ);

    /* The backend stream is a process-local transport, never VMState. */
    pci_set_irq(&s->parent_obj, 0);
    s->irq_level = false;

    s->hello_done = false;
    s->backend_connected = false;
    s->input_have = 0;
    s->staged_valid = false;

    if (s->epoch == 0 || ram_epoch != s->epoch ||
        ram_producer != s->producer || ram_consumer != s->saved_consumer ||
        s->producer - ram_consumer > GHID_RING_ENTRIES ||
        ram_sequence != s->next_sequence ||
        ldl_le_p(s->ram + GHID_RING_OFF_MAGIC) != GHID_RING_MAGIC ||
        lduw_le_p(s->ram + GHID_RING_OFF_MAJOR) != 1 ||
        lduw_le_p(s->ram + GHID_RING_OFF_MINOR) != 0 ||
        lduw_le_p(s->ram + GHID_RING_OFF_HDR_BYTES) != GHID_HEADER_SIZE ||
        lduw_le_p(s->ram + GHID_RING_OFF_REC_BYTES) != GHID_RECORD_SIZE ||
        lduw_le_p(s->ram + GHID_RING_OFF_ENTRIES) != GHID_RING_ENTRIES ||
        ldl_le_p(s->ram + GHID_RING_OFF_FEATURES) != GHID_FEATURES ||
        (s->irq_status & ~GHID_IRQ_ALL) != 0 ||
        (s->irq_mask & ~GHID_IRQ_ALL) != 0 ||
        (s->driver_ready &&
         ldl_le_p(s->ram + GHID_RING_OFF_LAST_EPOCH) != s->epoch)) {
        error_report("gallery-hid-pci: inconsistent VMState/BAR2 "
                     "epoch=%u/%u producer=%u/%u consumer=%u/%u "
                     "sequence=%u/%u",
                     s->epoch, ram_epoch, s->producer, ram_producer,
                     s->saved_consumer, ram_consumer,
                     s->next_sequence, ram_sequence);
        return -EINVAL;
    }

    /* Saved causes/INTA are stale until a fresh backend hello and guest rearm. */
    s->irq_status = 0;
    s->stalled = s->producer - ram_consumer == GHID_RING_ENTRIES;
    s->rearm_required = true;
    return 0;
}

static const VMStateDescription vmstate_gallery_hid = {
    .name = TYPE_GALLERY_HID_PCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = gallery_pre_save,
    .post_load = gallery_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, GalleryHIDState),
        VMSTATE_UINT32(irq_status, GalleryHIDState),
        VMSTATE_UINT32(irq_mask, GalleryHIDState),
        VMSTATE_UINT32(epoch, GalleryHIDState),
        VMSTATE_UINT32(producer, GalleryHIDState),
        VMSTATE_UINT32(protocol_errors, GalleryHIDState),
        VMSTATE_UINT32(ring_stalls, GalleryHIDState),
        VMSTATE_UINT32(saved_consumer, GalleryHIDState),
        VMSTATE_UINT16(next_sequence, GalleryHIDState),
        VMSTATE_BOOL(driver_ready, GalleryHIDState),
        VMSTATE_BOOL(reset_required, GalleryHIDState),
        VMSTATE_BOOL(stalled, GalleryHIDState),
        VMSTATE_BOOL(irq_level, GalleryHIDState),
        VMSTATE_END_OF_LIST()
    },
};

static void gallery_realize(PCIDevice *pdev, Error **errp)
{
    GalleryHIDState *s = GALLERY_HID_PCI(pdev);

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "gallery-hid-pci requires a chardev");
        return;
    }
    pdev->config[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->bar0, OBJECT(s), &gallery_bar0_ops, s,
                          "gallery-hid-control", GHID_BAR0_SIZE);
    memory_region_init_ram(&s->bar2, OBJECT(s), "gallery-hid-ring",
                           GHID_BAR2_SIZE, errp);
    if (*errp) {
        return;
    }
    s->ram = memory_region_get_ram_ptr(&s->bar2);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(pdev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_PREFETCH, &s->bar2);
    qemu_chr_fe_set_handlers(&s->chr, gallery_can_receive, gallery_receive,
                             gallery_chr_event, NULL, s, NULL, true);
}

static void gallery_exit(PCIDevice *pdev)
{
    GalleryHIDState *s = GALLERY_HID_PCI(pdev);

    qemu_chr_fe_set_handlers(&s->chr, NULL, NULL, NULL, NULL,
                             NULL, NULL, false);
    pci_set_irq(pdev, 0);
}

static const Property gallery_properties[] = {
    DEFINE_PROP_CHR("chardev", GalleryHIDState, chr),
};

static void gallery_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = gallery_realize;
    pc->exit = gallery_exit;
    pc->vendor_id = GHID_PCI_VENDOR_ID;
    pc->device_id = GHID_PCI_DEVICE_ID;
    pc->revision = GHID_PCI_REVISION;
    pc->class_id = GHID_PCI_CLASS;
    dc->vmsd = &vmstate_gallery_hid;
    device_class_set_legacy_reset(dc, gallery_reset);
    device_class_set_props(dc, gallery_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo gallery_type_info = {
    .name = TYPE_GALLERY_HID_PCI,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(GalleryHIDState),
    .class_init = gallery_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void gallery_register_types(void)
{
    type_register_static(&gallery_type_info);
}
type_init(gallery_register_types)
