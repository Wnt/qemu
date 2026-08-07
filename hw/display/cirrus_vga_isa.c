/*
 * QEMU Cirrus CLGD 54xx VGA Emulator, ISA bus support
 *
 * Copyright (c) 2004 Fabrice Bellard
 * Copyright (c) 2004 Makoto Suzuki (suzu)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/isa/isa.h"
#include "migration/vmstate.h"
#include "cirrus_vga_internal.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_ISA_CIRRUS_VGA "isa-cirrus-vga"
OBJECT_DECLARE_SIMPLE_TYPE(ISACirrusVGAState, ISA_CIRRUS_VGA)

struct ISACirrusVGAState {
    ISADevice parent_obj;

    CirrusVGAState cirrus_vga;
};

static void isa_cirrus_vga_realizefn(DeviceState *dev, Error **errp)
{
    ISADevice *isadev = ISA_DEVICE(dev);
    ISACirrusVGAState *d = ISA_CIRRUS_VGA(dev);
    VGACommonState *s = &d->cirrus_vga.vga;

    /* follow real hardware, cirrus card emulated has 4 MB video memory.
       Also accept 8 MB/16 MB for backward compatibility. */
    if (s->vram_size_mb != 4 && s->vram_size_mb != 8 &&
        s->vram_size_mb != 16) {
        error_setg(errp, "Invalid cirrus_vga ram size '%u'",
                   s->vram_size_mb);
        return;
    }
    if (!vga_common_init(s, OBJECT(dev), errp)) {
        return;
    }
    cirrus_init_common(&d->cirrus_vga, OBJECT(dev), CIRRUS_ID_CLGD5430, 0,
                       isa_address_space(isadev),
                       isa_address_space_io(isadev));
    s->con = graphic_console_init(dev, 0, s->hw_ops, s);
    rom_add_vga(VGABIOS_CIRRUS_FILENAME);
    /* XXX ISA-LFB support */
    /* FIXME not qdev yet */
}

static const Property isa_cirrus_vga_properties[] = {
    DEFINE_PROP_UINT32("vgamem_mb", struct ISACirrusVGAState,
                       cirrus_vga.vga.vram_size_mb, 4),
    DEFINE_PROP_BOOL("blitter", struct ISACirrusVGAState,
                     cirrus_vga.enable_blitter, true),
    DEFINE_PROP_BOOL("global-vmstate", struct ISACirrusVGAState,
                     cirrus_vga.vga.global_vmstate, false),
};

/*
 * vmstate fix (angle=trace): the ISA device's vmsd must descend into the
 * embedded CirrusVGAState.  Pointing dc->vmsd straight at vmstate_cirrus_vga
 * resolves that VMSD's CirrusVGAState field offsets against the *device*
 * (ISACirrusVGAState) base, which is 160 bytes before cirrus_vga.  The
 * VGACommonState-relative fields still round-tripped, but the cirrus-direct
 * fields (cirrus_hidden_dac_data / _lockindex, cirrus_shadow_gr0/1) were saved
 * and restored from the wrong location, so the hi-colour DAC depth was lost on
 * loadvm (16bpp 565 -> 15bpp 555 = the blue/lavender corruption).  Wrap exactly
 * like the PCI variant (vmstate_pci_cirrus_vga) so the substate opaque is the
 * real CirrusVGAState.  Wire format and section name ("cirrus_vga") are
 * unchanged.
 */
static const VMStateDescription vmstate_isa_cirrus_vga = {
    .name = "cirrus_vga",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(cirrus_vga, ISACirrusVGAState, 0,
                       vmstate_cirrus_vga, CirrusVGAState),
        VMSTATE_END_OF_LIST()
    }
};

static void isa_cirrus_vga_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd  = &vmstate_isa_cirrus_vga;
    dc->realize = isa_cirrus_vga_realizefn;
    device_class_set_props(dc, isa_cirrus_vga_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo isa_cirrus_vga_info = {
    .name          = TYPE_ISA_CIRRUS_VGA,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISACirrusVGAState),
    .class_init = isa_cirrus_vga_class_init,
};

static void cirrus_vga_isa_register_types(void)
{
    type_register_static(&isa_cirrus_vga_info);
}

type_init(cirrus_vga_isa_register_types)
