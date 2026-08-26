/*
 * QEMU Matrox MGA G200 (GXT130P-compatible) display adapter
 *
 * A minimal model of the Matrox G200 PCI graphics chip (0x102B:0x0520),
 * sufficient for IBM AIX 4.3's GXT130P driver (devices.pci.2b102005) to
 * drive a linear framebuffer on the IBM 40p (PReP) machine.
 *
 * Modelled pieces:
 *  - PCI config: OPTION/OPTION2/OPTION3 scratch regs, MGA_INDEX/MGA_DATA
 *    config-space window into the control aperture
 *  - BAR0: framebuffer aperture (VRAM, prefetchable)
 *  - BAR1: 16KB control aperture:
 *      - drawing engine registers are stored only (no acceleration)
 *      - host registers: FIFOSTATUS, STATUS, ICLEAR, IEN, VCOUNT,
 *        RESET, OPMODE (with synthesized vsync/vline)
 *      - VGA register mirror at 0x1F00..0x1FFF (forwarded to VGA core)
 *      - CRTCEXT0..8 via 0x1FDE/0x1FDF
 *      - integrated DAC ("DAC1064"-style) at 0x3C00: palette (forwarded
 *        to the VGA DAC), X-registers, PLLs (always report locked),
 *        GPIO-bitbanged I2C with a DDC monitor (EDID)
 *  - BAR2: ILOAD aperture stub
 *
 * Mode setting follows the ati-vga approach: extended ("power graphics")
 * modes are translated into the VGA core's Bochs VBE registers, which
 * takes care of rendering, dirty tracking and panning.
 *
 * Register-level references: Linux matroxfb, Xorg xf86-video-mga.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/range.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_ids.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/bitbang_i2c.h"
#include "hw/display/i2c-ddc.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "vga_int.h"
#include "vga_regs.h"
#include "qom/object.h"

#define TYPE_MGA "mga"
OBJECT_DECLARE_SIMPLE_TYPE(MGAState, MGA)

/* control aperture layout */
#define MGA_DWG_BASE        0x1C00  /* drawing engine registers            */
#define MGA_DWG_END         0x1E00  /* 0x1D00.. are the "GO" aliases       */
#define MGA_HOST_BASE       0x1E00
#define MGA_HOST_END        0x1F00
#define MGA_VGA_BASE        0x1F00  /* VGA mirror: offset-0x1C00 = port    */
#define MGA_VGA_END         0x2000
#define MGA_RAMDAC_BASE     0x3C00
#define MGA_RAMDAC_END      0x3D00

/* host registers */
#define MGA_FIFOSTATUS      0x1E10
#define MGA_STATUS          0x1E14
#define MGA_ICLEAR          0x1E18
#define MGA_IEN             0x1E1C
#define MGA_VCOUNT          0x1E20
#define MGA_RESET           0x1E40
#define MGA_OPMODE          0x1E54

#define MGA_STATUS_SOFTRAPEN    0x00000001
#define MGA_STATUS_PICKPEN      0x00000004
#define MGA_STATUS_VSYNCSTS     0x00000008
#define MGA_STATUS_VLINEPEN     0x00000020
#define MGA_STATUS_EXTPEN       0x00000040
#define MGA_STATUS_DWGENGSTS    0x00010000

/* CRTCEXT index/data in the VGA mirror */
#define MGA_CRTCEXT_INDEX   0x1FDE
#define MGA_CRTCEXT_DATA    0x1FDF

/* DAC direct registers (offsets from MGA_RAMDAC_BASE) */
#define MGA_DAC_PALWRADD    0x00    /* also X register index */
#define MGA_DAC_PALDATA     0x01
#define MGA_DAC_PIXRDMSK    0x02
#define MGA_DAC_PALRDADD    0x03
#define MGA_DAC_X_DATAREG   0x0A
#define MGA_DAC_CURPOSXL    0x0C
#define MGA_DAC_CURPOSXH    0x0D
#define MGA_DAC_CURPOSYL    0x0E
#define MGA_DAC_CURPOSYH    0x0F

/* X registers */
#define MGA_XCURCTRL        0x06
#define MGA_XMULCTRL        0x19
#define MGA_XPIXCLKCTRL     0x1A
#define MGA_XMISCCTRL       0x1E
#define MGA_XGENIOCTRL      0x2A
#define MGA_XGENIODATA      0x2B
#define MGA_XSYSPLLSTAT     0x2F
#define MGA_XSENSETEST      0x3A
#define MGA_XPIXPLLSTAT     0x4F

#define MGA_XMISCCTRL_DAC_8BIT  0x08

/* DDC (primary head, G100/G200): XGENIO bits */
#define MGA_DDC_DATA        0x02
#define MGA_DDC_CLK         0x08

/* PCI config */
#define MGA_PCI_OPTION      0x40
#define MGA_PCI_MGA_INDEX   0x44
#define MGA_PCI_MGA_DATA    0x48
#define MGA_PCI_OPTION2     0x50
#define MGA_PCI_OPTION3     0x54

#define MGA_CTRL_SIZE       0x4000
#define MGA_ILOAD_SIZE      (8 * MiB)

#define MGA_VLINE_PERIOD_NS (NANOSECONDS_PER_SECOND / 60)

struct MGAState {
    PCIDevice dev;
    VGACommonState vga;

    MemoryRegion ctrl;
    MemoryRegion iload;

    uint8_t crtcext_idx;
    uint8_t crtcext[16];
    uint8_t xindex;
    uint8_t xreg[256];
    uint8_t dwgreg[256];

    uint32_t status;        /* latched interrupt-pending bits */
    uint32_t ien;
    uint32_t opmode;
    uint32_t chip_reset;
    uint32_t mga_index;     /* PCI config 0x44 */

    bitbang_i2c_interface bbi2c;
    I2CDDCState ddc;
    uint8_t sda_line;

    QEMUTimer vline_timer;

    /* cached VBE-programmed mode, to avoid re-programming */
    uint32_t mode_width, mode_height, mode_bpp;
    uint32_t mode_pitch, mode_offs;
    bool mode_ext, mode_be;
};

static void mga_update_irq(MGAState *s)
{
    pci_set_irq(&s->dev, (s->status & s->ien) ? 1 : 0);
}

static void mga_vline_timer_cb(void *opaque)
{
    MGAState *s = opaque;

    if (s->ien & MGA_STATUS_VLINEPEN) {
        s->status |= MGA_STATUS_VLINEPEN;
        mga_update_irq(s);
        timer_mod(&s->vline_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MGA_VLINE_PERIOD_NS);
    }
}

/* synthesized beam position: ~60Hz frame, 1066 total lines */
#define MGA_FAKE_VTOTAL 1066
#define MGA_FAKE_VDISP  1024

static uint32_t mga_vcount(MGAState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t in_frame = now % MGA_VLINE_PERIOD_NS;

    return (uint32_t)(in_frame * MGA_FAKE_VTOTAL / MGA_VLINE_PERIOD_NS);
}

static uint32_t mga_status(MGAState *s)
{
    uint32_t val = s->status;

    if (mga_vcount(s) >= MGA_FAKE_VDISP) {
        val |= MGA_STATUS_VSYNCSTS;
    }
    /* drawing engine always idle: DWGENGSTS stays 0 */
    return val;
}

/*
 * Mode set: decode CRTC + CRTCEXT + XMULCTRL into a linear mode and
 * program it through the VGA core's VBE registers (ati-vga style).
 */
static void mga_update_mode(MGAState *s)
{
    VGACommonState *vga = &s->vga;
    bool ext = (s->crtcext[3] & 0x80) && !(vga->sr[VGA_SEQ_CLOCK_MODE] & 0x20);
    uint32_t width, height, vd, wd, pitch, offs, bpp = 0, bypp;
    bool be;

    if (!ext) {
        if (s->mode_ext) {
            s->mode_ext = false;
            vbe_ioport_write_index(vga, 0, VBE_DISPI_INDEX_ENABLE);
            vbe_ioport_write_data(vga, 0, VBE_DISPI_DISABLED);
        }
        return;
    }

    width = (vga->cr[VGA_CRTC_H_DISP] + 1) * 8;
    vd = vga->cr[VGA_CRTC_V_DISP_END]
        | ((vga->cr[VGA_CRTC_OVERFLOW] & 0x02) << 7)
        | ((vga->cr[VGA_CRTC_OVERFLOW] & 0x40) << 3)
        | ((uint32_t)(s->crtcext[2] & 0x04) << 8);
    height = vd + 1;

    /* pitch: CRTC[19] + CRTCEXT0[5:4], in units of 64 bits */
    wd = vga->cr[VGA_CRTC_OFFSET] | ((uint32_t)(s->crtcext[0] & 0x30) << 4);
    pitch = wd * 8;

    /* start address: CRTC[13,12] + CRTCEXT0[3:0]<<16 + CRTCEXT0[6]->bit20,
     * in units of 32 bits */
    offs = (vga->cr[VGA_CRTC_START_LO]
            | ((uint32_t)vga->cr[VGA_CRTC_START_HI] << 8)
            | ((uint32_t)(s->crtcext[0] & 0x0F) << 16)
            | ((uint32_t)(s->crtcext[0] & 0x40) << 14)) * 4;

    switch (s->xreg[MGA_XMULCTRL] & 7) {
    case 0: /* 8 bpp */
        bpp = 8;
        break;
    case 1: /* 15 bpp + overlay */
        bpp = 15;
        break;
    case 2: /* 16 bpp */
    case 5:
    case 6:
        bpp = 16;
        break;
    case 3: /* packed 24 bpp */
        bpp = 24;
        break;
    case 4: /* 24 + 8 overlay */
    case 7: /* 32 bpp */
        bpp = 32;
        break;
    }
    bypp = DIV_ROUND_UP(bpp, 8);

    if (!width || !height || !bpp || !pitch) {
        return;
    }

    be = bpp > 8 && ((s->opmode >> 16) & 3) != 0;

    if (s->mode_ext && s->mode_width == width && s->mode_height == height &&
        s->mode_bpp == bpp && s->mode_pitch == pitch &&
        s->mode_offs == offs && s->mode_be == be) {
        return;
    }
    s->mode_ext = true;
    s->mode_width = width;
    s->mode_height = height;
    s->mode_bpp = bpp;
    s->mode_pitch = pitch;
    s->mode_offs = offs;
    s->mode_be = be;

    qemu_log_mask(LOG_UNIMP,
                  "mga: mode switch %ux%u bpp %u pitch %u offs 0x%x %s\n",
                  width, height, bpp, pitch, offs, be ? "BE" : "LE");

    vbe_ioport_write_index(vga, 0, VBE_DISPI_INDEX_ENABLE);
    vbe_ioport_write_data(vga, 0, VBE_DISPI_DISABLED);
    vga->big_endian_fb = be;
    vga->vbe_regs[VBE_DISPI_INDEX_XRES] = width;
    vga->vbe_regs[VBE_DISPI_INDEX_YRES] = height;
    vga->vbe_regs[VBE_DISPI_INDEX_BPP] = bpp;
    vbe_ioport_write_index(vga, 0, VBE_DISPI_INDEX_ENABLE);
    vbe_ioport_write_data(vga, 0, VBE_DISPI_ENABLED |
                          VBE_DISPI_LFB_ENABLED | VBE_DISPI_NOCLEARMEM |
                          (s->xreg[MGA_XMISCCTRL] & MGA_XMISCCTRL_DAC_8BIT ?
                           VBE_DISPI_8BIT_DAC : 0));
    vbe_ioport_write_index(vga, 0, VBE_DISPI_INDEX_VIRT_WIDTH);
    vbe_ioport_write_data(vga, 0, pitch / bypp);
    if (offs / 4 < vga->vbe_size / 4) {
        vga->vbe_start_addr = offs / 4;
    }
}

/* GPIO-bitbanged I2C: an XGENIOCTRL bit set to 1 enables the output
 * driver; the line is pulled low when the XGENIODATA latch bit is 0
 * (the usual open-drain usage keeps the latch at 0 and toggles OE). */
static void mga_i2c_update(MGAState *s)
{
    uint8_t ctrl = s->xreg[MGA_XGENIOCTRL];
    uint8_t data = s->xreg[MGA_XGENIODATA];
    int scl_level = ((ctrl & MGA_DDC_CLK) && !(data & MGA_DDC_CLK)) ? 0 : 1;
    int sda_level = ((ctrl & MGA_DDC_DATA) && !(data & MGA_DDC_DATA)) ? 0 : 1;

    bitbang_i2c_set(&s->bbi2c, BITBANG_I2C_SCL, scl_level);
    s->sda_line = bitbang_i2c_set(&s->bbi2c, BITBANG_I2C_SDA, sda_level);
}

static uint8_t mga_i2c_read(MGAState *s)
{
    uint8_t ctrl = s->xreg[MGA_XGENIOCTRL];
    uint8_t data = s->xreg[MGA_XGENIODATA];
    uint8_t val = 0;

    if (!((ctrl & MGA_DDC_CLK) && !(data & MGA_DDC_CLK))) {
        val |= MGA_DDC_CLK;
    }
    if (s->sda_line) {
        val |= MGA_DDC_DATA;
    }
    return val;
}

static uint8_t mga_xreg_read(MGAState *s, uint8_t idx)
{
    switch (idx) {
    case MGA_XGENIODATA:
        return mga_i2c_read(s);
    case MGA_XSYSPLLSTAT:
    case MGA_XPIXPLLSTAT:
        return s->xreg[idx] | 0x40;     /* PLL always locked */
    case MGA_XSENSETEST:
        /* report a connected monitor on all guns */
        return s->xreg[idx] | 0x07;
    default:
        return s->xreg[idx];
    }
}

static void mga_xreg_write(MGAState *s, uint8_t idx, uint8_t val)
{
    s->xreg[idx] = val;

    switch (idx) {
    case MGA_XGENIOCTRL:
    case MGA_XGENIODATA:
        mga_i2c_update(s);
        break;
    case MGA_XMULCTRL:
    case MGA_XMISCCTRL:
        mga_update_mode(s);
        break;
    case MGA_XCURCTRL:
        if (val & 3) {
            qemu_log_mask(LOG_UNIMP, "mga: hardware cursor enabled "
                          "(mode %d) - not implemented\n", val & 3);
        }
        break;
    default:
        break;
    }
}

static uint8_t mga_dac_read(MGAState *s, uint32_t off)
{
    switch (off) {
    case MGA_DAC_PALWRADD:
        return vga_ioport_read(&s->vga, VGA_PEL_IW);
    case MGA_DAC_PALDATA:
        return vga_ioport_read(&s->vga, VGA_PEL_D);
    case MGA_DAC_PIXRDMSK:
        return vga_ioport_read(&s->vga, VGA_PEL_MSK);
    case MGA_DAC_PALRDADD:
        return vga_ioport_read(&s->vga, VGA_PEL_IR);
    case MGA_DAC_X_DATAREG:
        return mga_xreg_read(s, s->xindex);
    case MGA_DAC_CURPOSXL ... MGA_DAC_CURPOSYH:
        return s->xreg[0xF0 + (off & 3)];   /* cursor pos stash */
    default:
        qemu_log_mask(LOG_UNIMP, "mga: unimplemented DAC read 0x%x\n", off);
        return 0;
    }
}

static void mga_dac_write(MGAState *s, uint32_t off, uint8_t val)
{
    switch (off) {
    case MGA_DAC_PALWRADD:
        s->xindex = val;
        vga_ioport_write(&s->vga, VGA_PEL_IW, val);
        break;
    case MGA_DAC_PALDATA:
        vga_ioport_write(&s->vga, VGA_PEL_D, val);
        break;
    case MGA_DAC_PIXRDMSK:
        vga_ioport_write(&s->vga, VGA_PEL_MSK, val);
        break;
    case MGA_DAC_PALRDADD:
        vga_ioport_write(&s->vga, VGA_PEL_IR, val);
        break;
    case MGA_DAC_X_DATAREG:
        mga_xreg_write(s, s->xindex, val);
        break;
    case MGA_DAC_CURPOSXL ... MGA_DAC_CURPOSYH:
        s->xreg[0xF0 + (off & 3)] = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mga: unimplemented DAC write 0x%x = 0x%x\n",
                      off, val);
        break;
    }
}

static uint32_t mga_host_read(MGAState *s, hwaddr addr)
{
    switch (addr) {
    case MGA_FIFOSTATUS:
        return 0x200 | 64;              /* FIFO empty, 64 slots free */
    case MGA_STATUS:
        return mga_status(s);
    case MGA_IEN:
        return s->ien;
    case MGA_VCOUNT:
        return mga_vcount(s);
    case MGA_RESET:
        return s->chip_reset;
    case MGA_OPMODE:
        return s->opmode;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mga: unimplemented host reg read 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void mga_host_write(MGAState *s, hwaddr addr, uint32_t val)
{
    switch (addr) {
    case MGA_ICLEAR:
        s->status &= ~val;
        mga_update_irq(s);
        break;
    case MGA_IEN:
        s->ien = val;
        if (s->ien & MGA_STATUS_VLINEPEN) {
            timer_mod(&s->vline_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      MGA_VLINE_PERIOD_NS);
        } else {
            timer_del(&s->vline_timer);
        }
        mga_update_irq(s);
        break;
    case MGA_RESET:
        s->chip_reset = val & 1;
        break;
    case MGA_OPMODE:
        s->opmode = val;
        mga_update_mode(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mga: unimplemented host reg write 0x%" HWADDR_PRIx
                      " = 0x%x\n", addr, val);
        break;
    }
}

static uint64_t mga_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    MGAState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (addr >= MGA_HOST_BASE && addr < MGA_HOST_END) {
        /* 32-bit registers; handle sub-word access via shifting */
        uint32_t v = mga_host_read(s, addr & ~3);
        val = extract32(v, (addr & 3) * 8, size * 8);
        return val;
    }
    if (addr >= MGA_RAMDAC_BASE && addr < MGA_RAMDAC_END) {
        for (i = 0; i < size; i++) {
            val |= (uint64_t)mga_dac_read(s, (addr - MGA_RAMDAC_BASE) + i)
                   << (i * 8);
        }
        return val;
    }
    if (addr >= MGA_VGA_BASE && addr < MGA_VGA_END) {
        for (i = 0; i < size; i++) {
            hwaddr a = addr + i;
            uint8_t b;

            if (a == MGA_CRTCEXT_INDEX) {
                b = s->crtcext_idx;
            } else if (a == MGA_CRTCEXT_DATA) {
                b = s->crtcext[s->crtcext_idx & 0xF];
            } else {
                b = vga_ioport_read(&s->vga, (a - 0x1C00) & 0x3FF);
            }
            val |= (uint64_t)b << (i * 8);
        }
        return val;
    }
    if (addr >= MGA_DWG_BASE && addr < MGA_DWG_END) {
        for (i = 0; i < size; i++) {
            val |= (uint64_t)s->dwgreg[(addr + i) & 0xFF] << (i * 8);
        }
        return val;
    }
    qemu_log_mask(LOG_UNIMP,
                  "mga: unimplemented ctrl read 0x%" HWADDR_PRIx " size %u\n",
                  addr, size);
    return 0;
}

static void mga_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MGAState *s = opaque;
    unsigned i;

    if (addr >= MGA_HOST_BASE && addr < MGA_HOST_END) {
        uint32_t v = (uint32_t)val;

        if (size < 4) {
            uint32_t cur = 0;
            if ((addr & ~3) == MGA_IEN || (addr & ~3) == MGA_OPMODE) {
                cur = mga_host_read(s, addr & ~3);
            }
            v = deposit32(cur, (addr & 3) * 8, size * 8, (uint32_t)val);
        }
        mga_host_write(s, addr & ~3, v);
        return;
    }
    if (addr >= MGA_RAMDAC_BASE && addr < MGA_RAMDAC_END) {
        for (i = 0; i < size; i++) {
            mga_dac_write(s, (addr - MGA_RAMDAC_BASE) + i,
                          (val >> (i * 8)) & 0xFF);
        }
        return;
    }
    if (addr >= MGA_VGA_BASE && addr < MGA_VGA_END) {
        for (i = 0; i < size; i++) {
            hwaddr a = addr + i;
            uint8_t b = (val >> (i * 8)) & 0xFF;

            if (a == MGA_CRTCEXT_INDEX) {
                s->crtcext_idx = b;
            } else if (a == MGA_CRTCEXT_DATA) {
                s->crtcext[s->crtcext_idx & 0xF] = b;
                if ((s->crtcext_idx & 0xF) <= 3 ||
                    (s->crtcext_idx & 0xF) == 8) {
                    mga_update_mode(s);
                }
            } else {
                vga_ioport_write(&s->vga, (a - 0x1C00) & 0x3FF, b);
                if ((a - 0x1C00) == VGA_CRT_DC ||
                    (a - 0x1C00) == VGA_SEQ_D) {
                    mga_update_mode(s);
                }
            }
        }
        return;
    }
    if (addr >= MGA_DWG_BASE && addr < MGA_DWG_END) {
        for (i = 0; i < size; i++) {
            s->dwgreg[(addr + i) & 0xFF] = (val >> (i * 8)) & 0xFF;
        }
        if (addr & 0x100) {
            qemu_log_mask(LOG_UNIMP,
                          "mga: drawing engine start (reg 0x%" HWADDR_PRIx
                          ") - acceleration not implemented\n", addr & 0xFF);
        }
        return;
    }
    /* 0x0000..0x1BFF: pseudo-DMA window - drawing engine data, ignored */
    if (addr < MGA_DWG_BASE) {
        qemu_log_mask(LOG_UNIMP,
                      "mga: pseudo-DMA write 0x%" HWADDR_PRIx " ignored\n",
                      addr);
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "mga: unimplemented ctrl write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 " size %u\n", addr, val, size);
}

static const MemoryRegionOps mga_ctrl_ops = {
    .read = mga_ctrl_read,
    .write = mga_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t mga_iload_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void mga_iload_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mga: ILOAD write ignored\n");
}

static const MemoryRegionOps mga_iload_ops = {
    .read = mga_iload_read,
    .write = mga_iload_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* PCI config space: OPTION regs and the MGA_INDEX/MGA_DATA window */
static uint32_t mga_pci_config_read(PCIDevice *d, uint32_t addr, int len)
{
    MGAState *s = MGA(d);

    if (ranges_overlap(addr, len, MGA_PCI_MGA_DATA, 4)) {
        uint32_t v = mga_ctrl_read(s, s->mga_index & 0x3FFC, 4);

        if (addr == MGA_PCI_MGA_DATA && len == 4) {
            return v;
        }
        if (addr >= MGA_PCI_MGA_DATA) {
            return extract32(v, (addr - MGA_PCI_MGA_DATA) * 8,
                             MIN(len, 4 - (int)(addr - MGA_PCI_MGA_DATA)) * 8);
        }
        /* unaligned overlap: fall back to default for simplicity */
    }
    return pci_default_read_config(d, addr, len);
}

static void mga_pci_config_write(PCIDevice *d, uint32_t addr, uint32_t val,
                                 int len)
{
    MGAState *s = MGA(d);

    if (addr == MGA_PCI_MGA_DATA && len == 4) {
        mga_ctrl_write(s, s->mga_index & 0x3FFC, val, 4);
        return;
    }
    if (ranges_overlap(addr, len, MGA_PCI_MGA_DATA, 4) &&
        addr >= MGA_PCI_MGA_DATA) {
        uint32_t cur = mga_ctrl_read(s, s->mga_index & 0x3FFC, 4);

        cur = deposit32(cur, (addr - MGA_PCI_MGA_DATA) * 8,
                        MIN(len, 4 - (int)(addr - MGA_PCI_MGA_DATA)) * 8, val);
        mga_ctrl_write(s, s->mga_index & 0x3FFC, cur, 4);
        return;
    }
    pci_default_write_config(d, addr, val, len);
    if (ranges_overlap(addr, len, MGA_PCI_MGA_INDEX, 4)) {
        s->mga_index = pci_get_long(d->config + MGA_PCI_MGA_INDEX);
    }
}

static void mga_reset(DeviceState *d)
{
    MGAState *s = MGA(d);

    vga_common_reset(&s->vga);
    memset(s->crtcext, 0, sizeof(s->crtcext));
    memset(s->xreg, 0, sizeof(s->xreg));
    memset(s->dwgreg, 0, sizeof(s->dwgreg));
    s->crtcext_idx = 0;
    s->xindex = 0;
    s->status = 0;
    s->ien = 0;
    s->opmode = 0;
    s->chip_reset = 0;
    s->mga_index = 0;
    s->sda_line = 1;
    s->mode_ext = false;
    s->mode_width = s->mode_height = s->mode_bpp = 0;
    s->mode_pitch = s->mode_offs = 0;
    s->mode_be = false;
    timer_del(&s->vline_timer);
}

static const VMStateDescription vmstate_mga = {
    .name = TYPE_MGA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, MGAState),
        VMSTATE_STRUCT(vga, MGAState, 0, vmstate_vga_common, VGACommonState),
        VMSTATE_UINT8(crtcext_idx, MGAState),
        VMSTATE_UINT8_ARRAY(crtcext, MGAState, 16),
        VMSTATE_UINT8(xindex, MGAState),
        VMSTATE_UINT8_ARRAY(xreg, MGAState, 256),
        VMSTATE_UINT8_ARRAY(dwgreg, MGAState, 256),
        VMSTATE_UINT32(status, MGAState),
        VMSTATE_UINT32(ien, MGAState),
        VMSTATE_UINT32(opmode, MGAState),
        VMSTATE_UINT32(chip_reset, MGAState),
        VMSTATE_UINT32(mga_index, MGAState),
        VMSTATE_UINT8(sda_line, MGAState),
        VMSTATE_UINT32(mode_width, MGAState),
        VMSTATE_UINT32(mode_height, MGAState),
        VMSTATE_UINT32(mode_bpp, MGAState),
        VMSTATE_UINT32(mode_pitch, MGAState),
        VMSTATE_UINT32(mode_offs, MGAState),
        VMSTATE_BOOL(mode_ext, MGAState),
        VMSTATE_BOOL(mode_be, MGAState),
        VMSTATE_END_OF_LIST()
    },
};

static void mga_realize(PCIDevice *dev, Error **errp)
{
    MGAState *s = MGA(dev);
    Object *o = OBJECT(dev);
    I2CBus *i2cbus;

    /* setup VGA core */
    if (!vga_common_init(&s->vga, o, errp)) {
        return;
    }
    s->vga.con = graphic_console_init(DEVICE(s), 0, s->vga.hw_ops, &s->vga);

    memory_region_init_io(&s->ctrl, o, &mga_ctrl_ops, s,
                          "mga-ctrl", MGA_CTRL_SIZE);
    memory_region_init_io(&s->iload, o, &mga_iload_ops, s,
                          "mga-iload", MGA_ILOAD_SIZE);

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_MEM_PREFETCH, &s->vga.vram);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->ctrl);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->iload);

    /* Matrox OPTION registers: plain read/write scratch */
    pci_set_long(dev->wmask + MGA_PCI_OPTION, 0xFFFFFFFF);
    pci_set_long(dev->wmask + MGA_PCI_MGA_INDEX, 0xFFFFFFFF);
    pci_set_long(dev->wmask + MGA_PCI_OPTION2, 0xFFFFFFFF);
    pci_set_long(dev->wmask + MGA_PCI_OPTION3, 0xFFFFFFFF);

    dev->config[PCI_INTERRUPT_PIN] = 1;

    /* DDC monitor with EDID */
    i2cbus = i2c_init_bus(DEVICE(s), "mga.ddc");
    bitbang_i2c_init(&s->bbi2c, i2cbus);
    i2c_slave_set_address(I2C_SLAVE(&s->ddc), 0x50);
    qdev_realize(DEVICE(&s->ddc), BUS(i2cbus), &error_abort);
    s->sda_line = 1;

    timer_init_ns(&s->vline_timer, QEMU_CLOCK_VIRTUAL,
                  mga_vline_timer_cb, s);
}

static void mga_exit(PCIDevice *dev)
{
    MGAState *s = MGA(dev);

    timer_del(&s->vline_timer);
    graphic_console_close(s->vga.con);
}

static void mga_init(Object *obj)
{
    MGAState *s = MGA(obj);

    object_initialize_child(obj, "ddc", &s->ddc, TYPE_I2CDDC);
}

static const Property mga_properties[] = {
    DEFINE_PROP_UINT32("vram_size_mb", MGAState, vga.vram_size_mb, 8),
    DEFINE_EDID_PROPERTIES(MGAState, ddc.edid_info),
};

static void mga_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = mga_realize;
    k->exit = mga_exit;
    k->config_read = mga_pci_config_read;
    k->config_write = mga_pci_config_write;
    k->vendor_id = PCI_VENDOR_ID_MATROX;
    k->device_id = PCI_DEVICE_ID_MATROX_G200;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    k->subsystem_vendor_id = PCI_VENDOR_ID_MATROX;
    k->subsystem_id = 0xFF03;   /* Millennium G200 */
    device_class_set_legacy_reset(dc, mga_reset);
    dc->desc = "Matrox MGA G200 (GXT130P)";
    dc->vmsd = &vmstate_mga;
    device_class_set_props(dc, mga_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo mga_info = {
    .name          = TYPE_MGA,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MGAState),
    .instance_init = mga_init,
    .class_init    = mga_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void mga_register_types(void)
{
    type_register_static(&mga_info);
}

type_init(mga_register_types)
