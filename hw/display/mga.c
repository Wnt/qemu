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
#include "trace.h"

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
#define MGA_MEMRDBK         0x1E44
#define MGA_OPMODE          0x1E54
#define MGA_PRIMADDRESS     0x1E58
#define MGA_PRIMEND         0x1E5C

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
    uint32_t memrdbk;
    uint32_t mga_index;     /* PCI config 0x44 */

    bitbang_i2c_interface bbi2c;
    I2CDDCState ddc;
    uint8_t sda_line;

    QEMUTimer vline_timer;

    /* cached VBE-programmed mode, to avoid re-programming */
    uint32_t mode_width, mode_height, mode_bpp;
    uint32_t mode_pitch, mode_offs;
    bool mode_ext, mode_be;

    uint32_t primaddr, primend;

    /* ILOAD (host->screen) transfer in progress */
    bool il_active;
    uint8_t il_fmt;         /* MGA_IL_* */
    bool il_linear, il_transc;
    uint8_t il_bop;
    uint32_t il_fcol, il_bcol, il_plnwt;
    uint32_t il_pitch, il_ydstorg, il_bypp;
    uint32_t il_x0, il_x1, il_x, il_y;
    uint32_t il_rows_left;
    uint32_t il_ytop, il_ybot, il_cxl, il_cxr;
};

#define MGA_IL_MONO_LSB 0       /* BMONOLEF */
#define MGA_IL_MONO_MSB 1       /* BMONOWF */
#define MGA_IL_PIXELS   2       /* BFCOL */

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
    /* make sure the attribute controller "display enable" flag is set */
    vga->ar_index |= 0x20;
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

/* drawing engine register offsets (within dwgreg[]) */
#define DWG_DWGCTL      0x00
#define DWG_MACCESS     0x04
#define DWG_PLNWT       0x1C
#define DWG_BCOL        0x20
#define DWG_FCOL        0x24
#define DWG_SRC0        0x30
#define DWG_XYSTRT      0x40
#define DWG_XYEND       0x44
#define DWG_SHIFT       0x50
#define DWG_SGN         0x58
#define DWG_LEN         0x5C
#define DWG_AR0         0x60
#define DWG_AR3         0x6C
#define DWG_AR5         0x74
#define DWG_CXBNDRY     0x80
#define DWG_FXBNDRY     0x84
#define DWG_YDSTLEN     0x88
#define DWG_PITCH       0x8C
#define DWG_YDST        0x90
#define DWG_YDSTORG     0x94
#define DWG_YTOP        0x98
#define DWG_YBOT        0x9C
#define DWG_CXLEFT      0xA0
#define DWG_CXRIGHT     0xA4
#define DWG_FXLEFT      0xA8
#define DWG_FXRIGHT     0xAC

/* DWGCTL fields */
#define DWG_OPCOD(c)    ((c) & 0xF)
#define DWG_BOP(c)      (((c) >> 16) & 0xF)
#define DWG_BLTMOD(c)   (((c) >> 25) & 0xF)
#define DWGCTL_LINEAR   (1u << 7)
#define DWGCTL_SOLID    (1u << 11)
#define DWGCTL_TRANSC   (1u << 30)

#define OPCOD_LINE_OPEN      0x0
#define OPCOD_AUTOLINE_OPEN  0x1
#define OPCOD_LINE_CLOSE     0x2
#define OPCOD_AUTOLINE_CLOSE 0x3
#define OPCOD_TRAP           0x4
#define OPCOD_BITBLT         0x8
#define OPCOD_ILOAD          0x9

#define BLTMOD_BMONOLEF 0x0
#define BLTMOD_BFCOL    0x2
#define BLTMOD_BMONOWF  0x4

static inline uint32_t dwg32(MGAState *s, unsigned off)
{
    return ldl_le_p(&s->dwgreg[off]);
}

/* MGA boolean raster op: 4-entry truth table indexed by (src<<1)|dst */
static inline uint32_t mga_rop(uint8_t bop, uint32_t d, uint32_t src)
{
    switch (bop) {
    case 0xC:                   /* copy (by far the common case) */
        return src;
    case 0x6:                   /* xor */
        return d ^ src;
    default: {
        uint32_t r = 0;
        unsigned i;

        for (i = 0; i < 32; i++) {
            unsigned idx = (((src >> i) & 1) << 1) | ((d >> i) & 1);

            r |= ((uint32_t)(bop >> idx) & 1) << i;
        }
        return r;
    }
    }
}

/* clip + rop + plane-write-mask context shared by the primitives */
typedef struct MGADrawCtx {
    MGAState *s;
    uint32_t pitch, ydstorg, bypp;
    uint32_t plnwt;
    uint8_t bop;
    uint32_t ytop, ybot;        /* linear pixel addresses; ybot=0: no clip */
    uint32_t cxl, cxr;          /* x clip; cxr=0: no clip */
} MGADrawCtx;

static void mga_ctx_init(MGADrawCtx *c, MGAState *s, uint32_t dwgctl)
{
    uint32_t maccess = dwg32(s, DWG_MACCESS);
    uint32_t cxb = dwg32(s, DWG_CXBNDRY);

    c->s = s;
    c->pitch = dwg32(s, DWG_PITCH) & 0x1FFF;
    c->ydstorg = dwg32(s, DWG_YDSTORG);
    c->bypp = ((maccess & 3) == 3) ? 3 : (1 << (maccess & 3));
    c->plnwt = dwg32(s, DWG_PLNWT);
    c->bop = DWG_BOP(dwgctl);
    c->ytop = dwg32(s, DWG_YTOP) & 0xFFFFFF;
    c->ybot = dwg32(s, DWG_YBOT) & 0xFFFFFF;
    (void)cxb;
    c->cxl = dwg32(s, DWG_CXLEFT) & 0xFFF;
    c->cxr = dwg32(s, DWG_CXRIGHT) & 0xFFF;
}

static uint32_t mga_rdpix(const MGADrawCtx *c, int64_t pixaddr)
{
    VGACommonState *vga = &c->s->vga;
    uint64_t off;
    uint32_t v = 0;
    unsigned b;

    if (pixaddr < 0) {
        return 0;
    }
    off = (uint64_t)pixaddr * c->bypp;
    if (off + c->bypp > vga->vram_size) {
        return 0;
    }
    for (b = 0; b < c->bypp; b++) {
        v |= (uint32_t)vga->vram_ptr[off + b] << (b * 8);
    }
    return v;
}

static void mga_plot(const MGADrawCtx *c, int32_t x, int32_t y, uint32_t src)
{
    VGACommonState *vga = &c->s->vga;
    uint64_t a, off;
    uint32_t old = 0, val;
    unsigned b;

    if (x < 0 || y < 0) {
        return;
    }
    if (c->cxr && ((uint32_t)x < c->cxl || (uint32_t)x > c->cxr)) {
        return;
    }
    a = (uint64_t)y * c->pitch + c->ydstorg + x;
    if (c->ybot && (a < c->ytop || a > c->ybot)) {
        return;
    }
    off = a * c->bypp;
    if (off + c->bypp > vga->vram_size) {
        return;
    }
    for (b = 0; b < c->bypp; b++) {
        old |= (uint32_t)vga->vram_ptr[off + b] << (b * 8);
    }
    val = mga_rop(c->bop, old, src);
    val = (old & ~c->plnwt) | (val & c->plnwt);
    for (b = 0; b < c->bypp; b++) {
        vga->vram_ptr[off + b] = (val >> (b * 8)) & 0xFF;
    }
    memory_region_set_dirty(&vga->vram, off, c->bypp);
}

/* TRAP: rectangle/trapezoid fill (only the rectangular case: arzero) */
static void mga_dwg_trap(MGAState *s, uint32_t dwgctl)
{
    MGADrawCtx c;
    VGACommonState *vga = &s->vga;
    uint32_t x1 = dwg32(s, DWG_FXLEFT) & 0xFFFF;
    uint32_t x2 = dwg32(s, DWG_FXRIGHT) & 0xFFFF;
    uint32_t ydst = dwg32(s, DWG_YDST);
    uint32_t len = dwg32(s, DWG_LEN) & 0xFFFF;
    uint32_t fcol = dwg32(s, DWG_FCOL);
    uint32_t y, x;

    mga_ctx_init(&c, s, dwgctl);
    if (!c.pitch || x2 <= x1 || !len) {
        return;
    }
    if (!(dwgctl & DWGCTL_SOLID)) {
        qemu_log_mask(LOG_UNIMP, "mga: TRAP pattern fill drawn solid "
                      "(dwgctl=0x%x)\n", dwgctl);
    }
    for (y = 0; y < len; y++) {
        uint64_t pix = ((uint64_t)(ydst + y)) * c.pitch + c.ydstorg;
        uint64_t off = (pix + x1) * c.bypp;
        uint64_t end = (pix + x2) * c.bypp;

        if (end > vga->vram_size) {
            break;
        }
        if (c.bop == 0xC && c.plnwt == 0xFFFFFFFF && c.bypp == 1 &&
            !c.cxr && !c.ybot) {
            memset(vga->vram_ptr + off, fcol & 0xFF, end - off);
            memory_region_set_dirty(&vga->vram, off, end - off);
        } else {
            for (x = x1; x < x2; x++) {
                mga_plot(&c, x, ydst + y, fcol);
            }
        }
    }
}

/* BITBLT: screen-to-screen copy */
static void mga_dwg_bitblt(MGAState *s, uint32_t dwgctl)
{
    MGADrawCtx c;
    int32_t x1 = (int16_t)dwg32(s, DWG_FXLEFT);
    int32_t x2 = (int16_t)dwg32(s, DWG_FXRIGHT);
    uint32_t len = dwg32(s, DWG_LEN) & 0xFFFF;
    int32_t ydst = (int32_t)(dwg32(s, DWG_YDST) & 0x7FFFFF);
    uint32_t ar3 = dwg32(s, DWG_AR3) & 0xFFFFFF;
    int32_t ar5 = sextract32(dwg32(s, DWG_AR5), 0, 18);
    uint32_t sgn = dwg32(s, DWG_SGN);
    bool scanleft = sgn & 1;
    int ydir = (sgn & 4) ? -1 : 1;
    int32_t w = x2 - x1 + 1;
    uint32_t bltmod = DWG_BLTMOD(dwgctl);
    g_autofree uint32_t *line = NULL;
    uint32_t r, k;

    if (bltmod != BLTMOD_BFCOL) {
        qemu_log_mask(LOG_UNIMP, "mga: BITBLT bltmod 0x%x not implemented "
                      "(dwgctl=0x%x)\n", bltmod, dwgctl);
        return;
    }
    mga_ctx_init(&c, s, dwgctl);
    if (w <= 0 || !len || !c.pitch) {
        return;
    }
    line = g_new(uint32_t, w);
    for (r = 0; r < len; r++) {
        int64_t sline = (int64_t)ar3 + (int64_t)r * ar5;
        int32_t dy = ydst + ydir * (int32_t)r;

        /* read the whole source line first: overlap-safe */
        for (k = 0; k < (uint32_t)w; k++) {
            int64_t sa = sline + (scanleft ? -(int64_t)k : (int64_t)k);

            line[k] = mga_rdpix(&c, sa);
        }
        for (k = 0; k < (uint32_t)w; k++) {
            int32_t dx = scanleft ? x2 - (int32_t)k : x1 + (int32_t)k;

            mga_plot(&c, dx, dy, line[k]);
        }
    }
}

/* LINE/AUTOLINE: Bresenham from XYSTRT to XYEND */
static void mga_dwg_line(MGAState *s, uint32_t dwgctl)
{
    MGADrawCtx c;
    uint32_t xystrt = dwg32(s, DWG_XYSTRT);
    uint32_t xyend = dwg32(s, DWG_XYEND);
    int32_t x0 = (int16_t)xystrt, y0 = (int16_t)(xystrt >> 16);
    int32_t x1 = (int16_t)xyend, y1 = (int16_t)(xyend >> 16);
    uint32_t fcol = dwg32(s, DWG_FCOL);
    uint32_t bcol = dwg32(s, DWG_BCOL);
    bool close = DWG_OPCOD(dwgctl) & 2;
    bool solid = dwgctl & DWGCTL_SOLID;
    bool transc = dwgctl & DWGCTL_TRANSC;
    int32_t dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int32_t sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy;
    unsigned patbit = 0;
    int guard = 1 << 14;

    mga_ctx_init(&c, s, dwgctl);
    if (!c.pitch) {
        return;
    }
    while (guard--) {
        bool bit = true;

        if (!solid) {
            unsigned i = patbit++ & 127;

            bit = (s->dwgreg[DWG_SRC0 + (i >> 3)] >> (i & 7)) & 1;
        }
        if (x0 == x1 && y0 == y1 && !close) {
            break;
        }
        if (bit) {
            mga_plot(&c, x0, y0, fcol);
        } else if (!transc) {
            mga_plot(&c, x0, y0, bcol);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        if (2 * err >= dy) {
            err += dy;
            x0 += sx;
        }
        if (2 * err <= dx) {
            err += dx;
            y0 += sy;
        }
    }
    if (DWG_OPCOD(dwgctl) & 1) {        /* AUTOLINE: chain */
        stl_le_p(&s->dwgreg[DWG_XYSTRT], xyend);
    }
}

/* ILOAD: host->screen image transfer; GO arms it, data arrives through
 * the ILOAD aperture (BAR2) or the pseudo-DMA window */
static void mga_dwg_iload_start(MGAState *s, uint32_t dwgctl)
{
    MGADrawCtx c;
    int32_t x1 = (int16_t)dwg32(s, DWG_FXLEFT);
    int32_t x2 = (int16_t)dwg32(s, DWG_FXRIGHT);
    uint32_t len = dwg32(s, DWG_LEN) & 0xFFFF;
    uint32_t ydst = dwg32(s, DWG_YDST) & 0x7FFFFF;
    uint32_t bltmod = DWG_BLTMOD(dwgctl);
    int32_t w = x2 - x1 + 1;

    if (w <= 0 || !len) {
        return;
    }
    switch (bltmod) {
    case BLTMOD_BMONOLEF:
        s->il_fmt = MGA_IL_MONO_LSB;
        break;
    case BLTMOD_BMONOWF:
        s->il_fmt = MGA_IL_MONO_MSB;
        break;
    case BLTMOD_BFCOL:
        s->il_fmt = MGA_IL_PIXELS;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mga: ILOAD bltmod 0x%x not implemented "
                      "(dwgctl=0x%x)\n", bltmod, dwgctl);
        return;
    }
    mga_ctx_init(&c, s, dwgctl);
    if (!c.pitch) {
        return;
    }
    if (s->il_fmt == MGA_IL_PIXELS && c.bypp != 1) {
        qemu_log_mask(LOG_UNIMP, "mga: ILOAD BFCOL bypp %u not implemented\n",
                      c.bypp);
        return;
    }
    s->il_active = true;
    s->il_linear = dwgctl & DWGCTL_LINEAR;
    s->il_transc = dwgctl & DWGCTL_TRANSC;
    s->il_bop = c.bop;
    s->il_fcol = dwg32(s, DWG_FCOL);
    s->il_bcol = dwg32(s, DWG_BCOL);
    s->il_plnwt = c.plnwt;
    s->il_pitch = c.pitch;
    s->il_ydstorg = c.ydstorg;
    s->il_bypp = c.bypp;
    s->il_x0 = x1;
    s->il_x1 = x2;
    s->il_x = x1;
    s->il_y = ydst;
    s->il_rows_left = len;
    s->il_ytop = c.ytop;
    s->il_ybot = c.ybot;
    s->il_cxl = c.cxl;
    s->il_cxr = c.cxr;
}

static void mga_iload_ctx(MGAState *s, MGADrawCtx *c)
{
    c->s = s;
    c->pitch = s->il_pitch;
    c->ydstorg = s->il_ydstorg;
    c->bypp = s->il_bypp;
    c->plnwt = s->il_plnwt;
    c->bop = s->il_bop;
    c->ytop = s->il_ytop;
    c->ybot = s->il_ybot;
    c->cxl = s->il_cxl;
    c->cxr = s->il_cxr;
}

/* advance destination cursor; returns false when the transfer completed */
static bool mga_iload_advance(MGAState *s)
{
    if (s->il_x < s->il_x1) {
        s->il_x++;
        return true;
    }
    s->il_x = s->il_x0;
    s->il_y++;
    if (--s->il_rows_left == 0) {
        s->il_active = false;
        return false;
    }
    return true;
}

/*
 * Feed data to an armed ILOAD.  Bytes are taken in guest order (the
 * aperture is big-endian: the MSB of val is the lowest-addressed byte).
 * Mono WF consumes each byte MSB first, LEF LSB first.  A non-linear
 * transfer discards the remainder of the (dword-sized) access at the
 * end of each scan line.
 */
static void mga_iload_data(MGAState *s, uint64_t val, unsigned size)
{
    MGADrawCtx c;
    unsigned i, b;

    if (!s->il_active) {
        return;
    }
    mga_iload_ctx(s, &c);
    for (i = 0; i < size && s->il_active; i++) {
        uint8_t byte = (val >> (8 * (size - 1 - i))) & 0xFF;

        if (s->il_fmt == MGA_IL_PIXELS) {
            mga_plot(&c, s->il_x, s->il_y, byte);
            if (!mga_iload_advance(s)) {
                return;
            }
            continue;
        }
        for (b = 0; b < 8; b++) {
            bool bit;
            bool row_end;

            if (s->il_fmt == MGA_IL_MONO_MSB) {
                bit = (byte >> (7 - b)) & 1;
            } else {
                bit = (byte >> b) & 1;
            }
            if (bit) {
                mga_plot(&c, s->il_x, s->il_y, s->il_fcol);
            } else if (!s->il_transc) {
                mga_plot(&c, s->il_x, s->il_y, s->il_bcol);
            }
            row_end = s->il_x == s->il_x1;
            if (!mga_iload_advance(s)) {
                return;
            }
            if (row_end && !s->il_linear) {
                /* non-linear: skip to the next dword for the next line */
                return;
            }
        }
    }
}

/*
 * Execute a drawing engine operation (started by a write to the GO alias
 * at 0x1Dxx).
 */
static void mga_dwg_execute(MGAState *s)
{
    uint32_t dwgctl = dwg32(s, DWG_DWGCTL);
    uint32_t opcod = DWG_OPCOD(dwgctl);

    trace_mga_dwg_go(dwgctl, dwg32(s, DWG_XYSTRT), dwg32(s, DWG_XYEND),
                     dwg32(s, DWG_FXLEFT), dwg32(s, DWG_FXRIGHT),
                     dwg32(s, DWG_YDST), dwg32(s, DWG_LEN),
                     dwg32(s, DWG_SGN), dwg32(s, DWG_FCOL));
    trace_mga_dwg_go2(dwg32(s, DWG_AR0), dwg32(s, DWG_AR3),
                      dwg32(s, DWG_AR5), dwg32(s, DWG_CXLEFT),
                      dwg32(s, DWG_CXRIGHT), dwg32(s, DWG_YTOP),
                      dwg32(s, DWG_YBOT), dwg32(s, DWG_YDSTORG));

    /* a new operation cancels any half-fed ILOAD */
    s->il_active = false;

    switch (opcod) {
    case OPCOD_LINE_OPEN:
        if (!dwgctl) {
            /* GO with a zeroed DWGCTL: idle op, used as a fence */
            return;
        }
        /* fall through */
    case OPCOD_AUTOLINE_OPEN:
    case OPCOD_LINE_CLOSE:
    case OPCOD_AUTOLINE_CLOSE:
        mga_dwg_line(s, dwgctl);
        return;
    case OPCOD_TRAP:
        mga_dwg_trap(s, dwgctl);
        return;
    case OPCOD_BITBLT:
        mga_dwg_bitblt(s, dwgctl);
        return;
    case OPCOD_ILOAD:
        mga_dwg_iload_start(s, dwgctl);
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "mga: drawing engine opcod 0x%x not implemented "
                      "(dwgctl=0x%x)\n", opcod, dwgctl);
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
    case MGA_MEMRDBK:
        return s->memrdbk;
    case MGA_OPMODE:
        return s->opmode;
    case MGA_PRIMADDRESS:
        return s->primaddr;
    case MGA_PRIMEND:
        return s->primend;
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
    case MGA_MEMRDBK:
        s->memrdbk = val;
        break;
    case MGA_OPMODE:
        s->opmode = val;
        mga_update_mode(s);
        break;
    case MGA_PRIMADDRESS:
        s->primaddr = val;
        break;
    case MGA_PRIMEND:
        s->primend = val;
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
        if (((addr & 0xFC) == DWG_YDSTLEN) && size == 4) {
            /* YDSTLEN also loads YDST (hi16) and LEN (lo16) */
            stl_le_p(&s->dwgreg[DWG_YDST], (uint32_t)(val >> 16));
            stl_le_p(&s->dwgreg[DWG_LEN], (uint32_t)(val & 0xFFFF));
        }
        if (((addr & 0xFC) == DWG_CXBNDRY) && size == 4) {
            stl_le_p(&s->dwgreg[DWG_CXLEFT], (uint32_t)(val & 0xFFFF));
            stl_le_p(&s->dwgreg[DWG_CXRIGHT], (uint32_t)(val >> 16));
        }
        if (((addr & 0xFC) == DWG_FXBNDRY) && size == 4) {
            stl_le_p(&s->dwgreg[DWG_FXLEFT], (uint32_t)(val & 0xFFFF));
            stl_le_p(&s->dwgreg[DWG_FXRIGHT], (uint32_t)(val >> 16));
        }
        if (addr & 0x100) {
            mga_dwg_execute(s);
        }
        return;
    }
    /* 0x0000..0x1BFF: pseudo-DMA window - drawing engine data */
    if (addr < MGA_DWG_BASE) {
        mga_iload_data(s, val, size);
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "mga: unimplemented ctrl write 0x%" HWADDR_PRIx
                  " = 0x%" PRIx64 " size %u\n", addr, val, size);
}

static const MemoryRegionOps mga_ctrl_ops = {
    .read = mga_ctrl_read,
    .write = mga_ctrl_write,
    /*
     * The GXT130P presents the G200 register apertures byte-swapped to
     * the big-endian host: the AIX DDX reads FIFOSTATUS etc with plain
     * lwz and masks the fifocount bits directly (it spins forever on a
     * little-endian view).
     */
    .endianness = DEVICE_BIG_ENDIAN,
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
    mga_iload_data(opaque, val, size);
}

static const MemoryRegionOps mga_iload_ops = {
    .read = mga_iload_read,
    .write = mga_iload_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
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
    s->memrdbk = 0;
    s->mga_index = 0;
    s->sda_line = 1;
    s->mode_ext = false;
    s->mode_width = s->mode_height = s->mode_bpp = 0;
    s->mode_pitch = s->mode_offs = 0;
    s->mode_be = false;
    s->primaddr = 0;
    s->primend = 0;
    s->il_active = false;
    timer_del(&s->vline_timer);
}

static const VMStateDescription vmstate_mga = {
    .name = TYPE_MGA,
    .version_id = 2,
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
        VMSTATE_UINT32(memrdbk, MGAState),
        VMSTATE_UINT32(mga_index, MGAState),
        VMSTATE_UINT8(sda_line, MGAState),
        VMSTATE_UINT32(mode_width, MGAState),
        VMSTATE_UINT32(mode_height, MGAState),
        VMSTATE_UINT32(mode_bpp, MGAState),
        VMSTATE_UINT32(mode_pitch, MGAState),
        VMSTATE_UINT32(mode_offs, MGAState),
        VMSTATE_BOOL(mode_ext, MGAState),
        VMSTATE_BOOL(mode_be, MGAState),
        VMSTATE_UINT32_V(primaddr, MGAState, 2),
        VMSTATE_UINT32_V(primend, MGAState, 2),
        VMSTATE_BOOL_V(il_active, MGAState, 2),
        VMSTATE_UINT8_V(il_fmt, MGAState, 2),
        VMSTATE_BOOL_V(il_linear, MGAState, 2),
        VMSTATE_BOOL_V(il_transc, MGAState, 2),
        VMSTATE_UINT8_V(il_bop, MGAState, 2),
        VMSTATE_UINT32_V(il_fcol, MGAState, 2),
        VMSTATE_UINT32_V(il_bcol, MGAState, 2),
        VMSTATE_UINT32_V(il_plnwt, MGAState, 2),
        VMSTATE_UINT32_V(il_pitch, MGAState, 2),
        VMSTATE_UINT32_V(il_ydstorg, MGAState, 2),
        VMSTATE_UINT32_V(il_bypp, MGAState, 2),
        VMSTATE_UINT32_V(il_x0, MGAState, 2),
        VMSTATE_UINT32_V(il_x1, MGAState, 2),
        VMSTATE_UINT32_V(il_x, MGAState, 2),
        VMSTATE_UINT32_V(il_y, MGAState, 2),
        VMSTATE_UINT32_V(il_rows_left, MGAState, 2),
        VMSTATE_UINT32_V(il_ytop, MGAState, 2),
        VMSTATE_UINT32_V(il_ybot, MGAState, 2),
        VMSTATE_UINT32_V(il_cxl, MGAState, 2),
        VMSTATE_UINT32_V(il_cxr, MGAState, 2),
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
