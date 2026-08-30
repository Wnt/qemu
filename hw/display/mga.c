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
#include "ui/pixel_ops.h"
#include "ui/input.h"
#include "chardev/char-fe.h"
#include "hw/core/qdev-properties-system.h"
#include "system/runstate.h"
#include <math.h>

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
#define MGA_XCURADDL        0x04
#define MGA_XCURADDH        0x05
#define MGA_XCURCTRL        0x06
#define MGA_XCURCOL0RED     0x08    /* ..0x0A: R,G,B */
#define MGA_XCURCOL1RED     0x0C    /* ..0x0E: R,G,B */
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


/* ---- closed-loop 1:1 pointer: see the engine further down ------------ */
#define PTR_RXMAX            512
#define PTR_QMAX             64
#define PTR_SPRITE_MAX       63     /* a hotspot lives inside the 64x64 sprite */
#define PTR_OSC_FLIPS        3      /* sign reversals within ONE target       */
#define PTR_INFL_DECAY       0.5    /* per window                             */
#define PTR_GAIN_MARGIN      1.10   /* undershoot margin on every step        */
#define PTR_GAIN_MIN_COUNTS  6
#define PTR_GAIN_LO          0.25
#define PTR_GAIN_HI          4.0
#define PTR_GAIN_ALPHA_UP    0.35
#define PTR_GAIN_ALPHA_DOWN  0.10
#define PTR_GAIN_EDGE_MARGIN 2
/* Above this the correction is not a limit cycle, it is a control failure:
 * let the give-up cap own it rather than accepting the pointer that far off. */
#define PTR_OSC_MAX_ERR      32
#define PTR_HOME_MAX_WINDOWS 96
#define PTR_HOME_STILL       3
#define PTR_HOT_SLOTS        24
#define PTR_PIN_WINDOWS      3
/* How long a step may stay "on the wire" before it is declared landed. */
#define PTR_AWAIT_MAX        6

typedef struct MGAPtrVerb {
    char kind;              /* 't' target, 'b' button edge, 's' sync fence */
    int x, y;               /* 't' */
    int btn;                /* 'b': 0..2 = left, right, middle             */
    bool down;              /* 'b'                                         */
    uint64_t seq;           /* client seq; 0 = internal, no ack            */
} MGAPtrVerb;

typedef struct MGAPtrLoop {
    CharFrontend chr;       /* the control socket; one client at a time    */
    bool enabled;           /* a control chardev was configured            */
    bool open;              /* ... and a client is connected               */
    bool trace;
    bool trace_pos;
    bool track_hotspot;
    uint32_t window_ms, dead, move_step, tries, btn_gap_ms;
    uint32_t gain_x100;     /* seed for gx/gy, in hundredths of a px/count */

    QEMUTimer timer;
    VMChangeStateEntry *vmse;

    char rx[PTR_RXMAX];
    uint32_t rxlen;

    /* the converging target */
    bool ta_active;
    int ta_x, ta_y;
    uint64_t ta_seq;
    uint32_t windows;

    /* control state -- all of it derived, none of it migrated */
    double infl_x, infl_y;  /* px issued but not yet observed              */
    double gx, gy;          /* px per count; a STEP SIZER, not a model     */
    int obs_x, obs_y;       /* last reading                                */
    int bel_x, bel_y;       /* belief, used only while the sprite is off   */
    bool obs_valid;
    int last_cx, last_cy;
    /* A step is on the wire and the registers have not moved yet. Nothing
     * new may be issued until they do -- see mga_ptr_step. */
    bool awaiting;
    uint32_t await_windows;
    int osc_sx, osc_sy;
    uint32_t osc_nx, osc_ny;

    /* glyph hotspot, measured from the compensating register write */
    int hot_x, hot_y;
    bool hot_dirty;
    uint64_t cur_sig;       /* signature of the sprite bytes plus its address */
    bool sig_valid;
    bool hot_exact;         /* the current glyph's hotspot was MEASURED       */
    /* Opportunistic clamp calibration: windows spent pushing an axis whose
     * registers refuse to move, and the reading they refused at. */
    uint32_t pinned_x, pinned_y;
    int last_cx_dir, last_cy_dir;
    int pin_rx, pin_ry;
    /* Glyph signature -> its measured hotspot. A glyph has to be caught at
     * rest exactly once, ever; after that every swap to it is exact. */
    struct {
        uint64_t sig;
        int hx, hy;
        bool valid;
    } hot_tab[PTR_HOT_SLOTS];
    uint32_t hot_next;
    bool latch_valid;
    int latch_x, latch_y;

    /* One-time clamp calibration (see mga_ptr_home). */
    bool homing;
    uint32_t home_windows;
    uint32_t home_still;
    int home_last_x, home_last_y;

    /* verbs deferred behind the converging target */
    MGAPtrVerb q[PTR_QMAX];
    uint32_t qhead, qlen;
    int64_t btn_ready_ns;
    uint8_t held;           /* buttons this engine is holding down         */

    /* STAT counters */
    int res_x, res_y;
    uint64_t steps, converged, gaveup, hot_seen;
} MGAPtrLoop;

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

    /* hardware cursor bookkeeping (derived from xreg[], not migrated) */
    int hw_cursor_size;
    int hw_cursor_last_x, hw_cursor_last_y;

    /* closed-loop 1:1 pointer over those same cursor registers */
    MGAPtrLoop ptr;
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

/*
 * Hardware cursor: DAC1064/G200 style.  The 64x64x2bpp shape lives in
 * VRAM at XCURADD<<10 (16 bytes per scan line: 8 bytes plane 0, then 8
 * bytes plane 1, MSB = leftmost pixel).  The position registers hold the
 * screen position of the sprite's top-left corner biased by +64 so that
 * 0 parks it off screen.  The sprite is composed into the display
 * surface from the vga core's cursor hooks, so every consumer of the
 * console surface (streamhost capture, screendump, VNC) sees it.
 */
/*
 * The DAC1064 biases the cursor position registers by +64, so that 0 parks the
 * 64x64 sprite entirely off screen. THE BIAS IS 64 AND NOT 57, which is worth
 * writing down because the measurement says 57 until you look at the glyph.
 *
 * Shove the pointer past the top-left corner until the X server pins it -- the
 * pointer is then (0,0) by construction -- and these registers read 57. The
 * same 7 shows up at the bottom-right clamp, on both axes. That looks exactly
 * like a bias off by 7, and subtracting 57 does make the sprite origin land on
 * the pointer.
 *
 * It is not a bias. The sprite CDE installs over the root window is the
 * X_cursor -- a 16x16 saltire whose hotspot is its CENTRE -- so the origin is
 * SUPPOSED to sit 7 px up and left of the pointer, and forcing it onto the
 * pointer draws the X 7 px down and right of where a click lands. The proof is
 * in the shape: at bias 57 the corner shows the whole X with its arms at the
 * screen edge; at bias 64 the top-left 7 px are correctly clipped, because
 * that is where a centre-hotspot cursor sits when its hotspot is at (0,0).
 *
 * So the 7 is a HOTSPOT, and hotspots are the pointer engine's business, not
 * the DAC's: see mga_ptr_home and mga_ptr_glyph_sample below.
 */
static int mga_cursor_x(MGAState *s)
{
    return (s->xreg[0xF0] | ((int)s->xreg[0xF1] << 8)) - 64;
}

static int mga_cursor_y(MGAState *s)
{
    return (s->xreg[0xF2] | ((int)s->xreg[0xF3] << 8)) - 64;
}

static void mga_cursor_invalidate_rect(MGAState *s, int y)
{
    vga_invalidate_scanlines(&s->vga, MAX(y, 0), MAX(y + 64, 0));
}

static void mga_cursor_invalidate(VGACommonState *vga)
{
    MGAState *s = container_of(vga, MGAState, vga);
    int size = (s->xreg[MGA_XCURCTRL] & 3) ? 64 : 0;
    int x = mga_cursor_x(s), y = mga_cursor_y(s);

    if (size != s->hw_cursor_size || x != s->hw_cursor_last_x ||
        y != s->hw_cursor_last_y) {
        if (s->hw_cursor_size) {
            mga_cursor_invalidate_rect(s, s->hw_cursor_last_y);
        }
        s->hw_cursor_size = size;
        s->hw_cursor_last_x = x;
        s->hw_cursor_last_y = y;
        if (size) {
            mga_cursor_invalidate_rect(s, y);
        }
    }
}

static void mga_cursor_draw_line(VGACommonState *vga, uint8_t *d, int scr_y)
{
    MGAState *s = container_of(vga, MGAState, vga);
    unsigned mode = s->xreg[MGA_XCURCTRL] & 3;
    const uint8_t *p0, *p1;
    uint32_t addr, col0, col1;
    int cx, cy, x;

    if (!mode) {
        return;
    }
    cx = mga_cursor_x(s);
    cy = mga_cursor_y(s);
    if (scr_y < cy || scr_y >= cy + 64) {
        return;
    }
    addr = (((uint32_t)s->xreg[MGA_XCURADDH] << 8) | s->xreg[MGA_XCURADDL])
           << 10;
    if (addr + 1024 > vga->vram_size) {
        return;
    }
    p0 = vga->vram_ptr + addr + (scr_y - cy) * 16;
    p1 = p0 + 8;
    col0 = rgb_to_pixel32(s->xreg[MGA_XCURCOL0RED],
                          s->xreg[MGA_XCURCOL0RED + 1],
                          s->xreg[MGA_XCURCOL0RED + 2]);
    col1 = rgb_to_pixel32(s->xreg[MGA_XCURCOL1RED],
                          s->xreg[MGA_XCURCOL1RED + 1],
                          s->xreg[MGA_XCURCOL1RED + 2]);
    for (x = 0; x < 64; x++) {
        int sx = cx + x;
        /*
         * The 8 bytes of each plane row are BYTE-REVERSED in VRAM: byte 7
         * holds the leftmost 8 pixels.  Not a guess -- dumped live from
         * 0xC3000000 + XCURADD<<10 with a cursor on screen, and only this
         * reading produces a cursor at all (every other candidate layout is
         * blank or noise; the arrow's tip lands at sprite 0,0 under it).
         * AIX's DDX uploads the sprite with 64-bit stores -- what a PowerPC
         * graphics driver naturally uses for a bulk copy -- and they land
         * byte-swapped in this model's little-endian VRAM aperture.  Nothing
         * else is affected because CDE draws through the BITBLT/ILOAD engine,
         * not through direct framebuffer stores.
         *
         * Read the wrong way round the sprite still RENDERS, which is why it
         * survived bring-up: an arrow appeared and tracked the mouse. It
         * appeared ~48 px to the RIGHT of the actual pointer, which no
         * open-loop test could see and a 1:1 cursor cannot live with.
         */
        bool b0 = (p0[7 - (x >> 3)] >> (7 - (x & 7))) & 1;
        bool b1 = (p1[7 - (x >> 3)] >> (7 - (x & 7))) & 1;
        uint32_t *px;

        if (sx < 0 || sx >= vga->last_scr_width) {
            continue;
        }
        px = (uint32_t *)d + sx;
        switch (mode) {
        case 3:                 /* X-windows: plane 1 = mask */
            if (b1) {
                *px = b0 ? col1 : col0;
            }
            break;
        case 2:                 /* XGA: plane 1 = transparency/complement */
            if (!b1) {
                *px = b0 ? col1 : col0;
            } else if (b0) {
                *px = ~*px;
            }
            break;
        default:                /* 3-colour */
            if (b0 || b1) {
                *px = b1 ? col1 : col0;
            }
            break;
        }
    }
}

/* ===================================================================== *
 * Closed-loop 1:1 pointer ("mgaptr/1")
 *
 * The 40p has a PS/2 mouse and nothing else -- no tablet, no absolute
 * device -- so the daemon used to reckon absolute coordinates itself:
 * pin the guest cursor into a corner once, then send deltas from where
 * it BELIEVES the cursor is.  A belief is wrong the moment the guest
 * accelerates, clamps at a screen edge, or warps the pointer, and the
 * visitor's cursor and the guest's part company with nothing to pull
 * them back together.
 *
 * This station does not have to guess.  The X server drives the Matrox
 * HARDWARE cursor, so the guest writes the pointer position into the
 * DAC's CURPOSX/CURPOSY registers on every move -- and those registers
 * are ours to read.  That closes the loop, exactly as `irix` closes it
 * over the Newport VC2's cursor registers (docs/IO-PATHS.md, the
 * `mamesock (closed loop)` row):
 *
 *     err = target - reading - hotspot - in-flight
 *     counts = trunc(err / (gain * margin)),  capped, one step per window
 *
 * The control law and its three binding rules are ported from the MAME
 * ctlsock module's landed MOVEA engine, whose dead ends are recorded in
 * docs/guests/irix.md "MOVEA engine -- closed loop over the reading":
 *
 *   - IN-FLIGHT (`infl`, geometrically decayed) holds pixels issued but
 *     not yet observed, so a lagging reading cannot make the loop
 *     re-issue counts that are already on the wire.  That was v1's
 *     rubber-band.
 *   - NO OPPOSING STEP: a step may never contradict the sign of the
 *     measured error.  Never extrapolate the cursor ahead of real
 *     movement -- undershoot and trim.
 *   - THE OSCILLATION LATCH: a target that reverses its correction
 *     OSC_FLIPS times is finished where it stands, which is what keeps
 *     a glyph whose hotspot flips at a boundary from being chased in
 *     and out of the hot zone forever (the "repelling magnet").
 *
 * GAIN IS A STEP SIZER ONLY.  It sizes how far one window may reach;
 * it never decides where the pointer ends up, because the next window
 * measures the result.  A wrong gain costs convergence SPEED, never
 * accuracy -- which is the whole reason this is a closed loop and not
 * the dead-reckoned bridge it replaces.
 *
 * THE HOTSPOT IS READ, NOT INFERRED.  The registers hold the cursor
 * SPRITE's top-left corner, so the reading is `pointer - hotspot`, and
 * every glyph X installs (I-beam, resize handle, watch) has a different
 * hotspot.  Driving the READING onto the target would park the POINTER
 * a hotspot away from the visitor's cursor.  No register holds the
 * hotspot -- but swapping the glyph FORCES a compensating write to the
 * position registers with the pointer standing still, and that write IS
 * the hotspot delta.  So the shape/enable registers ARM a sampler, and
 * the next window books the position delta as a hotspot change -- but
 * only when the loop itself commanded nothing, and only when the step
 * fits inside the 64x64 sprite.  Both guards exist because on irix four
 * separate attempts to INFER the hotspot from samples and statistics
 * each looked right and each failed in the field.
 *
 * NOTHING HERE IS MIGRATED.  Every field below is derived from the
 * registers (which are migrated) or from the live connection, and the
 * engine re-bases itself after a `loadvm`.  That is deliberate: adding
 * a save item would change the machine's migration signature and cost
 * the station a golden recapture (AGENTS.md rule 6).
 * ===================================================================== */

static void G_GNUC_PRINTF(2, 3) mga_ptr_trace(MGAState *s,
                                             const char *fmt, ...)
{
    va_list ap;

    if (!s->ptr.trace) {
        return;
    }
    va_start(ap, fmt);
    fprintf(stderr, "mgaptr: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    fflush(stderr);
}

static int mga_ptr_surf_w(MGAState *s)
{
    return s->mode_width ? (int)s->mode_width : 1024;
}

static int mga_ptr_surf_h(MGAState *s)
{
    return s->mode_height ? (int)s->mode_height : 768;
}

/*
 * ONE observation of the hardware-cursor registers.  Returns false when
 * the sprite is disabled: the reading is then stale and the loop must
 * fall back on its belief rather than chase a frozen number.
 */
static bool mga_ptr_reading(MGAState *s, int *rx, int *ry)
{
    *rx = mga_cursor_x(s);
    *ry = mga_cursor_y(s);
    return (s->xreg[MGA_XCURCTRL] & 3) != 0;
}

static void G_GNUC_PRINTF(2, 3) mga_ptr_send(MGAState *s, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int n;

    if (!s->ptr.open) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 2) {
        n = sizeof(buf) - 2;
    }
    buf[n++] = '\n';
    qemu_chr_fe_write_all(&s->ptr.chr, (const uint8_t *)buf, n);
}

/* ---- injection: straight at the PS/2 mouse ---------------------------
 *
 * The engine is the SINGLE INJECTOR for this guest's pointer.  Nothing
 * else may push motion or button edges at the mouse while the socket is
 * connected -- a second injector fights the loop for the guest's PS/2
 * accumulator and the two of them will never agree on where the cursor
 * is.  streamhost enforces its half by routing the whole pointer here
 * when SH_INPUT_BACKEND=mgactl.
 */
static void mga_ptr_inject_rel(MGAState *s, int cx, int cy)
{
    if (cx) {
        qemu_input_queue_rel(NULL, INPUT_AXIS_X, cx);
    }
    if (cy) {
        qemu_input_queue_rel(NULL, INPUT_AXIS_Y, cy);
    }
    qemu_input_event_sync();
}

static void mga_ptr_inject_btn(MGAState *s, int btn, bool down)
{
    static const InputButton map[3] = {
        INPUT_BUTTON_LEFT, INPUT_BUTTON_RIGHT, INPUT_BUTTON_MIDDLE
    };

    if (btn < 0 || btn > 2) {
        return;
    }
    qemu_input_queue_btn(NULL, map[btn], down);
    qemu_input_event_sync();
    if (down) {
        s->ptr.held |= 1 << btn;
    } else {
        s->ptr.held &= ~(1 << btn);
    }
    mga_ptr_trace(s, "btn %d %s", btn + 1, down ? "down" : "up");
}

static void mga_ptr_release_all(MGAState *s)
{
    int b;

    for (b = 0; b < 3; b++) {
        if (s->ptr.held & (1 << b)) {
            mga_ptr_inject_btn(s, b, false);
        }
    }
}

/* ---- the verb queue --------------------------------------------------
 * Everything the client sends behind a converging target waits here, in
 * order, so a click can never apply while the pointer is still flying
 * to the place it was aimed at.
 */
static void mga_ptr_qclear(MGAState *s)
{
    s->ptr.qhead = 0;
    s->ptr.qlen = 0;
}

static MGAPtrVerb *mga_ptr_qpush(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    MGAPtrVerb *v;

    if (p->qlen >= PTR_QMAX) {
        /* Overflow means the guest has stopped absorbing input entirely.
         * Drop the OLDEST: the newest target is the one the visitor's
         * cursor is actually at. */
        v = &p->q[p->qhead];
        if (v->seq) {
            mga_ptr_send(s, "%" PRIu64 " ERR overflow", v->seq);
        }
        p->qhead = (p->qhead + 1) % PTR_QMAX;
        p->qlen--;
    }
    v = &p->q[(p->qhead + p->qlen) % PTR_QMAX];
    p->qlen++;
    memset(v, 0, sizeof(*v));
    return v;
}

static void mga_ptr_qpop(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;

    if (p->qlen) {
        p->qhead = (p->qhead + 1) % PTR_QMAX;
        p->qlen--;
    }
}

/* ---- control-law helpers -------------------------------------------- */

/*
 * Size one step.  `eff` is the error with in-flight pixels removed,
 * `raw` the measured error.  A step may never oppose the measured error
 * -- an over-estimated in-flight balance must cost a window of waiting,
 * never a backwards move -- and the gain margin keeps a full-gain guest
 * short of the target rather than past it.
 */
static int mga_ptr_step_counts(MGAState *s, int eff, int raw, double g)
{
    MGAPtrLoop *p = &s->ptr;
    int c;

    if (abs(eff) <= (int)p->dead) {
        return 0;
    }
    if (g < 0.05) {
        g = 0.05;
    }
    c = (int)trunc((double)eff / (g * PTR_GAIN_MARGIN));
    if (c == 0) {
        c = (eff > 0) ? 1 : -1;   /* sub-gain residue: one count is the
                                   * smallest motion the wire can express */
    }
    if ((c > 0 && raw <= 0) || (c < 0 && raw >= 0)) {
        return 0;
    }
    return MAX(-(int)p->move_step, MIN((int)p->move_step, c));
}

/* Per-axis sign-reversal counter behind the oscillation latch. */
static bool mga_ptr_osc(int c, int *lastsign, uint32_t *flips)
{
    int sign;

    if (c == 0) {
        return false;
    }
    sign = (c > 0) ? 1 : -1;
    if (*lastsign != 0 && sign != *lastsign) {
        (*flips)++;
    }
    *lastsign = sign;
    return *flips >= PTR_OSC_FLIPS;
}

/*
 * Gain learning, closed-loop safe: the loop's ACCURACY never depends on
 * g, only its speed, so this is a cheap EMA over the last step's
 * observed/issued ratio.  ASYMMETRIC on purpose -- a step only partly
 * absorbed reads LOW, and a gain estimated too low over-issues, the one
 * direction the no-overshoot rule forbids.  So g rises fast, falls slow.
 */
static void mga_ptr_learn_gain(MGAState *s, char ax, int c, int d,
                               bool lo_edge, bool hi_edge, double *g)
{
    double r;

    if (abs(c) < PTR_GAIN_MIN_COUNTS || lo_edge || hi_edge) {
        return;   /* too small to measure, or the guest clamped: that is a
                   * lower bound, not a measurement */
    }
    r = (double)d / (double)c;
    if (r < PTR_GAIN_LO || r > PTR_GAIN_HI) {
        return;   /* partial absorption or a glyph step: corrupt sample */
    }
    *g += ((r > *g) ? PTR_GAIN_ALPHA_UP : PTR_GAIN_ALPHA_DOWN) * (r - *g);
    mga_ptr_trace(s, "gain %c obs=%.3f g=%.3f", ax, r, *g);
}

/*
 * Book what the registers moved by since the last window against what
 * was issued, and decay the in-flight balance.  A gain OVER-estimate
 * self-clears here instead of parking a phantom balance in front of the
 * loop forever.
 */
static void mga_ptr_observe(MGAState *s, int rx, int ry, bool trusted)
{
    MGAPtrLoop *p = &s->ptr;

    if (p->obs_valid) {
        int dx = rx - p->obs_x;
        int dy = ry - p->obs_y;

        if (dx || dy) {
            p->awaiting = false;
            p->await_windows = 0;
        }
        p->infl_x -= dx;
        p->infl_y -= dy;
        if (trusted) {
            mga_ptr_learn_gain(s, 'x', p->last_cx, dx,
                               rx <= PTR_GAIN_EDGE_MARGIN,
                               rx >= mga_ptr_surf_w(s) - 1 - PTR_GAIN_EDGE_MARGIN,
                               &p->gx);
            mga_ptr_learn_gain(s, 'y', p->last_cy, dy,
                               ry <= PTR_GAIN_EDGE_MARGIN,
                               ry >= mga_ptr_surf_h(s) - 1 - PTR_GAIN_EDGE_MARGIN,
                               &p->gy);
        }
    }
    p->infl_x *= PTR_INFL_DECAY;
    p->infl_y *= PTR_INFL_DECAY;
    p->obs_x = rx;
    p->obs_y = ry;
    p->obs_valid = true;
    p->last_cx = 0;
    p->last_cy = 0;
    if (trusted) {
        p->bel_x = rx;
        p->bel_y = ry;
    }
}

/*
 * Shift the hotspot and keep the observer honest: the pointer estimate
 * is reading + hot, so observe() must see the step added back to the raw
 * reading delta, or it books a sprite jump as pointer motion.
 */
static void mga_ptr_hot_shift(MGAState *s, int nx, int ny)
{
    MGAPtrLoop *p = &s->ptr;

    if (abs(nx) > PTR_SPRITE_MAX || abs(ny) > PTR_SPRITE_MAX) {
        /* A hotspot lives inside the 64x64 sprite. Nothing should be able to
         * propose otherwise; if something does, it is not a hotspot. */
        mga_ptr_trace(s, "hotspot %d,%d out of the sprite -- refused", nx, ny);
        return;
    }
    if (nx == p->hot_x && ny == p->hot_y) {
        return;
    }
    mga_ptr_trace(s, "hot %d,%d -> %d,%d", p->hot_x, p->hot_y, nx, ny);
    p->obs_x -= nx - p->hot_x;
    p->obs_y -= ny - p->hot_y;
    p->hot_x = nx;
    p->hot_y = ny;
    p->hot_dirty = true;    /* ... and the at-rest pass owes it a correction */
    p->hot_seen++;
}

/*
 * THE GLYPH SAMPLER: the hotspot is read, or it is not known.
 *
 * The registers hold the cursor SPRITE's origin, so the reading is
 * `pointer - hotspot`, and the hotspot is different for every glyph X
 * installs. Measured on this desktop: the CDE root window's X_cursor is a
 * saltire whose hotspot is its CENTRE, (7,7); the desktop backdrop's pointer
 * sits at (7,1); Netscape's pointing hand puts its hotspot on the fingertip.
 * Drive the READING onto the target with the wrong hotspot and both the drawn
 * cursor and the click land up to 7 px from the visitor's mouse.
 *
 * ONE fact names a hotspot on this guest, and it is the SCREEN CLAMP: the X
 * server pins the pointer at 0 (or at W-1), so whatever the registers then
 * read is its negation, on that axis, exactly. mga_ptr_home forces one at
 * connect, and mga_ptr_step takes one whenever a visitor's target sits on an
 * edge -- which is what the deliberate over-clamp in there is for.
 *
 * THE OTHER FACT DOES NOT EXIST HERE, and the record of why is worth more than
 * the code that tried it. On irix a glyph swap forces a compensating write to
 * the cursor register with the pointer standing still, and that write IS the
 * hotspot delta; the module reads it off a device accumulator. The same idea
 * was built here and it cannot work, for a reason particular to this guest:
 * X installs the new glyph the instant the pointer CROSSES a window border,
 * which is always mid-flight, so the compensating write and the commanded
 * motion are one register delta and nothing can separate them. Traced live --
 * every swap on this desktop, without exception, arrives while the loop is
 * still stepping. Running the delta anyway behind a "the loop has been quiet
 * for two windows" test did exactly what irix warns of: it booked the guest's
 * own lagging motion as hotspot and walked the offset to -67 px in one sweep.
 * So there is no delta path here. A glyph no clamp has named is UNKNOWN.
 *
 * What makes that survivable is that the shape signature is also the glyph's
 * NAME. A glyph has to be measured once, ever: every later swap back to it is
 * a table lookup, and exact. Until then the previous glyph's value is carried
 * -- the smallest wrong answer available, and bounded by the sprite -- and
 * `hot_exact=0` in STAT says so out loud instead of pretending.
 */
static uint64_t mga_ptr_shape_sig(MGAState *s)
{
    uint32_t addr = (((uint32_t)s->xreg[MGA_XCURADDH] << 8) |
                     s->xreg[MGA_XCURADDL]) << 10;
    const uint8_t *p;
    uint64_t sig;
    int i;

    if (addr + 1024 > s->vga.vram_size) {
        return 0;
    }
    p = s->vga.vram_ptr + addr;
    sig = 1469598103934665603ULL ^ addr;            /* FNV-1a, 64-bit */
    for (i = 0; i < 1024; i++) {
        sig = (sig ^ p[i]) * 1099511628211ULL;
    }
    return sig;
}

/* Remember this glyph's hotspot under its signature. Newest wins on a
 * collision: a later exact measurement is later evidence, not a duplicate. */
static void mga_ptr_hot_record(MGAState *s, uint64_t sig, int hx, int hy)
{
    MGAPtrLoop *p = &s->ptr;
    uint32_t i, slot = p->hot_next;

    for (i = 0; i < PTR_HOT_SLOTS; i++) {
        if (p->hot_tab[i].valid && p->hot_tab[i].sig == sig) {
            slot = i;
            break;
        }
    }
    if (i == PTR_HOT_SLOTS) {
        p->hot_next = (p->hot_next + 1) % PTR_HOT_SLOTS;
    }
    p->hot_tab[slot].sig = sig;
    p->hot_tab[slot].hx = hx;
    p->hot_tab[slot].hy = hy;
    p->hot_tab[slot].valid = true;
    mga_ptr_trace(s, "glyph %016" PRIx64 " hotspot %d,%d recorded", sig, hx, hy);
}

static bool mga_ptr_hot_lookup(MGAState *s, uint64_t sig, int *hx, int *hy)
{
    MGAPtrLoop *p = &s->ptr;
    uint32_t i;

    for (i = 0; i < PTR_HOT_SLOTS; i++) {
        if (p->hot_tab[i].valid && p->hot_tab[i].sig == sig) {
            *hx = p->hot_tab[i].hx;
            *hy = p->hot_tab[i].hy;
            return true;
        }
    }
    return false;
}

static void mga_ptr_glyph_sample(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    uint64_t sig;
    int rx, ry, hx, hy;

    if (!p->track_hotspot || p->homing) {
        return;
    }
    if (!mga_ptr_reading(s, &rx, &ry)) {
        return;
    }
    sig = mga_ptr_shape_sig(s);
    if (!p->sig_valid) {
        p->cur_sig = sig;
        p->sig_valid = true;
        return;
    }
    if (sig == p->cur_sig) {
        return;
    }

    /* --- a swap --- */
    p->cur_sig = sig;
    p->hot_seen++;
    if (mga_ptr_hot_lookup(s, sig, &hx, &hy)) {
        mga_ptr_hot_shift(s, hx, hy);       /* measured before: exact */
        p->hot_exact = true;
    } else {
        /*
         * Never named by a clamp. Fall back to ZERO -- the sprite's own origin
         * -- and not to the previous glyph's value, which is the mistake this
         * used to make. The one glyph a clamp can always name here is the CDE
         * root window's X_cursor, and it is the ATYPICAL one: a centred
         * saltire, hotspot (7,7). Every other glyph on this desktop puts its
         * hotspot at the sprite origin -- measured by walking the commanded
         * pixel across the Netscape window's left frame and watching which
         * glyph X installs: the frame's cursor appears at reading 9 against a
         * frame that starts at screen x=10, and the page's at reading 24
         * against a page that starts at x=25. Carrying the X_cursor's 7 into
         * them put the visitor's mouse 7 px INSIDE the arrow instead of on its
         * tip, and made the pointer jitter at every window border, because the
         * reading shifted under a hotspot that did not.
         *
         * Zero is the neutral prior -- "the sprite origin is the pointer until
         * the guest says otherwise" -- not an inference from samples, and
         * `hot_exact=0` still says it has not been measured. A clamp under
         * this glyph replaces it with the fact.
         */
        mga_ptr_trace(s, "swap to glyph %016" PRIx64
                      " no clamp has named -- hotspot reset to 0,0", sig);
        mga_ptr_hot_shift(s, 0, 0);
        p->hot_exact = false;
    }
}

/* ---- targets --------------------------------------------------------- */

static void mga_ptr_target_reset(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;

    p->windows = 0;
    p->osc_sx = p->osc_sy = 0;
    p->osc_nx = p->osc_ny = 0;
}

static void mga_ptr_epoch(MGAState *s, int x, int y, uint64_t seq)
{
    MGAPtrLoop *p = &s->ptr;

    p->ta_x = MAX(0, MIN(mga_ptr_surf_w(s) - 1, x));
    p->ta_y = MAX(0, MIN(mga_ptr_surf_h(s) - 1, y));
    p->ta_seq = seq;
    p->ta_active = true;
    mga_ptr_target_reset(s);
    mga_ptr_trace(s, "target %d,%d seq=%" PRIu64, p->ta_x, p->ta_y, seq);
}

static void mga_ptr_drain(MGAState *s);

/*
 * Finish the in-flight target.  Its VALUE is latched on a clean finish,
 * so the at-rest pass knows where the visitor last aimed; a GIVE-UP does
 * not latch -- a pointer freed from a modal grab must be able to try
 * again.
 */
static void mga_ptr_accept(MGAState *s, int ex, int ey, bool gaveup)
{
    MGAPtrLoop *p = &s->ptr;

    if (!gaveup) {
        p->latch_x = p->ta_x;
        p->latch_y = p->ta_y;
        p->latch_valid = true;
    }
    p->ta_active = false;
    p->awaiting = false;
    p->await_windows = 0;
    p->converged += !gaveup;
    p->gaveup += gaveup;
    p->res_x = ex;
    p->res_y = ey;
    mga_ptr_drain(s);
}

/* Release what queued behind the finished target, up to the next target. */
static void mga_ptr_drain(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;

    while (!p->ta_active && !p->homing && p->qlen) {
        MGAPtrVerb *v = &p->q[p->qhead];
        int64_t now;

        switch (v->kind) {
        case 'b':
            now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            if (now < p->btn_ready_ns) {
                return;   /* the pacer owes the previous edge its gap: a
                           * DOWN+UP pair inside one 10 ms PS/2 sample is
                           * a click the guest never sees */
            }
            mga_ptr_inject_btn(s, v->btn, v->down);
            p->btn_ready_ns = now +
                (int64_t)p->btn_gap_ms * SCALE_MS;
            if (v->seq) {
                mga_ptr_send(s, "%" PRIu64 " OK", v->seq);
            }
            mga_ptr_qpop(s);
            break;
        case 's':
            if (v->seq) {
                mga_ptr_send(s, "%" PRIu64 " OK", v->seq);
            }
            mga_ptr_qpop(s);
            break;
        default: {
            int x = v->x, y = v->y;
            uint64_t seq = v->seq;
            mga_ptr_qpop(s);
            mga_ptr_epoch(s, x, y, seq);
            break;
        }
        }
    }
}

/*
 * One window of the loop.  At most one step is issued per window and
 * never while the previous correction is still unobserved -- by the next
 * window it is in the registers and the fresh error already accounts for
 * it.
 */
static void mga_ptr_step(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    int rx, ry, ex, ey, cx, cy;
    bool have, ox, oy, edge_x, edge_y, want_pin;

    if (!p->ta_active || p->homing) {
        return;
    }
    have = mga_ptr_reading(s, &rx, &ry);
    if (!have) {
        /* No trustworthy reading (the sprite is disabled): run this
         * window off the belief so a hidden pointer still moves, and let
         * the give-up cap bound it. */
        rx = p->bel_x;
        ry = p->bel_y;
    }
    mga_ptr_observe(s, rx, ry, have);

    ex = p->ta_x - rx - p->hot_x;
    ey = p->ta_y - ry - p->hot_y;

    /*
     * A target ON AN EDGE, under a glyph whose hotspot has never been
     * measured, is the one chance this loop gets to measure it -- so it is
     * spent rather than converged away. Landing 1 px short of the edge is
     * "arrived" by the deadband, and the pointer then never pins, so the clamp
     * below never fires and the glyph stays unknown for the whole session.
     * Push PAST the edge instead: the overshoot dies against the guest's own
     * clamp, which is the entire point, and the visitor sees nothing because
     * the pointer is already there.
     */
    edge_x = p->ta_x <= 1 || p->ta_x >= mga_ptr_surf_w(s) - 2;
    edge_y = p->ta_y <= 1 || p->ta_y >= mga_ptr_surf_h(s) - 2;
    want_pin = !p->hot_exact && have &&
               ((edge_x && p->pinned_x < PTR_PIN_WINDOWS) ||
                (edge_y && p->pinned_y < PTR_PIN_WINDOWS));

    if (!want_pin && abs(ex) <= (int)p->dead && abs(ey) <= (int)p->dead) {
        mga_ptr_trace(s, "converge r=%d,%d err=%d,%d g=%.2f,%.2f",
                      rx, ry, ex, ey, p->gx, p->gy);
        mga_ptr_accept(s, ex, ey, false);
        return;
    }
    if (++p->windows > p->tries) {
        mga_ptr_trace(s, "giveup r=%d,%d err=%d,%d w=%u",
                      rx, ry, ex, ey, p->windows);
        mga_ptr_accept(s, ex, ey, true);
        return;
    }
    /*
     * NEVER ISSUE WHILE THE LAST CORRECTION IS STILL ON THE WIRE. `infl` is
     * a decaying ESTIMATE of what has not landed yet, and under TCG the guest
     * takes several windows to absorb a PS/2 packet, walk it through X's
     * acceleration and write the cursor register -- so the estimate decays
     * away long before the motion arrives, and the loop re-issues counts that
     * are already in flight. Measured on this guest before this gate existed:
     * two windows of -48 counts against an unchanged reading, then a 288 px
     * overshoot and an oscillation that accepted 30 px off target. The
     * registers moving is the ONLY evidence that a step has landed; until
     * they do, this window observes and waits. The give-up cap above still
     * bounds it, so a guest that has stopped absorbing input cannot wedge the
     * loop.
     */
    if (p->awaiting) {
        /*
         * ... but a step that never lands must not own the loop for ever.
         * Against the guest's screen clamp the registers NEVER move again, so
         * the gate would stay shut, the pointer would stop responding to every
         * later target, and the station would look wedged -- measured exactly
         * that way the first time this gate existed without the bound. After
         * this many windows the counts are gone: absorbed by a clamp, or
         * dropped. Declare them landed and let the loop measure again.
         */
        if (++p->await_windows <= PTR_AWAIT_MAX) {
            return;
        }
        p->awaiting = false;
        p->await_windows = 0;
        p->infl_x = p->infl_y = 0.0;
    }

    cx = mga_ptr_step_counts(s, ex - (int)llround(p->infl_x), ex, p->gx);
    cy = mga_ptr_step_counts(s, ey - (int)llround(p->infl_y), ey, p->gy);
    if (want_pin) {
        /* Deliberate over-clamp: the no-opposing-step rule is about not
         * extrapolating the pointer ahead of real movement, and there is no
         * movement left to get ahead of. */
        if (edge_x) {
            cx = (p->ta_x <= 1) ? -(int)p->move_step : (int)p->move_step;
        }
        if (edge_y) {
            cy = (p->ta_y <= 1) ? -(int)p->move_step : (int)p->move_step;
        }
    }

    ox = mga_ptr_osc(cx, &p->osc_sx, &p->osc_nx);
    oy = mga_ptr_osc(cy, &p->osc_sy, &p->osc_ny);
    if (!want_pin && (ox || oy) &&
        abs(ex) <= PTR_OSC_MAX_ERR && abs(ey) <= PTR_OSC_MAX_ERR) {
        /* The correction keeps reversing: that is a reading which moves
         * WITH the pointer, not a pointer that keeps missing its target.
         * Accept where we are. */
        mga_ptr_trace(s, "osc-accept r=%d,%d err=%d,%d flips=%u,%u",
                      rx, ry, ex, ey, p->osc_nx, p->osc_ny);
        mga_ptr_accept(s, ex, ey, false);
        return;
    }
    if (cx == 0 && cy == 0) {
        return;   /* everything outstanding is already on the wire */
    }
    /*
     * OPPORTUNISTIC CLAMP CALIBRATION. A glyph's hotspot is only ever
     * observable at a screen clamp or at an at-rest glyph swap, and on this
     * guest the at-rest swap never happens: X installs the new glyph the
     * instant the pointer crosses a window border, which is always mid-flight,
     * so the compensating write and the commanded motion are one register
     * delta and cannot be told apart. Traced live -- every swap on this
     * desktop, without exception.
     *
     * The clamp does happen, constantly, because visitors sweep to the edges.
     * When the target is ON an edge and the loop has pushed at it for several
     * windows with the registers refusing to move, the X server has pinned the
     * pointer at 0 or at W-1, and that names the CURRENT glyph's hotspot
     * outright on that axis, with no model of anything in it. Per axis,
     * because a sweep into the left edge says nothing about y.
     */
    if (cx && ((cx > 0) - (cx < 0)) == p->last_cx_dir && rx == p->pin_rx) {
        p->pinned_x++;
    } else {
        p->pinned_x = 0;
    }
    if (cy && ((cy > 0) - (cy < 0)) == p->last_cy_dir && ry == p->pin_ry) {
        p->pinned_y++;
    } else {
        p->pinned_y = 0;
    }
    p->pin_rx = rx;
    p->pin_ry = ry;
    p->last_cx_dir = (cx > 0) - (cx < 0);
    p->last_cy_dir = (cy > 0) - (cy < 0);
    if (have && p->sig_valid) {
        int nhx = p->hot_x, nhy = p->hot_y;
        bool got = false;

        if (p->pinned_x >= PTR_PIN_WINDOWS) {
            if (p->ta_x <= 1 && cx < 0) {
                nhx = -rx;
                got = true;
            } else if (p->ta_x >= mga_ptr_surf_w(s) - 2 && cx > 0) {
                nhx = mga_ptr_surf_w(s) - 1 - rx;
                got = true;
            }
        }
        if (p->pinned_y >= PTR_PIN_WINDOWS) {
            if (p->ta_y <= 1 && cy < 0) {
                nhy = -ry;
                got = true;
            } else if (p->ta_y >= mga_ptr_surf_h(s) - 2 && cy > 0) {
                nhy = mga_ptr_surf_h(s) - 1 - ry;
                got = true;
            }
        }
        if (got && abs(nhx) <= PTR_SPRITE_MAX && abs(nhy) <= PTR_SPRITE_MAX &&
            (nhx != p->hot_x || nhy != p->hot_y || !p->hot_exact)) {
            mga_ptr_trace(s, "clamp calibration: hot %d,%d -> %d,%d",
                          p->hot_x, p->hot_y, nhx, nhy);
            mga_ptr_hot_shift(s, nhx, nhy);
            mga_ptr_hot_record(s, p->cur_sig, p->hot_x, p->hot_y);
            p->hot_exact = true;
        }
    }

    p->last_cx = cx;
    p->last_cy = cy;
    p->awaiting = true;
    p->await_windows = 0;
    p->infl_x += cx * p->gx;
    p->infl_y += cy * p->gy;
    p->bel_x += (int)llround(cx * p->gx);
    p->bel_y += (int)llround(cy * p->gy);
    p->steps++;
    mga_ptr_inject_rel(s, cx, cy);
    mga_ptr_trace(s, "step r=%d,%d err=%d,%d c=%d,%d infl=%.1f,%.1f w=%u",
                  rx, ry, ex, ey, cx, cy, p->infl_x, p->infl_y, p->windows);
}

/*
 * The at-rest observer.  X swaps the cursor glyph when the pointer
 * ARRIVES somewhere -- after the loop has converged and gone idle -- so
 * without a pass that runs with no target active, a hotspot change would
 * stay invisible until the pointer next moved, which is exactly when it
 * is too late.  On a detected shift the last accepted target is re-armed
 * internally (seq 0, so the client is told nothing about a target it
 * never sent) and the pointer goes back under the visitor's cursor.
 */
static void mga_ptr_rest(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    int rx, ry, lx, ly;
    bool dirty;

    if (p->ta_active || p->homing || p->qlen) {
        return;
    }
    if (!mga_ptr_reading(s, &rx, &ry)) {
        return;
    }
    lx = p->latch_x;
    ly = p->latch_y;
    dirty = p->hot_dirty && p->latch_valid;
    p->hot_dirty = false;
    mga_ptr_observe(s, rx, ry, true);
    if (dirty && (abs(lx - rx - p->hot_x) > (int)p->dead ||
                  abs(ly - ry - p->hot_y) > (int)p->dead)) {
        mga_ptr_epoch(s, lx, ly, 0);
    }
}


/*
 * ONE-TIME CLAMP CALIBRATION, and the only thing on this station that is not
 * already a fact the device reports.
 *
 * The registers hold the cursor SPRITE's origin, biased by the DAC's own
 * constant and offset by whatever the current glyph's hotspot is. Neither
 * number is readable, and their SUM is exactly what stands between the
 * reading and the pointer. But the guest's own screen clamp names it: shove
 * the pointer past the top-left corner and the X server pins it at 0,0 --
 * that is the one event where guest and model agree by construction -- so
 * whatever the registers then read is the negation of the whole offset, at
 * once, without a model of either half.
 *
 * (irix rejected clamp calibration because it could not FORCE a clamp: it had
 * to wait for a visitor to shove the pointer into an edge, and it misfired on
 * a merely-stalled loop. Here the calibration is a deliberate burst at
 * connect, with no visitor and nothing else moving the pointer, so neither
 * objection applies. Glyph swaps after this are tracked incrementally by
 * mga_ptr_glyph_sample.)
 *
 * Runs once per client connection, and on an explicit HOME. It is NOT re-run
 * on an idle-pause resume or a loadvm: the registers restore with the machine
 * and the offset with them, and yanking a watching visitor's pointer into a
 * corner mid-session would be a worse bug than the one it fixed.
 */
static void mga_ptr_home(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    int rx, ry;

    if (!p->homing) {
        return;
    }
    if (!mga_ptr_reading(s, &rx, &ry)) {
        /* the sprite is disabled: nothing to calibrate against */
        if (++p->home_windows > PTR_HOME_MAX_WINDOWS) {
            p->homing = false;
            mga_ptr_trace(s, "home abandoned: cursor disabled");
        }
        return;
    }
    if (p->home_windows && rx == p->home_last_x && ry == p->home_last_y) {
        p->home_still++;
    } else {
        p->home_still = 0;
    }
    p->home_last_x = rx;
    p->home_last_y = ry;

    if (p->home_still >= PTR_HOME_STILL) {
        /* Pinned. pointer == 0,0, so hot == -reading, whole and exact. */
        p->hot_x = -rx;
        p->hot_y = -ry;
        p->obs_x = rx;
        p->obs_y = ry;
        p->bel_x = rx;
        p->bel_y = ry;
        p->obs_valid = true;
        p->infl_x = p->infl_y = 0.0;
        p->awaiting = false;
        p->homing = false;
        p->hot_seen++;
        p->hot_exact = true;
        p->cur_sig = mga_ptr_shape_sig(s);
        p->sig_valid = true;
        mga_ptr_hot_record(s, p->cur_sig, p->hot_x, p->hot_y);
        mga_ptr_trace(s, "home done r=%d,%d -> hot=%d,%d (%u windows)",
                      rx, ry, p->hot_x, p->hot_y, p->home_windows);
        mga_ptr_drain(s);
        return;
    }
    if (++p->home_windows > PTR_HOME_MAX_WINDOWS) {
        p->homing = false;
        mga_ptr_trace(s, "home gave up at r=%d,%d; hotspot left at %d,%d",
                      rx, ry, p->hot_x, p->hot_y);
        mga_ptr_drain(s);
        return;
    }
    /* Deliberately over-clamped: the overshoot has to DIE against the guest's
     * edge, which is the whole point. */
    mga_ptr_inject_rel(s, -(int)p->move_step, -(int)p->move_step);
}

static void mga_ptr_home_start(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;

    p->homing = true;
    p->home_windows = 0;
    p->home_still = 0;
    p->home_last_x = p->home_last_y = INT_MIN;
    p->ta_active = false;
}

static void mga_ptr_tick(void *opaque)
{
    MGAState *s = opaque;

    mga_ptr_home(s);
    mga_ptr_glyph_sample(s);
    mga_ptr_step(s);
    mga_ptr_rest(s);
    mga_ptr_drain(s);
    timer_mod(&s->ptr.timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (int64_t)s->ptr.window_ms * SCALE_MS);
}

/*
 * Re-base every derived belief against the registers.  Run on reset, on
 * a fresh connection and after a loadvm: the restored CURPOSX/Y ARE the
 * restored pointer, but nothing in the stream says which GLYPH is in
 * force, so the hotspot starts at zero and the first swap re-measures it.
 */
static void mga_ptr_rebase(MGAState *s)
{
    MGAPtrLoop *p = &s->ptr;
    int rx, ry;

    p->ta_active = false;
    p->windows = 0;
    p->infl_x = p->infl_y = 0.0;
    p->gx = p->gy = p->gain_x100 / 100.0;
    p->last_cx = p->last_cy = 0;
    p->awaiting = false;
    p->await_windows = 0;
    p->osc_sx = p->osc_sy = 0;
    p->osc_nx = p->osc_ny = 0;
    p->hot_x = p->hot_y = 0;
    p->hot_dirty = false;
    p->sig_valid = false;
    p->hot_exact = false;
    p->pinned_x = p->pinned_y = 0;
    p->last_cx_dir = p->last_cy_dir = 0;
    p->pin_rx = p->pin_ry = INT_MIN;
    memset(p->hot_tab, 0, sizeof(p->hot_tab));
    p->hot_next = 0;
    p->latch_valid = false;
    p->homing = false;
    p->btn_ready_ns = 0;
    mga_ptr_qclear(s);
    mga_ptr_reading(s, &rx, &ry);
    p->obs_x = p->bel_x = rx;
    p->obs_y = p->bel_y = ry;
    p->obs_valid = true;
}

/* ---- the wire -------------------------------------------------------- */

static void mga_ptr_line(MGAState *s, char *line)
{
    MGAPtrLoop *p = &s->ptr;
    char verb[16];
    unsigned long long seq = 0;
    int x = 0, y = 0, n;
    MGAPtrVerb *v;

    n = sscanf(line, "%llu %15s %d %d", &seq, verb, &x, &y);
    if (n < 2) {
        mga_ptr_send(s, "0 ERR parse");
        return;
    }

    if (!strcmp(verb, "MOVEA")) {
        if (n < 4) {
            mga_ptr_send(s, "%llu ERR movea-args", seq);
            return;
        }
        x = MAX(0, MIN(mga_ptr_surf_w(s) - 1, x));
        y = MAX(0, MIN(mga_ptr_surf_h(s) - 1, y));
        /* A target acks on ACCEPT, not on convergence: the client streams
         * targets faster than they converge and must never block on one. */
        mga_ptr_send(s, "%llu OK", seq);
        if (!p->ta_active && !p->homing) {
            mga_ptr_epoch(s, x, y, seq);
        } else if (p->ta_active && !p->qlen) {
            /* Nothing deferred behind the pending target: LATEST WINS. A
             * moving finger streams targets faster than they converge, so
             * a changed value gets a fresh window budget and a fresh
             * oscillation history. */
            if (x != p->ta_x || y != p->ta_y) {
                p->ta_x = x;
                p->ta_y = y;
                mga_ptr_target_reset(s);
            }
            p->ta_seq = seq;
        } else {
            v = &p->q[(p->qhead + p->qlen - 1) % PTR_QMAX];
            if (v->kind == 't') {
                v->x = x;      /* coalesce onto the queued tail target */
                v->y = y;
                v->seq = seq;
            } else {
                v = mga_ptr_qpush(s);
                v->kind = 't';
                v->x = x;
                v->y = y;
                v->seq = seq;
            }
        }
        return;
    }

    if ((strlen(verb) == 5 && !strncmp(verb, "DOWN", 4)) ||
        (strlen(verb) == 3 && !strncmp(verb, "UP", 2))) {
        bool down = verb[0] == 'D';
        int btn = verb[down ? 4 : 2] - '1';

        if (btn < 0 || btn > 2) {
            mga_ptr_send(s, "%llu ERR button", seq);
            return;
        }
        v = mga_ptr_qpush(s);
        v->kind = 'b';
        v->btn = btn;
        v->down = down;
        v->seq = seq;
        mga_ptr_drain(s);
        return;
    }

    if (!strcmp(verb, "HOME")) {
        mga_ptr_home_start(s);
        mga_ptr_send(s, "%llu OK", seq);
        return;
    }

    if (!strcmp(verb, "SYNC")) {
        if (!p->qlen && !p->ta_active && !p->homing) {
            mga_ptr_send(s, "%llu OK", seq);
        } else {
            v = mga_ptr_qpush(s);
            v->kind = 's';
            v->seq = seq;
        }
        return;
    }

    if (!strcmp(verb, "STAT")) {
        int rx, ry;
        bool have = mga_ptr_reading(s, &rx, &ry);

        mga_ptr_send(s, "%llu OK r=%d,%d trust=%d hot=%d,%d g=%.2f,%.2f "
                     "q=%u active=%d homing=%d res=%d,%d mode=%u cadd=0x%x "
                     "sig=%016" PRIx64 " hot_exact=%d "
                     "steps=%" PRIu64
                     " conv=%" PRIu64 " giveup=%" PRIu64 " hot_seen=%" PRIu64,
                     seq, rx, ry, have, p->hot_x, p->hot_y, p->gx, p->gy,
                     p->qlen, p->ta_active, p->homing, p->res_x, p->res_y,
                     s->xreg[MGA_XCURCTRL] & 3,
                     ((((uint32_t)s->xreg[MGA_XCURADDH] << 8) |
                       s->xreg[MGA_XCURADDL]) << 10),
                     p->cur_sig, p->hot_exact,
                     p->steps, p->converged, p->gaveup, p->hot_seen);
        return;
    }

    mga_ptr_send(s, "%llu ERR verb", seq);
}

static int mga_ptr_can_read(void *opaque)
{
    MGAState *s = opaque;

    return PTR_RXMAX - s->ptr.rxlen;
}

static void mga_ptr_read(void *opaque, const uint8_t *buf, int size)
{
    MGAState *s = opaque;
    MGAPtrLoop *p = &s->ptr;
    int i;

    for (i = 0; i < size; i++) {
        char c = buf[i];

        if (c == '\n' || c == '\r') {
            if (p->rxlen) {
                p->rx[p->rxlen] = '\0';
                mga_ptr_line(s, p->rx);
                p->rxlen = 0;
            }
            continue;
        }
        if (p->rxlen < PTR_RXMAX - 1) {
            p->rx[p->rxlen++] = c;
        } else {
            p->rxlen = 0;   /* oversized line: drop it whole */
            mga_ptr_send(s, "0 ERR line-too-long");
        }
    }
}

static void mga_ptr_event(void *opaque, QEMUChrEvent event)
{
    MGAState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->ptr.open = true;
        s->ptr.rxlen = 0;
        mga_ptr_rebase(s);
        mga_ptr_home_start(s);
        mga_ptr_send(s, "HELLO mgaptr/1 caps=movea,btn,sync,stat,home surf=%dx%d",
                     mga_ptr_surf_w(s), mga_ptr_surf_h(s));
        mga_ptr_trace(s, "client connected");
        break;
    case CHR_EVENT_CLOSED:
        /* A dropped client must not strand a button down on the guest. */
        mga_ptr_release_all(s);
        s->ptr.open = false;
        s->ptr.rxlen = 0;
        mga_ptr_qclear(s);
        s->ptr.ta_active = false;
        s->ptr.homing = false;
        mga_ptr_trace(s, "client gone");
        break;
    default:
        break;
    }
}

/*
 * The window runs on VIRTUAL time, so it stops with the guest.  A paused
 * station (idle-pause is aggressive here: the guest is TCG and burns a
 * core whenever it runs) would otherwise leave every queued verb unacked
 * until the daemon declared the backend dead, so a stop finishes them
 * where they stand and a resume re-bases against the registers.
 */
static void mga_ptr_vm_state(void *opaque, bool running, RunState state)
{
    MGAState *s = opaque;

    if (!s->ptr.enabled) {
        return;
    }
    if (running) {
        mga_ptr_rebase(s);
        timer_mod(&s->ptr.timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (int64_t)s->ptr.window_ms * SCALE_MS);
    } else {
        timer_del(&s->ptr.timer);
        while (s->ptr.qlen) {
            MGAPtrVerb *v = &s->ptr.q[s->ptr.qhead];
            if (v->seq) {
                mga_ptr_send(s, "%" PRIu64 " OK paused=1", v->seq);
            }
            mga_ptr_qpop(s);
        }
        s->ptr.ta_active = false;
    }
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
    case MGA_XCURADDL:
    case MGA_XCURADDH:
    case MGA_XCURCOL0RED ... MGA_XCURCOL0RED + 2:
    case MGA_XCURCOL1RED ... MGA_XCURCOL1RED + 2:
        /* shape, colours or enable changed in place: repaint the sprite */
        mga_cursor_invalidate_rect(s, mga_cursor_y(s));
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
        if (s->ptr.trace_pos) {
            static const char *nm[4] = { "XL", "XH", "YL", "YH" };
            fprintf(stderr, "mgapos: %s=0x%02x pos=%d,%d sig=%016llx\n",
                    nm[off & 3], val, mga_cursor_x(s), mga_cursor_y(s),
                    (unsigned long long)mga_ptr_shape_sig(s));
        }
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
#define DWGCTL_ARZERO   (1u << 12)
#define DWGCTL_SGNZERO  (1u << 13)
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
    /* sgnzero: the SGN register contents are ignored (treated as 0) */
    uint32_t sgn = (dwgctl & DWGCTL_SGNZERO) ? 0 : dwg32(s, DWG_SGN);
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
    s->hw_cursor_size = 0;
    s->hw_cursor_last_x = 0;
    s->hw_cursor_last_y = 0;
    if (s->ptr.enabled) {
        mga_ptr_rebase(s);
    }
    timer_del(&s->vline_timer);
}

/* After loadvm the converted 256-colour palette must be rebuilt, and one input
 * to it is NOT part of the migration stream.
 *
 * VGACommonState::dac_8bit is a DERIVED field: vga.c sets it only as a side
 * effect of a VBE_DISPI_INDEX_ENABLE write (and clears it on reset), and
 * vmstate_vga_common does not carry it. So a restored guest always came back
 * with dac_8bit == 0, i.e. a 6-bit VGA DAC, and update_palette256() shifted
 * every component left by 2. On this station that is highly visible: AIX runs
 * CDE at 1024x768x8 PseudoColor with an 8-bit DAC, so a restored desktop came
 * back structurally perfect but colour-crushed -- greys to black, mid tones to
 * saturated magenta/cyan/yellow.
 *
 * The authoritative bit is already in the stream, in our own xreg[] copy of
 * XMISCCTRL, so re-derive it here instead of growing the migration format.
 * That deliberately keeps the wire format unchanged: checkpoints taken before
 * this fix still load, and now restore with correct colour.
 */
static int mga_post_load(void *opaque, int version_id)
{
    MGAState *s = opaque;

    s->vga.dac_8bit = (s->xreg[MGA_XMISCCTRL] & MGA_XMISCCTRL_DAC_8BIT) != 0;
    /* Force a full redraw so the palette is re-converted for every pixel. */
    if (s->vga.con) {
        graphic_hw_invalidate(s->vga.con);
    }
    /* The restored CURPOSX/Y ARE the restored pointer, but nothing in the
     * stream says which cursor GLYPH is in force, so the closed loop
     * re-bases its hotspot and its beliefs against the registers. */
    if (s->ptr.enabled) {
        mga_ptr_rebase(s);
    }
    return 0;
}

static const VMStateDescription vmstate_mga = {
    .name = TYPE_MGA,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = mga_post_load,
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
    s->vga.cursor_invalidate = mga_cursor_invalidate;
    s->vga.cursor_draw_line = mga_cursor_draw_line;

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

    /* Closed-loop pointer: armed only when a control chardev was given. */
    if (qemu_chr_fe_backend_connected(&s->ptr.chr)) {
        if (s->ptr.window_ms < 1) {
            s->ptr.window_ms = 1;
        }
        s->ptr.enabled = true;
        timer_init_ns(&s->ptr.timer, QEMU_CLOCK_VIRTUAL, mga_ptr_tick, s);
        s->ptr.vmse = qemu_add_vm_change_state_handler(mga_ptr_vm_state, s);
        qemu_chr_fe_set_handlers(&s->ptr.chr, mga_ptr_can_read, mga_ptr_read,
                                 mga_ptr_event, NULL, s, NULL, true);
        mga_ptr_rebase(s);
        timer_mod(&s->ptr.timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  (int64_t)s->ptr.window_ms * SCALE_MS);
    }
}

static void mga_exit(PCIDevice *dev)
{
    MGAState *s = MGA(dev);

    timer_del(&s->vline_timer);
    if (s->ptr.enabled) {
        timer_del(&s->ptr.timer);
        if (s->ptr.vmse) {
            qemu_del_vm_change_state_handler(s->ptr.vmse);
            s->ptr.vmse = NULL;
        }
    }
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
    /* Closed-loop 1:1 pointer. Unset ptrctl = the device behaves exactly as
     * it did before: no timer, no handlers, no injection. */
    DEFINE_PROP_CHR("ptrctl", MGAState, ptr.chr),
    DEFINE_PROP_UINT32("ptr-window-ms", MGAState, ptr.window_ms, 16),
    DEFINE_PROP_UINT32("ptr-deadband", MGAState, ptr.dead, 1),
    DEFINE_PROP_UINT32("ptr-move-step", MGAState, ptr.move_step, 48),
    DEFINE_PROP_UINT32("ptr-tries", MGAState, ptr.tries, 40),
    DEFINE_PROP_UINT32("ptr-btn-gap-ms", MGAState, ptr.btn_gap_ms, 24),
    /* Seed for the step sizer, in hundredths of a px per PS/2 count. Too HIGH
     * only costs windows; too LOW overshoots, the one direction the
     * no-overshoot rule forbids -- so the default is the measured value on
     * this guest (AIX X acceleration, ~3 px/count), not 1. */
    DEFINE_PROP_UINT32("ptr-gain-x100", MGAState, ptr.gain_x100, 300),
    DEFINE_PROP_BOOL("ptr-hotspot", MGAState, ptr.track_hotspot, true),
    DEFINE_PROP_BOOL("ptr-trace", MGAState, ptr.trace, false),
    DEFINE_PROP_BOOL("ptr-trace-pos", MGAState, ptr.trace_pos, false),
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
