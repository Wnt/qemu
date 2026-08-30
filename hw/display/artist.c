/*
 * QEMU HP Artist Emulation
 *
 * Copyright (c) 2019-2022 Sven Schnelle <svens@stackframe.org>
 * Copyright (c) 2022 Helge Deller <deller@gmx.de>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "hw/core/qdev.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "trace.h"
#include "framebuffer.h"
#include "qom/object.h"
#include "chardev/char-fe.h"
#include "hw/core/qdev-properties-system.h"
#include "qemu/timer.h"
#include "ui/input.h"
#include "system/runstate.h"

#define TYPE_ARTIST "artist"
OBJECT_DECLARE_SIMPLE_TYPE(ARTISTState, ARTIST)

struct vram_buffer {
    MemoryRegion mr;
    uint8_t *data;
    unsigned int size;
    unsigned int width;
    unsigned int height;
};

#define PTR_GLYPH_SLOTS     16
#define PTR_EDGE_CAP        64

struct ARTISTState {
    SysBusDevice parent_obj;

    QemuConsole *con;
    MemoryRegion vram_mem;
    MemoryRegion mem_as_root;
    MemoryRegion reg;
    MemoryRegionSection fbsection;

    void *vram_int_mr;
    AddressSpace as;

    struct vram_buffer vram_buffer[16];

    bool disable;
    uint16_t width;
    uint16_t height;
    uint16_t depth;

    uint32_t fg_color;
    uint32_t bg_color;

    uint32_t vram_char_y;
    uint32_t vram_bitmask;

    uint32_t vram_start;
    uint32_t vram_pos;

    uint32_t vram_size;

    uint32_t blockmove_source;
    uint32_t blockmove_dest;
    uint32_t blockmove_size;

    uint32_t line_size;
    uint32_t line_end;
    uint32_t line_xy;
    uint32_t line_pattern_start;
    uint32_t line_pattern_skip;

    uint32_t cursor_pos;
    uint32_t cursor_cntrl;

    uint32_t cursor_height;
    uint32_t cursor_width;

    uint32_t plane_mask;

    uint32_t reg_100080;
    uint32_t horiz_backporch;
    uint32_t active_lines_low;
    uint32_t misc_video;
    uint32_t misc_ctrl;

    uint32_t dst_bm_access;
    uint32_t src_bm_access;
    uint32_t control_plane;
    uint32_t transfer_data;
    uint32_t image_bitmap_op;

    uint32_t font_write1;
    uint32_t font_write2;
    uint32_t font_write_pos_y;

    int draw_line_pattern;

    /*
     * ---- closed-loop pointer engine (kernel-hive) ----
     * DELIBERATELY ABSENT FROM vmstate_artist. Every field is re-derived from
     * registers the guest owns or from the live control socket, so arming the
     * loop does not change the migration format and the station's golden
     * checkpoint keeps restoring. Do not migrate any of it.
     */
    CharFrontend ptrctl;
    QEMUTimer *ptr_timer;
    bool ptr_open;
    char ptr_rx[256];
    int ptr_rxlen;

    uint32_t ptr_window_ms;
    uint32_t ptr_deadband;
    uint32_t ptr_move_step;
    uint32_t ptr_tries;
    uint32_t ptr_btn_gap_ms;
    uint32_t ptr_gain_x100;
    bool ptr_trace;
    bool ptr_trace_pos;

    bool ptr_have_target;
    int ptr_tx, ptr_ty;
    int ptr_win;
    int ptr_osc;
    int ptr_sign_x, ptr_sign_y;
    int ptr_infl_x, ptr_infl_y;
    int ptr_wait;
    int ptr_last_x, ptr_last_y;
    bool ptr_have_last;

    int ptr_hot_x, ptr_hot_y;
    bool ptr_hot_exact;
    uint32_t ptr_last_sig;
    uint32_t ptr_glyph_sig[PTR_GLYPH_SLOTS];
    int ptr_glyph_hx[PTR_GLYPH_SLOTS];
    int ptr_glyph_hy[PTR_GLYPH_SLOTS];

    int ptr_home_state;
    int ptr_home_win;
    int ptr_home_kick;
    int ptr_home_still;
    int ptr_home_lx, ptr_home_ly;
    bool ptr_home_moved;

    uint8_t ptr_edge_btn[PTR_EDGE_CAP];
    bool ptr_edge_down[PTR_EDGE_CAP];
    uint64_t ptr_edge_seq[PTR_EDGE_CAP];
    unsigned ptr_edge_head, ptr_edge_tail;
    uint32_t ptr_btn_state;
    int64_t ptr_edge_gap_until;

    uint32_t ptr_reaims;
    uint32_t ptr_giveups;
};

/* hardware allows up to 64x64, but we emulate 32x32 only. */
#define NGLE_MAX_SPRITE_SIZE    32

typedef enum {
    ARTIST_BUFFER_AP = 1,
    ARTIST_BUFFER_OVERLAY = 2,
    ARTIST_BUFFER_CURSOR1 = 6,
    ARTIST_BUFFER_CURSOR2 = 7,
    ARTIST_BUFFER_ATTRIBUTE = 13,
    ARTIST_BUFFER_CMAP = 15,
} artist_buffer_t;

typedef enum {
    VRAM_IDX = 0x1004a0,
    VRAM_BITMASK = 0x1005a0,
    VRAM_WRITE_INCR_X = 0x100600,
    VRAM_WRITE_INCR_X2 = 0x100604,
    VRAM_WRITE_INCR_Y = 0x100620,
    VRAM_START = 0x100800,
    BLOCK_MOVE_SIZE = 0x100804,
    BLOCK_MOVE_SOURCE = 0x100808,
    TRANSFER_DATA = 0x100820,
    FONT_WRITE_INCR_Y = 0x1008a0,
    VRAM_START_TRIGGER = 0x100a00,
    VRAM_SIZE_TRIGGER = 0x100a04,
    FONT_WRITE_START = 0x100aa0,
    BLOCK_MOVE_DEST_TRIGGER = 0x100b00,
    BLOCK_MOVE_SIZE_TRIGGER = 0x100b04,
    LINE_XY = 0x100ccc,
    PATTERN_LINE_START = 0x100ecc,
    LINE_SIZE = 0x100e04,
    LINE_END = 0x100e44,
    DST_SRC_BM_ACCESS = 0x118000,
    DST_BM_ACCESS = 0x118004,
    SRC_BM_ACCESS = 0x118008,
    CONTROL_PLANE = 0x11800c,
    FG_COLOR = 0x118010,
    BG_COLOR = 0x118014,
    PLANE_MASK = 0x118018,
    IMAGE_BITMAP_OP = 0x11801c,
    CURSOR_POS = 0x300100,      /* reg17 */
    CURSOR_CTRL = 0x300104,     /* reg18 */
    MISC_VIDEO = 0x300218,      /* reg21 */
    MISC_CTRL = 0x300308,       /* reg27 */
    HORIZ_BACKPORCH = 0x300200, /* reg19 */
    ACTIVE_LINES_LOW = 0x300208,/* reg20 */
    FIFO1 = 0x300008,           /* reg34 */
    FIFO2 = 0x380008,
} artist_reg_t;

typedef enum {
    ARTIST_ROP_CLEAR = 0,
    ARTIST_ROP_COPY = 3,
    ARTIST_ROP_XOR = 6,
    ARTIST_ROP_NOT_DST = 10,
    ARTIST_ROP_SET = 15,
} artist_rop_t;

#define REG_NAME(_x) case _x: return " "#_x;
static const char *artist_reg_name(uint64_t addr)
{
    switch ((artist_reg_t)addr) {
    REG_NAME(VRAM_IDX);
    REG_NAME(VRAM_BITMASK);
    REG_NAME(VRAM_WRITE_INCR_X);
    REG_NAME(VRAM_WRITE_INCR_X2);
    REG_NAME(VRAM_WRITE_INCR_Y);
    REG_NAME(VRAM_START);
    REG_NAME(BLOCK_MOVE_SIZE);
    REG_NAME(BLOCK_MOVE_SOURCE);
    REG_NAME(FG_COLOR);
    REG_NAME(BG_COLOR);
    REG_NAME(PLANE_MASK);
    REG_NAME(VRAM_START_TRIGGER);
    REG_NAME(VRAM_SIZE_TRIGGER);
    REG_NAME(BLOCK_MOVE_DEST_TRIGGER);
    REG_NAME(BLOCK_MOVE_SIZE_TRIGGER);
    REG_NAME(TRANSFER_DATA);
    REG_NAME(CONTROL_PLANE);
    REG_NAME(IMAGE_BITMAP_OP);
    REG_NAME(DST_SRC_BM_ACCESS);
    REG_NAME(DST_BM_ACCESS);
    REG_NAME(SRC_BM_ACCESS);
    REG_NAME(CURSOR_POS);
    REG_NAME(CURSOR_CTRL);
    REG_NAME(HORIZ_BACKPORCH);
    REG_NAME(ACTIVE_LINES_LOW);
    REG_NAME(MISC_VIDEO);
    REG_NAME(MISC_CTRL);
    REG_NAME(LINE_XY);
    REG_NAME(PATTERN_LINE_START);
    REG_NAME(LINE_SIZE);
    REG_NAME(LINE_END);
    REG_NAME(FONT_WRITE_INCR_Y);
    REG_NAME(FONT_WRITE_START);
    REG_NAME(FIFO1);
    REG_NAME(FIFO2);
    }
    return "";
}
#undef REG_NAME

static void artist_invalidate(void *opaque);

/* artist has a fixed line length of 2048 bytes. */
#define ADDR_TO_Y(addr) extract32(addr, 11, 11)
#define ADDR_TO_X(addr) extract32(addr, 0, 11)

static int16_t artist_get_x(uint32_t reg)
{
    return reg >> 16;
}

static int16_t artist_get_y(uint32_t reg)
{
    return reg & 0xffff;
}

static void artist_invalidate_lines(struct vram_buffer *buf,
                                    int starty, int height)
{
    int start = starty * buf->width;
    int size;

    if (starty + height > buf->height) {
        height = buf->height - starty;
    }

    size = height * buf->width;

    if (start + size <= buf->size) {
        memory_region_set_dirty(&buf->mr, start, size);
    }
}

static int vram_write_bufidx(ARTISTState *s)
{
    return (s->dst_bm_access >> 12) & 0x0f;
}

static int vram_read_bufidx(ARTISTState *s)
{
    return (s->src_bm_access >> 12) & 0x0f;
}

static struct vram_buffer *vram_read_buffer(ARTISTState *s)
{
    return &s->vram_buffer[vram_read_bufidx(s)];
}

static struct vram_buffer *vram_write_buffer(ARTISTState *s)
{
    return &s->vram_buffer[vram_write_bufidx(s)];
}

static uint8_t artist_get_color(ARTISTState *s)
{
    if (s->image_bitmap_op & 2) {
        return s->fg_color;
    } else {
        return s->bg_color;
    }
}

static artist_rop_t artist_get_op(ARTISTState *s)
{
    return (s->image_bitmap_op >> 8) & 0xf;
}

static void artist_rop8(ARTISTState *s, struct vram_buffer *buf,
                        unsigned int offset, uint8_t val)
{
    const artist_rop_t op = artist_get_op(s);
    uint8_t plane_mask;
    uint8_t *dst;

    if (offset >= buf->size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rop8 offset:%u bufsize:%u\n", offset, buf->size);
        return;
    }
    dst = buf->data + offset;
    plane_mask = s->plane_mask & 0xff;

    switch (op) {
    case ARTIST_ROP_CLEAR:
        *dst &= ~plane_mask;
        break;

    case ARTIST_ROP_COPY:
        *dst = (*dst & ~plane_mask) | (val & plane_mask);
        break;

    case ARTIST_ROP_XOR:
        *dst ^= val & plane_mask;
        break;

    case ARTIST_ROP_NOT_DST:
        *dst ^= plane_mask;
        break;

    case ARTIST_ROP_SET:
        *dst |= plane_mask;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unsupported rop %d\n", __func__, op);
        break;
    }
}

static void artist_get_cursor_pos(ARTISTState *s, int *x, int *y)
{
    /*
     * The emulated Artist graphic is like a CRX graphic, and as such
     * it's usually fixed at 1280x1024 pixels.
     * Other resolutions may work, but no guarantee.
     */

    unsigned int hbp_times_vi, horizBackPorch;
    int16_t xHi, xLo;
    const int videoInterleave = 4;
    const int pipelineDelay = 4;

    /* ignore if uninitialized */
    if (s->cursor_pos == 0) {
        *x = *y = 0;
        return;
    }

    /*
     * Calculate X position based on backporch and interleave values.
     * Based on code from Xorg X11R6.6
     */
    horizBackPorch = ((s->horiz_backporch & 0xff0000) >> 16) +
                     ((s->horiz_backporch & 0xff00) >> 8) + 2;
    hbp_times_vi = horizBackPorch * videoInterleave;
    xHi = s->cursor_pos >> 19;
    *x = ((xHi + pipelineDelay) * videoInterleave) - hbp_times_vi;

    xLo = (s->cursor_pos >> 16) & 0x07;
    *x += ((xLo - hbp_times_vi) & (videoInterleave - 1)) + 8 - 1;

    /* subtract cursor offset from cursor control register */
    *x -= (s->cursor_cntrl & 0xf0) >> 4;

    /* Calculate Y position */
    *y = s->height - artist_get_y(s->cursor_pos);
    *y -= (s->cursor_cntrl & 0x0f);

    if (*x > s->width) {
        *x = s->width;
    }

    if (*y > s->height) {
        *y = s->height;
    }
}

static inline bool cursor_visible(ARTISTState *s)
{
    /* cursor is visible if bit 0x80 is set in cursor_cntrl */
    return s->cursor_cntrl & 0x80;
}

static void artist_invalidate_cursor(ARTISTState *s)
{
    int x, y;

    if (!cursor_visible(s)) {
        return;
    }

    artist_get_cursor_pos(s, &x, &y);
    artist_invalidate_lines(&s->vram_buffer[ARTIST_BUFFER_AP],
                            y, s->cursor_height);
}

static void block_move(ARTISTState *s,
                       unsigned int source_x, unsigned int source_y,
                       unsigned int dest_x,   unsigned int dest_y,
                       unsigned int width,    unsigned int height)
{
    struct vram_buffer *buf;
    int line, endline, lineincr, startcolumn, endcolumn, columnincr, column;
    unsigned int dst, src;

    trace_artist_block_move(source_x, source_y, dest_x, dest_y, width, height);

    if (s->control_plane != 0) {
        /* We don't support CONTROL_PLANE accesses */
        qemu_log_mask(LOG_UNIMP, "%s: CONTROL_PLANE: %08x\n", __func__,
                      s->control_plane);
        return;
    }

    buf = &s->vram_buffer[ARTIST_BUFFER_AP];
    if (height > buf->height) {
        height = buf->height;
    }
    if (width > buf->width) {
        width = buf->width;
    }

    if (dest_y > source_y) {
        /* move down */
        line = height - 1;
        endline = -1;
        lineincr = -1;
    } else {
        /* move up */
        line = 0;
        endline = height;
        lineincr = 1;
    }

    if (dest_x > source_x) {
        /* move right */
        startcolumn = width - 1;
        endcolumn = -1;
        columnincr = -1;
    } else {
        /* move left */
        startcolumn = 0;
        endcolumn = width;
        columnincr = 1;
    }

    for ( ; line != endline; line += lineincr) {
        src = source_x + ((line + source_y) * buf->width) + startcolumn;
        dst = dest_x + ((line + dest_y) * buf->width) + startcolumn;

        for (column = startcolumn; column != endcolumn; column += columnincr) {
            if (dst >= buf->size || src >= buf->size) {
                continue;
            }
            artist_rop8(s, buf, dst, buf->data[src]);
            src += columnincr;
            dst += columnincr;
        }
    }

    artist_invalidate_lines(buf, dest_y, height);
}

static void fill_window(ARTISTState *s,
                        unsigned int startx, unsigned int starty,
                        unsigned int width,  unsigned int height)
{
    unsigned int offset;
    uint8_t color = artist_get_color(s);
    struct vram_buffer *buf;
    int x, y;

    trace_artist_fill_window(startx, starty, width, height,
                             s->image_bitmap_op, s->control_plane);

    if (s->control_plane != 0) {
        /* We don't support CONTROL_PLANE accesses */
        qemu_log_mask(LOG_UNIMP, "%s: CONTROL_PLANE: %08x\n", __func__,
                      s->control_plane);
        return;
    }

    if (s->reg_100080 == 0x7d) {
        /*
         * Not sure what this register really does, but
         * 0x7d seems to enable autoincremt of the Y axis
         * by the current block move height.
         */
        height = artist_get_y(s->blockmove_size);
        s->vram_start += height;
    }

    buf = &s->vram_buffer[ARTIST_BUFFER_AP];

    for (y = starty; y < starty + height; y++) {
        offset = y * s->width;

        for (x = startx; x < startx + width; x++) {
            artist_rop8(s, buf, offset + x, color);
        }
    }
    artist_invalidate_lines(buf, starty, height);
}

static void draw_line(ARTISTState *s,
                      unsigned int x1, unsigned int y1,
                      unsigned int x2, unsigned int y2,
                      bool update_start, int skip_pix, int max_pix)
{
    struct vram_buffer *buf = &s->vram_buffer[ARTIST_BUFFER_AP];
    uint8_t color;
    int dx, dy, t, e, x, y, incy, diago, horiz;
    bool c1;

    trace_artist_draw_line(x1, y1, x2, y2);

    if ((x1 >= buf->width && x2 >= buf->width) ||
        (y1 >= buf->height && y2 >= buf->height)) {
        return;
    }

    if (update_start) {
        s->vram_start = (x2 << 16) | y2;
    }

    if (x2 > x1) {
        dx = x2 - x1;
    } else {
        dx = x1 - x2;
    }
    if (y2 > y1) {
        dy = y2 - y1;
    } else {
        dy = y1 - y2;
    }

    c1 = false;
    if (dy > dx) {
        t = y2;
        y2 = x2;
        x2 = t;

        t = y1;
        y1 = x1;
        x1 = t;

        t = dx;
        dx = dy;
        dy = t;

        c1 = true;
    }

    if (x1 > x2) {
        t = y2;
        y2 = y1;
        y1 = t;

        t = x1;
        x1 = x2;
        x2 = t;
    }

    horiz = dy << 1;
    diago = (dy - dx) << 1;
    e = (dy << 1) - dx;

    if (y1 <= y2) {
        incy = 1;
    } else {
        incy = -1;
    }
    x = x1;
    y = y1;
    color = artist_get_color(s);

    do {
        unsigned int ofs;

        if (c1) {
            ofs = x * s->width + y;
        } else {
            ofs = y * s->width + x;
        }

        if (skip_pix > 0) {
            skip_pix--;
        } else {
            artist_rop8(s, buf, ofs, color);
        }

        if (e > 0) {
            y  += incy;
            e  += diago;
        } else {
            e += horiz;
        }
        x++;
    } while (x <= x2 && (max_pix == -1 || --max_pix > 0));

    if (c1) {
        artist_invalidate_lines(buf, x1, x2 - x1);
    } else {
        artist_invalidate_lines(buf, y1 > y2 ? y2 : y1, x2 - x1);
    }
}

static void draw_line_pattern_start(ARTISTState *s)
{
    int startx = artist_get_x(s->vram_start);
    int starty = artist_get_y(s->vram_start);
    int endx = artist_get_x(s->blockmove_size);
    int endy = artist_get_y(s->blockmove_size);
    int pstart = s->line_pattern_start >> 16;

    draw_line(s, startx, starty, endx, endy, false, -1, pstart);
    s->line_pattern_skip = pstart;
}

static void draw_line_pattern_next(ARTISTState *s)
{
    int startx = artist_get_x(s->vram_start);
    int starty = artist_get_y(s->vram_start);
    int endx = artist_get_x(s->blockmove_size);
    int endy = artist_get_y(s->blockmove_size);
    int line_xy = s->line_xy >> 16;

    draw_line(s, startx, starty, endx, endy, false, s->line_pattern_skip,
              s->line_pattern_skip + line_xy);
    s->line_pattern_skip += line_xy;
    s->image_bitmap_op ^= 2;
}

static void draw_line_size(ARTISTState *s, bool update_start)
{
    int startx = artist_get_x(s->vram_start);
    int starty = artist_get_y(s->vram_start);
    int endx = artist_get_x(s->line_size);
    int endy = artist_get_y(s->line_size);

    draw_line(s, startx, starty, endx, endy, update_start, -1, -1);
}

static void draw_line_xy(ARTISTState *s, bool update_start)
{
    int startx = artist_get_x(s->vram_start);
    int starty = artist_get_y(s->vram_start);
    int sizex = artist_get_x(s->blockmove_size);
    int sizey = artist_get_y(s->blockmove_size);
    int linexy = s->line_xy >> 16;
    int endx, endy;

    endx = startx;
    endy = starty;

    if (sizex > 0) {
        endx = startx + linexy;
    }

    if (sizex < 0) {
        endx = startx;
        startx -= linexy;
    }

    if (sizey > 0) {
        endy = starty + linexy;
    }

    if (sizey < 0) {
        endy = starty;
        starty -= linexy;
    }

    if (startx < 0) {
        startx = 0;
    }

    if (endx < 0) {
        endx = 0;
    }

    if (starty < 0) {
        starty = 0;
    }

    if (endy < 0) {
        endy = 0;
    }

    draw_line(s, startx, starty, endx, endy, false, -1, -1);
}

static void draw_line_end(ARTISTState *s, bool update_start)
{
    int startx = artist_get_x(s->vram_start);
    int starty = artist_get_y(s->vram_start);
    int endx = artist_get_x(s->line_end);
    int endy = artist_get_y(s->line_end);

    draw_line(s, startx, starty, endx, endy, update_start, -1, -1);
}

static void font_write16(ARTISTState *s, uint16_t val)
{
    struct vram_buffer *buf;
    uint32_t color = (s->image_bitmap_op & 2) ? s->fg_color : s->bg_color;
    uint16_t mask;
    int i;

    unsigned int startx = artist_get_x(s->vram_start);
    unsigned int starty = artist_get_y(s->vram_start) + s->font_write_pos_y;
    unsigned int offset = starty * s->width + startx;

    buf = &s->vram_buffer[ARTIST_BUFFER_AP];

    if (startx >= buf->width || starty >= buf->height ||
        offset + 16 >= buf->size) {
        return;
    }

    for (i = 0; i < 16; i++) {
        mask = 1 << (15 - i);
        if (val & mask) {
            artist_rop8(s, buf, offset + i, color);
        } else {
            if (!(s->image_bitmap_op & 0x20000000)) {
                artist_rop8(s, buf, offset + i, s->bg_color);
            }
        }
    }
    artist_invalidate_lines(buf, starty, 1);
}

static void font_write(ARTISTState *s, uint32_t val)
{
    font_write16(s, val >> 16);
    if (++s->font_write_pos_y == artist_get_y(s->blockmove_size)) {
        s->vram_start += (s->blockmove_size & 0xffff0000);
        return;
    }

    font_write16(s, val & 0xffff);
    if (++s->font_write_pos_y == artist_get_y(s->blockmove_size)) {
        s->vram_start += (s->blockmove_size & 0xffff0000);
        return;
    }
}

static void combine_write_reg(hwaddr addr, uint64_t val, int size, void *out)
{
    /*
     * FIXME: is there a qemu helper for this?
     */

#if !HOST_BIG_ENDIAN
    addr ^= 3;
#endif

    switch (size) {
    case 1:
        *(uint8_t *)(out + (addr & 3)) = val;
        break;

    case 2:
        *(uint16_t *)(out + (addr & 2)) = val;
        break;

    case 4:
        *(uint32_t *)out = val;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "unsupported write size: %d\n", size);
    }
}

static void artist_vram_write4(ARTISTState *s, struct vram_buffer *buf,
                               uint32_t offset, uint32_t data)
{
    int i;
    int mask = s->vram_bitmask >> 28;

    for (i = 0; i < 4; i++) {
        if (!(s->image_bitmap_op & 0x20000000) || (mask & 8)) {
            artist_rop8(s, buf, offset + i, data >> 24);
            data <<= 8;
            mask <<= 1;
        }
    }
    memory_region_set_dirty(&buf->mr, offset, 3);
}

static void artist_vram_write32(ARTISTState *s, struct vram_buffer *buf,
                                uint32_t offset, int size, uint32_t data,
                                int fg, int bg)
{
    uint32_t mask, vram_bitmask = s->vram_bitmask >> ((4 - size) * 8);
    int i, pix_count = size * 8;

    for (i = 0; i < pix_count && offset + i < buf->size; i++) {
        mask = 1 << (pix_count - 1 - i);

        if (!(s->image_bitmap_op & 0x20000000) || (vram_bitmask & mask)) {
            if (data & mask) {
                artist_rop8(s, buf, offset + i, fg);
            } else {
                if (!(s->image_bitmap_op & 0x10000002)) {
                    artist_rop8(s, buf, offset + i, bg);
                }
            }
        }
    }
    memory_region_set_dirty(&buf->mr, offset, pix_count);
}

static int get_vram_offset(ARTISTState *s, struct vram_buffer *buf,
                           int pos, int posy)
{
    unsigned int posx, width;

    width = buf->width;
    posx = ADDR_TO_X(pos);
    posy += ADDR_TO_Y(pos);
    return posy * width + posx;
}

static int vram_bit_write(ARTISTState *s, uint32_t pos, int posy,
                          uint32_t data, int size)
{
    struct vram_buffer *buf = vram_write_buffer(s);

    switch (s->dst_bm_access >> 16) {
    case 0x3ba0:
    case 0xbbe0:
        artist_vram_write4(s, buf, pos, bswap32(data));
        pos += 4;
        break;

    case 0x1360: /* linux */
        artist_vram_write4(s, buf, get_vram_offset(s, buf, pos, posy), data);
        pos += 4;
        break;

    case 0x13a0:
        artist_vram_write4(s, buf, get_vram_offset(s, buf, pos >> 2, posy),
                           data);
        pos += 16;
        break;

    case 0x2ea0:
        artist_vram_write32(s, buf, get_vram_offset(s, buf, pos >> 2, posy),
                            size, data, s->fg_color, s->bg_color);
        pos += 4;
        break;

    case 0x28a0:
        artist_vram_write32(s, buf, get_vram_offset(s, buf, pos >> 2, posy),
                            size, data, 1, 0);
        pos += 4;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unknown dst bm access %08x\n",
                      __func__, s->dst_bm_access);
        break;
    }

    if (vram_write_bufidx(s) == ARTIST_BUFFER_CURSOR1 ||
        vram_write_bufidx(s) == ARTIST_BUFFER_CURSOR2) {
        artist_invalidate_cursor(s);
    }
    return pos;
}

static void artist_vram_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    ARTISTState *s = opaque;

    s->vram_char_y = 0;
    trace_artist_vram_write(size, addr, val);
    vram_bit_write(opaque, addr, 0, val, size);
}

static uint64_t artist_vram_read(void *opaque, hwaddr addr, unsigned size)
{
    ARTISTState *s = opaque;
    struct vram_buffer *buf;
    unsigned int offset;
    uint64_t val;

    buf = vram_read_buffer(s);
    if (!buf->size) {
        return 0;
    }

    offset = get_vram_offset(s, buf, addr >> 2, 0);

    if (offset > buf->size) {
        return 0;
    }

    switch (s->src_bm_access >> 16) {
    case 0x3ba0:
        val = *(uint32_t *)(buf->data + offset);
        break;

    case 0x13a0:
    case 0x2ea0:
        val = bswap32(*(uint32_t *)(buf->data + offset));
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unknown src bm access %08x\n",
                      __func__, s->dst_bm_access);
        val = -1ULL;
        break;
    }
    trace_artist_vram_read(size, addr, val);
    return val;
}

static void artist_reg_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ARTISTState *s = opaque;
    int width, height;
    uint64_t oldval;

    trace_artist_reg_write(size, addr, artist_reg_name(addr & ~3ULL), val);

    switch (addr & ~3ULL) {
    case 0x100080:
        combine_write_reg(addr, val, size, &s->reg_100080);
        break;

    case FG_COLOR:
        combine_write_reg(addr, val, size, &s->fg_color);
        break;

    case BG_COLOR:
        combine_write_reg(addr, val, size, &s->bg_color);
        break;

    case VRAM_BITMASK:
        combine_write_reg(addr, val, size, &s->vram_bitmask);
        break;

    case VRAM_WRITE_INCR_Y:
        vram_bit_write(s, s->vram_pos, s->vram_char_y++, val, size);
        break;

    case VRAM_WRITE_INCR_X:
    case VRAM_WRITE_INCR_X2:
        s->vram_pos = vram_bit_write(s, s->vram_pos, s->vram_char_y, val, size);
        break;

    case VRAM_IDX:
        combine_write_reg(addr, val, size, &s->vram_pos);
        s->vram_char_y = 0;
        s->draw_line_pattern = 0;
        break;

    case VRAM_START:
        combine_write_reg(addr, val, size, &s->vram_start);
        s->draw_line_pattern = 0;
        break;

    case VRAM_START_TRIGGER:
        combine_write_reg(addr, val, size, &s->vram_start);
        fill_window(s, artist_get_x(s->vram_start),
                    artist_get_y(s->vram_start),
                    artist_get_x(s->blockmove_size),
                    artist_get_y(s->blockmove_size));
        break;

    case VRAM_SIZE_TRIGGER:
        combine_write_reg(addr, val, size, &s->vram_size);

        if (size == 2 && !(addr & 2)) {
            height = artist_get_y(s->blockmove_size);
        } else {
            height = artist_get_y(s->vram_size);
        }

        if (size == 2 && (addr & 2)) {
            width = artist_get_x(s->blockmove_size);
        } else {
            width = artist_get_x(s->vram_size);
        }

        fill_window(s, artist_get_x(s->vram_start),
                    artist_get_y(s->vram_start),
                    width, height);
        break;

    case LINE_XY:
        combine_write_reg(addr, val, size, &s->line_xy);
        if (s->draw_line_pattern) {
            draw_line_pattern_next(s);
        } else {
            draw_line_xy(s, true);
        }
        break;

    case PATTERN_LINE_START:
        combine_write_reg(addr, val, size, &s->line_pattern_start);
        s->draw_line_pattern = 1;
        draw_line_pattern_start(s);
        break;

    case LINE_SIZE:
        combine_write_reg(addr, val, size, &s->line_size);
        draw_line_size(s, true);
        break;

    case LINE_END:
        combine_write_reg(addr, val, size, &s->line_end);
        draw_line_end(s, true);
        break;

    case BLOCK_MOVE_SIZE:
        combine_write_reg(addr, val, size, &s->blockmove_size);
        break;

    case BLOCK_MOVE_SOURCE:
        combine_write_reg(addr, val, size, &s->blockmove_source);
        break;

    case BLOCK_MOVE_DEST_TRIGGER:
        combine_write_reg(addr, val, size, &s->blockmove_dest);

        block_move(s, artist_get_x(s->blockmove_source),
                   artist_get_y(s->blockmove_source),
                   artist_get_x(s->blockmove_dest),
                   artist_get_y(s->blockmove_dest),
                   artist_get_x(s->blockmove_size),
                   artist_get_y(s->blockmove_size));
        break;

    case BLOCK_MOVE_SIZE_TRIGGER:
        combine_write_reg(addr, val, size, &s->blockmove_size);

        block_move(s,
                   artist_get_x(s->blockmove_source),
                   artist_get_y(s->blockmove_source),
                   artist_get_x(s->vram_start),
                   artist_get_y(s->vram_start),
                   artist_get_x(s->blockmove_size),
                   artist_get_y(s->blockmove_size));
        break;

    case PLANE_MASK:
        combine_write_reg(addr, val, size, &s->plane_mask);
        break;

    case DST_SRC_BM_ACCESS:
        combine_write_reg(addr, val, size, &s->dst_bm_access);
        combine_write_reg(addr, val, size, &s->src_bm_access);
        break;

    case DST_BM_ACCESS:
        combine_write_reg(addr, val, size, &s->dst_bm_access);
        break;

    case SRC_BM_ACCESS:
        combine_write_reg(addr, val, size, &s->src_bm_access);
        break;

    case CONTROL_PLANE:
        combine_write_reg(addr, val, size, &s->control_plane);
        break;

    case TRANSFER_DATA:
        combine_write_reg(addr, val, size, &s->transfer_data);
        break;

    case HORIZ_BACKPORCH:
        /* overwrite HP-UX settings to fix X cursor position. */
        val = (NGLE_MAX_SPRITE_SIZE << 16) + (NGLE_MAX_SPRITE_SIZE << 8);
        combine_write_reg(addr, val, size, &s->horiz_backporch);
        break;

    case ACTIVE_LINES_LOW:
        combine_write_reg(addr, val, size, &s->active_lines_low);
        break;

    case MISC_VIDEO:
        oldval = s->misc_video;
        combine_write_reg(addr, val, size, &s->misc_video);
        /* Invalidate and hide screen if graphics signal is turned off. */
        if (((oldval & 0x0A000000) == 0x0A000000) &&
            ((val & 0x0A000000) != 0x0A000000)) {
            artist_invalidate(s);
        }
        /* Invalidate and redraw screen if graphics signal is turned back on. */
        if (((oldval & 0x0A000000) != 0x0A000000) &&
            ((val & 0x0A000000) == 0x0A000000)) {
            artist_invalidate(s);
        }
        break;

    case MISC_CTRL:
        combine_write_reg(addr, val, size, &s->misc_ctrl);
        break;

    case CURSOR_POS:
        artist_invalidate_cursor(s);
        combine_write_reg(addr, val, size, &s->cursor_pos);
        artist_invalidate_cursor(s);
        break;

    case CURSOR_CTRL:
        combine_write_reg(addr, val, size, &s->cursor_cntrl);
        break;

    case IMAGE_BITMAP_OP:
        combine_write_reg(addr, val, size, &s->image_bitmap_op);
        break;

    case FONT_WRITE_INCR_Y:
        combine_write_reg(addr, val, size, &s->font_write1);
        font_write(s, s->font_write1);
        break;

    case FONT_WRITE_START:
        combine_write_reg(addr, val, size, &s->font_write2);
        s->font_write_pos_y = 0;
        font_write(s, s->font_write2);
        break;

    case 300104:
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unknown register: reg=%08" HWADDR_PRIx
                      " val=%08" PRIx64 " size=%d\n",
                      __func__, addr, val, size);
        break;
    }
}

static uint64_t combine_read_reg(hwaddr addr, int size, void *in)
{
    /*
     * FIXME: is there a qemu helper for this?
     */

#if !HOST_BIG_ENDIAN
    addr ^= 3;
#endif

    switch (size) {
    case 1:
        return *(uint8_t *)(in + (addr & 3));

    case 2:
        return *(uint16_t *)(in + (addr & 2));

    case 4:
        return *(uint32_t *)in;

    default:
        qemu_log_mask(LOG_UNIMP, "unsupported read size: %d\n", size);
        return 0;
    }
}

static uint64_t artist_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    ARTISTState *s = opaque;
    uint32_t val = 0;

    switch (addr & ~3ULL) {
        /* Unknown status registers */
    case 0:
        break;

    case 0x211110:
        val = (s->width << 16) | s->height;
        if (s->depth == 1) {
            val |= 1 << 31;
        }
        break;

    case 0x100000:
    case 0x300000:
    case 0x300004:
    case 0x380000:
        break;

    case FIFO1:
    case FIFO2:
        /*
         * FIFO ready flag. we're not emulating the FIFOs
         * so we're always ready
         */
        val = 0x10;
        break;

    case HORIZ_BACKPORCH:
        val = s->horiz_backporch;
        break;

    case ACTIVE_LINES_LOW:
        val = s->active_lines_low;
        /* activeLinesLo for cursor is in reg20.b.b0 */
        val &= ~(0xff << 24);
        val |= (s->height & 0xff) << 24;
        break;

    case MISC_VIDEO:
        /* emulate V-blank */
        s->misc_video ^= 0x00040000;
        /* activeLinesHi for cursor is in reg21.b.b2 */
        val = s->misc_video;
        val &= ~0xff00UL;
        val |= (s->height & 0xff00);
        break;

    case MISC_CTRL:
        val = s->misc_ctrl;
        break;

    case 0x30023c:
        val = 0xac4ffdac;
        break;

    case 0x380004:
        /* magic number detected by SeaBIOS-hppa */
        val = s->disable ? 0 : 0x6dc20006;
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unknown register: %08" HWADDR_PRIx
                      " size %d\n", __func__, addr, size);
        break;
    }
    val = combine_read_reg(addr, size, &val);
    trace_artist_reg_read(size, addr, artist_reg_name(addr & ~3ULL), val);
    return val;
}

static const MemoryRegionOps artist_reg_ops = {
    .read = artist_reg_read,
    .write = artist_reg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps artist_vram_ops = {
    .read = artist_vram_read,
    .write = artist_vram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void artist_draw_cursor(ARTISTState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t *data = (uint32_t *)surface_data(surface);
    struct vram_buffer *cursor0, *cursor1 , *buf;
    int cx, cy, cursor_pos_x, cursor_pos_y;

    if (!cursor_visible(s)) {
        return;
    }

    cursor0 = &s->vram_buffer[ARTIST_BUFFER_CURSOR1];
    cursor1 = &s->vram_buffer[ARTIST_BUFFER_CURSOR2];
    buf = &s->vram_buffer[ARTIST_BUFFER_AP];

    artist_get_cursor_pos(s, &cursor_pos_x, &cursor_pos_y);

    for (cy = 0; cy < s->cursor_height; cy++) {

        for (cx = 0; cx < s->cursor_width; cx++) {

            if (cursor_pos_y + cy < 0 ||
                cursor_pos_x + cx < 0 ||
                cursor_pos_y + cy > buf->height - 1 ||
                cursor_pos_x + cx > buf->width) {
                continue;
            }

            int dstoffset = (cursor_pos_y + cy) * s->width +
                (cursor_pos_x + cx);

            if (cursor0->data[cy * cursor0->width + cx]) {
                data[dstoffset] = 0;
            } else {
                if (cursor1->data[cy * cursor1->width + cx]) {
                    data[dstoffset] = 0xffffff;
                }
            }
        }
    }
}

static bool artist_screen_enabled(ARTISTState *s)
{
    /*  We could check for (s->misc_ctrl & 0x00800000) too... */
    return ((s->misc_video & 0x0A000000) == 0x0A000000);
}

static void artist_draw_line(void *opaque, uint8_t *d, const uint8_t *src,
                             int width, int pitch)
{
    ARTISTState *s = ARTIST(opaque);
    uint32_t *cmap, *data = (uint32_t *)d;
    int x;

    if (!artist_screen_enabled(s)) {
        /* clear screen */
        memset(data, 0, s->width * sizeof(uint32_t));
        return;
    }

    cmap = (uint32_t *)(s->vram_buffer[ARTIST_BUFFER_CMAP].data + 0x400);

    for (x = 0; x < s->width; x++) {
        *data++ = cmap[*src++];
    }
}

static void artist_update_display(void *opaque)
{
    ARTISTState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int first = 0, last;

    framebuffer_update_display(surface, &s->fbsection, s->width, s->height,
                               s->width, s->width * 4, 0, 0, artist_draw_line,
                               s, &first, &last);

    artist_draw_cursor(s);

    if (first >= 0) {
        dpy_gfx_update(s->con, 0, first, s->width, last - first + 1);
    }
}

static void artist_invalidate(void *opaque)
{
    ARTISTState *s = ARTIST(opaque);
    struct vram_buffer *buf = &s->vram_buffer[ARTIST_BUFFER_AP];

    memory_region_set_dirty(&buf->mr, 0, buf->size);
}

static const GraphicHwOps artist_ops = {
    .invalidate  = artist_invalidate,
    .gfx_update = artist_update_display,
};

static void artist_initfn(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    ARTISTState *s = ARTIST(obj);

    memory_region_init_io(&s->reg, obj, &artist_reg_ops, s, "artist.reg",
                          4 * MiB);
    memory_region_init_io(&s->vram_mem, obj, &artist_vram_ops, s, "artist.vram",
                          8 * MiB);
    sysbus_init_mmio(sbd, &s->reg);
    sysbus_init_mmio(sbd, &s->vram_mem);
}

static void artist_create_buffer(ARTISTState *s, const char *name,
                                 hwaddr *offset, unsigned int idx,
                                 int width, int height)
{
    struct vram_buffer *buf = s->vram_buffer + idx;

    memory_region_init_ram(&buf->mr, OBJECT(s), name, width * height,
                           &error_fatal);
    memory_region_add_subregion_overlap(&s->mem_as_root, *offset, &buf->mr, 0);

    buf->data = memory_region_get_ram_ptr(&buf->mr);
    buf->size = height * width;
    buf->width = width;
    buf->height = height;

    *offset += buf->size;
}


/*
 * ============================================================================
 * kernel-hive: closed-loop 1:1 absolute pointer over the Artist hardware cursor
 * ============================================================================
 *
 * WHY THIS EXISTS. The daemon (streamhost) knows where the visitor pointed, in
 * guest pixels. The B160L has no absolute pointer path at all -- LASI PS/2,
 * relative only, no USB, no tablet -- so an absolute target can only be reached
 * by injecting relative counts and CHECKING WHERE THEY LANDED. HP-UX 10.20's X
 * server drives the Artist HARDWARE cursor, which means the guest continuously
 * publishes its own idea of the pointer position into CURSOR_POS/CURSOR_CTRL.
 * That is a sensor, so the control loop can close INSIDE the emulator:
 *
 *     reading = artist_get_cursor_pos()          (the DRAWN SPRITE ORIGIN)
 *     pointer = reading + hotspot
 *     err     = target - pointer - in_flight
 *     counts  = trunc(err / (gain * margin)), capped, ONE step per window
 *
 * `absolute: true` on this station is therefore EARNED BY MEASUREMENT, not
 * provided by a device.
 *
 * READ THE POSITION THROUGH artist_get_cursor_pos(), NEVER FROM THE RAW
 * REGISTERS. This is not a style preference, it is the bug that this port found
 * the hard way. CURSOR_CTRL's low nibbles ((&0xf0)>>4, &0x0f) are an OFFSET that
 * the accessor subtracts to reach the drawn sprite origin; they are NOT a
 * hotspot. A loop closed on a private decode of CURSOR_POS lands every target a
 * constant 8 px to the left, and -- this is the dangerous part -- the raw
 * register and the framebuffer still agree with each other EXACTLY at every
 * target, err +0,+0. Two observers agreeing is not proof; only the COMMANDED
 * TARGET is the third observer that separates "self-consistent" from "correct".
 *
 * NOTHING HERE IS IN VMSTATE. Every field below is derived from registers the
 * guest already owns or from the live socket, so vmstate_artist is untouched and
 * the station's golden checkpoint keeps restoring. Do not add a field here to
 * vmstate_artist -- that would force a golden re-bake for no gain.
 *
 * SINGLE INJECTOR. While the control socket is connected this engine OWNS the
 * guest pointer. No rel bridge, no QMP input-send-event, no labctl pointer
 * helper may run at the same time, or the two injectors fight and the loop reads
 * motion it did not cause.
 *
 * Wire dialect `artistptr/1`, spoken over a -chardev socket:
 *      <- HELLO artistptr/1 caps=movea,btn,sync,stat surf=1280x1024
 *      -> <seq> MOVEA <x> <y>        <- <seq> OK   (acks on target-ACCEPT)
 *      -> <seq> DOWN1|UP1|DOWN2|...  <- <seq> OK   (acks when the edge APPLIES)
 *      -> <seq> SYNC | STAT          <- <seq> OK [k=v ...]
 */

#define PTR_SPRITE_MAX      NGLE_MAX_SPRITE_SIZE  /* hotspot must live in here */
#define PTR_HOME_STILL      3       /* windows of stillness that mean "pinned" */
#define PTR_HOME_MAX_WIN    96      /* give-up bound, NOT a success criterion  */
#define PTR_INFL_DECAY      2       /* in-flight halves each window            */
#define PTR_OSC_LIMIT       6
/*
 * HARD IN-FLIGHT GATE. Never issue a step while the previous one is still on
 * the wire. TCG hppa absorbs PS/2 packets on its own schedule, so a window can
 * elapse with the last step unconsumed; without this gate the loop keeps
 * issuing against a stale reading, overshoots, reverses, trips the oscillation
 * latch and gives up several pixels short. BOUNDED, because an unbounded wait
 * wedges the pointer wherever the guest legitimately cannot move it -- at a
 * screen clamp the step never lands and the loop would never issue again.
 */
#define PTR_INFL_GATE_WIN   6       /* sign reversals that mean "accept here"  */

enum {
    PTR_HOME_IDLE = 0,
    PTR_HOME_KICK,      /* move AWAY from the corner so homing can prove motion */
    PTR_HOME_RUN,
    PTR_HOME_DONE,
};

/*
 * A content signature over both cursor sprite planes. A glyph swap changes it,
 * which is what lets a per-glyph hotspot be measured ONCE per glyph and then
 * recalled, instead of being re-derived (or, far worse, guessed) on every aim.
 */
static uint32_t artist_ptr_glyph_sig(ARTISTState *s)
{
    uint32_t h = 2166136261u;
    int b, i;

    for (b = 0; b < 2; b++) {
        struct vram_buffer *buf =
            &s->vram_buffer[b ? ARTIST_BUFFER_CURSOR2 : ARTIST_BUFFER_CURSOR1];
        if (!buf->data) {
            continue;
        }
        for (i = 0; i < PTR_SPRITE_MAX * PTR_SPRITE_MAX; i++) {
            h = (h ^ buf->data[i]) * 16777619u;
        }
    }
    return h ? h : 1;
}

/* The sprite origin as the DEVICE MODEL itself computes it. See the warning. */
static bool artist_ptr_reading(ARTISTState *s, int *x, int *y)
{
    if (!cursor_visible(s) || s->cursor_pos == 0) {
        return false;               /* a hidden cursor's registers stop tracking */
    }
    artist_get_cursor_pos(s, x, y);
    return true;
}

static bool artist_ptr_hot_sane(int hx, int hy)
{
    /*
     * A hotspot lives INSIDE the sprite. Bound it AT EVERY PATH THAT RECORDS
     * ONE, not only where one is used: an engine that can store an impossible
     * value and then report it as exact is an engine whose health signal is
     * worthless precisely when it matters.
     */
    return hx > -PTR_SPRITE_MAX && hx < PTR_SPRITE_MAX &&
           hy > -PTR_SPRITE_MAX && hy < PTR_SPRITE_MAX;
}

static void artist_ptr_glyph_store(ARTISTState *s, uint32_t sig, int hx, int hy)
{
    int i;

    if (!artist_ptr_hot_sane(hx, hy)) {
        return;
    }
    for (i = 0; i < PTR_GLYPH_SLOTS; i++) {
        if (s->ptr_glyph_sig[i] == sig || s->ptr_glyph_sig[i] == 0) {
            s->ptr_glyph_sig[i] = sig;
            s->ptr_glyph_hx[i] = hx;
            s->ptr_glyph_hy[i] = hy;
            return;
        }
    }
    s->ptr_glyph_sig[0] = sig;      /* bank full: evict slot 0 */
    s->ptr_glyph_hx[0] = hx;
    s->ptr_glyph_hy[0] = hy;
}

static bool artist_ptr_glyph_recall(ARTISTState *s, uint32_t sig, int *hx, int *hy)
{
    int i;

    for (i = 0; i < PTR_GLYPH_SLOTS; i++) {
        if (s->ptr_glyph_sig[i] == sig) {
            *hx = s->ptr_glyph_hx[i];
            *hy = s->ptr_glyph_hy[i];
            return true;
        }
    }
    return false;
}

static void G_GNUC_PRINTF(2, 3) artist_ptr_send(ARTISTState *s,
                                               const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int n;

    if (!s->ptr_open) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        qemu_chr_fe_write_all(&s->ptrctl, (const uint8_t *)buf, MIN(n, (int)sizeof(buf) - 1));
    }
}

static void artist_ptr_ack(ARTISTState *s, uint64_t seq)
{
    artist_ptr_send(s, "%" PRIu64 " OK\n", seq);
}

static void artist_ptr_inject(ARTISTState *s, int dx, int dy)
{
    if (dx) {
        qemu_input_queue_rel(s->con, INPUT_AXIS_X, dx);
    }
    if (dy) {
        qemu_input_queue_rel(s->con, INPUT_AXIS_Y, dy);
    }
    if (dx || dy) {
        qemu_input_event_sync();
    }
}

/*
 * Apply one pending button edge. Edges are deferred behind a converging target
 * so a click can never be delivered while the pointer is still walking -- that
 * is how a press-at-A / motion / release-at-B DRAG gets manufactured out of an
 * ordinary click.
 */
static void artist_ptr_apply_edges(ARTISTState *s)
{
    static const InputButton map[3] = {
        INPUT_BUTTON_LEFT, INPUT_BUTTON_MIDDLE, INPUT_BUTTON_RIGHT
    };
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);

    while (s->ptr_edge_head != s->ptr_edge_tail) {
        int i = s->ptr_edge_head % PTR_EDGE_CAP;

        if (now < s->ptr_edge_gap_until) {
            return;                 /* pace consecutive edges apart */
        }
        qemu_input_queue_btn(s->con, map[s->ptr_edge_btn[i]], s->ptr_edge_down[i]);
        qemu_input_event_sync();
        s->ptr_btn_state = s->ptr_edge_down[i]
            ? (s->ptr_btn_state | (1u << s->ptr_edge_btn[i]))
            : (s->ptr_btn_state & ~(1u << s->ptr_edge_btn[i]));
        if (s->ptr_trace) {
            qemu_log("artist-ptr: edge btn%d %s applied\n",
                     s->ptr_edge_btn[i] + 1, s->ptr_edge_down[i] ? "DOWN" : "UP");
        }
        artist_ptr_ack(s, s->ptr_edge_seq[i]);   /* ack when it APPLIES */
        s->ptr_edge_gap_until = now + s->ptr_btn_gap_ms;
        s->ptr_edge_head++;
    }
}

/*
 * HOME: drive the pointer into the top-left corner so the X server clamps it to
 * a KNOWN position (0,0), which makes the reading the negated hotspot.
 *
 * A QUIESCENT SENSOR IS NOT A CONVERGED SENSOR. An unchanged reading means
 * "pinned against the edge" only if the reading CHANGED FIRST. Stillness from a
 * standing start is indistinguishable from a guest that has simply not consumed
 * the injected motion yet, and concluding on it records `hot = -(wherever the
 * pointer happened to be)` -- an impossible value, reported as exact. So:
 * require proof of motion, THEN require stillness.
 */
/*
 * KICK: shove the pointer AWAY from the corner before homing into it.
 *
 * Homing proves it is pinned by watching the reading stop changing, which is
 * only meaningful if the reading changed in the first place. A session that
 * reconnects while the pointer is ALREADY parked in the corner -- the ordinary
 * case, since the previous session left it there -- would otherwise see perfect
 * stillness from the very first window, never satisfy proof-of-motion, and burn
 * its whole budget before honestly reporting "I do not know". One kick outward
 * makes the subsequent stillness mean something.
 */
static void artist_ptr_home_kick(ARTISTState *s)
{
    int x, y, step = (int)s->ptr_move_step;

    if (!artist_ptr_reading(s, &x, &y)) {
        return;
    }
    if (s->ptr_home_kick > 0 &&
        (x != s->ptr_home_lx || y != s->ptr_home_ly)) {
        s->ptr_home_state = PTR_HOME_RUN;   /* it moved; now home for real */
        s->ptr_home_win = 0;
        s->ptr_home_still = 0;
        s->ptr_home_moved = false;
        return;
    }
    if (++s->ptr_home_kick >= 16) {
        s->ptr_home_state = PTR_HOME_RUN;   /* proceed; RUN still proves motion */
        s->ptr_home_win = 0;
        s->ptr_home_still = 0;
        s->ptr_home_moved = false;
        return;
    }
    s->ptr_home_lx = x;
    s->ptr_home_ly = y;
    artist_ptr_inject(s, step, step);
}

static void artist_ptr_home_window(ARTISTState *s)
{
    int x, y, step;
    bool at_corner, changed;

    if (!artist_ptr_reading(s, &x, &y)) {
        return;
    }
    changed = s->ptr_home_win > 0 &&
              (x != s->ptr_home_lx || y != s->ptr_home_ly);
    if (changed) {
        s->ptr_home_moved = true;       /* the guest really did move it */
        s->ptr_home_still = 0;
    } else if (s->ptr_home_win > 0) {
        s->ptr_home_still++;
    }
    s->ptr_home_lx = x;
    s->ptr_home_ly = y;

    /*
     * "Pinned" is VERIFIED, not inferred. At the top-left clamp the pointer is
     * (0,0), so the sprite origin must be -hotspot -- which lives inside the
     * sprite. If the reading is not within a sprite of the corner we are not
     * pinned, however still the sensor looks: under TCG the guest consumes PS/2
     * packets on its own schedule, and three windows can pass with motion still
     * queued. That is exactly how a home step records an impossible hotspot and
     * then reports it as exact. Requiring proof of MOTION and proof of PLACE
     * makes both failure modes non-conclusive instead of silently wrong.
     */
    at_corner = abs(x) < PTR_SPRITE_MAX && abs(y) < PTR_SPRITE_MAX;

    if (s->ptr_home_moved && at_corner &&
        s->ptr_home_still >= PTR_HOME_STILL) {
        int hx = -x, hy = -y;

        if (artist_ptr_hot_sane(hx, hy)) {
            s->ptr_hot_x = hx;
            s->ptr_hot_y = hy;
            s->ptr_hot_exact = true;
            s->ptr_last_sig = artist_ptr_glyph_sig(s);
            artist_ptr_glyph_store(s, s->ptr_last_sig, hx, hy);
        } else {
            s->ptr_hot_x = s->ptr_hot_y = 0;
            s->ptr_hot_exact = false;
        }
        s->ptr_home_state = PTR_HOME_DONE;
        return;
    }
    if (++s->ptr_home_win >= PTR_HOME_MAX_WIN) {
        /* Never converged. Say "I do not know" instead of inventing a number. */
        s->ptr_hot_x = s->ptr_hot_y = 0;
        s->ptr_hot_exact = false;
        s->ptr_home_state = PTR_HOME_DONE;
        return;
    }

    /*
     * Pace the drive to the guest rather than to the window: inject when the
     * last step was visibly consumed, and nudge again if we have stalled short
     * of the corner. Injecting every window instead floods the PS/2 queue with
     * thousands of counts that keep shoving at the clamp long after homing.
     */
    if (s->ptr_home_still != 0 && s->ptr_home_still < 2) {
        return;
    }
    if (at_corner && s->ptr_home_still) {
        return;                         /* pinned: let stillness accumulate */
    }
    step = -(int)s->ptr_move_step;
    artist_ptr_inject(s, step, step);
}

/*
 * A glyph swap moves the DRAWN SPRITE without moving the POINTER: the X server
 * re-states the position for the new hotspot. So while the pointer is at rest,
 *      d(origin) = -d(hotspot)
 * and the compensation can be read straight off the write. This is the
 * continuity rule; sampling the registers per window instead would walk the
 * hotspot away over a single sweep.
 */
static void artist_ptr_track_glyph(ARTISTState *s, int x, int y)
{
    uint32_t sig = artist_ptr_glyph_sig(s);
    int hx, hy;

    if (sig == s->ptr_last_sig) {
        return;
    }
    if (s->ptr_last_sig != 0 && s->ptr_hot_exact && s->ptr_have_last) {
        if (artist_ptr_glyph_recall(s, sig, &hx, &hy)) {
            s->ptr_hot_x = hx;
            s->ptr_hot_y = hy;
        } else {
            hx = s->ptr_hot_x - (x - s->ptr_last_x);
            hy = s->ptr_hot_y - (y - s->ptr_last_y);
            if (artist_ptr_hot_sane(hx, hy)) {
                s->ptr_hot_x = hx;
                s->ptr_hot_y = hy;
                artist_ptr_glyph_store(s, sig, hx, hy);
            } else {
                s->ptr_hot_exact = false;   /* honest: this glyph is unmeasured */
            }
        }
    }
    s->ptr_last_sig = sig;
}

static void artist_ptr_window(void *opaque)
{
    ARTISTState *s = opaque;
    int x, y, px, py, ex, ey, sx, sy, cap;

    timer_mod(s->ptr_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + s->ptr_window_ms);

    if (!s->ptr_open) {
        return;
    }
    if (s->ptr_home_state == PTR_HOME_KICK) {
        artist_ptr_home_kick(s);
        return;
    }
    if (s->ptr_home_state == PTR_HOME_RUN) {
        artist_ptr_home_window(s);
        return;
    }
    if (!artist_ptr_reading(s, &x, &y)) {
        return;                     /* untrusted this window; do NOT guess */
    }

    if (!s->ptr_have_target) {
        /* At rest is the only safe moment to learn a glyph's hotspot. */
        artist_ptr_track_glyph(s, x, y);
        s->ptr_last_x = x;
        s->ptr_last_y = y;
        s->ptr_have_last = true;
        artist_ptr_apply_edges(s);
        return;
    }

    /* Retire in-flight by pixels we OBSERVED move, then decay the remainder. */
    if (s->ptr_have_last) {
        s->ptr_infl_x -= (x - s->ptr_last_x);
        s->ptr_infl_y -= (y - s->ptr_last_y);
    }
    s->ptr_infl_x /= PTR_INFL_DECAY;
    s->ptr_infl_y /= PTR_INFL_DECAY;
    s->ptr_last_x = x;
    s->ptr_last_y = y;
    s->ptr_have_last = true;

    px = x + s->ptr_hot_x;
    py = y + s->ptr_hot_y;
    ex = s->ptr_tx - px - (int)s->ptr_infl_x;
    ey = s->ptr_ty - py - (int)s->ptr_infl_y;

    if (s->ptr_trace_pos) {
        qemu_log("artist-ptr: r=%d,%d hot=%d,%d(%s) p=%d,%d t=%d,%d e=%d,%d w=%d\n",
                 x, y, s->ptr_hot_x, s->ptr_hot_y,
                 s->ptr_hot_exact ? "exact" : "UNKNOWN",
                 px, py, s->ptr_tx, s->ptr_ty, ex, ey, s->ptr_win);
    }

    /*
     * Converged means AT REST at the target, not merely predicted to arrive.
     * The error already has the in-flight estimate subtracted, so a target can
     * look reached while counts are still queued in the guest's PS/2 buffer --
     * and those counts then carry the pointer PAST it, typically into a screen
     * clamp where it cannot come back on its own. Settling the in-flight before
     * declaring victory costs a couple of windows and is the difference between
     * landing on the pixel and landing near it.
     */
    if (abs(ex) <= (int)s->ptr_deadband && abs(ey) <= (int)s->ptr_deadband &&
        !s->ptr_infl_x && !s->ptr_infl_y) {
        s->ptr_have_target = false;
        s->ptr_infl_x = s->ptr_infl_y = 0;
        artist_ptr_apply_edges(s);
        return;
    }

    /*
     * OSCILLATION LATCH. Repeated sign reversal means the READING is moving
     * with the pointer (a glyph swap under the cursor), not that we keep
     * missing: accept where we are rather than hunt forever.
     */
    if ((ex > 0) != (s->ptr_sign_x > 0) || (ey > 0) != (s->ptr_sign_y > 0)) {
        s->ptr_osc++;
    }
    s->ptr_sign_x = ex;
    s->ptr_sign_y = ey;
    if (s->ptr_osc >= PTR_OSC_LIMIT || ++s->ptr_win >= (int)s->ptr_tries) {
        s->ptr_have_target = false;
        s->ptr_giveups++;
        s->ptr_infl_x = s->ptr_infl_y = 0;
        artist_ptr_apply_edges(s);
        return;
    }

    if ((s->ptr_infl_x || s->ptr_infl_y) && ++s->ptr_wait < PTR_INFL_GATE_WIN) {
        return;                     /* last step still unconsumed; do not stack */
    }

    /*
     * GAIN IS A STEP SIZER ONLY. A wrong gain costs convergence speed, never
     * accuracy, because the next window re-reads the truth. The margin keeps
     * the step just short of the error so we never extrapolate ahead of real
     * observed movement.
     */
    sx = (ex * 100) / (int)(s->ptr_gain_x100 * 110 / 100);
    sy = (ey * 100) / (int)(s->ptr_gain_x100 * 110 / 100);
    cap = (int)s->ptr_move_step;
    sx = MAX(-cap, MIN(cap, sx));
    sy = MAX(-cap, MIN(cap, sy));
    if (sx == 0 && ex) {
        sx = ex > 0 ? 1 : -1;       /* never a step OPPOSING the measured error */
    }
    if (sy == 0 && ey) {
        sy = ey > 0 ? 1 : -1;
    }
    if (s->ptr_trace) {
        qemu_log("artist-ptr: step %+d,%+d (err %+d,%+d, infl %+d,%+d)\n",
                 sx, sy, ex, ey, s->ptr_infl_x, s->ptr_infl_y);
    }
    artist_ptr_inject(s, sx, sy);
    s->ptr_infl_x += sx;
    s->ptr_infl_y += sy;
    s->ptr_wait = 0;
}

static void artist_ptr_line(ARTISTState *s, char *line)
{
    uint64_t seq = 0;
    char verb[16];
    int x, y, n;

    if (s->ptr_trace) {
        qemu_log("artist-ptr: rx %s\n", line);
    }
    if (sscanf(line, "%" SCNu64 " %15s%n", &seq, verb, &n) < 2) {
        return;
    }
    if (!strcmp(verb, "MOVEA")) {
        if (sscanf(line + n, " %d %d", &x, &y) == 2) {
            s->ptr_tx = MAX(0, MIN((int)s->width - 1, x));
            s->ptr_ty = MAX(0, MIN((int)s->height - 1, y));
            s->ptr_have_target = true;
            s->ptr_win = 0;
            s->ptr_osc = 0;
            s->ptr_wait = 0;
            s->ptr_reaims++;
        }
        artist_ptr_ack(s, seq);             /* acks on ACCEPT, not on arrival */
    } else if (!strncmp(verb, "DOWN", 4) || !strncmp(verb, "UP", 2)) {
        bool down = verb[0] == 'D';
        int btn = (down ? verb[4] : verb[2]) - '1';
        int slot;

        if (btn < 0 || btn > 2) {
            artist_ptr_ack(s, seq);
            return;
        }
        if (s->ptr_edge_tail - s->ptr_edge_head >= PTR_EDGE_CAP) {
            artist_ptr_ack(s, seq);         /* bounded queue: drop, never block */
            return;
        }
        slot = s->ptr_edge_tail % PTR_EDGE_CAP;
        s->ptr_edge_btn[slot] = btn;
        s->ptr_edge_down[slot] = down;
        s->ptr_edge_seq[slot] = seq;
        s->ptr_edge_tail++;
    } else if (!strcmp(verb, "STAT")) {
        int rx = 0, ry = 0;
        bool ok = artist_ptr_reading(s, &rx, &ry);

        artist_ptr_send(s,
            "%" PRIu64 " OK reading=%d,%d valid=%d hot=%d,%d hot_exact=%d "
            "home=%s target=%d,%d aiming=%d gain=%u queue=%d reaims=%u "
            "giveups=%u running=%d\n",
            seq, rx, ry, ok, s->ptr_hot_x, s->ptr_hot_y, s->ptr_hot_exact,
            s->ptr_home_state == PTR_HOME_DONE
                ? (s->ptr_hot_exact ? "measured" : "UNKNOWN")
                : (s->ptr_home_state == PTR_HOME_KICK ? "kick" : "running"),
            s->ptr_tx, s->ptr_ty, s->ptr_have_target, s->ptr_gain_x100,
            (int)(s->ptr_edge_tail - s->ptr_edge_head), s->ptr_reaims,
            s->ptr_giveups, runstate_is_running());
    } else {
        artist_ptr_ack(s, seq);             /* SYNC and anything else */
    }
}

static int artist_ptr_can_receive(void *opaque)
{
    ARTISTState *s = opaque;
    return sizeof(s->ptr_rx) - s->ptr_rxlen - 1;
}

static void artist_ptr_receive(void *opaque, const uint8_t *buf, int size)
{
    ARTISTState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        if (buf[i] == '\n' || s->ptr_rxlen >= (int)sizeof(s->ptr_rx) - 1) {
            s->ptr_rx[s->ptr_rxlen] = '\0';
            if (s->ptr_rxlen) {
                artist_ptr_line(s, s->ptr_rx);
            }
            s->ptr_rxlen = 0;
        } else {
            s->ptr_rx[s->ptr_rxlen++] = buf[i];
        }
    }
}

static void artist_ptr_event(void *opaque, QEMUChrEvent ev)
{
    ARTISTState *s = opaque;

    switch (ev) {
    case CHR_EVENT_OPENED:
        s->ptr_open = true;
        s->ptr_rxlen = 0;
        s->ptr_have_target = false;
        s->ptr_have_last = false;
        s->ptr_edge_head = s->ptr_edge_tail = 0;
        s->ptr_btn_state = 0;
        s->ptr_last_sig = 0;
        s->ptr_hot_exact = false;
        s->ptr_hot_x = s->ptr_hot_y = 0;
        /* Force ONE clamp per session so the hotspot is named, never guessed. */
        s->ptr_home_state = PTR_HOME_KICK;
        s->ptr_home_win = 0;
        s->ptr_home_kick = 0;
        s->ptr_home_still = 0;
        s->ptr_home_moved = false;
        artist_ptr_send(s, "HELLO artistptr/1 caps=movea,btn,sync,stat "
                           "surf=%ux%u\n", s->width, s->height);
        break;
    case CHR_EVENT_CLOSED:
        s->ptr_open = false;
        s->ptr_have_target = false;
        break;
    default:
        break;
    }
}

static void artist_ptr_init(ARTISTState *s)
{
    if (!qemu_chr_fe_backend_connected(&s->ptrctl)) {
        return;                     /* loop not armed; station runs dbus-rel */
    }
    qemu_chr_fe_set_handlers(&s->ptrctl, artist_ptr_can_receive,
                             artist_ptr_receive, artist_ptr_event,
                             NULL, s, NULL, true);
    /*
     * QEMU_CLOCK_VIRTUAL is deliberate: the window measures GUEST time, and the
     * loop must not advance while the guest is not running -- it would burn its
     * try budget against a pointer that cannot move. The consequence is that a
     * paused guest acks nothing, which matters on this station more than on any
     * other: hpuxvue starts `-loadvm golden -S` AND idle-auto-pauses after 60 s,
     * so a returning visitor is the COMMON path, not an edge case. MOVEA acks on
     * ACCEPT rather than on convergence precisely so that a pause cannot stall
     * the daemon's ack pipeline; only button edges wait for the guest, and the
     * daemon resumes it on input well inside the ack deadline.
     */
    s->ptr_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, artist_ptr_window, s);
    timer_mod(s->ptr_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + s->ptr_window_ms);
}

static void artist_realizefn(DeviceState *dev, Error **errp)
{
    ARTISTState *s = ARTIST(dev);
    struct vram_buffer *buf;
    hwaddr offset = 0;

    if (s->width > 2048 || s->height > 2048) {
        error_report("artist: screen size can not exceed 2048 x 2048 pixel.");
        s->width = MIN(s->width, 2048);
        s->height = MIN(s->height, 2048);
    }

    if (s->width < 640 || s->height < 480) {
        error_report("artist: minimum screen size is 640 x 480 pixel.");
        s->width = MAX(s->width, 640);
        s->height = MAX(s->height, 480);
    }

    memory_region_init(&s->mem_as_root, OBJECT(dev), "artist", ~0ull);
    address_space_init(&s->as, &s->mem_as_root, "artist");

    artist_create_buffer(s, "cmap", &offset, ARTIST_BUFFER_CMAP, 2048, 4);
    artist_create_buffer(s, "ap", &offset, ARTIST_BUFFER_AP,
                         s->width, s->height);
    artist_create_buffer(s, "cursor1", &offset, ARTIST_BUFFER_CURSOR1, 64, 64);
    artist_create_buffer(s, "cursor2", &offset, ARTIST_BUFFER_CURSOR2, 64, 64);
    artist_create_buffer(s, "attribute", &offset, ARTIST_BUFFER_ATTRIBUTE,
                         64, 64);

    buf = &s->vram_buffer[ARTIST_BUFFER_AP];
    framebuffer_update_memory_section(&s->fbsection, &buf->mr, 0,
                                      buf->width, buf->height);
    /*
     * Artist cursor max size
     */
    s->cursor_height = NGLE_MAX_SPRITE_SIZE;
    s->cursor_width = NGLE_MAX_SPRITE_SIZE;

    /*
     * These two registers are not initialized by seabios's STI implementation.
     * Initialize them here to sane values so artist also works with older
     * (not-fixed) seabios versions.
     */
    s->image_bitmap_op = 0x23000300;
    s->plane_mask = 0xff;

    /* enable screen */
    s->misc_video |= 0x0A000000;
    s->misc_ctrl  |= 0x00800000;

    s->con = graphic_console_init(dev, 0, &artist_ops, s);
    qemu_console_resize(s->con, s->width, s->height);

    artist_ptr_init(s);
}

static int vmstate_artist_post_load(void *opaque, int version_id)
{
    artist_invalidate(opaque);
    return 0;
}

static const VMStateDescription vmstate_artist = {
    .name = "artist",
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = vmstate_artist_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(height, ARTISTState),
        VMSTATE_UINT16(width, ARTISTState),
        VMSTATE_UINT16(depth, ARTISTState),
        VMSTATE_UINT32(fg_color, ARTISTState),
        VMSTATE_UINT32(bg_color, ARTISTState),
        VMSTATE_UINT32(vram_char_y, ARTISTState),
        VMSTATE_UINT32(vram_bitmask, ARTISTState),
        VMSTATE_UINT32(vram_start, ARTISTState),
        VMSTATE_UINT32(vram_pos, ARTISTState),
        VMSTATE_UINT32(vram_size, ARTISTState),
        VMSTATE_UINT32(blockmove_source, ARTISTState),
        VMSTATE_UINT32(blockmove_dest, ARTISTState),
        VMSTATE_UINT32(blockmove_size, ARTISTState),
        VMSTATE_UINT32(line_size, ARTISTState),
        VMSTATE_UINT32(line_end, ARTISTState),
        VMSTATE_UINT32(line_xy, ARTISTState),
        VMSTATE_UINT32(cursor_pos, ARTISTState),
        VMSTATE_UINT32(cursor_cntrl, ARTISTState),
        VMSTATE_UINT32(cursor_height, ARTISTState),
        VMSTATE_UINT32(cursor_width, ARTISTState),
        VMSTATE_UINT32(plane_mask, ARTISTState),
        VMSTATE_UINT32(reg_100080, ARTISTState),
        VMSTATE_UINT32(horiz_backporch, ARTISTState),
        VMSTATE_UINT32(active_lines_low, ARTISTState),
        VMSTATE_UINT32(misc_video, ARTISTState),
        VMSTATE_UINT32(misc_ctrl, ARTISTState),
        VMSTATE_UINT32(dst_bm_access, ARTISTState),
        VMSTATE_UINT32(src_bm_access, ARTISTState),
        VMSTATE_UINT32(control_plane, ARTISTState),
        VMSTATE_UINT32(transfer_data, ARTISTState),
        VMSTATE_UINT32(image_bitmap_op, ARTISTState),
        VMSTATE_UINT32(font_write1, ARTISTState),
        VMSTATE_UINT32(font_write2, ARTISTState),
        VMSTATE_UINT32(font_write_pos_y, ARTISTState),
        VMSTATE_BOOL(disable, ARTISTState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property artist_properties[] = {
    DEFINE_PROP_UINT16("width",        ARTISTState, width, 1280),
    DEFINE_PROP_UINT16("height",       ARTISTState, height, 1024),
    DEFINE_PROP_UINT16("depth",        ARTISTState, depth, 8),
    DEFINE_PROP_BOOL("disable",        ARTISTState, disable, false),
    /*
     * Closed-loop pointer. Absent `ptrctl` the engine never arms and the device
     * behaves exactly as before, which is what makes the rollback two lines:
     * drop `-global artist.ptrctl=` and set SH_INPUT_BACKEND=dbus-rel.
     */
    DEFINE_PROP_CHR("ptrctl",          ARTISTState, ptrctl),
    DEFINE_PROP_UINT32("ptr-window-ms", ARTISTState, ptr_window_ms, 16),
    DEFINE_PROP_UINT32("ptr-deadband", ARTISTState, ptr_deadband, 1),
    DEFINE_PROP_UINT32("ptr-move-step", ARTISTState, ptr_move_step, 48),
    DEFINE_PROP_UINT32("ptr-tries",    ARTISTState, ptr_tries, 90),
    DEFINE_PROP_UINT32("ptr-btn-gap-ms", ARTISTState, ptr_btn_gap_ms, 24),
    /*
     * px per injected count x100. 100 == 1:1. The current golden actually shows
     * ~1.9x (X pointer acceleration; see the station.env.fixture note), but this
     * is only a STEP SIZER -- a wrong value costs convergence windows, never
     * accuracy, because the next window re-reads the truth from the sensor.
     */
    DEFINE_PROP_UINT32("ptr-gain-x100", ARTISTState, ptr_gain_x100, 190),
    DEFINE_PROP_BOOL("ptr-trace",      ARTISTState, ptr_trace, false),
    DEFINE_PROP_BOOL("ptr-trace-pos",  ARTISTState, ptr_trace_pos, false),
};

static void artist_reset(DeviceState *qdev)
{
}

static void artist_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = artist_realizefn;
    dc->vmsd = &vmstate_artist;
    device_class_set_legacy_reset(dc, artist_reset);
    device_class_set_props(dc, artist_properties);
}

static const TypeInfo artist_info = {
    .name          = TYPE_ARTIST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ARTISTState),
    .instance_init = artist_initfn,
    .class_init    = artist_class_init,
};

static void artist_register_types(void)
{
    type_register_static(&artist_info);
}

type_init(artist_register_types)
