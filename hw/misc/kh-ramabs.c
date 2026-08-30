/*
 * kh-ramabs — absolute pointer by writing the guest's OWN pointer coordinate.
 *
 * Copyright (c) 2026 osgallery contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * WHAT THIS IS, AND WHAT IT IS NOT
 *
 * It is NOT a closed loop. The two closed-loop stations in this museum (irix
 * over the Newport VC2's cursor registers, aix432 over the Matrox DAC's) read
 * a HARDWARE CURSOR position back and walk the pointer to a target with a
 * control law: gain, in-flight accounting, an oscillation latch, a give-up cap,
 * and — the hard half — a per-glyph HOTSPOT that has to be measured, because a
 * guessed hotspot is a magnet rather than a small error.
 *
 * Some guests make all of that unnecessary. If the guest OS keeps its own idea
 * of the pointer position in a structure at a known guest-physical address,
 * that coordinate can simply be WRITTEN. Then:
 *
 *   - there is no control law, no gain and no convergence criterion;
 *   - the hotspot leaves the control path entirely. It remains a property of
 *     the drawn sprite (the guest still draws at pointer - hotspot), but
 *     nothing here ever needs to know it, so the magnet failure mode is
 *     impossible rather than merely guarded against;
 *   - the display adapter is not involved at all, so the station's device set
 *     and its golden checkpoint are untouched.
 *
 * PUBLISHING. A write alone is usually invisible: a window server repaints the
 * cursor when an input event arrives, not when memory changes. So each write is
 * followed by a "publish" step that makes the guest act on it. The publish step
 * is guest-specific and lives here, behind one property, not on the wire.
 *
 *   publish=nudge (rhapsody)  inject ONE small relative event at the emulated
 *                             mouse. The guest's driver adds its scaled delta
 *                             to the coordinate we just wrote and repaints.
 *                             The written value is therefore pre-compensated
 *                             by the delta the nudge is expected to produce.
 *
 * The nudge is NOT deterministic and this is designed around, not wished away.
 * Rhapsody DR2's PS/2 driver scales by ~0.478 px/unit through a fractional
 * accumulator: two injected units are one guest pixel on 38 of 40 measured
 * trials and zero pixels on the other two. So every write is confirmed by
 * READING THE COORDINATE BACK, and a miss is corrected by repeating the
 * absolute write with the residual folded in. That is a re-issued write, not a
 * loop step: there is no gain to get wrong, and each attempt is complete in
 * itself.
 *
 * FAIL CLOSED. A guest-physical address that is only valid for one checkpoint
 * is a dangerous artifact: a write to a stale address is not a pointer bug, it
 * is memory corruption in whatever structure now lives there, and it will
 * surface weeks later as "the guest randomly misbehaves". So this device
 * VERIFIES its address before it will write anything:
 *
 *   1. the value there must already be a plausible on-screen point;
 *   2. a probe publish must move it by the expected amount.
 *
 * Until both hold, `verified` is false, every MOVEA is REFUSED with an error
 * rather than serviced, and STAT reports pos=unknown. Refusing is the correct
 * outcome: the station's client falls back to its relative path, which is
 * merely worse, instead of scribbling on guest memory.
 *
 * NO MIGRATION STATE. This device registers no VMStateDescription and holds
 * nothing the guest can observe, so it adds no section to the migration stream
 * and `loadvm <golden>` is unaffected by its presence. Every field here is
 * derived from the live connection or re-read from guest memory.
 *
 * SINGLE INJECTOR (BINDING). While the control socket is connected this device
 * owns the guest pointer. Nothing else — no dead-reckoning bridge in the
 * daemon, no QMP input-send-event, no labctl pointer helper — may push motion
 * or button edges at the same mouse, or the two injectors will disagree about
 * where the pointer is and neither will be able to tell.
 *
 * WIRE (`ramabs/1`, a deliberate subset of the museum's other pointer sockets
 * so they all read the same way):
 *
 *   <- HELLO ramabs/1 caps=movea,btn,sync,stat surf=<W>x<H>
 *   -> <seq> MOVEA <x> <y>          <- <seq> OK        (acks on accept)
 *   -> <seq> DOWN<n> | UP<n>        <- <seq> OK        (acks when it applies)
 *   -> <seq> SYNC                   <- <seq> OK
 *   -> <seq> STAT                   <- <seq> OK k=v ...
 *   (any refusal)                   <- <seq> ERR <reason>
 *
 * A target acks on ACCEPT, never on arrival at the pixel: the browser streams
 * targets far faster than any guest repaints and a client that waited would
 * stall the stream. Button edges ack when they APPLY, which is after the
 * position they were restated behind has landed — that deferral is what makes
 * a click land where the visitor aimed it.
 */

#include "qemu/osdep.h"
#include "chardev/char-fe.h"
#include "exec/cpu-common.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "system/runstate.h"
#include "ui/input.h"

#define TYPE_KH_RAMABS "kh-ramabs"
OBJECT_DECLARE_SIMPLE_TYPE(KhRamAbsState, KH_RAMABS)

#define RX_MAX          256
#define TX_MAX          512
#define PUBLISH_TRIES   6      /* re-issues before a target is given up on */
#define PROBE_TRIES     8      /* verification attempts before failing closed */

/* Layouts: how the guest stores its pointer coordinate. */
typedef enum {
    KH_LAYOUT_POINT16LE = 0,   /* struct { int16_t x; int16_t y; } little-end */
} KhLayout;

struct KhRamAbsState {
    DeviceState parent_obj;

    /* --- properties --------------------------------------------------- */
    CharFrontend chr;
    uint64_t addr;             /* guest-physical address of the coordinate  */
    char *layout;              /* "point16le"                               */
    uint32_t width, height;    /* surface bounds, for the sanity check      */
    int32_t nudge_units;       /* units injected to publish one write       */
    int32_t nudge_px;          /* pixels that many units are expected to move */
    uint32_t settle_ms;        /* wait before the read-back                 */
    bool trace;

    /* --- derived ------------------------------------------------------ */
    KhLayout layout_id;

    /* --- connection state (never migrated) ---------------------------- */
    bool connected;
    bool verified;
    bool probing;
    int probe_tries;

    char rx[RX_MAX];
    int rx_len;

    /* the target currently being published, and the request that owns it */
    bool have_target;
    int tgt_x, tgt_y;
    int want_x, want_y;        /* what we wrote, plus the expected nudge   */
    int publish_tries;
    uint64_t tgt_seq;

    /* one deferred button edge, released once the target has landed */
    bool have_btn;
    uint64_t btn_seq;
    InputButton btn;
    bool btn_down;

    /* last confirmed position, for STAT and for the resync preamble */
    bool pos_known;
    int pos_x, pos_y;

    uint64_t stat_refused, stat_reissued, stat_probe_fail;

    QEMUTimer *timer;
};

/* ------------------------------------------------------------------ */
/* guest memory                                                        */

static bool kh_read_point(KhRamAbsState *s, int *x, int *y)
{
    uint8_t b[4];

    cpu_physical_memory_read(s->addr, b, sizeof(b));
    *x = (int16_t)lduw_le_p(b);
    *y = (int16_t)lduw_le_p(b + 2);
    return true;
}

/*
 * Write, then READ IT STRAIGHT BACK. A write to unbacked guest-physical memory
 * is silently discarded, and that failure is invisible from anywhere else: the
 * verification probe below would then read the guest's real (unchanged) pointer,
 * find it exactly where it wanted it, and declare the address good. Proving the
 * write landed is the half of verification that a probe cannot supply.
 */
static bool kh_write_point(KhRamAbsState *s, int x, int y)
{
    uint8_t b[4];
    int rx, ry;

    stw_le_p(b, (uint16_t)(int16_t)x);
    stw_le_p(b + 2, (uint16_t)(int16_t)y);
    cpu_physical_memory_write(s->addr, b, sizeof(b));

    kh_read_point(s, &rx, &ry);
    return rx == x && ry == y;
}

static bool kh_onscreen(KhRamAbsState *s, int x, int y)
{
    return x >= 0 && y >= 0 && x < (int)s->width && y < (int)s->height;
}

/*
 * Nudge AWAY from the nearer screen edge on each axis. The write is
 * pre-compensated by the nudge, so nudging towards the edge would put the
 * written value off-screen and let the guest's own clamp eat the publish.
 */
static int kh_dir(int v, int hi)
{
    return v < hi / 2 ? -1 : +1;
}

/* ------------------------------------------------------------------ */
/* wire                                                                */

static void G_GNUC_PRINTF(2, 3) kh_send(KhRamAbsState *s,
                                       const char *fmt, ...)
{
    char buf[TX_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = sizeof(buf) - 1;
    }
    qemu_chr_fe_write_all(&s->chr, (const uint8_t *)buf, n);
    if (s->trace) {
        info_report("kh-ramabs: tx %.*s", n - 1, buf);
    }
}

static void kh_ok(KhRamAbsState *s, uint64_t seq)
{
    kh_send(s, "%" PRIu64 " OK\n", seq);
}

static void kh_err(KhRamAbsState *s, uint64_t seq, const char *why)
{
    s->stat_refused++;
    kh_send(s, "%" PRIu64 " ERR %s\n", seq, why);
}

/* ------------------------------------------------------------------ */
/* publishing                                                          */

static void kh_arm(KhRamAbsState *s)
{
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + s->settle_ms);
}

/* A write that did not land means the address is not writable guest RAM.
 * There is nothing to retry and nothing safe to assume, so stop writing. */
static void kh_write_failed(KhRamAbsState *s)
{
    s->verified = false;
    s->probing = false;
    s->have_target = false;
    s->pos_known = false;
    s->stat_probe_fail++;
    error_report("kh-ramabs: a write to 0x%" PRIx64 " did not read back - that "
                 "address is not writable guest RAM. Refusing every write; the "
                 "station must fall back to its relative path.", s->addr);
    /* Having failed closed, inject nothing further: refuse the deferred edge
     * rather than releasing it at a position we can no longer vouch for. The
     * client still gets its ack, so nothing stalls waiting on us. */
    if (s->have_btn) {
        s->have_btn = false;
        kh_err(s, s->btn_seq, "unverified-address");
    }
}

/*
 * One complete absolute write: pre-compensate for the nudge, write, publish.
 * Each call stands alone — nothing is carried over from the previous attempt
 * except the target itself, which is what makes a re-issue safe.
 */
static void kh_issue(KhRamAbsState *s)
{
    int dx = 0, dy = 0;

    if (!runstate_is_running()) {
        /*
         * The guest cannot consume a publish while it is stopped. Do not spin
         * against a stopped runstate — write the coordinate so a resume shows
         * the right place, and re-arm; the read-back will confirm later. (There
         * is no engine window here and nothing runs on QEMU_CLOCK_VIRTUAL, so a
         * paused guest costs a wait, never a wedge.)
         */
        if (!kh_write_point(s, s->tgt_x, s->tgt_y)) {
            kh_write_failed(s);
            return;
        }
        s->want_x = s->tgt_x;
        s->want_y = s->tgt_y;
        kh_arm(s);
        return;
    }

    dx = -kh_dir(s->tgt_x, s->width) * s->nudge_px;
    dy = -kh_dir(s->tgt_y, s->height) * s->nudge_px;

    if (!kh_write_point(s, s->tgt_x - dx, s->tgt_y - dy)) {
        kh_write_failed(s);
        return;
    }
    s->want_x = s->tgt_x;
    s->want_y = s->tgt_y;

    qemu_input_queue_rel(NULL, INPUT_AXIS_X,
                         (dx / s->nudge_px) * s->nudge_units);
    qemu_input_queue_rel(NULL, INPUT_AXIS_Y,
                         (dy / s->nudge_px) * s->nudge_units);
    qemu_input_event_sync();

    if (s->trace) {
        info_report("kh-ramabs: issue target=%d,%d wrote=%d,%d nudge=%d,%d try=%d",
                    s->tgt_x, s->tgt_y, s->tgt_x - dx, s->tgt_y - dy,
                    (dx / s->nudge_px) * s->nudge_units,
                    (dy / s->nudge_px) * s->nudge_units, s->publish_tries);
    }
    kh_arm(s);
}

static void kh_release_btn(KhRamAbsState *s)
{
    if (!s->have_btn) {
        return;
    }
    qemu_input_queue_btn(NULL, s->btn, s->btn_down);
    qemu_input_event_sync();
    kh_ok(s, s->btn_seq);
    s->have_btn = false;
}

static void kh_probe_step(KhRamAbsState *s)
{
    int x, y;

    kh_read_point(s, &x, &y);
    if (x == s->want_x && y == s->want_y) {
        s->verified = true;
        s->probing = false;
        s->pos_known = true;
        s->pos_x = x;
        s->pos_y = y;
        info_report("kh-ramabs: address 0x%" PRIx64 " VERIFIED (probe landed "
                    "at %d,%d)", s->addr, x, y);
        return;
    }
    if (++s->probe_tries >= PROBE_TRIES) {
        s->probing = false;
        s->verified = false;
        s->stat_probe_fail++;
        error_report("kh-ramabs: address 0x%" PRIx64 " FAILED verification "
                     "(probe wanted %d,%d, guest holds %d,%d) - refusing every "
                     "write; the station must fall back to its relative path. "
                     "The address is derived against one golden checkpoint: if "
                     "the golden was re-baked it must be re-derived.",
                     s->addr, s->want_x, s->want_y, x, y);
        return;
    }
    kh_issue(s);
}

static void kh_tick(void *opaque)
{
    KhRamAbsState *s = opaque;
    int x, y;

    if (s->probing) {
        kh_probe_step(s);
        return;
    }
    if (!s->have_target) {
        return;
    }

    kh_read_point(s, &x, &y);
    if (x == s->want_x && y == s->want_y) {
        s->have_target = false;
        s->pos_known = true;
        s->pos_x = x;
        s->pos_y = y;
        kh_release_btn(s);
        return;
    }
    if (++s->publish_tries >= PUBLISH_TRIES) {
        /*
         * The publish never took. Say so rather than pretending: an unknown
         * position is a legitimate answer and STAT reports it.
         */
        s->have_target = false;
        s->pos_known = false;
        s->stat_reissued++;
        error_report("kh-ramabs: gave up publishing %d,%d after %d tries "
                     "(guest holds %d,%d)", s->tgt_x, s->tgt_y,
                     s->publish_tries, x, y);
        kh_release_btn(s);
        return;
    }
    s->stat_reissued++;
    kh_issue(s);
}

/* ------------------------------------------------------------------ */
/* commands                                                            */

static void kh_start_probe(KhRamAbsState *s)
{
    int x, y;

    s->verified = false;
    s->probing = false;
    s->probe_tries = 0;

    kh_read_point(s, &x, &y);
    if (!kh_onscreen(s, x, y)) {
        s->stat_probe_fail++;
        error_report("kh-ramabs: address 0x%" PRIx64 " holds %d,%d, which is "
                     "not a point on a %ux%u surface - refusing to write. "
                     "Either the address is wrong for this checkpoint or the "
                     "golden was re-baked; re-derive it.",
                     s->addr, x, y, s->width, s->height);
        return;
    }
    /*
     * Probe by re-stating where the guest already is. The visitor's pointer does
     * not move, but this is NOT a no-op and it cannot pass from a standing
     * start: kh_issue writes a DELIBERATELY WRONG value (the target minus the
     * nudge) and proves the write landed by reading it back, and the guest then
     * has to turn that wrong value into the right one by acting on the injected
     * event. Quiescence therefore fails the probe rather than passing it.
     */
    s->probing = true;
    s->tgt_x = x;
    s->tgt_y = y;
    s->publish_tries = 0;
    kh_issue(s);
}

static void kh_hello(KhRamAbsState *s)
{
    kh_send(s, "HELLO ramabs/1 caps=movea,btn,sync,stat surf=%ux%u\n",
            s->width, s->height);
    kh_start_probe(s);
}

static void kh_stat(KhRamAbsState *s, uint64_t seq)
{
    if (s->pos_known) {
        kh_send(s, "%" PRIu64 " OK addr=0x%" PRIx64 " layout=%s verified=%s "
                "pos=%d,%d nudge=%d/%dpx refused=%" PRIu64 " reissued=%" PRIu64
                " probefail=%" PRIu64 "\n",
                seq, s->addr, s->layout, s->verified ? "yes" : "no",
                s->pos_x, s->pos_y, s->nudge_units, s->nudge_px,
                s->stat_refused, s->stat_reissued, s->stat_probe_fail);
    } else {
        kh_send(s, "%" PRIu64 " OK addr=0x%" PRIx64 " layout=%s verified=%s "
                "pos=unknown nudge=%d/%dpx refused=%" PRIu64 " reissued=%"
                PRIu64 " probefail=%" PRIu64 "\n",
                seq, s->addr, s->layout, s->verified ? "yes" : "no",
                s->nudge_units, s->nudge_px,
                s->stat_refused, s->stat_reissued, s->stat_probe_fail);
    }
}

static void kh_movea(KhRamAbsState *s, uint64_t seq, int x, int y)
{
    if (!s->verified) {
        kh_err(s, seq, "unverified-address");
        return;
    }
    x = MIN(MAX(x, 0), (int)s->width - 1);
    y = MIN(MAX(y, 0), (int)s->height - 1);

    s->tgt_x = x;
    s->tgt_y = y;
    s->tgt_seq = seq;
    s->have_target = true;
    s->publish_tries = 0;
    kh_issue(s);
    kh_ok(s, seq);            /* acks on ACCEPT, never on arrival */
}

static void kh_button(KhRamAbsState *s, uint64_t seq, InputButton b, bool down)
{
    if (!s->verified) {
        kh_err(s, seq, "unverified-address");
        return;
    }
    if (s->have_btn) {
        /* one edge in flight at a time: release the old one first */
        kh_release_btn(s);
    }
    s->btn = b;
    s->btn_down = down;
    s->btn_seq = seq;
    s->have_btn = true;
    if (!s->have_target) {
        kh_release_btn(s);    /* nothing to wait behind */
    }
}

static bool kh_parse_btn(const char *verb, InputButton *b, bool *down)
{
    int n;

    if (g_str_has_prefix(verb, "DOWN")) {
        *down = true;
        n = atoi(verb + 4);
    } else if (g_str_has_prefix(verb, "UP")) {
        *down = false;
        n = atoi(verb + 2);
    } else {
        return false;
    }
    switch (n) {
    case 1: *b = INPUT_BUTTON_LEFT;   return true;
    case 2: *b = INPUT_BUTTON_MIDDLE; return true;
    case 3: *b = INPUT_BUTTON_RIGHT;  return true;
    default: return false;
    }
}

static void kh_line(KhRamAbsState *s, char *line)
{
    uint64_t seq = 0;
    char verb[32];
    int x, y, n;
    InputButton b;
    bool down;

    if (s->trace) {
        info_report("kh-ramabs: rx %s", line);
    }
    n = sscanf(line, "%" SCNu64 " %31s %d %d", &seq, verb, &x, &y);
    if (n < 2) {
        return;
    }
    if (!strcmp(verb, "MOVEA")) {
        if (n == 4) {
            kh_movea(s, seq, x, y);
        } else {
            kh_err(s, seq, "bad-movea");
        }
    } else if (!strcmp(verb, "STAT")) {
        kh_stat(s, seq);
    } else if (!strcmp(verb, "SYNC")) {
        kh_ok(s, seq);
    } else if (kh_parse_btn(verb, &b, &down)) {
        kh_button(s, seq, b, down);
    } else {
        kh_err(s, seq, "bad-verb");
    }
}

/* ------------------------------------------------------------------ */
/* chardev plumbing                                                    */

static int kh_can_receive(void *opaque)
{
    KhRamAbsState *s = opaque;

    return RX_MAX - s->rx_len - 1;
}

static void kh_receive(void *opaque, const uint8_t *buf, int size)
{
    KhRamAbsState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        if (buf[i] == '\n') {
            s->rx[s->rx_len] = '\0';
            kh_line(s, s->rx);
            s->rx_len = 0;
        } else if (s->rx_len < RX_MAX - 1) {
            s->rx[s->rx_len++] = (char)buf[i];
        } else {
            s->rx_len = 0;    /* overlong line: drop it, resync on the newline */
        }
    }
}

static void kh_chr_event(void *opaque, QEMUChrEvent event)
{
    KhRamAbsState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->connected = true;
        s->rx_len = 0;
        s->have_target = false;
        s->have_btn = false;
        /* Nothing is known until this connection's own probe says so. */
        s->pos_known = false;
        kh_hello(s);
        break;
    case CHR_EVENT_CLOSED:
        s->connected = false;
        s->have_target = false;
        s->have_btn = false;
        s->verified = false;
        s->probing = false;
        timer_del(s->timer);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */

static void kh_realize(DeviceState *dev, Error **errp)
{
    KhRamAbsState *s = KH_RAMABS(dev);

    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "kh-ramabs needs chardev=<id>");
        return;
    }
    if (s->addr == 0) {
        error_setg(errp, "kh-ramabs needs addr=<guest-physical address of the "
                   "guest's own pointer coordinate>");
        return;
    }
    if (strcmp(s->layout, "point16le") != 0) {
        error_setg(errp, "kh-ramabs: unknown layout=%s (have: point16le)",
                   s->layout);
        return;
    }
    if (s->nudge_px == 0 || s->nudge_units == 0) {
        error_setg(errp, "kh-ramabs: nudge-units and nudge-px must be non-zero");
        return;
    }
    s->layout_id = KH_LAYOUT_POINT16LE;
    s->timer = timer_new_ms(QEMU_CLOCK_REALTIME, kh_tick, s);

    qemu_chr_fe_set_handlers(&s->chr, kh_can_receive, kh_receive,
                             kh_chr_event, NULL, s, NULL, true);
}

static void kh_unrealize(DeviceState *dev)
{
    KhRamAbsState *s = KH_RAMABS(dev);

    qemu_chr_fe_set_handlers(&s->chr, NULL, NULL, NULL, NULL, NULL, NULL,
                             false);
    if (s->timer) {
        timer_free(s->timer);
        s->timer = NULL;
    }
}

static const Property kh_ramabs_props[] = {
    DEFINE_PROP_CHR("chardev", KhRamAbsState, chr),
    DEFINE_PROP_UINT64("addr", KhRamAbsState, addr, 0),
    DEFINE_PROP_STRING("layout", KhRamAbsState, layout),
    DEFINE_PROP_UINT32("width", KhRamAbsState, width, 1024),
    DEFINE_PROP_UINT32("height", KhRamAbsState, height, 768),
    DEFINE_PROP_INT32("nudge-units", KhRamAbsState, nudge_units, 2),
    DEFINE_PROP_INT32("nudge-px", KhRamAbsState, nudge_px, 1),
    DEFINE_PROP_UINT32("settle-ms", KhRamAbsState, settle_ms, 24),
    DEFINE_PROP_BOOL("trace", KhRamAbsState, trace, false),
};

static void kh_instance_init(Object *obj)
{
    KhRamAbsState *s = KH_RAMABS(obj);

    s->layout = g_strdup("point16le");
}

static void kh_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = kh_realize;
    dc->unrealize = kh_unrealize;
    device_class_set_props(dc, kh_ramabs_props);
    dc->desc = "Absolute pointer by writing the guest's own pointer coordinate";
    dc->user_creatable = true;
    dc->hotpluggable = false;
    /*
     * Deliberately NO dc->vmsd: this device contributes no section to the
     * migration stream, so adding it does not invalidate a station's golden
     * checkpoint.
     */
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo kh_ramabs_info = {
    .name          = TYPE_KH_RAMABS,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(KhRamAbsState),
    .instance_init = kh_instance_init,
    .class_init    = kh_class_init,
};

static void kh_register_types(void)
{
    type_register_static(&kh_ramabs_info);
}

type_init(kh_register_types)
