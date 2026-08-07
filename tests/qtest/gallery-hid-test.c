/*
 * qtests for the gallery-hid-pci v1 transport.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <glib/gstdio.h>
#include "hw/misc/gallery-hid.h"
#include "hw/pci/pci.h"
#include "libqos/libqos-pc.h"
#include "libqtest.h"

typedef struct TestState {
    QOSState *qs;
    QPCIDevice *dev;
    QPCIBar bar0;
    QPCIBar bar2;
    char *dir;
    char *sock;
    int fd;
} TestState;

static void save_device(QPCIDevice *dev, int devfn, void *data)
{
    *(QPCIDevice **)data = dev;
}

static uint32_t bar0_read(TestState *s, uint32_t off)
{
    return qpci_io_readl(s->dev, s->bar0, off);
}

static void bar0_write(TestState *s, uint32_t off, uint32_t value)
{
    qpci_io_writel(s->dev, s->bar0, off, value);
}

static uint32_t ring_read32(TestState *s, uint32_t off)
{
    uint8_t data[4];

    qpci_memread(s->dev, s->bar2, off, data, sizeof(data));
    return ldl_le_p(data);
}

static uint16_t ring_read16(TestState *s, uint32_t off)
{
    uint8_t data[2];

    qpci_memread(s->dev, s->bar2, off, data, sizeof(data));
    return lduw_le_p(data);
}

static void ring_write32(TestState *s, uint32_t off, uint32_t value)
{
    uint8_t data[4];

    stl_le_p(data, value);
    qpci_memwrite(s->dev, s->bar2, off, data, sizeof(data));
}

static TestState *test_start_args(const char *extra_args)
{
    TestState *s = g_new0(TestState, 1);
    uint64_t size;
    g_autofree char *cmd = NULL;
    g_autofree char *data_option = NULL;
    const char *data_dir = g_getenv("QTEST_QEMU_DATA_DIR");

    s->fd = -1;
    s->dir = g_dir_make_tmp("gallery-hid-qtest-XXXXXX", NULL);
    g_assert_nonnull(s->dir);
    s->sock = g_build_filename(s->dir, "backend.sock", NULL);
    data_option = data_dir ? g_strdup_printf("-L %s ", data_dir) :
                             g_strdup("");
    cmd = g_strdup_printf("%s-chardev socket,id=ghid0,path=%s,"
                          "server=on,wait=off "
                          "-device gallery-hid-pci,id=ghid0,"
                          "chardev=ghid0,addr=1e.0 %s", data_option, s->sock,
                          extra_args ? extra_args : "");
    s->qs = qtest_pc_boot("%s", cmd);
    qpci_device_foreach(s->qs->pcibus, GHID_PCI_VENDOR_ID,
                        GHID_PCI_DEVICE_ID, save_device, &s->dev);
    g_assert_nonnull(s->dev);
    s->bar0 = qpci_iomap(s->dev, 0, &size);
    g_assert_cmpuint(size, ==, GHID_BAR0_SIZE);
    s->bar2 = qpci_iomap(s->dev, 2, &size);
    g_assert_cmpuint(size, ==, GHID_BAR2_SIZE);
    qpci_device_enable(s->dev);
    return s;
}

static TestState *test_start(void)
{
    return test_start_args(NULL);
}

static void test_stop(TestState *s)
{
    if (s->fd >= 0) {
        close(s->fd);
    }
    g_free(s->dev);
    qtest_pc_shutdown(s->qs);
    g_unlink(s->sock);
    g_rmdir(s->dir);
    g_free(s->sock);
    g_free(s->dir);
    g_free(s);
}

static int connect_backend(TestState *s)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd;
    int i;

    g_assert_cmpuint(strlen(s->sock), <, sizeof(addr.sun_path));
    g_strlcpy(addr.sun_path, s->sock, sizeof(addr.sun_path));
    for (i = 0; i < 100; i++) {
        fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        g_assert_cmpint(fd, >=, 0);
        if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
            return fd;
        }
        close(fd);
        g_usleep(10000);
    }
    g_error("could not connect to %s: %s", s->sock, strerror(errno));
}

static void send_all(int fd, const void *data, size_t len)
{
    const uint8_t *p = data;

    while (len) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);

        g_assert_cmpint(n, >, 0);
        p += n;
        len -= n;
    }
}

static bool recv_all(int fd, void *data, size_t len)
{
    uint8_t *p = data;

    while (len) {
        ssize_t n = recv(fd, p, len, 0);

        if (n <= 0) {
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

static void hello(int fd, unsigned split)
{
    uint8_t request[GHID_FRAME_SIZE] = { 0 };
    uint8_t reply[GHID_FRAME_SIZE];

    memcpy(request, GHID_HELLO_MAGIC, 4);
    stw_le_p(request + 4, GHID_BACKEND_MAJOR);
    stw_le_p(request + 6, GHID_BACKEND_MINOR);
    stw_le_p(request + 8, GHID_RECORD_SIZE);
    if (split) {
        send_all(fd, request, split);
        g_usleep(10000);
        send_all(fd, request + split, sizeof(request) - split);
    } else {
        send_all(fd, request, sizeof(request));
    }
    g_assert_true(recv_all(fd, reply, sizeof(reply)));
    g_assert_cmpmem(reply, 4, GHID_OK_MAGIC, 4);
    g_assert_cmpuint(lduw_le_p(reply + 4), ==, 1);
    g_assert_cmpuint(lduw_le_p(reply + 6), ==, 0);
    g_assert_cmpuint(ldl_le_p(reply + 8), !=, 0);
    g_assert_cmpuint(ldl_le_p(reply + 12) & GHID_STATUS_CONNECTED, !=, 0);
}

static void make_pointer(uint8_t frame[GHID_RECORD_SIZE],
                         uint16_t x, uint16_t y, uint32_t stamp)
{
    memset(frame, 0, GHID_RECORD_SIZE);
    frame[0] = GHID_EVENT_POINTER_ABS;
    stw_le_p(frame + 4, x);
    stw_le_p(frame + 6, y);
    stl_le_p(frame + 12, stamp);
}

static void wait_producer(TestState *s, uint32_t expected)
{
    int i;

    for (i = 0; i < 200; i++) {
        if (ring_read32(s, GHID_RING_OFF_PRODUCER) == expected) {
            return;
        }
        g_usleep(5000);
    }
    g_assert_cmpuint(ring_read32(s, GHID_RING_OFF_PRODUCER), ==, expected);
}

static void arm_driver(TestState *s)
{
    uint32_t producer = ring_read32(s, GHID_RING_OFF_PRODUCER);
    uint32_t epoch = ring_read32(s, GHID_RING_OFF_EPOCH);

    ring_write32(s, GHID_RING_OFF_CONSUMER, producer);
    ring_write32(s, GHID_RING_OFF_LAST_EPOCH, epoch);
    bar0_write(s, GHID_REG_DRIVER_READY, epoch);
    g_assert_cmpuint(bar0_read(s, GHID_REG_STATUS) &
                     GHID_STATUS_DRIVER_READY, !=, 0);
}

/* Models the first two operations of a shared-INTx guest ISR. */
static bool guest_isr_claims(TestState *s)
{
    uint32_t status = bar0_read(s, GHID_REG_IRQ_STATUS);
    uint32_t mask = bar0_read(s, GHID_REG_IRQ_MASK);

    return (status & mask) != 0;
}

static void test_ids_bars_header(void)
{
    TestState *s = test_start();
    uint32_t bar0 = qpci_config_readl(s->dev, PCI_BASE_ADDRESS_0);
    uint32_t bar1 = qpci_config_readl(s->dev, PCI_BASE_ADDRESS_1);
    uint32_t bar2 = qpci_config_readl(s->dev, PCI_BASE_ADDRESS_2);

    g_assert_cmphex(qpci_config_readw(s->dev, PCI_VENDOR_ID), ==,
                    GHID_PCI_VENDOR_ID);
    g_assert_cmphex(qpci_config_readw(s->dev, PCI_DEVICE_ID), ==,
                    GHID_PCI_DEVICE_ID);
    g_assert_cmphex(qpci_config_readb(s->dev, PCI_REVISION_ID), ==,
                    GHID_PCI_REVISION);
    g_assert_cmphex(qpci_config_readw(s->dev, PCI_CLASS_DEVICE), ==,
                    GHID_PCI_CLASS);
    g_assert_cmphex(qpci_config_readb(s->dev, PCI_HEADER_TYPE), ==, 0);
    g_assert_cmphex(qpci_config_readb(s->dev, PCI_INTERRUPT_PIN), ==, 1);
    g_assert_cmphex(bar0 & 0xf, ==, PCI_BASE_ADDRESS_SPACE_MEMORY);
    g_assert_cmphex(bar0 & PCI_BASE_ADDRESS_MEM_PREFETCH, ==, 0);
    g_assert_cmphex(bar1, ==, 0);
    g_assert_cmphex(bar2 & PCI_BASE_ADDRESS_MEM_TYPE_MASK, ==,
                    PCI_BASE_ADDRESS_MEM_TYPE_32);
    g_assert_cmphex(bar2 & PCI_BASE_ADDRESS_MEM_PREFETCH, !=, 0);
    g_assert_cmphex(bar0_read(s, GHID_REG_DEVICE_MAGIC), ==,
                    GHID_DEVICE_MAGIC);
    g_assert_cmphex(bar0_read(s, GHID_REG_ABI_VERSION), ==,
                    GHID_ABI_VERSION);
    g_assert_cmphex(bar0_read(s, GHID_REG_FEATURES), ==, GHID_FEATURES);
    g_assert_cmphex(ring_read32(s, GHID_RING_OFF_MAGIC), ==,
                    GHID_RING_MAGIC);
    g_assert_cmpuint(ring_read16(s, GHID_RING_OFF_HDR_BYTES), ==,
                     GHID_HEADER_SIZE);
    g_assert_cmpuint(ring_read16(s, GHID_RING_OFF_REC_BYTES), ==,
                     GHID_RECORD_SIZE);
    g_assert_cmpuint(ring_read16(s, GHID_RING_OFF_ENTRIES), ==,
                     GHID_RING_ENTRIES);
    test_stop(s);
}

static void test_hello_split_malformed(void)
{
    TestState *s = test_start();
    uint8_t bad[GHID_FRAME_SIZE] = { 0 };
    uint8_t byte;

    s->fd = connect_backend(s);
    memcpy(bad, "NOPE", 4);
    send_all(s->fd, bad, sizeof(bad));
    g_assert_cmpint(recv(s->fd, &byte, 1, 0), ==, 0);
    close(s->fd);
    s->fd = connect_backend(s);
    hello(s->fd, 3);
    g_assert_cmpuint(ring_read32(s, GHID_RING_OFF_PROTO_ERRS), ==, 1);
    test_stop(s);
}

static void test_level_irq_and_ack_race(void)
{
    TestState *s = test_start();
    uint8_t frame[GHID_RECORD_SIZE];
    uint32_t status_before;
    uint32_t producer_before;

    /* Shared line, no enabled cause: ISR returns UNCLAIMED, no side effect. */
    status_before = bar0_read(s, GHID_REG_IRQ_STATUS);
    producer_before = ring_read32(s, GHID_RING_OFF_PRODUCER);
    g_assert_false(guest_isr_claims(s));
    g_assert_cmphex(bar0_read(s, GHID_REG_IRQ_STATUS), ==, status_before);
    g_assert_cmpuint(ring_read32(s, GHID_RING_OFF_PRODUCER), ==,
                     producer_before);

    arm_driver(s);
    s->fd = connect_backend(s);
    hello(s->fd, 0);
    bar0_write(s, GHID_REG_IRQ_MASK, 0);
    g_assert_false(guest_isr_claims(s));
    bar0_write(s, GHID_REG_IRQ_MASK, GHID_IRQ_LINK);
    g_assert_true(guest_isr_claims(s));
    bar0_write(s, GHID_REG_IRQ_MASK, 0);
    g_assert_false(guest_isr_claims(s)); /* status is level-persistent */
    bar0_write(s, GHID_REG_IRQ_MASK, GHID_IRQ_ALL);
    g_assert_true(guest_isr_claims(s));
    bar0_write(s, GHID_REG_IRQ_ACK, GHID_IRQ_LINK | GHID_IRQ_RESET);
    g_assert_false(guest_isr_claims(s));

    make_pointer(frame, 100, 200, 1);
    send_all(s->fd, frame, sizeof(frame));
    wait_producer(s, 1);
    g_assert_true(guest_isr_claims(s));

    /* Consumer update races a second enqueue before ACK: ACK must reassert. */
    ring_write32(s, GHID_RING_OFF_CONSUMER, 1);
    make_pointer(frame, 101, 201, 2);
    send_all(s->fd, frame, sizeof(frame));
    wait_producer(s, 2);
    bar0_write(s, GHID_REG_IRQ_ACK, GHID_IRQ_RING);
    g_assert_true(guest_isr_claims(s));
    g_assert_cmphex(bar0_read(s, GHID_REG_IRQ_STATUS) & GHID_IRQ_RING, !=, 0);
    ring_write32(s, GHID_RING_OFF_CONSUMER, 2);
    bar0_write(s, GHID_REG_GUEST_KICK, 0);
    bar0_write(s, GHID_REG_IRQ_ACK, GHID_IRQ_RING);
    g_assert_false(guest_isr_claims(s));
    test_stop(s);
}

static void test_wrap_full_backpressure(void)
{
    TestState *s = test_start();
    uint8_t frame[GHID_RECORD_SIZE];
    unsigned i;

    arm_driver(s);
    s->fd = connect_backend(s);
    hello(s->fd, 0);
    bar0_write(s, GHID_REG_IRQ_MASK, GHID_IRQ_ALL);
    bar0_write(s, GHID_REG_IRQ_ACK, GHID_IRQ_LINK | GHID_IRQ_RESET);
    for (i = 0; i < GHID_RING_ENTRIES + 1; i++) {
        make_pointer(frame, i & 0x7fff, (i * 3) & 0x7fff, i);
        send_all(s->fd, frame, sizeof(frame));
    }
    wait_producer(s, GHID_RING_ENTRIES);
    g_assert_cmpuint(bar0_read(s, GHID_REG_STATUS) &
                     GHID_STATUS_STALLED, !=, 0);
    g_assert_cmpuint(ring_read32(s, GHID_RING_OFF_STALLS), ==, 1);
    g_assert_true(guest_isr_claims(s));

    /* Free slot 0, kick, and prove the staged record wraps into that slot. */
    ring_write32(s, GHID_RING_OFF_CONSUMER, 1);
    bar0_write(s, GHID_REG_GUEST_KICK, 0);
    wait_producer(s, GHID_RING_ENTRIES + 1);
    g_assert_cmpuint(ring_read16(s, GHID_RING_OFF_RECORDS + 2), ==,
                     GHID_RING_ENTRIES);
    g_assert_cmpuint(bar0_read(s, GHID_REG_STATUS) &
                     GHID_STATUS_STALLED, !=, 0);
    ring_write32(s, GHID_RING_OFF_CONSUMER, GHID_RING_ENTRIES + 1);
    bar0_write(s, GHID_REG_GUEST_KICK, 0);
    bar0_write(s, GHID_REG_IRQ_ACK, GHID_IRQ_RING);
    g_assert_cmpuint(bar0_read(s, GHID_REG_STATUS) &
                     GHID_STATUS_STALLED, ==, 0);
    g_assert_false(guest_isr_claims(s));
    test_stop(s);
}

static void test_vmstate_save_load(void)
{
    TestState *src = test_start();
    TestState *dst;
    uint8_t frame[GHID_RECORD_SIZE];
    g_autofree char *mig_sock = g_build_filename(src->dir, "migrate.sock",
                                                  NULL);
    g_autofree char *uri = g_strdup_printf("unix:%s", mig_sock);
    g_autofree char *incoming = g_strdup_printf("-incoming %s", uri);
    uint32_t epoch;

    arm_driver(src);
    src->fd = connect_backend(src);
    hello(src->fd, 0);
    bar0_write(src, GHID_REG_IRQ_MASK, GHID_IRQ_ALL);
    bar0_write(src, GHID_REG_IRQ_ACK, GHID_IRQ_LINK | GHID_IRQ_RESET);
    make_pointer(frame, 1234, 5678, 42);
    send_all(src->fd, frame, sizeof(frame));
    wait_producer(src, 1);
    g_assert_true(guest_isr_claims(src));
    epoch = ring_read32(src, GHID_RING_OFF_EPOCH);

    dst = test_start_args(incoming);
    migrate(src->qs, dst->qs, uri);

    /* PCI/BAR2/programmed state migrated, but backend and stale INTA did not. */
    g_assert_cmpuint(ring_read32(dst, GHID_RING_OFF_EPOCH), ==, epoch);
    g_assert_cmpuint(ring_read32(dst, GHID_RING_OFF_PRODUCER), ==, 1);
    g_assert_cmpuint(ring_read32(dst, GHID_RING_OFF_CONSUMER), ==, 0);
    g_assert_cmpuint(ring_read16(dst, GHID_RING_OFF_NEXT_SEQ), ==, 1);
    g_assert_cmpuint(ring_read16(dst, GHID_RING_OFF_RECORDS + 4), ==, 1234);
    g_assert_cmpuint(ring_read16(dst, GHID_RING_OFF_RECORDS + 6), ==, 5678);
    g_assert_cmpuint(bar0_read(dst, GHID_REG_IRQ_MASK), ==, GHID_IRQ_ALL);
    g_assert_cmpuint(bar0_read(dst, GHID_REG_IRQ_STATUS), ==, 0);
    g_assert_false(guest_isr_claims(dst));
    g_assert_cmpuint(bar0_read(dst, GHID_REG_STATUS) &
                     GHID_STATUS_CONNECTED, ==, 0);
    g_assert_cmpuint(bar0_read(dst, GHID_REG_STATUS) &
                     GHID_STATUS_DRIVER_READY, !=, 0);
    g_assert_cmpuint(bar0_read(dst, GHID_REG_STATUS) &
                     GHID_STATUS_RESET_REQ, !=, 0);

    /* Fresh hello raises LINK; the guest rearm write unlocks new frames. */
    dst->fd = connect_backend(dst);
    hello(dst->fd, 0);
    g_assert_true(guest_isr_claims(dst));
    arm_driver(dst);
    bar0_write(dst, GHID_REG_IRQ_ACK, GHID_IRQ_ALL);
    g_assert_cmpuint(bar0_read(dst, GHID_REG_STATUS) &
                     GHID_STATUS_RESET_REQ, ==, 0);
    make_pointer(frame, 2222, 7777, 43);
    send_all(dst->fd, frame, sizeof(frame));
    wait_producer(dst, 2);
    g_assert_cmpuint(ring_read16(dst, GHID_RING_OFF_RECORDS +
                                GHID_RECORD_SIZE + 2), ==, 1);

    g_unlink(mig_sock);
    test_stop(src);
    test_stop(dst);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/gallery-hid/ids-bars-header", test_ids_bars_header);
    g_test_add_func("/gallery-hid/hello-split-malformed",
                    test_hello_split_malformed);
    g_test_add_func("/gallery-hid/level-irq-ack-race",
                    test_level_irq_and_ack_race);
    g_test_add_func("/gallery-hid/wrap-full-backpressure",
                    test_wrap_full_backpressure);
    g_test_add_func("/gallery-hid/vmstate-save-load",
                    test_vmstate_save_load);
    return g_test_run();
}
