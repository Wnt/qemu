/* gallery-hid v1 guest/backend ABI.  All multibyte fields are little-endian. */
#ifndef GALLERY_HID_PROTO_H
#define GALLERY_HID_PROTO_H

#define GHID_PCI_VENDOR_ID       0x1b36
#define GHID_PCI_DEVICE_ID       0x0015
#define GHID_PCI_REVISION        0x01
#define GHID_PCI_CLASS           0xff00
#define GHID_BAR0_SIZE           0x1000
#define GHID_BAR2_SIZE           0x2000

#define GHID_REG_DEVICE_MAGIC    0x000
#define GHID_REG_ABI_VERSION     0x004
#define GHID_REG_FEATURES        0x008
#define GHID_REG_STATUS          0x00c
#define GHID_REG_IRQ_STATUS      0x010
#define GHID_REG_IRQ_MASK        0x014
#define GHID_REG_IRQ_ACK         0x018
#define GHID_REG_DRIVER_READY    0x01c
#define GHID_REG_GUEST_KICK      0x020

#define GHID_DEVICE_MAGIC        0x44494847U
#define GHID_ABI_VERSION         0x00010000U
#define GHID_FEATURES            0x0000000fU
#define GHID_STATUS_CONNECTED    (1U << 0)
#define GHID_STATUS_DRIVER_READY (1U << 1)
#define GHID_STATUS_RESET_REQ    (1U << 2)
#define GHID_STATUS_STALLED      (1U << 3)
#define GHID_IRQ_RING            (1U << 0)
#define GHID_IRQ_RESET           (1U << 1)
#define GHID_IRQ_LINK            (1U << 2)
#define GHID_IRQ_ALL             0x00000007U

#define GHID_RING_MAGIC          0x4e494c47U
#define GHID_HEADER_SIZE         0x0100
#define GHID_RECORD_SIZE         16
#define GHID_RING_ENTRIES        256
#define GHID_RING_MASK           255
#define GHID_RING_OFF_MAGIC      0x000
#define GHID_RING_OFF_MAJOR      0x004
#define GHID_RING_OFF_MINOR      0x006
#define GHID_RING_OFF_HDR_BYTES  0x008
#define GHID_RING_OFF_REC_BYTES  0x00a
#define GHID_RING_OFF_ENTRIES    0x00c
#define GHID_RING_OFF_FEATURES   0x010
#define GHID_RING_OFF_EPOCH      0x014
#define GHID_RING_OFF_PRODUCER   0x040
#define GHID_RING_OFF_NEXT_SEQ   0x044
#define GHID_RING_OFF_PROTO_ERRS 0x048
#define GHID_RING_OFF_STALLS     0x04c
#define GHID_RING_OFF_CONSUMER   0x080
#define GHID_RING_OFF_LAST_EPOCH 0x084
#define GHID_RING_OFF_RECORDS    0x100

#define GHID_EVENT_POINTER_ABS   0x01
#define GHID_EVENT_KEY           0x02
#define GHID_EVENT_RELEASE_ALL   0x03
#define GHID_HELLO_MAGIC         "GHIN"
#define GHID_OK_MAGIC            "GHOK"
#define GHID_BACKEND_MAJOR       1
#define GHID_BACKEND_MINOR       0
#define GHID_FRAME_SIZE          16

#endif
