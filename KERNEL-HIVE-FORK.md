# Kernel Hive fork

This is a patch-carrying fork of [qemu/qemu](https://github.com/qemu/qemu),
published in support of [Kernel Hive](https://github.com/Wnt/osgallery), a
browser-streamed museum of emulated and virtualized operating systems. Its
exhibits run under QEMU, and getting low-latency remote display and input
working for a browser-streamed setup required a handful of local changes to
QEMU itself.

The patches live on a branch, not on `master`:

- [`kernel-hive`](../../tree/kernel-hive) — 5 commits, based on upstream tag
  `v11.0.2`:
  - `ui/dbus,ui/console: add opt-in fast-poll interval, gated on run-state`
    — an env-var knob (`SH_DBUS_UPDATE_MS`) to shorten the dbus display
    listener's poll interval below the stock 30ms, plus a run-state gate so
    a paused guest never pays for the faster poll. Inert unless the env var
    is set.
  - `docs: build Sphinx documentation serially` — works around a Sphinx
    8.1.3 / Python 3.13 multiprocessing crash (`EOFError`) in worker
    processes during the docs build.
  - `hw/misc: add gallery-hid-pci, a low-latency virtual input device` — a
    new, purely additive PCI device for driving guest pointer/keyboard input
    from an external low-latency source, used by one exhibit instead of the
    existing PS/2, USB-HID or virtio-input paths.
  - `hw/display/cirrus: do not require a source pitch for ROP1 fills` and
    `hw/display/cirrus: fix ISA vmstate to descend into the embedded
    CirrusVGAState` — these two look like genuine upstream-worthy Cirrus
    bugs (a spurious source-pitch check dropping valid ROP1 fills, and an
    ISA vmstate pointed at the wrong struct offset causing loadvm framebuffer
    corruption), found while getting Windows NT 3.x/4 guests running
    reliably. Described as bug fixes, not project-specific workarounds,
    because that is what they are.

Each commit has a descriptive subject and body, so any of them can be read,
reviewed, or cherry-picked independently — see the branch's commit log for
what changed and why.

These patches are maintained for Kernel Hive and are published here so the
QEMU community can see and evaluate them, and upstream any ideas that seem
worthwhile — particularly the two Cirrus fixes. The author does not intend
to submit them upstream himself.

This fork carries forward qemu/qemu's existing license terms unchanged; see
`LICENSE` and the per-file license headers throughout the tree.
