#include "qemu/osdep.h"
#include "exec/gdbstub.h"

const GDBFeature gdb_static_features[] = {
    {
        .xmlname = "power-core.xml",
        .xml = 
            "<?xml version=\"1.0\"?>\n"
            "<!-- Copyright (C) 2007, 2008 Free Software Foundation, Inc.\n"
            "\n"
            "     Copying and distribution of this file, with or without modification,\n"
            "     are permitted in any medium without royalty provided the copyright\n"
            "     notice and this notice are preserved.  -->\n"
            "\n"
            "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
            "<feature name=\"org.gnu.gdb.power.core\">\n"
            "  <reg name=\"r0\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r1\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r2\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r3\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r4\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r5\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r6\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r7\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r8\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r9\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r10\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r11\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r12\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r13\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r14\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r15\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r16\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r17\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r18\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r19\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r20\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r21\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r22\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r23\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r24\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r25\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r26\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r27\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r28\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r29\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r30\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"r31\" bitsize=\"32\" type=\"uint32\"/>\n"
            "\n"
            "  <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\" regnum=\"64\"/>\n"
            "  <reg name=\"msr\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"cr\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"lr\" bitsize=\"32\" type=\"code_ptr\"/>\n"
            "  <reg name=\"ctr\" bitsize=\"32\" type=\"uint32\"/>\n"
            "  <reg name=\"xer\" bitsize=\"32\" type=\"uint32\"/>\n"
            "</feature>\n",
        .name = "org.gnu.gdb.power.core",
        .regs = (const char * const [70]) {
            [0] =
                "r0",
            [1] =
                "r1",
            [2] =
                "r2",
            [3] =
                "r3",
            [4] =
                "r4",
            [5] =
                "r5",
            [6] =
                "r6",
            [7] =
                "r7",
            [8] =
                "r8",
            [9] =
                "r9",
            [10] =
                "r10",
            [11] =
                "r11",
            [12] =
                "r12",
            [13] =
                "r13",
            [14] =
                "r14",
            [15] =
                "r15",
            [16] =
                "r16",
            [17] =
                "r17",
            [18] =
                "r18",
            [19] =
                "r19",
            [20] =
                "r20",
            [21] =
                "r21",
            [22] =
                "r22",
            [23] =
                "r23",
            [24] =
                "r24",
            [25] =
                "r25",
            [26] =
                "r26",
            [27] =
                "r27",
            [28] =
                "r28",
            [29] =
                "r29",
            [30] =
                "r30",
            [31] =
                "r31",
            [64] =
                "pc",
            [65] =
                "msr",
            [66] =
                "cr",
            [67] =
                "lr",
            [68] =
                "ctr",
            [69] =
                "xer",
        },
        .base_reg = 0,
        .num_regs = 70,
    },
    {
        .xmlname = "power-fpu.xml",
        .xml = 
            "<?xml version=\"1.0\"?>\n"
            "<!-- Copyright (C) 2007, 2008 Free Software Foundation, Inc.\n"
            "\n"
            "     Copying and distribution of this file, with or without modification,\n"
            "     are permitted in any medium without royalty provided the copyright\n"
            "     notice and this notice are preserved.  -->\n"
            "\n"
            "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
            "<feature name=\"org.gnu.gdb.power.fpu\">\n"
            "  <reg name=\"f0\" bitsize=\"64\" type=\"ieee_double\" regnum=\"71\"/>\n"
            "  <reg name=\"f1\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f2\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f3\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f4\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f5\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f6\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f7\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f8\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f9\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f10\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f11\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f12\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f13\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f14\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f15\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f16\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f17\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f18\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f19\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f20\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f21\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f22\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f23\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f24\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f25\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f26\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f27\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f28\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f29\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f30\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "  <reg name=\"f31\" bitsize=\"64\" type=\"ieee_double\"/>\n"
            "\n"
            "  <reg name=\"fpscr\" bitsize=\"32\" group=\"float\"/>\n"
            "</feature>\n",
        .name = "org.gnu.gdb.power.fpu",
        .regs = (const char * const [33]) {
            [0] =
                "f0",
            [1] =
                "f1",
            [2] =
                "f2",
            [3] =
                "f3",
            [4] =
                "f4",
            [5] =
                "f5",
            [6] =
                "f6",
            [7] =
                "f7",
            [8] =
                "f8",
            [9] =
                "f9",
            [10] =
                "f10",
            [11] =
                "f11",
            [12] =
                "f12",
            [13] =
                "f13",
            [14] =
                "f14",
            [15] =
                "f15",
            [16] =
                "f16",
            [17] =
                "f17",
            [18] =
                "f18",
            [19] =
                "f19",
            [20] =
                "f20",
            [21] =
                "f21",
            [22] =
                "f22",
            [23] =
                "f23",
            [24] =
                "f24",
            [25] =
                "f25",
            [26] =
                "f26",
            [27] =
                "f27",
            [28] =
                "f28",
            [29] =
                "f29",
            [30] =
                "f30",
            [31] =
                "f31",
            [32] =
                "fpscr",
        },
        .base_reg = 71,
        .num_regs = 33,
    },
    {
        .xmlname = "power-altivec.xml",
        .xml = 
            "<?xml version=\"1.0\"?>\n"
            "<!-- Copyright (C) 2007, 2008 Free Software Foundation, Inc.\n"
            "\n"
            "     Copying and distribution of this file, with or without modification,\n"
            "     are permitted in any medium without royalty provided the copyright\n"
            "     notice and this notice are preserved.  -->\n"
            "\n"
            "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
            "<feature name=\"org.gnu.gdb.power.altivec\">\n"
            "  <vector id=\"v4f\" type=\"ieee_single\" count=\"4\"/>\n"
            "  <vector id=\"v4i32\" type=\"int32\" count=\"4\"/>\n"
            "  <vector id=\"v8i16\" type=\"int16\" count=\"8\"/>\n"
            "  <vector id=\"v16i8\" type=\"int8\" count=\"16\"/>\n"
            "  <union id=\"vec128\">\n"
            "    <field name=\"uint128\" type=\"uint128\"/>\n"
            "    <field name=\"v4_float\" type=\"v4f\"/>\n"
            "    <field name=\"v4_int32\" type=\"v4i32\"/>\n"
            "    <field name=\"v8_int16\" type=\"v8i16\"/>\n"
            "    <field name=\"v16_int8\" type=\"v16i8\"/>\n"
            "  </union>\n"
            "\n"
            "  <reg name=\"vr0\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr1\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr2\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr3\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr4\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr5\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr6\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr7\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr8\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr9\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr10\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr11\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr12\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr13\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr14\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr15\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr16\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr17\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr18\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr19\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr20\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr21\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr22\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr23\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr24\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr25\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr26\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr27\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr28\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr29\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr30\" bitsize=\"128\" type=\"vec128\"/>\n"
            "  <reg name=\"vr31\" bitsize=\"128\" type=\"vec128\"/>\n"
            "\n"
            "  <reg name=\"vscr\" bitsize=\"32\" group=\"vector\"/>\n"
            "  <reg name=\"vrsave\" bitsize=\"32\" group=\"vector\"/>\n"
            "</feature>\n",
        .name = "org.gnu.gdb.power.altivec",
        .regs = (const char * const [34]) {
            [0] =
                "vr0",
            [1] =
                "vr1",
            [2] =
                "vr2",
            [3] =
                "vr3",
            [4] =
                "vr4",
            [5] =
                "vr5",
            [6] =
                "vr6",
            [7] =
                "vr7",
            [8] =
                "vr8",
            [9] =
                "vr9",
            [10] =
                "vr10",
            [11] =
                "vr11",
            [12] =
                "vr12",
            [13] =
                "vr13",
            [14] =
                "vr14",
            [15] =
                "vr15",
            [16] =
                "vr16",
            [17] =
                "vr17",
            [18] =
                "vr18",
            [19] =
                "vr19",
            [20] =
                "vr20",
            [21] =
                "vr21",
            [22] =
                "vr22",
            [23] =
                "vr23",
            [24] =
                "vr24",
            [25] =
                "vr25",
            [26] =
                "vr26",
            [27] =
                "vr27",
            [28] =
                "vr28",
            [29] =
                "vr29",
            [30] =
                "vr30",
            [31] =
                "vr31",
            [32] =
                "vscr",
            [33] =
                "vrsave",
        },
        .base_reg = 0,
        .num_regs = 34,
    },
    {
        .xmlname = "power-spe.xml",
        .xml = 
            "<?xml version=\"1.0\"?>\n"
            "<!-- Copyright (C) 2007, 2008 Free Software Foundation, Inc.\n"
            "\n"
            "     Copying and distribution of this file, with or without modification,\n"
            "     are permitted in any medium without royalty provided the copyright\n"
            "     notice and this notice are preserved.  -->\n"
            "\n"
            "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
            "<feature name=\"org.gnu.gdb.power.spe\">\n"
            "  <reg name=\"ev0h\" bitsize=\"32\" regnum=\"71\"/>\n"
            "  <reg name=\"ev1h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev2h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev3h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev4h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev5h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev6h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev7h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev8h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev9h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev10h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev11h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev12h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev13h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev14h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev15h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev16h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev17h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev18h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev19h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev20h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev21h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev22h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev23h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev24h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev25h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev26h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev27h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev28h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev29h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev30h\" bitsize=\"32\"/>\n"
            "  <reg name=\"ev31h\" bitsize=\"32\"/>\n"
            "\n"
            "  <reg name=\"acc\" bitsize=\"64\"/>\n"
            "  <reg name=\"spefscr\" bitsize=\"32\"/>\n"
            "</feature>\n",
        .name = "org.gnu.gdb.power.spe",
        .regs = (const char * const [34]) {
            [0] =
                "ev0h",
            [1] =
                "ev1h",
            [2] =
                "ev2h",
            [3] =
                "ev3h",
            [4] =
                "ev4h",
            [5] =
                "ev5h",
            [6] =
                "ev6h",
            [7] =
                "ev7h",
            [8] =
                "ev8h",
            [9] =
                "ev9h",
            [10] =
                "ev10h",
            [11] =
                "ev11h",
            [12] =
                "ev12h",
            [13] =
                "ev13h",
            [14] =
                "ev14h",
            [15] =
                "ev15h",
            [16] =
                "ev16h",
            [17] =
                "ev17h",
            [18] =
                "ev18h",
            [19] =
                "ev19h",
            [20] =
                "ev20h",
            [21] =
                "ev21h",
            [22] =
                "ev22h",
            [23] =
                "ev23h",
            [24] =
                "ev24h",
            [25] =
                "ev25h",
            [26] =
                "ev26h",
            [27] =
                "ev27h",
            [28] =
                "ev28h",
            [29] =
                "ev29h",
            [30] =
                "ev30h",
            [31] =
                "ev31h",
            [32] =
                "acc",
            [33] =
                "spefscr",
        },
        .base_reg = 71,
        .num_regs = 34,
    },
    { NULL }
};
