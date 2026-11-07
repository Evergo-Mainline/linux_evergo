/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Which panel module the bootloader found on this unit.
 *
 * The evergo board ships with two interchangeable modules wired to the same
 * DSI and touch controller:
 *
 *     k16a_36_02_0a_vdo   Tianma,  needs novatek_ts_fw.bin
 *     k16a_42_02_0b_vdo   CSOT,    needs novatek_ts_ct_fw.bin
 *
 * Running the wrong touch blob does not fail loudly; the controller boots and
 * reports touches, but the geometry is shifted -- on a CSOT unit the Tianma
 * blob moves every button of the right-hand PIN column up by one row, and on a
 * Tianma unit the CSOT blob leaves a dead strip along the right edge.
 *
 * The value compiled into the DTS cannot settle this: the kernel boots the
 * embedded tree, so whatever panel@0 says is a property of whoever wrote the
 * DTS, not of the unit in hand.  LK does know -- it probes the candidate
 * modules and prints "we will use lcm: <name>" -- so setup.c reads the answer
 * out of the tree LK hands over and stores it here.
 */
#ifndef _LINUX_EVERGO_LCM_H
#define _LINUX_EVERGO_LCM_H

enum evergo_lcm_id {
	EVERGO_LCM_UNKNOWN = 0,
	EVERGO_LCM_TIANMA,	/* k16a_36_02_0a_vdo */
	EVERGO_LCM_CSOT,	/* k16a_42_02_0b_vdo */
};

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
extern enum evergo_lcm_id evergo_lcm;
#else
static const enum evergo_lcm_id evergo_lcm = EVERGO_LCM_UNKNOWN;
#endif

#endif /* _LINUX_EVERGO_LCM_H */
