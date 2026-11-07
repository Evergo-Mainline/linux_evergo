/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG         "[BT-MOD-INIT]"

#include "wmt_detect.h"
#include "bluetooth_drv_init.h"

#ifdef CONFIG_MTK_COMBO_BT
int __attribute__((weak)) mtk_wcn_stpbt_drv_init()
{
	WMT_DETECT_PR_INFO("Not implement mtk_wcn_stpbt_drv_init\n");
	return 0;
}
#endif

int do_bluetooth_drv_init(int chip_id)
{
	/*
	 * CONFIG_MTK_COMBO_BT gates the vendor /dev/stpbt transport, which this
	 * tree does not build: Bluetooth is served by bt/hci/btmtk_stp.c, a
	 * mainline-style hci_dev driver on top of the WMT STP channel.  Not
	 * building the vendor one is therefore normal, not a failure -- but the
	 * stub used to return -1, which do_connectivity_driver_init() sums into
	 * its return value and reports as "do common driver init failed" even
	 * though WLAN came up fine.
	 */
	int i_ret = 0;

#ifdef CONFIG_MTK_COMBO_BT
	WMT_DETECT_PR_INFO("start to do bluetooth driver init\n");
	i_ret = mtk_wcn_stpbt_drv_init();
	WMT_DETECT_PR_INFO("finish bluetooth driver init, i_ret:%d\n", i_ret);
#else
	WMT_DETECT_PR_INFO("CONFIG_MTK_COMBO_BT is not defined\n");
#endif
	return i_ret;
}
