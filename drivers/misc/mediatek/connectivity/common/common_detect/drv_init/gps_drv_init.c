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
#define DFT_TAG         "[GPS-MOD-INIT]"

#include "wmt_detect.h"
#include "gps_drv_init.h"

#ifdef CONFIG_MTK_COMBO_GPS
int __attribute__((weak)) mtk_wcn_stpgps_drv_init()
{
	WMT_DETECT_PR_INFO("no impl. mtk_wcn_stpgps_drv_init\n");
	return 0;
}
#endif

int do_gps_drv_init(int chip_id)
{
	/* Same as do_bluetooth_drv_init(): an unbuilt optional module is not a
	 * failure.  No GPS driver exists in this tree yet, so returning -1 only
	 * poisoned the aggregate return of do_connectivity_driver_init(). */
	int i_ret = 0;

#ifdef CONFIG_MTK_COMBO_GPS
	WMT_DETECT_PR_INFO("start to do gps driver init\n");
	i_ret = mtk_wcn_stpgps_drv_init();
	WMT_DETECT_PR_INFO("finish gps driver init, i_ret:%d\n", i_ret);
#else
	WMT_DETECT_PR_INFO("CONFIG_MTK_COMBO_GPS is not defined\n");
#endif
	return i_ret;

}
