/* drivers/video/msm/mdss/mdss_shdisp.h  (Display Driver)
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MDSS_SHDISP_H
#define MDSS_SHDISP_H

#include <linux/types.h>
#include "mdss_panel.h"
#include <linux/leds.h>

struct mdss_dsi_ctrl_pdata;  // It avoids by imperfect type declaration.

extern void mdss_shdisp_lock_recovery(void);
extern void mdss_shdisp_unlock_recovery(void);

extern bool mdss_shdisp_get_disp_status(void);

extern void mdss_shdisp_dsi_panel_power_on(void);
extern void mdss_shdisp_dsi_panel_power_off(void);
extern void mdss_shdisp_dsi_panel_on(struct mdss_panel_data *pdata);
extern void mdss_shdisp_dsi_panel_off(void);
extern void mdss_shdisp_dsi_panel_start_display(void);
extern void mdss_shdisp_dsi_panel_post_video_start(void);
extern void mdss_shdisp_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

extern void mdss_shdisp_bkl_ctl( u32 bl_level );
extern int mdss_shdisp_is_disp_on( void );

extern void mdss_shdisp_shutdown( void );
extern void mdss_shdisp_dsi_cmd_clk_ctrl(bool onoff);
extern void mdss_dsi_dln0_err_contention_lp1_mask(bool onoff);
extern int mdss_shdisp_display_done(void);
extern int mdss_shdisp_lcd_recovery(void);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00031 */
#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...) \
                pr_debug(",[SHDISP_PERFORM]" fmt, ## args);
#else /* CONFIG_ANDROID_ENGINEERING */
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...)
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif

#endif /* MDSS_SHDISP_H */
