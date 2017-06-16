/* drivers/video/msm/mdss/mdss_shdisp.c  (Display Driver)
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
#include "mdss_shdisp.h"
#include <linux/types.h>
#include <mach/board.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"



#define MDSS_DSI_CTRL_DSI_EN             BIT(0)
#define MDSS_DSI_CTRL_VIDEO_MODE_EN      BIT(1)
#define MDSS_DSI_CTRL_CMD_MODE_EN        BIT(2)

static int lcd_disp_on = 0;
struct mdss_panel_data     * mdss_panel_data;
struct mdss_dsi_ctrl_pdata * mdss_dsi_ctrl;
static int mdss_shdisp_callback_data = 0;
static int mdss_shdisp_collect_cmd_cnt;
static struct dsi_cmd_desc mdss_shdisp_collect_cmds[64];

static int mdss_shdisp_is_cmdmode_on(void);
static int mdss_shdisp_cmdmode_ctrl(int enable);

static struct semaphore mdss_shdisp_recovery_sem;
static int mdss_shdisp_initialized = 0;
static int mdss_shdisp_first_display_done = 0;

extern struct fb_info *registered_fb[FB_MAX] __read_mostly;
extern void mdss_mdp_video_transfer_ctrl(struct msm_fb_data_type *mfd, bool onoff);
extern void mdss_mdp_cmd_transfer_ctrl(struct msm_fb_data_type *mfd, bool onoff);

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_lock_recovery(void)
{
	if (!mdss_shdisp_initialized) {
		mdss_shdisp_initialized = 1;
		sema_init(&mdss_shdisp_recovery_sem,1);
	}
	down(&mdss_shdisp_recovery_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_unlock_recovery(void)
{
	up(&mdss_shdisp_recovery_sem);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
bool mdss_shdisp_get_disp_status(void)
{
	shdisp_api_get_boot_context();

	if( shdisp_api_get_boot_disp_status() ) {
		return true;
	}

	return false;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_on(void)
{
	shdisp_api_main_lcd_power_on();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_power_off(void)
{
	shdisp_api_main_lcd_power_off();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_on(struct mdss_panel_data *pdata)
{
	int iscmdmodeon;
	mdss_panel_data = pdata;
	mdss_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	iscmdmodeon = mdss_shdisp_is_cmdmode_on();
	if (!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	shdisp_api_main_lcd_disp_on();
	if (mdss_panel_data->panel_info.type == MIPI_VIDEO_PANEL) {
		shdisp_api_main_lcd_start_display();
		lcd_disp_on = 1;
	}
	if (mdss_panel_data->panel_info.type == MIPI_CMD_PANEL) {
		shdisp_api_main_lcd_start_display();
		lcd_disp_on = 1;
	}

	if (!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(0);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_ctrl = ctrl_pdata;
	mdss_panel_data = &ctrl_pdata->panel_data;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_off(void)
{
	int iscmdmodeon;

	iscmdmodeon = mdss_shdisp_is_cmdmode_on();
	if(!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(1);
	}

	mdss_shdisp_collect_cmd_cnt = 0;
	shdisp_api_main_lcd_disp_off();
	mdss_dsi_ctrl = NULL;
	mdss_panel_data = NULL;
	lcd_disp_on = 0;

	if(!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(0);
	}
	mdss_shdisp_first_display_done = 0;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_is_disp_on(void)
{
	return lcd_disp_on;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_start_display(void)
{
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_panel_post_video_start()
{
	shdisp_api_main_lcd_post_video_start();
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_bkl_ctl(u32 bl_level)
{
	struct shdisp_main_bkl_ctl bkl;

	pr_debug("%s: called bl_level=%u\n", __func__, bl_level);

	if( bl_level == 0 ) {
		shdisp_api_main_bkl_off();
	} else {
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_shutdown( void )
{
	shdisp_api_shutdown();
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_dsi_to_mdss_dsi(const struct shdisp_dsi_cmd_desc * shdisp_cmds,  struct dsi_cmd_desc * mdss_cmds,
								int size, int isAck)
{
	int cnt;
	for (cnt = 0; cnt != size; cnt++){
		mdss_cmds->dchdr.dtype  = shdisp_cmds->dtype;
		mdss_cmds->dchdr.last = shdisp_cmds->wait ? 1 : 0;
		mdss_cmds->dchdr.vc	    = 0;
		mdss_cmds->dchdr.ack    = isAck ? 1 : 0;
		mdss_cmds->dchdr.wait   = shdisp_cmds->wait ? ((shdisp_cmds->wait+1000)/1000) : 0; /* mdss_dsi(ms) <- shdisp_dsi(usec) */
		mdss_cmds->dchdr.dlen   = shdisp_cmds->dlen;
		mdss_cmds->payload      = shdisp_cmds->payload;
		mdss_cmds++;
		shdisp_cmds++;
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_collect_cmd(struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	mdss_shdisp_dsi_to_mdss_dsi(shdisp_cmds, &mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt], size, 0);
	mdss_shdisp_collect_cmd_cnt += size;
	pr_debug("%s: size=%d, mdss_shdisp_collect_cmd_cnt = %d\n", __func__, size, mdss_shdisp_collect_cmd_cnt );
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_shdisp_kick_collect_cmd(void)
{
	int iscmdmodeon;
	struct dcs_cmd_req cmdreq;

	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return;
	}

	pr_debug("%s: begin cnt=%d", __func__, mdss_shdisp_collect_cmd_cnt);
	if (!mdss_shdisp_collect_cmd_cnt) {
		return;
	}

	mdss_shdisp_collect_cmds[mdss_shdisp_collect_cmd_cnt-1].dchdr.last   = 1;
	pr_debug("%s: kick_count = %d\n", __func__, mdss_shdisp_collect_cmd_cnt );

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.cmds = mdss_shdisp_collect_cmds;
	cmdreq.cmds_cnt = mdss_shdisp_collect_cmd_cnt;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	iscmdmodeon = mdss_shdisp_is_cmdmode_on();
	if(!iscmdmodeon){
		mdss_shdisp_cmdmode_ctrl(1);
	}
	mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);
	if (!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(0);
	}

	mdss_shdisp_collect_cmd_cnt = 0;

	pr_debug("%s: end", __func__);
	return;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_is_cmdmode_on(void)
{
	int dsi_ctrl;
	int cmdmode_on = MDSS_DSI_CTRL_DSI_EN | MDSS_DSI_CTRL_CMD_MODE_EN;
	int ret = 0;

	dsi_ctrl = MIPI_INP((mdss_dsi_ctrl->ctrl_base) + 0x0004);

	if ((dsi_ctrl&cmdmode_on) == cmdmode_on) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_shdisp_cmdmode_ctrl(int enable)
{
	int mode = enable ? DSI_CMD_MODE : DSI_VIDEO_MODE;
	pr_debug("%s: request=%d\n", __func__, enable);
	mdss_dsi_op_mode_config(mode, mdss_panel_data);
	return mode;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_init_cbdata(void)
{
	mdss_shdisp_callback_data = 0xffff;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_get_cbdata(void)
{
	return mdss_shdisp_callback_data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_host_dsi_cb(int data)
{
	mdss_shdisp_callback_data = data;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_tx(int commit, struct shdisp_dsi_cmd_desc * shdisp_cmds, int size)
{
	if (!mdss_dsi_ctrl) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return SHDISP_RESULT_FAILURE;
	}
	mdss_shdisp_dsi_cmd_clk_ctrl(1);
	mdss_shdisp_collect_cmd(shdisp_cmds, size);

	if (commit) {
		mdss_shdisp_kick_collect_cmd();
	}
	mdss_shdisp_dsi_cmd_clk_ctrl(0);

	return SHDISP_RESULT_SUCCESS;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc * cmds, unsigned char * rx_data, int rx_size)
{
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc mdss_cmds;
	char payload[2];
	int iscmdmodeon;

	if ( !mdss_dsi_ctrl ) {
		pr_err("LCDERR: %s mdss_dsi_ctrl=0x%p", __func__, mdss_dsi_ctrl);
		return SHDISP_RESULT_FAILURE;
	}

	mdss_shdisp_dsi_cmd_clk_ctrl(1);
	mdss_shdisp_kick_collect_cmd();

	memset(&cmdreq, 0, sizeof(cmdreq) );
	memset(&mdss_cmds, 0, sizeof(mdss_cmds) );
	mdss_shdisp_dsi_to_mdss_dsi(cmds, &mdss_cmds, 1, 1);
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.cmds = &mdss_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.cb = mdss_shdisp_host_dsi_cb;
	cmdreq.rbuf = rx_data;
	cmdreq.rlen = rx_size;

	if (mdss_cmds.dchdr.dlen>1) {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = mdss_cmds.payload[1];
	} else {
		payload[0] = mdss_cmds.payload[0];
		payload[1] = 0;
	}

	mdss_cmds.payload = payload;

	iscmdmodeon = mdss_shdisp_is_cmdmode_on();
	if (!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(1);
	}

	mdss_shdisp_host_dsi_init_cbdata();
	mdss_dsi_cmdlist_put(mdss_dsi_ctrl, &cmdreq);
	mdss_shdisp_dsi_cmd_clk_ctrl(0);

	pr_debug( "rx_data: payload[0][1] = 0x%02x, 0x%02x\n", payload[0], payload[1] );

	if (cmdreq.rlen != mdss_shdisp_host_dsi_get_cbdata()) {
		pr_err("LCDERR: %s callback_data=%d\n", __func__, mdss_shdisp_host_dsi_get_cbdata());
		if (!iscmdmodeon) {
			mdss_shdisp_cmdmode_ctrl(0);
		}
		return SHDISP_RESULT_FAILURE;
	}

	if (!iscmdmodeon) {
		mdss_shdisp_cmdmode_ctrl(0);
	}
	return SHDISP_RESULT_SUCCESS;
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_video_transfer_ctrl(bool onoff)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	fbi = registered_fb[0];
	mfd = (struct msm_fb_data_type *)fbi->par;
	mdss_mdp_video_transfer_ctrl(mfd, onoff);
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_cmd_transfer_ctrl(bool onoff)
{
	struct fb_info * fbi;
	struct msm_fb_data_type *mfd;
	fbi = registered_fb[0];
	mfd = (struct msm_fb_data_type *)fbi->par;
	mdss_mdp_cmd_transfer_ctrl(mfd, onoff);
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_shdisp_dsi_cmd_clk_ctrl(bool onoff)
{
	if (onoff) {
		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, DSI_ALL_CLKS, 1);
	} else {
		mdss_dsi_clk_ctrl(mdss_dsi_ctrl, DSI_ALL_CLKS, 0);
	}
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
void mdss_dsi_dln0_err_contention_lp1_mask(bool onoff)
{
    u32 status;
    unsigned char *base;

    base = mdss_dsi_ctrl->ctrl_base;
    if (onoff) {
        /* disable */
        status = MIPI_INP(base + 0x010c);/* MMSS_DSI_0_ERR_INT_MASK0 */
        status |= BIT(25);/* BIT(25) DLN0_ERR_CONTENTION_LP1_MASK */
        MIPI_OUTP(base + 0x010c, status);
    } else {
        /* enable */
        status = MIPI_INP(base + 0x010c);/* MMSS_DSI_0_ERR_INT_MASK0 */
        status &= (~ BIT(25));/* BIT(25) DLN0_ERR_CONTENTION_LP1_MASK */
        MIPI_OUTP(base + 0x010c, status);
    }
}


/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_shdisp_display_done(void)
{
	int ret = SHDISP_RESULT_SUCCESS;

	if (mdss_shdisp_first_display_done == 0) {
		ret = shdisp_api_display_done();
		mdss_shdisp_first_display_done = 1;
	}
	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */

int mdss_shdisp_lcd_recovery(void)
{
	int ret = SHDISP_RESULT_SUCCESS;
	ret = shdisp_api_do_lcd_det_recovery();
	return ret;
}


