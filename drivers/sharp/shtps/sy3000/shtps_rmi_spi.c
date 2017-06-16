/* drivers/sharp/shtps/sy3000/shtps_rmi_spi.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>

#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/perflock.h>
#include <mach/cpuidle.h>
#include <linux/pm_qos.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#include "shtps_rmi_spi.h"
#include "shtps_rmi_devctl.h"
#include "shtps_rmi_debug.h"

#ifdef CONFIG_SHTERM
	#include <sharp/shterm_k.h>
#endif


/* -----------------------------------------------------------------------------------
 */
static DEFINE_MUTEX(shtps_ctrl_lock);
static DEFINE_MUTEX(shtps_loader_lock);

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_clock_ctrl_lock);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_idle_sleep_ctrl_lock);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_sleep_ctrl_for_fwupdate_lock);
#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	static DEFINE_MUTEX(shtps_proc_lock);
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	static DEFINE_MUTEX(shtps_facetouch_qosctrl_lock);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

/* -----------------------------------------------------------------------------------
 */
/* for sysfs I/F */
#if defined(SHTPS_ENGINEER_BUILD_ENABLE)
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUGO | S_IWUGO), show_func, store_func)
#else
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), show_func, store_func)
#endif /* SHTPS_ENGINEER_BUILD_ENABLE */

#define SHTPSIF_SHOW_COMMON		shtpsif_show_common
#define SHTPSIF_STORE_COMMON	shtpsif_store_common

#define SHTPSIF_LOG_FUNC_CALL()								\
    if((gShtpsIfLog & 0x01) != 0){							\
        printk(KERN_DEBUG "[shtpsif] %s()\n", __func__);	\
    }

static int gShtpsIfLog = 0;

static char shtps_fwupdate_buffer[SHTPS_FWDATA_BLOCK_SIZE_MAX];
static int shtps_fwupdate_datasize = 0;

static unsigned short shtpsif_reg_read_addr = 0;
static unsigned char shtpsif_reg_read_size = 1;

#define SHTPSIF_REG_READ_SIZE_MAX	100
#define SHTPSIF_REG_WRITE_SIZE_MAX	100

/* -----------------------------------------------------------------------------------
 */

struct shtps_irq_info{
	int							irq;
	u8							state;
	u8							wake;
};

struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_loader_info{
	int							ack;
	wait_queue_head_t			wait_ack;
};

struct shtps_diag_info{
	u8							pos_mode;
	u8							tm_mode;
	u8							tm_data[SHTPS_TM_TXNUM_MAX * SHTPS_TM_RXNUM_MAX * 2];
	int							event;
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	int							event_touchkey;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
	int							tm_ack;
	int							tm_stop;
	wait_queue_head_t			wait;
	wait_queue_head_t			tm_wait_ack;
};

struct shtps_facetouch_info{
	int							mode;
	int							state;
	int							detect;
	int							palm_thresh;
	int							wake_sig;
	u8							palm_det;
	u8							touch_num;
	wait_queue_head_t			wait_off;
	
	u8							wakelock_state;
	struct wake_lock            wake_lock;
	struct pm_qos_request		qos_cpu_latency;
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
	int							facetouch_off_notify_delayed_state;
	int							facetouch_off_force_flg;
	struct delayed_work			notify_interval_delayed_work;
	
	struct wake_lock            facetouch_off_delayed_wake_lock;
	struct pm_qos_request		facetouch_off_delayed_qos;
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
	int							rezero_disable;
#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
	
	
};

struct shtps_offset_info{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_polling_info{
	int							boot_rezero_flag;
	int							stop_margin;
	int							stop_count;
	int							single_fingers_count;
	int							single_fingers_max;
	u8							single_fingers_enable;
};

struct shtps_drag_hist{
	int							pre;
	u8							dir;
	u8							count;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int						history[SHTPS_DRAG_HISTORY_SIZE_MAX];
		int						pre_comp_history_FIXED;
		int						history_old;
		int						count_up_base;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
};

#if defined(SHTPS_LPWG_MODE_ENABLE)
struct shtps_lpwg_ctrl{
    u8                          lpwg_switch;
    u8                          lpwg_state;
    u8                          is_notified;
    u8                          notify_enable;
    u8                          doze_wakeup_threshold;
    u8                          clip[2];
    u8                          fn11_ctrl92_enable_settings[0x10];
    u8                          fn11_ctrl92_disable_settings[0x10];
	struct wake_lock            wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	signed long					pm_qos_idle_value;
	unsigned long				notify_time;
	struct delayed_work			notify_interval_delayed_work;
	u8							block_touchevent;
	unsigned long				wakeup_time;
	u8							tu_rezero_req;

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		struct shtps_touch_info		pre_info;
		unsigned long				swipe_check_time;
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */
};
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
struct shtps_req_msg {
	struct list_head queue;
	void	(*complete)(void *context);
	void	*context;
	int		status;
	int		event;
	void	*param_p;
};
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
enum{
	SHTPS_DETER_SUSPEND_SPI_PROC_IRQ = 0x00,
	SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR,
	SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE,
	SHTPS_DETER_SUSPEND_SPI_PROC_OPEN,
	SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE,
	SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE,

	SHTPS_DETER_SUSPEND_SPI_PROC_NUM,
};

struct shtps_deter_suspend_spi{
	u8							suspend;
	struct work_struct			pending_proc_work;
	u8							wake_lock_state;
	struct wake_lock			wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	
	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		struct delayed_work			pending_proc_work_delay;
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	
	struct shtps_deter_suspend_spi_pending_info{
		u8						pending;
		u8						param;
	} pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_NUM];

	u8							suspend_irq_state;
	u8							suspend_irq_wake_state;
	u8							suspend_irq_detect;
};
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
struct shtps_edge_fail_touch_info{
	unsigned short	x;
	unsigned short	y;
	unsigned char	z;
	u8				id;
	unsigned long	time;
};
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
struct shtps_pinch_fail_reject{
	u8				segmentation_aggressiveness_def;
	u8				segmentation_aggressiveness_set_state;
	u8				segmentation_aggressiveness_set_check;
	u32				finger_distance;
};
#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
struct shtps_lgm_split_touch_combining{
	u8					finger_swap;
	u8					finger_adjust;
};
#endif  /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

#if defined(SHTPS_CLING_REJECTION_ENABLE)
struct shtps_cling_reject{
	u8									mode0_cling_cnt[SHTPS_FINGER_MAX];
	u8									mode0_cling_cnt_max;
	unsigned long						mode0_cling_cnt_change_time;
	u8									mode0_cling_cnt_change_state;
	u8									mode0_state[SHTPS_FINGER_MAX];
	unsigned short						mode0_tap_x[SHTPS_FINGER_MAX];
	unsigned short						mode0_tap_y[SHTPS_FINGER_MAX];
	u8									mode1_cling_cnt_wx[SHTPS_FINGER_MAX];
	u8									mode1_cling_cnt_wy[SHTPS_FINGER_MAX];
};
#endif /* SHTPS_CLING_REJECTION_ENABLE */

#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
struct shtps_lgm_fail_touch_up_reject{
	u8					reg_set_state;
	u8					chatt_enable;
	unsigned long		chatt_time;
	u8					chatt_finger;
	unsigned long		chatt_finger_tu_ignore_time[SHTPS_FINGER_MAX];
	struct delayed_work	chatt_work;
	u8					chatt_work_enable;
	u16					pos_move[SHTPS_FINGER_MAX];
	u8					chatt_check_enable_finger;
	u16					td_pos[SHTPS_FINGER_MAX][2];
	u8					z_max[SHTPS_FINGER_MAX];
};
#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */

#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
struct shtps_quick_flick_fail_resolv{
	int									enable;
	int									finger;
	int									count;
	int									pre_posx;
	int									pre_posy;
	unsigned long						pre_time;
};
#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */

#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
struct shtps_scan_freq_change{
	int									count;
	int									detect;
	unsigned long						pre_time;
	unsigned char						reg_freq[2];
};
#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
struct shtps_filter_topofscreen_ghost_reject_info{
	u8						is_ghost[SHTPS_FINGER_MAX];
	unsigned long			timeout[SHTPS_FINGER_MAX];
	struct delayed_work		ghost_clear_check_delayed_work;
};

enum{
	TOP_OF_SCREEN_GHOST_NOT_DETECTED,
	TOP_OF_SCREEN_GHOST_DETECTED,
};

enum{
	TOP_OF_SCREEN_GHOST_NOT_CLEARED,
	TOP_OF_SCREEN_GHOST_CLEARED,
};
#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

struct shtps_rmi_spi {
	struct spi_device*			spi;
	struct input_dev*			input;
	int							rst_pin;
	struct shtps_irq_info		irq_mgr;
	struct shtps_touch_info		fw_report_info;
	struct shtps_touch_info		fw_report_info_store;
	struct shtps_touch_info		report_info;
	struct shtps_touch_info		center_info;
	struct shtps_state_info		state_mgr;
	struct shtps_loader_info	loader;
	struct shtps_diag_info		diag;
	struct shtps_facetouch_info	facetouch;
	struct shtps_offset_info	offset;
	struct shtps_polling_info	poll_info;
	struct rmi_map				map;
	struct shtps_touch_state	touch_state;
	wait_queue_head_t			wait_start;
	struct delayed_work 		tmo_check;
	unsigned char				finger_state[3];	/* SHTPS_FINGER_MAX/4+1 */
	struct hrtimer				rezero_delayed_timer;
	struct work_struct			rezero_delayed_work;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		struct workqueue_struct		*workqueue_p;
		struct work_struct			work_data;
		struct list_head			queue;
		spinlock_t					queue_lock;
		struct shtps_req_msg		*cur_msg_p;
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		u8							lpmode_req_state;
		u8							lpmode_continuous_req_state;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	u16							bt_ver;
	char						phys[32];

	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		u8							charger_armor_state;
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		struct input_dev*			input_key;
		u16							keycodes[2];
		u8							key_state;
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		u16							system_boot_mode;
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		struct perf_lock			perf_lock;
		struct delayed_work			perf_lock_disable_delayed_work;
		int							perf_lock_enable_time_ms;
		int							report_event;
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	struct shtps_drag_hist		drag_hist[SHTPS_FINGER_MAX][2];

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		struct pm_qos_request		qos_cpu_latency;
		int							wake_lock_idle_state;
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
		struct wake_lock			wake_lock_for_fwupdate;
		struct pm_qos_request		qos_cpu_latency_for_fwupdate;
		int							wake_lock_for_fwupdate_state;
	#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		u8							absorption_hold_enable;
		u8							absorption_hold_tu_finger;
		struct shtps_touch_info		absorption_hold_finger_info;
		struct delayed_work			absorption_hold_off_delayed_work;
		
		#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
			unsigned short			pre_diff_x;
			unsigned short			pre_diff_y;
		#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined( SHTPS_FINGER_WIDTH_MODERATION_ENABLE )
		int	w_before_gain[SHTPS_FINGER_MAX];
	#endif	/* SHTPS_FINGER_WIDTH_MODERATION_ENABLE */

	#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
		u8							pixel_touch_threshold_def_val;
	#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */
	
    #if defined(SHTPS_LPWG_MODE_ENABLE)
        struct shtps_lpwg_ctrl		lpwg;

		#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
			int							lpwg_proximity_get_data;
			struct wake_lock            wake_lock_proximity;
			struct pm_qos_request		pm_qos_lock_idle_proximity;
			struct delayed_work			proximity_check_delayed_work;
		#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
    #endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined(SHTPS_MULTI_FW_ENABLE)
		u8							multi_fw_type;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		u8							wakeup_touch_event_inhibit_state;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		struct shtps_deter_suspend_spi		deter_suspend_spi;
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	struct kobject				*kobj;

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		u8									edge_fail_touch_enable;
		u8									edge_fail_touch_inhibit_id;
		u8									edge_fail_touch_decide_mt;
		struct shtps_edge_fail_touch_info	edge_fail_touch_td_info[SHTPS_FINGER_MAX];
		u8									edge_fail_touch_td_cnt;
		u8									edge_fail_touch_pre_single;
		u8									edge_fail_touch_side_inhibit_id;
		struct shtps_touch_info				edge_fail_touch_side_td_info;
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		u8							grip_fail_touch_inhibit_id;
		u8							grip_fail_flick_inhibit_id;
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		struct shtps_pinch_fail_reject	pinch_fail_reject;
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	u8							dev_state;

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		struct shtps_lgm_split_touch_combining	lgm_split_touch_combining;
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		u8							key_down_reserved;
		u8							key_down_ignored;
		u8							key_proximity_check_state;
		struct delayed_work			touchkey_delayed_work;
		struct delayed_work			touchkey_inproxymity_delayed_work;
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

	#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
		struct delayed_work			wakeup_calib_delayed_work;
		u8							wakeup_calib_enable;
		u8							wakeup_calib_cancel_check_enable;
		u8							wakeup_calib_deferment;
		u8							wakeup_calib_lastone;
		u8							wakeup_calib_lastone_req;
		unsigned short				wakeup_calib_td_pos[SHTPS_FINGER_MAX][2];
		u8							wakeup_calib_cancel_check[SHTPS_FINGER_MAX];
		u8							wakeup_calib_event_count[SHTPS_FINGER_MAX];
		u8							wakeup_calib_rezero_exec_num;
	#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

	#if defined( SHTPS_CLING_REJECTION_ENABLE )
		struct shtps_cling_reject	cling_reject;
	#endif /* SHTPS_CLING_REJECTION_ENABLE */
	
	#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
		struct shtps_lgm_fail_touch_up_reject	lgm_fail_touch_up_reject;
	#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE)
		struct shtps_touch_info		edge_shake_td_info;
		u8							edge_shake_check_state[SHTPS_FINGER_MAX];
	#endif /* SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE */

	#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
		struct shtps_quick_flick_fail_resolv	quick_flick_fail_resolv;
	#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */

	#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
		struct shtps_scan_freq_change	scan_freq_change;
	#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		struct shtps_filter_topofscreen_ghost_reject_info	topofscreen_ghost_reject;
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */
};

static dev_t 					shtpsif_devid;
static struct class*			shtpsif_class;
static struct device*			shtpsif_device;
struct cdev 					shtpsif_cdev;
static struct shtps_rmi_spi*	gShtps_rmi_spi = NULL;

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static u8						gLogOutputEnable = 0;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_MODULE_PARAM_ENABLE )
	static int shtps_irq_wake_state = 0;
	static int shtps_spi_clk_ctrl_state = 0;
	static int shtps_rezero_state = 0;

	module_param(shtps_irq_wake_state, int, S_IRUGO);
	module_param(shtps_spi_clk_ctrl_state, int, S_IRUGO);
	module_param(shtps_rezero_state, int, S_IRUGO | S_IWUSR);
#endif /* SHTPS_MODULE_PARAM_ENABLE */

/* -----------------------------------------------------------------------------------
 */
enum{
	SHTPS_FWTESTMODE_V01 = 0x00,
	SHTPS_FWTESTMODE_V02,
	SHTPS_FWTESTMODE_V03,
};

enum{
	SHTPS_REZERO_REQUEST_REZERO				= 0x01,
	SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE	= 0x02,
	SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE	= 0x04,
	SHTPS_REZERO_REQUEST_WAKEUP_REZERO		= 0x08,
};

enum{
	SHTPS_REZERO_HANDLE_EVENT_MTD = 0,
	SHTPS_REZERO_HANDLE_EVENT_TOUCH,
	SHTPS_REZERO_HANDLE_EVENT_TOUCHUP,
};

enum{
	SHTPS_REZERO_TRIGGER_BOOT = 0,
	SHTPS_REZERO_TRIGGER_WAKEUP,
	SHTPS_REZERO_TRIGGER_ENDCALL,
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

enum{
	SHTPS_TOUCH_STATE_NO_TOUCH = 0,
	SHTPS_TOUCH_STATE_FINGER,
	SHTPS_TOUCH_STATE_PEN,
	SHTPS_TOUCH_STATE_HOVER,
};

enum{
	SHTPS_STARTUP_SUCCESS,
	SHTPS_STARTUP_FAILED
};

enum{
	SHTPS_IRQ_WAKE_DISABLE,
	SHTPS_IRQ_WAKE_ENABLE,
};

enum{
	SHTPS_IRQ_STATE_DISABLE,
	SHTPS_IRQ_STATE_ENABLE,
};

enum{
	SHTPS_MODE_NORMAL,
	SHTPS_MODE_LOADER,
};

enum{
	SHTPS_EVENT_START,
	SHTPS_EVENT_STOP,
	SHTPS_EVENT_SLEEP,
	SHTPS_EVENT_WAKEUP,
	SHTPS_EVENT_STARTLOADER,
	SHTPS_EVENT_STARTTM,
	SHTPS_EVENT_STOPTM,
	SHTPS_EVENT_FACETOUCHMODE_ON,
	SHTPS_EVENT_FACETOUCHMODE_OFF,
	SHTPS_EVENT_INTERRUPT,
	SHTPS_EVENT_TIMEOUT,
};

enum{
	SHTPS_STATE_IDLE,
	SHTPS_STATE_WAIT_WAKEUP,
	SHTPS_STATE_WAIT_READY,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_BOOTLOADER,
	SHTPS_STATE_FACETOUCH,
	SHTPS_STATE_FWTESTMODE,
	SHTPS_STATE_SLEEP,
	SHTPS_STATE_SLEEP_FACETOUCH,
};

enum{
	SHTPS_PHYSICAL_KEY_DOWN = 0,
	SHTPS_PHYSICAL_KEY_UP,
	SHTPS_PHYSICAL_KEY_NUM,
};

enum{
	SHTPS_IRQ_FLASH		= 0x01,
	SHTPS_IRQ_STATE		= 0x02,
	SHTPS_IRQ_ABS 		= 0x04,
	SHTPS_IRQ_ANALOG 	= 0x08,
	SHTPS_IRQ_BUTTON 	= 0x10,
	SHTPS_IRQ_SENSOR 	= 0x20,
	SHTPS_IRQ_ALL		= (  SHTPS_IRQ_FLASH
							| SHTPS_IRQ_STATE
							| SHTPS_IRQ_ABS
							| SHTPS_IRQ_ANALOG
							| SHTPS_IRQ_BUTTON
							| SHTPS_IRQ_SENSOR),
};

enum{
	SHTPS_LPMODE_TYPE_NON_CONTINUOUS = 0,
	SHTPS_LPMODE_TYPE_CONTINUOUS,
};

enum{
	SHTPS_LPMODE_REQ_NONE		= 0x00,
	SHTPS_LPMODE_REQ_COMMON		= 0x01,
	SHTPS_LPMODE_REQ_ECO		= 0x02,
	SHTPS_LPMODE_REQ_HOVER_OFF	= 0x04,
	SHTPS_LPMODE_REQ_LCD_BRIGHT	= 0x08,
};

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
enum{
	SHTPS_FUNC_REQ_EVEMT_OPEN = 0,
	SHTPS_FUNC_REQ_EVEMT_CLOSE,
	SHTPS_FUNC_REQ_EVEMT_ENABLE,
	SHTPS_FUNC_REQ_EVEMT_DISABLE,

	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK,
	#endif /*SHTPS_PROXIMITY_SUPPORT_ENABLE */
};
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

enum{
	SHTPS_DRAG_DIR_NONE = 0,
	SHTPS_DRAG_DIR_PLUS,
	SHTPS_DRAG_DIR_MINUS,
};

#if defined(SHTPS_LPWG_MODE_ENABLE)
enum{
	SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE			= 0x00,
	SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP	= 0x01,
	SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE		= 0x02,
};

enum{
	SHTPS_LPWG_STATE_OFF		= 0x00,
	SHTPS_LPWG_STATE_ON			= 0x01,
	SHTPS_LPWG_STATE_GRIP_ONLY	= 0x02,
};
#endif /*  SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET) || defined(SHTPS_CHECK_HWID_ENABLE)
enum{
	SHTPS_HW_TYPE_BOARD = 0,
	SHTPS_HW_TYPE_HANDSET,
};
#endif /* defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET) || defined(SHTPS_CHECK_HWID_ENABLE) */

#if defined(SHTPS_CHECK_HWID_ENABLE)
enum{
	SHTPS_HW_REV_ES_0 = 0,
	SHTPS_HW_REV_ES_1,
	SHTPS_HW_REV_PP_1,
	SHTPS_HW_REV_PP_2,
};
#endif /* SHTPS_CHECK_HWID_ENABLE */

#if defined(SHTPS_MULTI_FW_ENABLE)
enum{
	SHTPS_FWTYPE_NORMAL = 0,
	SHTPS_FWTYPE_ES0,
	SHTPS_FWTYPE_ES1,
	SHTPS_FWTYPE_UNKNOWN,
};
#endif /* SHTPS_MULTI_FW_ENABLE */

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
enum{
	SHTPS_PROXIMITY_ENABLE = SH_PROXIMITY_ENABLE,
	SHTPS_PROXIMITY_DISABLE = SH_PROXIMITY_DISABLE,
};

enum{
	SHTPS_PROXIMITY_NEAR = SH_PROXIMITY_NEAR,
	SHTPS_PROXIMITY_FAR = SH_PROXIMITY_FAR,
};
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

enum{
	SHTPS_DEV_STATE_SLEEP = 0,
	SHTPS_DEV_STATE_DOZE,
	SHTPS_DEV_STATE_ACTIVE,
};

typedef int (shtps_state_func)(struct shtps_rmi_spi *ts, int param);
struct shtps_state_func {
	shtps_state_func	*enter;
	shtps_state_func	*start;
	shtps_state_func	*stop;
	shtps_state_func	*sleep;
	shtps_state_func	*wakeup;
	shtps_state_func	*start_ldr;
	shtps_state_func	*start_tm;
	shtps_state_func	*stop_tm;
	shtps_state_func	*facetouch_on;
	shtps_state_func	*facetouch_off;
	shtps_state_func	*interrupt;
	shtps_state_func	*timeout;
};

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param);
static int state_change(struct shtps_rmi_spi *ts, int state);
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param);
static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts);
static int shtps_start(struct shtps_rmi_spi *ts);
static int shtps_wait_startup(struct shtps_rmi_spi *ts);
static u16 shtps_fwver(struct shtps_rmi_spi *ts);
static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param);
static void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event);
static void shtps_forcecal(struct shtps_rmi_spi *ts);
static void shtps_rezero(struct shtps_rmi_spi *ts);
inline static void shtps_report_touch_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
inline static void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
static void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state);
static void shtps_event_force_touchup(struct shtps_rmi_spi *ts);

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on);
static void shtps_get_lpwg_def_settings(struct shtps_rmi_spi *ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
static void shtps_wake_lock_idle(struct shtps_rmi_spi *ts);
static void shtps_wake_unlock_idle(struct shtps_rmi_spi *ts);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */
#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
static void shtps_wake_lock_for_fwupdate(struct shtps_rmi_spi *ts);
static void shtps_wake_unlock_for_fwupdate(struct shtps_rmi_spi *ts);
#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data);
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
	static void shtps_absorption_hold_cancel(struct shtps_rmi_spi *ts);
#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE) || defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	static void shtps_func_request_async( struct shtps_rmi_spi *ts, int event);
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE || SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	static void shtps_irq_enable(struct shtps_rmi_spi *ts);
	static void shtps_irq_disable(struct shtps_rmi_spi *ts);
	#if !defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts);
		static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts);
	#endif /* !SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	static int shtps_is_lpwg_active(struct shtps_rmi_spi *ts);
	static void shtps_set_lpwg_mode_on(struct shtps_rmi_spi *ts);
	static void shtps_set_lpwg_mode_off(struct shtps_rmi_spi *ts);
	static void shtps_sleep(struct shtps_rmi_spi *ts, int on);
	static int shtps_set_charger_armor(struct shtps_rmi_spi *ts, int on);
	static void shtps_set_lpmode(struct shtps_rmi_spi *ts, int type, int req, int on);
	
	static void shtps_irq_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_charger_armor_proc(struct shtps_rmi_spi *ts, u8 charger);
	static void shtps_setsleep_proc(struct shtps_rmi_spi *ts, u8 sleep);
	static void shtps_ioctl_setlpwg_proc(struct shtps_rmi_spi *ts, u8 on);
	static void shtps_ioctl_setlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	static void shtps_ioctl_setconlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	static void shtps_ioctl_setlcdbrightlpmode_proc(struct shtps_rmi_spi *ts, u8 on);
	static void shtps_async_open_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_async_close_proc(struct shtps_rmi_spi *ts, u8 dummy);
	static void shtps_async_enable_proc(struct shtps_rmi_spi *ts, u8 dummy);
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
static void shtps_event_touch_cancel(struct shtps_rmi_spi *ts, u8 id);
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT)
	static void shtps_set_touch_info(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info);
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
	static int shtps_wakeup_calib_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms);
	static int shtps_wakeup_calib_timer_stop(struct shtps_rmi_spi *ts);
#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
	static void shtps_scan_freq_dynamic_change_reg_init(struct shtps_rmi_spi *ts);
#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
const static struct shtps_state_func state_idle = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_idle_start,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_idle_start_ldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_idle_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_waiwakeup = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_waiwakeup_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_waiwakeup_int,
    .timeout        = shtps_statef_waiwakeup_tmo
};

const static struct shtps_state_func state_waiready = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_state_waiready_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_state_waiready_int,
    .timeout        = shtps_state_waiready_tmo
};

const static struct shtps_state_func state_active = {
    .enter          = shtps_statef_active_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_active_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_active_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_active_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .interrupt      = shtps_statef_active_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_loader = {
    .enter          = shtps_statef_loader_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_loader_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_nop,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_loader_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_facetouch = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_facetouch_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_facetouch_facetouch_off,
    .interrupt      = shtps_statef_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_fwtm = {
    .enter          = shtps_statef_fwtm_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_fwtm_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_fwtm_stoptm,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .interrupt      = shtps_statef_fwtm_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep = {
    .enter          = shtps_statef_sleep_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_sleep_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep_facetouch = {
    .enter          = shtps_statef_sleep_facetouch_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_sleep_facetouch_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_facetouch_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_sleep_facetouch_facetouch_off,
    .interrupt      = shtps_statef_sleep_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func *state_func_tbl[] = {
	&state_idle,
	&state_waiwakeup,
	&state_waiready,
	&state_active,
	&state_loader,
	&state_facetouch,
	&state_fwtm,
	&state_sleep,
	&state_sleep_facetouch,
};

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
typedef void (*shtps_deter_suspend_spi_pending_func_t)(struct shtps_rmi_spi*, u8);
static const shtps_deter_suspend_spi_pending_func_t SHTPS_SUSPEND_PENDING_FUNC_TBL[] = {
	shtps_irq_proc,							/**< SHTPS_DETER_SUSPEND_SPI_PROC_IRQ */
	shtps_charger_armor_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR */
	shtps_setsleep_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP */
	shtps_ioctl_setlpwg_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG */
	shtps_ioctl_setlpmode_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE */
	shtps_ioctl_setconlpmode_proc,			/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE */
	shtps_ioctl_setlcdbrightlpmode_proc,	/**< SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE */
	shtps_async_open_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_OPEN */
	shtps_async_close_proc,					/**< SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE */
	shtps_async_enable_proc,				/**< SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE */
};
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_PERFORMANCE_CHECK_STATE_START	(0)
#define SHTPS_PERFORMANCE_CHECK_STATE_CONT	(1)
#define SHTPS_PERFORMANCE_CHECK_STATE_END	(2)

#if defined(SHTPS_PERFORMANCE_CHECK_PIN_ENABLE)
	#define SHTPS_PERFORMANCE_CHECK_PIN		(32)
	static int shtps_performance_check_pin_state = 0;
#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
	static int shtps_performace_log_point = 0;
	static struct timeval shtps_performance_log_tv;
#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */

static inline void shtps_performance_check_init(void)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		int result;
		result = gpio_request(SHTPS_PERFORMANCE_CHECK_PIN, "tps_test");
		if(result < 0){
			DBG_PRINTK("test pin gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SHTPS_PERFORMANCE_CHECK_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			DBG_PRINTK("test pin gpio_tlmm_config() error : %d\n", result);
		}

		shtps_performance_check_pin_state = 0;
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		shtps_performace_log_point = 0;
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}

static inline void shtps_performance_check(int state)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performance_check_pin_state = 1;
		}else{
			shtps_performance_check_pin_state = 
				(shtps_performance_check_pin_state == 0)? 1 : 0;
		}
		
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
		
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_END){
			shtps_performance_check_pin_state = 0;
			gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
		}
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performace_log_point = 1;
		}else{
			static struct timeval tv;
			do_gettimeofday(&tv);
			
			DBG_PRINTK("[performace] pt:%02d time:%ldus\n",
				shtps_performace_log_point++,
				(tv.tv_sec * 1000000 + tv.tv_usec) - 
					(shtps_performance_log_tv.tv_sec * 1000000 + shtps_performance_log_tv.tv_usec));
		}
		do_gettimeofday(&shtps_performance_log_tv);
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
#ifdef CONFIG_SHTERM
static void shtps_send_shterm_event(int event_num)
{
	shbattlog_info_t info;
	memset(&info, 0x00, sizeof(info));
	info.event_num = event_num;
	shterm_k_set_event(&info);
}
#endif  /* CONFIG_SHTERM */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static int shtps_is_uimode(struct shtps_rmi_spi *ts)
{
	switch( ts->state_mgr.state ){
		case SHTPS_STATE_ACTIVE:
		case SHTPS_STATE_FACETOUCH:
		case SHTPS_STATE_SLEEP:
		case SHTPS_STATE_SLEEP_FACETOUCH:
			return 1;

		default:
			return 0;
	}
}
#endif /* SHTPS_CHARGER_ARMOR_ENABLE */

#if defined(SHTPS_CHECK_HWID_ENABLE)
static int shtps_system_get_hw_revision(void)
{
	return sh_boot_get_hw_revision();
}
#endif /* SHTPS_CHECK_HWID_ENABLE */

#if (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || \
		(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE))
static int shtps_system_get_hw_type(void)
{
	unsigned char handset;
	int ret = 0;

	handset = sh_boot_get_handset();

	if(handset == 0){
		ret = SHTPS_HW_TYPE_BOARD;
	}else{
		ret = SHTPS_HW_TYPE_HANDSET;
	}

	return ret;
}
#endif /* (defined(SHTPS_BOOT_FWUPDATE_ENABLE) && defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)) || 
			(defined(SHTPS_MULTI_FW_ENABLE) && defined(SHTPS_CHECK_HWID_ENABLE)) */

/* -----------------------------------------------------------------------------------
 */
static void shtps_system_set_sleep(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE)
		shtps_device_sleep(shtpsif_device);
	#endif /* SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE */
}

static void shtps_system_set_wakeup(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE)
		shtps_device_wakeup(shtpsif_device);
	#endif /* SHTPS_INPUT_POWER_MODE_CHANGE_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_device_access_setup(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_active(struct spi_device *spi);

		sh_spi_tps_active(ts->spi);
		SHTPS_LOG_DBG_PRINT("spi active\n");

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_spi_clk_ctrl_state = 1;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	#endif	/* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

static int shtps_device_access_teardown(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_standby(struct spi_device *spi);

		sh_spi_tps_standby(ts->spi);
		SHTPS_LOG_DBG_PRINT("spi standby\n");

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_spi_clk_ctrl_state = 0;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	#endif /* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
static int shtps_proximity_state_check(struct shtps_rmi_spi *ts)
{
	int state = -1;
	int data = -1;

	SHTPS_LOG_DBG_PRINT("[proximity] check state start\n");
	PROX_stateread_func(&state, &data);
	SHTPS_LOG_DBG_PRINT("[proximity] check state end\n");

	if (state == SHTPS_PROXIMITY_ENABLE &&
		data == SHTPS_PROXIMITY_NEAR) {
		return 1;
	} else {
		return 0;
	}
}

static int shtps_proximity_check(struct shtps_rmi_spi *ts)
{
	int data = -1;

	SHTPS_LOG_DBG_PRINT("[proximity] check start\n");
	PROX_dataread_func(&data);
	SHTPS_LOG_DBG_PRINT("[proximity] check end\n");

	return data;
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_write_fw_data(struct shtps_rmi_spi *ts, u16 addr, u8 *buf)
{
	struct spi_message	message;
#ifdef	SHTPS_SPI_FAST_FW_TRANSFER_ENABLE
	struct spi_transfer	t;
	u8					tx_buf[20];
#else
	struct spi_transfer	t[17];
	u8					tx_buf[2];
#endif	/* SHTPS_SPI_FAST_FW_TRANSFER_ENABLE */
	u16					blockSize;
	int					status = 0;
	int					i;
	int					retry = SHTPS_SPI_RETRY_COUNT;

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	do{
#ifdef	SHTPS_SPI_FAST_FW_TRANSFER_ENABLE
		memset(&t, 0, sizeof(t));

		tx_buf[0] = (addr >> 8) & 0xff;
		tx_buf[1] = addr & 0xff;

		for(i = 0; i < blockSize; i++){
			tx_buf[2+i] = buf[i];
		}

		spi_message_init(&message);

		t.tx_buf 		= tx_buf;
		t.rx_buf 		= NULL;
		t.len    		= 2+blockSize;
		t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t, &message);

		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
			status = spi_sync(ts->spi, &message);
		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
#else
		memset(&t, 0, sizeof(t));

		tx_buf[0] = (addr >> 8) & 0xff;
		tx_buf[1] = addr & 0xff;

		spi_message_init(&message);

		t[0].tx_buf 		= tx_buf;
		t[0].rx_buf 		= NULL;
		t[0].len    		= 2;
		t[0].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t[0].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t[0].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t[0], &message);

		for(i = 0; i < blockSize; i++){
			t[i+1].tx_buf 		= &buf[i];
			t[i+1].rx_buf 		= NULL;
			t[i+1].len    		= 1;
			t[i+1].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
			#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
				t[i+1].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
			#else
				t[i+1].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
			#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

			spi_message_add_tail(&t[i+1], &message);
		}

		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
			status = spi_sync(ts->spi, &message);
		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
#endif	/* SHTPS_SPI_FAST_FW_TRANSFER_ENABLE */
		if(status){
			struct timespec tu;
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
			SHTPS_LOG_DBG_PRINT("wait for spi write\n");
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}
#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_write_async_spicomplete(void *arg)
{
	complete(arg);
	udelay(SHTPS_SY3X00_SPI_TANSACTION_WRITE_WAIT);
}
static int shtps_rmi_write_async(struct shtps_rmi_spi *ts, u16 addr, u8 data)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8					tx_buf[3];
	int					status;

	memset(&t, 0, sizeof(t));

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);
	t.tx_buf 		= tx_buf;
	t.rx_buf 		= NULL;
	t.len    		= 3;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#else
		t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;
	tx_buf[2] = data;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	message.complete = shtps_rmi_write_async_spicomplete;
	message.context  = &done;
	status = spi_async(ts->spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	return status;
}

#if defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
static int shtps_rmi_write_block(struct shtps_rmi_spi *ts, u16 addr, u8 *data, u32 size)
{
	struct spi_message	message;
	struct spi_transfer	t_local[1 + SHTPS_SY3X00_SPIBLOCKWRITE_BUFSIZE];
	struct spi_transfer	*t;
	u8					tx_buf[2];
	int					status;
	int 				i;
	int 				allocSize = 0;

	if(size > SHTPS_SY3X00_SPIBLOCKWRITE_BUFSIZE){
		allocSize = sizeof(struct spi_transfer) * (1 + size);
		t = (struct spi_transfer *)kzalloc(allocSize, GFP_KERNEL);
		if(t == NULL){
			SHTPS_LOG_DBG_PRINT("shtps_rmi_write_block() alloc error. size = %d\n", allocSize);
			return -ENOMEM;
		}
		memset(t, 0, allocSize);
	} else {
		t = t_local;
		memset(t, 0, sizeof(t_local));
	}

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;

	spi_message_init(&message);

	t[0].tx_buf 		= tx_buf;
	t[0].rx_buf 		= NULL;
	t[0].len    		= 2;
	t[0].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t[0].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#else
		t[0].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	spi_message_add_tail(&t[0], &message);

	for(i = 0; i < size; i++){
		t[i+1].tx_buf 		= &data[i];
		t[i+1].rx_buf 		= NULL;
		t[i+1].len    		= 1;
		t[i+1].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t[i+1].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t[i+1].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t[i+1], &message);
	}

	status = spi_sync(ts->spi, &message);

	if(allocSize > 0){
		kfree(t);
	}

	return status;
}
#endif /* SHTPS_SPI_BLOCKACCESS_ENABLE */

static int shtps_rmi_write(struct shtps_rmi_spi *ts, u16 addr, u8 data)
{
	int retry = SHTPS_SPI_RETRY_COUNT;
	int err;

	do{
		err = shtps_rmi_write_async(ts, addr, data);
		if(err){
			struct timespec tu;
		SHTPS_LOG_ERR_PRINT("shtps_rmi_write err:%d\n",err);
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		}
	}while(err != 0 && retry-- > 0);

	if(err){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return err;
}

#if	defined( SHTPS_SPI_BLOCKACCESS_ENABLE )
static void shtps_rmi_one_cs_spicomplete(void *arg)
{
	complete(arg);
	udelay(SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT);
}
static int shtps_rmi_read_block(struct shtps_rmi_spi *ts, u16 addr, u8 *readbuf, u32 size, u8 *buf)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8 *txbuf_p;
	u8 *rxbuf_p;
	int status;
	int i;

	memset(&t, 0, sizeof(t));
	memset(buf, 0, (size+2*2));

	txbuf_p = buf;
	rxbuf_p = buf + sizeof(u8)*(size + 2);

	txbuf_p[0] = ((addr >> 8) & 0xff) | 0x80;
	txbuf_p[1] = addr & 0xff;

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	t.tx_buf		= txbuf_p;
	t.rx_buf  		= rxbuf_p;
	t.len			= size + 2;
	t.bits_per_word	= 8;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	message.complete = shtps_rmi_one_cs_spicomplete;
	message.context  = &done;
	status = spi_async(ts->spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	rxbuf_p += 2;
	for(i = 0;i < size;i++){
		*readbuf++ = *rxbuf_p++;
	}

	return status;
}
static int shtps_rmi_read(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int	i;
	u32	s;
	u8 transbuf[(SHTPS_SY3X00_SPIBLOCK_BUFSIZE+2)*2];
	int retry = SHTPS_SPI_RETRY_COUNT;

	do{
		s = size;
		for(i=0;i<size;i+=SHTPS_SY3X00_SPIBLOCK_BUFSIZE){
			status = shtps_rmi_read_block(ts,
				addr+i,
				buf+i,
				(s>SHTPS_SY3X00_SPIBLOCK_BUFSIZE)?(SHTPS_SY3X00_SPIBLOCK_BUFSIZE):(s),
				transbuf);
			//
			if(status){
				struct timespec tu;
				SHTPS_LOG_ERR_PRINT("shtps_rmi_read:status :%d\n",status);
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
				break;
			}
			s -= SHTPS_SY3X00_SPIBLOCK_BUFSIZE;
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi read error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */
	
	return status;
}
#else	/* SHTPS_SPI_BLOCKACCESS_ENABLE */
/* -----------------------------------------------------------------------------------
 */
struct shtps_spi_async_ctrl_tbl{
	struct completion		work;
	struct spi_message		message;
	struct spi_transfer		t;
	int						wait_utime;
	u8						tx_buf[2];
	u8						rx_buf[2];
};

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_readb_async_comp_1(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	if(wait_utime){
		udelay( wait_utime );
	}
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_readb_async_comp_2(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	complete( &(ptr_p->work) );

	if(wait_utime){
		udelay( wait_utime );
	}
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_readb(struct shtps_rmi_spi *ts, u16 addr, u8 *buf)
{
	struct	shtps_spi_async_ctrl_tbl	spi_work[2], *spi_p;

	int	status;

	*buf = 0;
	memset(spi_work, 0, sizeof(spi_work));

	/* Write Reg.Addr. */
	spi_p = &spi_work[0];

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT;
	spi_p->tx_buf[0]		= ((addr >> 8) & 0xff) | 0x80;
	spi_p->tx_buf[1]		= addr & 0xff;
	spi_p->t.tx_buf			= spi_p->tx_buf;
	spi_p->t.rx_buf			= NULL;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 2;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		spi_p->t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
	spi_p->message.complete	= shtps_rmi_readb_async_comp_1;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async(ts->spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status){
		return status;
	}

	/* Read data */
	spi_p = &spi_work[1];

	init_completion( &(spi_p->work) );

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT;
	spi_p->t.tx_buf			= NULL;
	spi_p->t.rx_buf			= spi_p->rx_buf;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 1;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		spi_p->t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
	spi_p->message.complete	= shtps_rmi_readb_async_comp_2;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async(ts->spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ_COMP, "0x%04X|0x%02X", addr, rx_buf[0]);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status == 0){
		wait_for_completion( &(spi_p->work) );
		status = spi_p->message.status;

	}else{
		return status;

	}

	*buf = spi_p->rx_buf[0];

	return status;
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_read(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int i;
	int status = 0;

	for(i = 0;i < size;i++){
		int retry;
		retry = SHTPS_SPI_RETRY_COUNT;
		do{
			status = shtps_rmi_readb(ts, addr + i, buf + i);
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
			}
		}while(status != 0 && retry-- > 0);

		if(status){
			SHTPS_LOG_ERR_PRINT("spi read error\n");
			goto err_exit;
		}
		#if defined( SHTPS_LOG_DEBUG_ENABLE )
		else if(retry < (SHTPS_SPI_RETRY_COUNT)){
			SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
		}
		#endif /* SHTPS_LOG_DEBUG_ENABLE */
	}
	return 0;

err_exit:
	return status;
}
#endif /* #if defined( SHTPS_SPI_BLOCKACCESS_ENABLE ) */

static int shtps_rmi_write_packet(struct shtps_rmi_spi *ts, u16 addr, u8 *data, u32 size)
{
	int status = 0;
	int retry = SHTPS_SPI_RETRY_COUNT;

	do{
		status = shtps_rmi_write_block(ts,
			addr,
			data,
			size);
		//
		if(status){
			struct timespec tu;
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}

static int shtps_rmi_read_packet(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int	i;
	u32	s;
	u8 transbuf[(SHTPS_SY3X00_SPIBLOCK_BUFSIZE+2)*2];
	int retry = SHTPS_SPI_RETRY_COUNT;
	u16 addr_org = addr;

	do{
		s = size;
		for(i=0;i<size;i+=SHTPS_SY3X00_SPIBLOCK_BUFSIZE){
			status = shtps_rmi_read_block(ts,
				addr,
				buf+i,
				(s>SHTPS_SY3X00_SPIBLOCK_BUFSIZE)?(SHTPS_SY3X00_SPIBLOCK_BUFSIZE):(s),
				transbuf);
			//
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
				addr = addr_org;
				break;
			}
			s -= SHTPS_SY3X00_SPIBLOCK_BUFSIZE;
			addr = 0x7fff;	/* specifying no address after first read */
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi read error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
static void shtps_irq_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		if(SHTPS_STATE_ACTIVE == ts->state_mgr.state){
			shtps_wake_lock_idle(ts);
		}
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(0 != ts->lpwg.lpwg_switch && SHTPS_STATE_SLEEP == ts->state_mgr.state){
			shtps_lpwg_wakelock(ts, 1);
		}
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

	request_event(ts, SHTPS_EVENT_INTERRUPT, 1);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock(ts, 0);
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		shtps_wake_unlock_idle(ts);
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_END);
}

static void shtps_charger_armor_proc(struct shtps_rmi_spi *ts, u8 charger)
{
	SHTPS_LOG_FUNC_CALL();

    mutex_lock(&shtps_ctrl_lock);
    shtps_set_charger_armor(ts, (int)charger);
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_setsleep_proc(struct shtps_rmi_spi *ts, u8 sleep)
{
	SHTPS_LOG_FUNC_CALL();

	if(sleep){
		shtps_device_access_teardown(ts);
		request_event(ts, SHTPS_EVENT_SLEEP, 0);
	}else{
		#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
			ts->system_boot_mode = SH_BOOT_NORMAL;
		#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

		request_event(ts, SHTPS_EVENT_WAKEUP, 0);
	}
}

static void shtps_ioctl_setlpwg_proc(struct shtps_rmi_spi *ts, u8 on)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_ctrl_lock);
	ts->lpwg.lpwg_state = on;
	ts->lpwg.notify_enable = 1;
    SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_state = %d\n", ts->lpwg.lpwg_state);
    if (SHTPS_STATE_SLEEP == ts->state_mgr.state) {
		u8 new_setting = shtps_is_lpwg_active(ts);
		
		if(new_setting != ts->lpwg.lpwg_switch){
			if(new_setting){
				#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
					shtps_irq_enable(ts);
				#else
					shtps_irq_wake_enable(ts);
				#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

				shtps_system_set_wakeup(ts);
				shtps_set_lpwg_mode_on(ts);
			}else{
				#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
					shtps_irq_disable(ts);
				#else
					shtps_irq_wake_disable(ts);
				#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

				shtps_set_lpwg_mode_off(ts);
				shtps_sleep(ts, 1);
				shtps_system_set_sleep(ts);
			}
		}
	}
    mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_ioctl_setlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
	mutex_lock(&shtps_ctrl_lock);
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, (int)on);
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_ioctl_setconlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
	mutex_lock(&shtps_ctrl_lock);
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, (int)on);
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_ioctl_setlcdbrightlpmode_proc(struct shtps_rmi_spi *ts, u8 on)
{
	mutex_lock(&shtps_ctrl_lock);
	shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, (int)on);
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_async_open_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
}

static void shtps_async_close_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
}

static void shtps_async_enable_proc(struct shtps_rmi_spi *ts, u8 dummy)
{
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
}

static void shtps_suspend_spi_wake_lock(struct shtps_rmi_spi *ts, u8 lock)
{
	if(lock){
		if(ts->deter_suspend_spi.wake_lock_state == 0){
			ts->deter_suspend_spi.wake_lock_state = 1;
			wake_lock(&ts->deter_suspend_spi.wake_lock);
			pm_qos_update_request(&ts->deter_suspend_spi.pm_qos_lock_idle, SHTPS_QOS_LATENCY_DEF_VALUE);
		    SHTPS_LOG_DBG_PRINT("[suspend spi] wake_lock\n");
		}
	}else{
		if(ts->deter_suspend_spi.wake_lock_state == 1){
			ts->deter_suspend_spi.wake_lock_state = 0;
			wake_unlock(&ts->deter_suspend_spi.wake_lock);
			pm_qos_update_request(&ts->deter_suspend_spi.pm_qos_lock_idle, PM_QOS_DEFAULT_VALUE);
		    SHTPS_LOG_DBG_PRINT("[suspend spi] wake_unlock\n");
		}
	}
}

static int shtps_check_suspend_state(struct shtps_rmi_spi *ts, int proc, u8 param)
{
	int ret = 0;
	
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_proc_lock);
	if(ts->deter_suspend_spi.suspend){
		shtps_suspend_spi_wake_lock(ts, 1);
		ts->deter_suspend_spi.pending_info[proc].pending= 1;
		ts->deter_suspend_spi.pending_info[proc].param  = param;
		ret = 1;
	}else{
		shtps_suspend_spi_wake_lock(ts, 0);
		ts->deter_suspend_spi.pending_info[proc].pending= 0;
		ret = 0;
	}
	
	if(proc == SHTPS_DETER_SUSPEND_SPI_PROC_OPEN ||
		proc == SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE)
	{
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Close Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE].pending = 0;
		}
	}else if(proc == SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE){
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_OPEN].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Open Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_OPEN].pending  = 0;
		}
		if(ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE].pending){
		    SHTPS_LOG_DBG_PRINT("[suspend spi] Pending flag of TPS Enable Reqeust clear\n");
			ts->deter_suspend_spi.pending_info[SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE].pending= 0;
		}
	}
	mutex_unlock(&shtps_proc_lock);
	return ret;
}

#ifdef SHTPS_DEVELOP_MODE_ENABLE
static void shtps_exec_suspend_pending_proc_delayed(struct shtps_rmi_spi *ts)
{
    SHTPS_LOG_DBG_PRINT("%s() cancel_delayed_work()\n", __func__);
	cancel_delayed_work(&ts->deter_suspend_spi.pending_proc_work_delay);

    SHTPS_LOG_DBG_PRINT("%s() schedule_delayed_work(%d ms)\n", __func__, SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY);
	schedule_delayed_work(&ts->deter_suspend_spi.pending_proc_work_delay, 
							msecs_to_jiffies(SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY));

    SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
}

static void shtps_deter_suspend_spi_pending_proc_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_deter_suspend_spi *dss = container_of(dw, struct shtps_deter_suspend_spi, pending_proc_work_delay);
	struct shtps_rmi_spi *ts = container_of(dss, struct shtps_rmi_spi, deter_suspend_spi);

	SHTPS_LOG_FUNC_CALL();

	schedule_work(&ts->deter_suspend_spi.pending_proc_work);
	return;
}
#endif /* SHTPS_DEVELOP_MODE_ENABLE */

static void shtps_exec_suspend_pending_proc(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_work_sync(&ts->deter_suspend_spi.pending_proc_work);

	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		if(SHTPS_SUSPEND_SPI_RESUME_FUNC_DELAY > 0){
			shtps_exec_suspend_pending_proc_delayed(ts);
		}else
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	schedule_work(&ts->deter_suspend_spi.pending_proc_work);

    SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
}

static void shtps_deter_suspend_spi_pending_proc_work_function(struct work_struct *work)
{
	struct shtps_deter_suspend_spi *dss = container_of(work, struct shtps_deter_suspend_spi, pending_proc_work);
	struct shtps_rmi_spi *ts = container_of(dss, struct shtps_rmi_spi, deter_suspend_spi);
	int i;
	
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_proc_lock);
	
	for(i = 0;i < SHTPS_DETER_SUSPEND_SPI_PROC_NUM;i++){
		if(ts->deter_suspend_spi.pending_info[i].pending){
			SHTPS_SUSPEND_PENDING_FUNC_TBL[i](ts, ts->deter_suspend_spi.pending_info[i].param);
			ts->deter_suspend_spi.pending_info[i].pending = 0;
		}
	}
	shtps_suspend_spi_wake_lock(ts, 0);

	mutex_unlock(&shtps_proc_lock);
}

static void shtps_set_suspend_state(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_proc_lock);
	ts->deter_suspend_spi.suspend = 1;
	mutex_unlock(&shtps_proc_lock);
}

static void shtps_clr_suspend_state(struct shtps_rmi_spi *ts)
{
	int i;
	int hold_process = 0;
	
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_proc_lock);
	ts->deter_suspend_spi.suspend = 0;
	for(i = 0;i < SHTPS_DETER_SUSPEND_SPI_PROC_NUM;i++){
		if(ts->deter_suspend_spi.pending_info[i].pending){
			hold_process = 1;
			break;
		}
	}
	mutex_unlock(&shtps_proc_lock);
	
	if(hold_process){
		shtps_exec_suspend_pending_proc(ts);
	}else{
		shtps_suspend_spi_wake_lock(ts, 0);
	}

	mutex_lock(&shtps_ctrl_lock);
	if(ts->deter_suspend_spi.suspend_irq_detect != 0){
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			if(ts->deter_suspend_spi.suspend_irq_state == SHTPS_IRQ_STATE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake enable\n");
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq enable\n");
				shtps_irq_enable(ts);
			}
		#else
			if(ts->deter_suspend_spi.suspend_irq_wake_state == SHTPS_IRQ_WAKE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake enable\n");
				shtps_irq_wake_enable(ts);
			}
			if(ts->deter_suspend_spi.suspend_irq_state == SHTPS_IRQ_STATE_ENABLE){
				SHTPS_LOG_DBG_PRINT("[suspend_spi] irq enable\n");
				shtps_irq_enable(ts);
			}
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

		ts->deter_suspend_spi.suspend_irq_detect = 0;
	}
	mutex_unlock(&shtps_ctrl_lock);
}
#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static inline int shtps_get_fingermax(struct shtps_rmi_spi *ts);

#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE )
static void shtps_get_reporting_mode(struct shtps_rmi_spi *ts, u8* val)
{
	shtps_rmi_read(ts, ts->map.fn11.ctrlBase, val, 1);
	return;
}

static void shtps_set_reporting_mode(struct shtps_rmi_spi *ts, u8 mode)
{
	u8 now_val;
	u8 val;
	
	if(SHTPS_FACETOUCH_DETECT_CHG_REPORTING_MODE_ENABLE == 0) return;
	
	shtps_get_reporting_mode(ts, &now_val);

	val = (now_val & (0xFF << 3)) | mode;
	
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase, val);
	SHTPS_LOG_DBG_PRINT("change reporting mode (0x%02x)\n", val);

	return;
}
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE */

static int shtps_get_facetouchmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->facetouch.mode;
}

static void shtps_set_facetouchmode(struct shtps_rmi_spi *ts, int mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__SET_FACETOUCH_MODE, "%d", mode);
	ts->facetouch.mode = mode;
	if(mode == 0){
		ts->facetouch.detect = 0;
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE )
		shtps_set_reporting_mode(ts, SHTPS_FACETOUCH_DETECT_REPORTING_MODE_DEFAULT);
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE */
	}
}

static void shtps_init_palmthresh(struct shtps_rmi_spi *ts)
{
	u8 val;

	ts->facetouch.palm_thresh = SHTPS_LOS_SINGLE;
	if(SHTPS_FACETOUCH_DETECT_CHANGE_LOS){
		shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x29, &val, 1);
		shtps_rmi_write(ts,ts->map.fn11.ctrlBase + 0x29, (val & 0x80) | (ts->facetouch.palm_thresh & 0x7F));
		SHTPS_LOG_DBG_PRINT("init palm thresh (0x%02x)\n", ts->facetouch.palm_thresh);
	}
}

static void shtps_set_palmthresh(struct shtps_rmi_spi *ts, int touch_num)
{
	u8 thresh;

	if(shtps_get_facetouchmode(ts)){
		if(touch_num >= 2){
			thresh = SHTPS_LOS_MULTI;
		}else{
			thresh = SHTPS_LOS_SINGLE;
		}
	}else{
		thresh = SHTPS_LOS_SINGLE;
	}
	
	if(ts->facetouch.palm_thresh != thresh){
		u8 val;

		ts->facetouch.palm_thresh = thresh;
		
		if(SHTPS_FACETOUCH_DETECT_CHANGE_LOS){
			shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x26, &val, 1);
			shtps_rmi_write(ts,ts->map.fn11.ctrlBase + 0x26, (val & 0x80) | (ts->facetouch.palm_thresh & 0x7F));
			SHTPS_LOG_DBG_PRINT("palm thresh = (0x%02x)\n", ts->facetouch.palm_thresh);
		}
	}
}

static void shtps_facetouch_init(struct shtps_rmi_spi *ts)
{
	ts->facetouch.mode = 0;
	ts->facetouch.state = 0;
	ts->facetouch.detect = 0;
	ts->facetouch.palm_thresh = -1;
	ts->facetouch.wake_sig = 0;
	ts->facetouch.palm_det = 0;
	ts->facetouch.wakelock_state = 0;
	ts->facetouch.touch_num = 0;

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		ts->facetouch.facetouch_off_notify_delayed_state = 0;
		ts->facetouch.facetouch_off_force_flg = 0;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
		ts->facetouch.rezero_disable = 0;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */

	init_waitqueue_head(&ts->facetouch.wait_off);
    wake_lock_init(&ts->facetouch.wake_lock, WAKE_LOCK_SUSPEND, "shtps_facetouch_wake_lock");
	pm_qos_add_request(&ts->facetouch.qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

static void shtps_facetouch_wakelock(struct shtps_rmi_spi *ts, u8 on)
{
	if(on){
		if(ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH){
			mutex_lock(&shtps_facetouch_qosctrl_lock);
			if(ts->facetouch.wakelock_state == 0){
				SHTPS_LOG_DBG_PRINT("face touch wake_lock on\n");
				wake_lock(&ts->facetouch.wake_lock);
				pm_qos_update_request(&ts->facetouch.qos_cpu_latency, SHTPS_QOS_LATENCY_DEF_VALUE);
				ts->facetouch.wakelock_state = 1;
			}
			mutex_unlock(&shtps_facetouch_qosctrl_lock);
		}
	}else{
		mutex_lock(&shtps_facetouch_qosctrl_lock);
		if(ts->facetouch.wakelock_state != 0){
			SHTPS_LOG_DBG_PRINT("face touch wake_lock off\n");
			wake_unlock(&ts->facetouch.wake_lock);
			pm_qos_update_request(&ts->facetouch.qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
			ts->facetouch.wakelock_state = 0;
		}
		mutex_unlock(&shtps_facetouch_qosctrl_lock);
	}
}

static void shtps_event_all_cancel(struct shtps_rmi_spi *ts)
{
	int	i;
	int isEvent = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(ts->report_info.fingers[i].state != 0x00){
			isEvent = 1;
			shtps_report_touch_on(ts, i,
								  SHTPS_TOUCH_CANCEL_COORDINATES_X,
								  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
								  1,
								  ts->report_info.fingers[i].wx,
								  ts->report_info.fingers[i].wy,
								  ts->report_info.fingers[i].z);
		}
	}
	if(isEvent){
		input_sync(ts->input);
		
		for(i = 0;i < fingerMax;i++){
			if(ts->report_info.fingers[i].state != 0x00){
				isEvent = 1;
				shtps_report_touch_off(ts, i,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  0,
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);
			}
		}
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
}

#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
static void shtps_delayed_facetouch_off_notify_wakelock(struct shtps_rmi_spi *ts, int on)
{
	if(on){
		wake_lock(&ts->facetouch.facetouch_off_delayed_wake_lock);
		pm_qos_update_request(&ts->facetouch.facetouch_off_delayed_qos, SHTPS_QOS_LATENCY_DEF_VALUE);
		SHTPS_LOG_DBG_PRINT("%s(): wake_lock\n", __func__);
	}else{
		wake_unlock(&ts->facetouch.facetouch_off_delayed_wake_lock);
		pm_qos_update_request(&ts->facetouch.facetouch_off_delayed_qos, PM_QOS_DEFAULT_VALUE);
		SHTPS_LOG_DBG_PRINT("%s(): wake_unlock\n", __func__);
	}
}

static void shtps_delayed_facetouch_off_notify_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_facetouch_info *fi = container_of(dw, struct shtps_facetouch_info, notify_interval_delayed_work);
	struct shtps_rmi_spi *ts = container_of(fi, struct shtps_rmi_spi, facetouch);

	SHTPS_LOG_FUNC_CALL();


	if(ts->facetouch.state != 0 || ts->facetouch.facetouch_off_force_flg != 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 0;
		ts->facetouch.detect = 1;
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch off detect(force flag = %d). wake_up() (by chatter timer)\n", ts->facetouch.facetouch_off_force_flg);
	}else{
		SHTPS_LOG_DBG_PRINT("touch off detect but pre-state isn't face touch (chatter timer).\n");
	}

	ts->facetouch.facetouch_off_notify_delayed_state = 0;
	ts->facetouch.facetouch_off_force_flg = 0;
	
	shtps_delayed_facetouch_off_notify_wakelock(ts, 0);
	
}

static void shtps_delayed_facetouch_off_notify(struct shtps_rmi_spi *ts, int delay)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->facetouch.facetouch_off_notify_delayed_state == 0){
		shtps_delayed_facetouch_off_notify_wakelock(ts, 1);
		schedule_delayed_work(&ts->facetouch.notify_interval_delayed_work, msecs_to_jiffies(delay));
		ts->facetouch.facetouch_off_notify_delayed_state = 1;
		SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("delayed facetouch_off notify.... [%d]ms", delay);
	}
}

static void shtps_delayed_facetouch_off_notify_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->facetouch.facetouch_off_notify_delayed_state == 1){
		shtps_delayed_facetouch_off_notify_wakelock(ts, 0);
		cancel_delayed_work(&ts->facetouch.notify_interval_delayed_work);
		ts->facetouch.facetouch_off_notify_delayed_state = 0;
		ts->facetouch.facetouch_off_force_flg = 0;
	}
	
	#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
		ts->facetouch.rezero_disable = 0;
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
	
}
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

static void shtps_notify_facetouch(struct shtps_rmi_spi *ts)
{
	if(ts->facetouch.state == 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 1;
		ts->facetouch.detect = 1;
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch detect. wake_up()\n");
	}else{
		SHTPS_LOG_DBG_PRINT("face touch detect but pre-state isn't face off touch.\n");
	}
}

static void shtps_notify_facetouchoff(struct shtps_rmi_spi *ts, int force)
{
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
	if(ts->facetouch.state != 0 || force != 0){
		if(ts->facetouch.facetouch_off_notify_delayed_state == 0){
			ts->facetouch.facetouch_off_force_flg = force;
			
			#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
				ts->facetouch.rezero_disable = 1;
			#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
			
			shtps_delayed_facetouch_off_notify(ts, SHTPS_FACETOUCH_OFF_DETECT_CHATT_THRESH_TIME);
		}else{
			SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("already detect facetouch off. watiting....");
		}
	}else{
		shtps_delayed_facetouch_off_notify_cancel(ts);
		SHTPS_LOG_DBG_PRINT("touch off detect but pre-state isn't face touch.\n");
	}
#else
	if(ts->facetouch.state != 0 || force != 0){
		shtps_facetouch_wakelock(ts, 1);
		ts->facetouch.state = 0;
		ts->facetouch.detect = 1;
		
		#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
			ts->facetouch.rezero_disable = 1;
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */
		
		wake_up_interruptible(&ts->facetouch.wait_off);
		SHTPS_LOG_DBG_PRINT("face touch off detect(force flag = %d). wake_up()\n", force);
	}else{
		SHTPS_LOG_DBG_PRINT("touch off detect but pre-state isn't face touch.\n");
	}
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
}

static void shtps_check_facetouch(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 	palm_det  = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);
	u8	numOfFingers = 0;

	SHTPS_LOG_DBG_PRINT("%s() gesture flag = 0x%02x\n", __func__, info->gs1);
	
	if(SHTPS_FACETOUCH_DETECT_CHECK_PALMFLAG == 1 && (info->gs1 & 0x02) == 0x02){
		SHTPS_LOG_DBG_PRINT("Detect palm touch by palm flag.\n");
		palm_det = 1;
	}else{
		for(i = 0;i < fingerMax;i++){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if(SHTPS_FACETOUCH_DETECT_CHECK_FINGERWIDTH == 1){
					if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
						if( (ts->fw_report_info.fingers[i].wx >= SHTPS_FACETOUCH_DETECT_PALMDET_W_8BIT_THRESHOLD) ||
							(ts->fw_report_info.fingers[i].wy >= SHTPS_FACETOUCH_DETECT_PALMDET_W_8BIT_THRESHOLD) )
						{
							SHTPS_LOG_DBG_PRINT("Detect palm touch by finger width.\n");
							palm_det = 1;
							break;
						}
					}else{
						if( (ts->fw_report_info.fingers[i].wx >= SHTPS_FACETOUCH_DETECT_PALMDET_W_THRESHOLD) ||
							(ts->fw_report_info.fingers[i].wy >= SHTPS_FACETOUCH_DETECT_PALMDET_W_THRESHOLD) )
						{
							SHTPS_LOG_DBG_PRINT("Detect palm touch by finger width.\n");
							palm_det = 1;
							break;
						}
					}
				}

				numOfFingers++;
			}
		}
	}
	
	if(SHTPS_FACETOUCH_DETECT_CHECK_MULTITOUCH == 1 && numOfFingers >= 2){
		SHTPS_LOG_DBG_PRINT("Detect palm touch by multi-touch.\n");
		palm_det = 1;
	}
	
	if(palm_det != 0 && ts->facetouch.state == 0){
		shtps_notify_facetouch(ts);
	}
	
	if(ts->facetouch.palm_det != 0){
		if(palm_det == 0 && ts->fw_report_info.finger_num == 0){
			ts->facetouch.palm_det = 0;
		}
	}else{
		ts->facetouch.palm_det = palm_det;
	}
}

static void shtps_check_facetouchoff(struct shtps_rmi_spi *ts)
{
	if(ts->facetouch.palm_det == 0 && ts->facetouch.state != 0){
		shtps_notify_facetouchoff(ts, 0);
	}
}

static void shtps_facetouch_forcecal_handle(struct shtps_rmi_spi *ts)
{
	ts->facetouch.palm_det = 0;
	if(ts->facetouch.state != 0){
		shtps_notify_facetouchoff(ts, 0);
	}
}

static void shtps_read_touchevent_infacetouchmode(struct shtps_rmi_spi *ts)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 5 + (SHTPS_FINGER_MAX / 4) + 1 + 5 + 5];
	u8 finger;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 offset, readsize;
	struct shtps_touch_info info;

	SHTPS_LOG_FUNC_CALL();

	shtps_device_access_setup(ts);

	if(fingerMax > 8){
		offset = 5;
		readsize = 3 + 5 * SHTPS_FINGER_MAX;
	}else if (fingerMax > 4){
		offset = 4;
		readsize = 2 + 5 * SHTPS_FINGER_MAX;
	}else{
		offset = 3;
		readsize = 1 + 5 * SHTPS_FINGER_MAX;
	}

	memset(buf, 0, sizeof(buf));
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	shtps_rmi_read(ts, ts->map.fn11.dataBase, &buf[2], readsize);
	shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
									   &buf[offset + fingerMax * 5], 1);
	if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
		shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + (fingerMax * 5) + 1,
										&buf[offset + (fingerMax * 5) + 1], SHTPS_FINGER_MAX);
	}

	ts->finger_state[0] = buf[2];
	if (fingerMax > 4){
		ts->finger_state[1] = buf[3];
	}
	if(fingerMax > 8){
		ts->finger_state[2] = buf[4];
	}
	
	shtps_set_touch_info(ts, &buf[2], &info);
	
	info.finger_num = 0;
	for(finger = 0;finger < fingerMax;finger++){
		if(info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			info.finger_num++;
		}
	}
	info.gs1 = buf[offset + fingerMax * 5];
	info.gs2 = 0;

	SHTPS_LOG_DBG_PRINT("%s() numOfFingers = %d, palm det = 0x%02x\n", __func__, ts->fw_report_info.finger_num, buf[offset + fingerMax * 5]);
	
	shtps_check_facetouch(ts, &info);
	shtps_check_facetouchoff(ts);

	if(SHTPS_FACETOUCH_DETECT_OFF_NOTIFY_BY_ALLTU){
		if( (ts->facetouch.touch_num != 0) && (ts->fw_report_info.finger_num == 0) ){
			if(SHTPS_FACETOUCH_DETECT_CHECK_PALMFLAG == 1){
				if(ts->facetouch.palm_det == 0){
					shtps_notify_facetouchoff(ts, 1);
				}
			}else{
				shtps_notify_facetouchoff(ts, 1);
			}
		}
	}
	
	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		if(ts->facetouch.facetouch_off_notify_delayed_state == 1){
			if(info.finger_num > 0){
				shtps_delayed_facetouch_off_notify_cancel(ts);
				SHTPS_FACETOUCH_OFF_DETECT_CHATT_PRINT("Detect touch. cancel touch off notify timer");
			}
		}
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */
	
	ts->facetouch.touch_num = info.finger_num;

	if(ts->facetouch.touch_num == 0){
		shtps_device_access_teardown(ts);
	}

	memcpy(&ts->fw_report_info_store, &ts->fw_report_info, sizeof(ts->fw_report_info));
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

/* -----------------------------------------------------------------------------------
 */
static irqreturn_t shtps_irq_handler(int irq, void *dev_id)
{
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_START);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t shtps_irq(int irq, void *dev_id)
{
	struct shtps_rmi_spi	*ts = dev_id;

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IRQ, 0) == 0){
			shtps_irq_proc(ts, 0);
		}else{
			mutex_lock(&shtps_ctrl_lock);
			ts->deter_suspend_spi.suspend_irq_state = ts->irq_mgr.state;
			ts->deter_suspend_spi.suspend_irq_wake_state = ts->irq_mgr.wake;
			ts->deter_suspend_spi.suspend_irq_detect = 1;
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq detect <irq_state:%d><irq_wake_state:%d>\n",
									ts->deter_suspend_spi.suspend_irq_state, ts->deter_suspend_spi.suspend_irq_wake_state);
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq disable\n");
			SHTPS_LOG_DBG_PRINT("[suspend_spi] irq wake disable\n");
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_disable(ts);
			#else
				shtps_irq_wake_disable(ts);
				shtps_irq_disable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
			mutex_unlock(&shtps_ctrl_lock);
		}
	#else
		shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

		#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
			if(SHTPS_STATE_ACTIVE == ts->state_mgr.state){
				shtps_wake_lock_idle(ts);
			}
		#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

		_log_msg_send( LOGMSG_ID__IRQ_NOTIFY, "");
		_log_msg_recv( LOGMSG_ID__IRQ_NOTIFY, "");

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if(0 != ts->lpwg.lpwg_switch && SHTPS_STATE_SLEEP == ts->state_mgr.state){
				shtps_lpwg_wakelock(ts, 1);
			}
		#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

		request_event(ts, SHTPS_EVENT_INTERRUPT, 1);

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_lpwg_wakelock(ts, 0);
		#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

		#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
			shtps_wake_unlock_idle(ts);
		#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

		shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_END);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------------
 */
static inline void shtps_set_dev_state(struct shtps_rmi_spi *ts, u8 state)
{
	SHTPS_LOG_DBG_PRINT("[dev_state] set (%s -> %s)\n",
							(ts->dev_state == SHTPS_DEV_STATE_SLEEP) ? "sleep" :
							(ts->dev_state == SHTPS_DEV_STATE_DOZE) ? "doze" :
							(ts->dev_state == SHTPS_DEV_STATE_ACTIVE) ? "active" : "unknown",
							(state == SHTPS_DEV_STATE_SLEEP) ? "sleep" :
							(state == SHTPS_DEV_STATE_DOZE) ? "doze" :
							(state == SHTPS_DEV_STATE_ACTIVE) ? "active" : "unknown");
	ts->dev_state = state;
}

static inline u8 shtps_get_dev_state(struct shtps_rmi_spi *ts)
{
	return ts->dev_state;
}

static int shtps_f54_command_completion_wait(struct shtps_rmi_spi *ts, int max, int interval)
{
	u8  buf   = 0xFF;
	int count = 0;
	int ret   = 0;

	if( shtps_get_dev_state(ts) == SHTPS_DEV_STATE_SLEEP ){
		SHTPS_LOG_DBG_PRINT("%s(): not wait by sleep state\n", __func__);
		return 0;
	}

	do{
		msleep(interval);
		shtps_rmi_read(ts, ts->map.fn54.commandBase, &buf, 1);
		SHTPS_LOG_DBG_PRINT("%s(): F54_COMMAND00 = 0x%02x (cnt=%d)\n", __func__, buf, count);
	}while(++count < max && buf != 0);
	
	if(buf != 0){
		SHTPS_LOG_DBG_PRINT("%s(): wait count over\n", __func__);
		ret = -1;
	}
	return ret;
}

static inline void shtps_command_force_update(struct shtps_rmi_spi *ts)
{
	if(ts->map.fn54.enable != 0){
		SHTPS_LOG_DBG_PRINT("[command] f54 force_update execute\n");
		shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x04);
	}
}

static inline void shtps_command_force_cal(struct shtps_rmi_spi *ts)
{
	if(ts->map.fn54.enable != 0){
		SHTPS_LOG_DBG_PRINT("[command] f54 force_cal execute\n");
		shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x02);
	}
}

static void shtps_work_tmof(struct work_struct *data)
{
	struct delayed_work  *dw = container_of(data, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, tmo_check);

	_log_msg_sync( LOGMSG_ID__TIMER_TIMEOUT, "");
	request_event(ts, SHTPS_EVENT_TIMEOUT, 0);
}

static enum hrtimer_restart shtps_delayed_rezero_timer_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, rezero_delayed_timer);

	_log_msg_send( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	schedule_work(&ts->rezero_delayed_work);
	return HRTIMER_NORESTART;
}
static void shtps_rezero_delayed_work_function(struct work_struct *work)
{
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, rezero_delayed_work);
	u8 rezero_exec = 0;

	_log_msg_recv( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	mutex_lock(&shtps_ctrl_lock);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(ts->lpwg.block_touchevent != 0){
			if((ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0){
				rezero_exec = 1;
				ts->lpwg.block_touchevent = 0;
				ts->lpwg.tu_rezero_req = 0;
				#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
					ts->wakeup_touch_event_inhibit_state = 0;
					SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
				#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
			}else{
				SHTPS_LOG_DBG_PRINT("LPWG delayed rezero fail by touch state\n");
				ts->lpwg.tu_rezero_req = 1;
			}
		}else{
			rezero_exec = 1;
			ts->lpwg.block_touchevent = 0;
			ts->lpwg.tu_rezero_req = 0;
			#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
				ts->wakeup_touch_event_inhibit_state = 0;
				SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
			#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
		}
	#else
		rezero_exec = 1;
		#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
			ts->wakeup_touch_event_inhibit_state = 0;
			SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
		#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	if(rezero_exec){
		shtps_rezero(ts);
	}

	#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
		if(SHTPS_WAKEUP_CALIB_ENABLE_SWITCH != 0){
			ts->wakeup_calib_enable = 1;
			ts->wakeup_calib_cancel_check_enable = 1;
			ts->wakeup_calib_deferment = 0;
			ts->wakeup_calib_lastone = 0;
			ts->wakeup_calib_lastone_req = 0;
			ts->wakeup_calib_rezero_exec_num = 0;
			memset(ts->wakeup_calib_td_pos, 0, sizeof(ts->wakeup_calib_td_pos));
			memset(ts->wakeup_calib_cancel_check, 0, sizeof(ts->wakeup_calib_cancel_check));
			memset(ts->wakeup_calib_event_count, 0, sizeof(ts->wakeup_calib_event_count));
			SHTPS_LOG_DBG_PRINT("[wakeup_calib] timer start\n");
			shtps_wakeup_calib_timer_start(ts, SHTPS_WAKEUP_CALIB_DELAY_MS);
		}
	#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

	mutex_unlock(&shtps_ctrl_lock);
}

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
static void shtps_perf_lock_enable(struct shtps_rmi_spi *ts)
{
	mutex_lock(&shtps_cpu_clock_ctrl_lock);

	if ( is_perf_lock_active(&ts->perf_lock) == 0 ){
		perf_lock(&ts->perf_lock);
		SHTPS_LOG_DBG_PRINT("perf_lock start (%d ms)\n", ts->perf_lock_enable_time_ms);
	}

	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_perf_lock_disable(struct shtps_rmi_spi *ts)
{
	mutex_lock(&shtps_cpu_clock_ctrl_lock);

	if (is_perf_lock_active(&ts->perf_lock)){
		perf_unlock(&ts->perf_lock);
		SHTPS_LOG_DBG_PRINT("perf_lock end\n");
	}

	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static int shtps_perf_lock_disable_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms)
{
	cancel_delayed_work(&ts->perf_lock_disable_delayed_work);
	schedule_delayed_work(&ts->perf_lock_disable_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

static void shtps_perf_lock_disable_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, perf_lock_disable_delayed_work);

	SHTPS_LOG_FUNC_CALL();

	shtps_perf_lock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by Timer\n");

	return;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_SET, "%lu", delay_us);
	hrtimer_cancel(&ts->rezero_delayed_timer);
	hrtimer_start(&ts->rezero_delayed_timer, ktime_set(0, delay_us * 1000), HRTIMER_MODE_REL);
}

static void shtps_delayed_rezero_cancel(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_CANCEL, "");
	hrtimer_try_to_cancel(&ts->rezero_delayed_timer);
}

static void shtps_forcecal(struct shtps_rmi_spi *ts)
{
	
	shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x02);
	SHTPS_LOG_DBG_PRINT("fw force cal execute\n");

	shtps_event_force_touchup(ts);

}

static void shtps_rezero(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
		u8  val;

		_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
		SHTPS_LOG_DBG_PRINT("fw rezero execute\n");
		shtps_rmi_read(ts, 0xF0, &val, 1);
		shtps_rmi_write(ts, 0xF0, val & ~0x01);
		shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
		shtps_rmi_write(ts, 0xF0, val | 0x01);
	#else
		_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
		SHTPS_LOG_DBG_PRINT("fw rezero execute\n");
		shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
	#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

	shtps_event_force_touchup(ts);

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_facetouch_forcecal_handle(ts);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */


	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		shtps_rezero_state = 1;
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
}

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
static u8 shtps_is_singlefinger(struct shtps_rmi_spi *ts, u8 gs_info)
{
#if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE )
	u8  ret = 0x00;
	u8  buf;

	shtps_rmi_read(ts, 0xF0, &buf, 1);
	if(buf & 0x02){
		ret = 0x01;
	}
	_log_msg_sync( LOGMSG_ID__GET_SINGLE_FINGER_FLAG, "%d", ret);
	return ret;
#else
	return 0x01;
#endif /* #if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE ) */
}

static void shtps_autorezero_disable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_DISABLE, "");
	SHTPS_LOG_DBG_PRINT("fw auto rezero disable\n");
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val | 0x01);
}

static void shtps_autorezero_enable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
	SHTPS_LOG_DBG_PRINT("fw auto rezero enable\n");
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val & ~0x01);
}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

static void shtps_rezero_request(struct shtps_rmi_spi *ts, u8 request, u8 trigger)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	int single_finger_count;

	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE){
		shtps_autorezero_disable(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE){
		shtps_autorezero_enable(ts);
	}
#else
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
			if(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECT_REZERO_ENABLE != 0){
				ts->wakeup_touch_event_inhibit_state = 1;
				SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit start until rezero exec\n");
			}
		#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_rezero_handle(struct shtps_rmi_spi *ts, u8 event, u8 gs)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	_log_msg_sync( LOGMSG_ID__REZERO_HANDLE, "%d|%d|%d", ts->poll_info.stop_margin, event, gs);
	if(!ts->poll_info.stop_margin){
		return;
	}

	if(event == SHTPS_REZERO_HANDLE_EVENT_MTD){
		ts->poll_info.single_fingers_enable = 0;

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCH){
		if(ts->poll_info.single_fingers_enable && shtps_is_singlefinger(ts, gs)){
			ts->poll_info.single_fingers_count++;
			SHTPS_LOG_DBG_PRINT("single finger count = %d\n",
									ts->poll_info.single_fingers_count);
		}

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCHUP){
		ts->poll_info.single_fingers_enable = 1;
		if((++ts->poll_info.stop_count >= ts->poll_info.stop_margin) &&
			(ts->poll_info.single_fingers_count >= ts->poll_info.single_fingers_max))
		{
			shtps_autorezero_disable(ts);

			ts->poll_info.stop_margin          = 0;
			ts->poll_info.single_fingers_count = 0;
			ts->poll_info.single_fingers_enable= 0;
			ts->poll_info.single_fingers_max   = 0;
		}
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_reset_startuptime(struct shtps_rmi_spi *ts)
{
	ts->state_mgr.starttime = jiffies + msecs_to_jiffies(SHTPS_STARTUP_MIN_TIME);
}

static unsigned long shtps_check_startuptime(struct shtps_rmi_spi *ts)
{
	unsigned long remainingtime;
	if(time_after(jiffies, ts->state_mgr.starttime)){
		return 0;
	}
	remainingtime = jiffies_to_msecs(ts->state_mgr.starttime - jiffies);
	if(remainingtime > SHTPS_STARTUP_MIN_TIME){
		remainingtime = SHTPS_STARTUP_MIN_TIME;
	}
	return remainingtime;
}

static int shtps_start(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_START, 0);
}

static void shtps_shutdown(struct shtps_rmi_spi *ts)
{
	request_event(ts, SHTPS_EVENT_STOP, 0);
}

/* -----------------------------------------------------------------------------------
 */
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
static int shtps_offset_area(struct shtps_rmi_spi *ts, int x, int y)
{
	if(y < ts->offset.base[2]){
		if(x < ts->offset.base[0]){
			return 0x00;
		}else if(x < ts->offset.base[1]){
			return 0x01;
		}else{
			return 0x02;
		}
	}else if(y < ts->offset.base[3]){
		if(x < ts->offset.base[0]){
			return 0x03;
		}else if(x < ts->offset.base[1]){
			return 0x04;
		}else{
			return 0x05;
		}
	}else if(y < ts->offset.base[4]){
		if(x < ts->offset.base[0]){
			return 0x06;
		}else if(x < ts->offset.base[1]){
			return 0x07;
		}else{
			return 0x08;
		}
	}else{
		if(x < ts->offset.base[0]){
			return 0x09;
		}else if(x < ts->offset.base[1]){
			return 0x0A;
		}else{
			return 0x0B;
		}
	}
	return 0x00;
}
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */

static int shtps_offset_pos(struct shtps_rmi_spi *ts, int *x, int *y)
{
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
	int area;
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;

	if(!ts->offset.enabled){
		return 0;
	}

	if((*x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (*y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		return 0;
	}

	area = shtps_offset_area(ts, *x, *y);

	xp = xq = xr = xs = yp = yq = yr = ys = 0;
	if(area == 0x00){
		xq = xs = ts->offset.diff[0];
		yr = ys = ts->offset.diff[1];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x01){
		xp = xr = ts->offset.diff[0];
		xq = xs = ts->offset.diff[2];
		yr = ts->offset.diff[1];
		ys = ts->offset.diff[3];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x02){
		xq = xr = ts->offset.diff[2];
		yr = ys = ts->offset.diff[3];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x03){
		xq = ts->offset.diff[0];
		xs = ts->offset.diff[4];
		yp = yq = ts->offset.diff[1];
		yr = ys = ts->offset.diff[5];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x04){
		xp = ts->offset.diff[0];
		xq = ts->offset.diff[2];
		xr = ts->offset.diff[4];
		xs = ts->offset.diff[6];
		yp = ts->offset.diff[1];
		yq = ts->offset.diff[3];
		yr = ts->offset.diff[5];
		ys = ts->offset.diff[7];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x05){
		xp = ts->offset.diff[2];
		xr = ts->offset.diff[6];
		yp = yq = ts->offset.diff[3];
		yr = ys = ts->offset.diff[7];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x06){
		xq = ts->offset.diff[4];
		xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[5];
		yr = ys = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x07){
		xp = ts->offset.diff[4];
		xq = ts->offset.diff[6];
		xr = ts->offset.diff[8];
		xs = ts->offset.diff[10];
		yp = ts->offset.diff[5];
		yq = ts->offset.diff[7];
		yr = ts->offset.diff[9];
		ys = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x08){
		xp = ts->offset.diff[6];
		xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[7];
		yr = ys = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x09){
		xq = xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = ts->offset.diff[8];
		xq = xs = ts->offset.diff[10];
		yp = ts->offset.diff[9];
		yq = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else{
		xq = xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);
	
	if(*x >= CONFIG_SHTPS_SY3000_LCD_SIZE_X){
		*x = CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1;
	}
	if(*x < 0){
		*x = 0;
	}
	if(*y >= CONFIG_SHTPS_SY3000_LCD_SIZE_Y){
		*y = CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1;
	}
	if(*y < 0){
		*y = 0;
	}
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */
	return 0;
}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
static int shtps_map_construct(struct shtps_rmi_spi *ts, u8 func_check)
#else
static int shtps_map_construct(struct shtps_rmi_spi *ts)
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
{
	struct rmi_pdt	pdt;
	int				i;
	int				rc;
	int				page;
	u8				maxPosition[4];
	char			productID[11];

	_log_msg_sync( LOGMSG_ID__MAP_CONSTRUCT, "");
	SHTPS_LOG_FUNC_CALL();

	msleep(3);
	memset(&ts->map, 0, sizeof(ts->map));

	/* Read the PDT */
	for(page = 0;page < SHTPS_PDT_PAGE_SIZE_MAX;page++){
		shtps_rmi_write(ts, 0xFF, page);
		for(i = 0xE9;i > 0x0a;i-=sizeof(pdt)){
			rc = shtps_rmi_read(ts, ((page & 0x0f) << 8) | i, (u8*)&pdt, sizeof(pdt));
			if(rc){
				goto err_exit;
			}

			if(!pdt.functionNumber){
				/** End of PDT */
				break;
			}

			SHTPS_LOG_DBG_PRINT("\n");
			switch(pdt.functionNumber){
			case 0x01:
				SHTPS_LOG_DBG_PRINT("Found: RMI Device Control\n");
				ts->map.fn01.enable		= 1;
				ts->map.fn01.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn01.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn01.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn01.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn01.queryBase,
									ts->map.fn01.query.data, sizeof(ts->map.fn01.query.data));

#if defined( SHTPS_SY_REGMAP_BASE3 )
				memcpy(productID, &F01_QUERY_PRODUCTID(ts->map.fn01.query.data), sizeof(productID));
				productID[10] = '\0';
				SHTPS_LOG_DBG_PRINT("F01 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("Manufacturer ID       : 0x%02x\n", F01_QUERY_MANUFACTURERID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CutomMap              : 0x%02x\n", F01_QUERY_CUSTOMMAP(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("NonCompliant          : 0x%02x\n", F01_QUERY_NONCOMPLIANT(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasSensorID           : 0x%02x\n", F01_QUERY_HASSENSORID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasAdjustableDoze     : 0x%02x\n", F01_QUERY_HASAJUSTABLEDOZE(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasAdjDozeHoldoff     : 0x%02x\n", F01_QUERY_HASADJDOZEHOLDOFF(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasProductProperties2 : 0x%02x\n", F01_QUERY_HASPRODUCTPROPERTIES2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo0          : 0x%02x\n", F01_QUERY_PRODUCTINFO0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo1          : 0x%02x\n", F01_QUERY_PRODUCTINFO1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Year                  : 0x%02x\n", F01_QUERY_DATECODEYEAR(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Month                 : 0x%02x\n", F01_QUERY_DATECODEMONTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Day                   : 0x%02x\n", F01_QUERY_DATECODEDAY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CP1                   : 0x%02x\n", F01_QUERY_CP1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CP2                   : 0x%02x\n", F01_QUERY_CP2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID0           : 0x%04x\n", F01_QUERY_WAFERLOTID0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID1           : 0x%04x\n", F01_QUERY_WAFERLOTID1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID2           : 0x%02x\n", F01_QUERY_WAFERLOTID2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("ProductID             : %s\n", productID);
				SHTPS_LOG_DBG_PRINT("HasDS4Queries         : 0x%02x\n", F01_QUERY_HASDS4QUERIES(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 Length            : 0x%02x\n", F01_QUERY_DS4_LENGTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasPackageQuery   : 0x%02x\n", F01_QUERY_DS4_HASPACKAGEIDQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasPackratQuery   : 0x%02x\n", F01_QUERY_DS4_HASPACKRATQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasResetQuery     : 0x%02x\n", F01_QUERY_DS4_HASRESETQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasMaskRevQuery   : 0x%02x\n", F01_QUERY_DS4_HASMASKREVQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasI2CControl     : 0x%02x\n", F01_QUERY_DS4_HASI2CCONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasSPIControl     : 0x%02x\n", F01_QUERY_DS4_HASSPICONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasATTNControl    : 0x%02x\n", F01_QUERY_DS4_HASATTNCONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Enabled         : 0x%02x\n", F01_QUERY_RESET_ENABLED(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Polarity        : 0x%02x\n", F01_QUERY_RESET_POLARITY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Pull-upEnabled  : 0x%02x\n", F01_QUERY_RESET_PULLUPENABLED(ts->map.fn01.query.data));
#else
				memcpy(productID, &F01_QUERY_PRODUCTID(ts->map.fn01.query.data), sizeof(productID));
				productID[10] = '\0';
				SHTPS_LOG_DBG_PRINT("F01 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("Manufacturer ID : 0x%02x\n", F01_QUERY_MANUFACTURERID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("NonCompliant    : 0x%02x\n", F01_QUERY_NONCOMPLIANT(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("cutomMap        : 0x%02x\n", F01_QUERY_CUSTOMMAP(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo0    : 0x%02x\n", F01_QUERY_PRODUCTINFO0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo1    : 0x%02x\n", F01_QUERY_PRODUCTINFO1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeYear    : 0x%02x\n", F01_QUERY_DATACODEYEAR(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeMonth   : 0x%02x\n", F01_QUERY_DATACODEMONTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeDay     : 0x%02x\n", F01_QUERY_DATACODEDAY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("testID_h        : 0x%02x\n", F01_QUERY_TESTID_HI(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("testID_l        : 0x%02x\n", F01_QUERY_TESTID_LO(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("serialNum_h     : 0x%02x\n", F01_QUERY_SERIALNUM_HI(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("serialNum_l     : 0x%02x\n", F01_QUERY_SERIALNUM_LO(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Product ID      : %s\n"    , productID);
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

				break;

			case 0x05:
				SHTPS_LOG_DBG_PRINT("Found: Image Reporting\n");
				ts->map.fn05.enable		= 1;
				ts->map.fn05.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn05.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn05.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn05.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn05.queryBase,
									ts->map.fn05.query.data, sizeof(ts->map.fn05.query.data));

				SHTPS_LOG_DBG_PRINT("F05 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("Has15bitDelta                 : 0x%02x\n", F05_QUERY_HAS16BIT(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("SizeOfF05ImageWindow          : 0x%02x\n", F05_QUERY_IMAGEWINDOWSIZE(ts->map.fn05.query.data));
				break;

			case 0x11:
				SHTPS_LOG_DBG_PRINT("Found: 2-D Sensors\n");
				ts->map.fn11.enable		= 1;
				ts->map.fn11.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn11.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn11.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn11.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn11.queryBase,
									ts->map.fn11.query.data, sizeof(ts->map.fn11.query.data));
				if(rc){
					goto err_exit;
				}
				rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x06, maxPosition, 4);
				if(rc){
					goto err_exit;
				}
				ts->map.fn11.ctrl.maxXPosition = maxPosition[0] | ((maxPosition[1] & 0x0F) << 0x08);
				ts->map.fn11.ctrl.maxYPosition = maxPosition[2] | ((maxPosition[3] & 0x0F) << 0x08);
#if defined( SHTPS_SY_REGMAP_BASE3 )
				SHTPS_LOG_DBG_PRINT("F11 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOf2DSensors              : 0x%02x\n", F11_QUERY_NUMOFSENSORS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery9                      : 0x%02x\n", F11_QUERY_HASQUERY9(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery11                     : 0x%02x\n", F11_QUERY_HASQUERY11(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery12                     : 0x%02x\n", F11_QUERY_HASQUERY12(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery27                     : 0x%02x\n", F11_QUERY_HASQUERY27(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery28                     : 0x%02x\n", F11_QUERY_HASQUERY28(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfFingers                : 0x%02x\n", F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasRelMode                     : 0x%02x\n", F11_QUERY_HASREL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAbsMode                     : 0x%02x\n", F11_QUERY_HASABS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasGestures                    : 0x%02x\n", F11_QUERY_HASGESTURES(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSensitivityAdjust           : 0x%02x\n", F11_QUERY_HASSENSADJUST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Configurable                   : 0x%02x\n", F11_QUERY_CONFIGURABLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfXElectrodes            : 0x%02x\n", F11_QUERY_NUMOFXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfYElectrodes            : 0x%02x\n", F11_QUERY_NUMOFYELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("MaximumElectrodes              : 0x%02x\n", F11_QUERY_MAXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("AbsoluteDataSize               : 0x%02x\n", F11_QUERY_ABSOLUTEDATASIZE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("AnchoredFinger                 : 0x%02x\n", F11_QUERY_ANCHOREDFINGER(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDribble                     : 0x%02x\n", F11_QUERY_HASDRIBBLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasBendingCorrection           : 0x%02x\n", F11_QUERY_HASBENDINGCORRECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLargeObjectSuppression      : 0x%02x\n", F11_QUERY_HASLARGEOBJECTSUPPRESSION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasJitterFilter                : 0x%02x\n", F11_QUERY_HASJITTERFILTER(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasZTuning                     : 0x%02x\n", F11_QUERY_HASZTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAlgorithmSelection          : 0x%02x\n", F11_QUERY_HASALGORITHMSELECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasWTuning                     : 0x%02x\n", F11_QUERY_HASWTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPitchInfo                   : 0x%02x\n", F11_QUERY_HASPITCHINFO(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasFingerSize                  : 0x%02x\n", F11_QUERY_HASFINGERSIZE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSegmentationAggressiveness  : 0x%02x\n", F11_QUERY_HASSEGAGGRESSIVENESS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasTxRxClip                    : 0x%02x\n", F11_QUERY_HASTXRXCLIP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDrummingFilter              : 0x%02x\n", F11_QUERY_HASDRUMMINGFILTER(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSmallObjectDetection        : 0x%02x\n", F11_QUERY_HASSMALLOBJECTDETECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSmallObjectDetectionTuning  : 0x%02x\n", F11_QUERY_HASSMALLOBJECTDETECTIONTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Has8bitW                       : 0x%02x\n", F11_QUERY_HAS8BITW(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Has2DAjustableMapping          : 0x%02x\n", F11_QUERY_HAS2DAJSTMAPPING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasFingerLimit                 : 0x%02x\n", F11_QUERY_HASFINGERLIMIT(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLinearCoefficient2          : 0x%02x\n", F11_QUERY_HASFINGERLIMIT(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPenPositionCorrection       : 0x%02x\n", F11_QUERY_HASPENPOSITIONCORRECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPenJitterFilterCoefficient  : 0x%02x\n", F11_QUERY_HASPENJITTERFILTERCOEFFICIENT(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasGroupDecomposition Control  : 0x%02x\n", F11_QUERY_HASGROUPDECOMPOSITIONCONTROL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLPWG                        : 0x%02x\n", F11_QUERY_HASLPWG(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSmallFingerCorrection       : 0x%02x\n", F11_QUERY_HASSMALLFINGERCORRECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasCtrl88                      : 0x%02x\n", F11_QUERY_HASCTRL88(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasCtrl92                      : 0x%02x\n", F11_QUERY_HASCTRL92(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasCtrl89                      : 0x%02x\n", F11_QUERY_HASCTRL89(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasCtrl90                      : 0x%02x\n", F11_QUERY_HASCTRL90(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasData39                      : 0x%02x\n", F11_QUERY_HASDATA39(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasGloveTuning                 : 0x%02x\n", F11_QUERY_HASGLOVETUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAdvancedFingerTracking      : 0x%02x\n", F11_QUERY_HASADVANCEDFINGERTRACKING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLandLock                    : 0x%02x\n", F11_QUERY_HASLANDLOCK(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("SizeofCtrl92                   : 0x%02x\n", F11_QUERY_SIZEOFCTRL92(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLPWGDoubleTap               : 0x%02x\n", F11_QUERY_HASLPWGDOUBLETAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLPWGSwipe                   : 0x%02x\n", F11_QUERY_HASLPWGSWIPE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLPWGSwipeDistance           : 0x%02x\n", F11_QUERY_HASLPWGSWIPEDISTANCE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDirectionalSwipe            : 0x%02x\n", F11_QUERY_HASDIRECTIONALSWIPE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLowPowerControl             : 0x%02x\n", F11_QUERY_HASLOWPOWERCONTROL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("SizeofCtrl90                   : 0x%02x\n", F11_QUERY_SIZEOFCTRL90(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("SizeofData39                   : 0x%02x\n", F11_QUERY_SIZEOFDATA39(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAggressiveFingerDisambiguation : 0x%02x\n", F11_QUERY_HASAGGRESSIVEFINGERDISAMBIGUATION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasEnhancedFingerTracking      : 0x%02x\n", F11_QUERY_HASENHANCEDFINGERTRACKING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAlternateFingerSorting      : 0x%02x\n", F11_QUERY_HASALTERNATEFINGERSORTING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasCtrl99                      : 0x%02x\n", F11_QUERY_HASCTRL99(ts->map.fn11.query.data));
#else
				SHTPS_LOG_DBG_PRINT("F11 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("numOfSensors    : 0x%02x\n", F11_QUERY_NUMOFSENSORS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("configurable    : 0x%02x\n", F11_QUERY_CONFIGURABLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasSensAdjust   : 0x%02x\n", F11_QUERY_HASSENSADJUST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasGestures     : 0x%02x\n", F11_QUERY_HASGESTURES(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasAbs          : 0x%02x\n", F11_QUERY_HASABS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasRel          : 0x%02x\n", F11_QUERY_HASREL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfFingers    : 0x%02x\n", F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfXElec      : 0x%02x\n", F11_QUERY_NUMOFXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfYElec      : 0x%02x\n", F11_QUERY_NUMOFYELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("maxElec         : 0x%02x\n", F11_QUERY_MAXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasAnchored     : 0x%02x\n", F11_QUERY_HASANCHORED(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("absDataSize     : 0x%02x\n", F11_QUERY_ABSDATASIZE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPinch        : 0x%02x\n", F11_QUERY_HASPINCH(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPress        : 0x%02x\n", F11_QUERY_HASPRESS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasFlick        : 0x%02x\n", F11_QUERY_HASFLICK(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasEarlyTap     : 0x%02x\n", F11_QUERY_HASEARLYTAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasDoubleTap    : 0x%02x\n", F11_QUERY_HASDOUBLETAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasTapHost      : 0x%02x\n", F11_QUERY_HASTAPHOST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasSingleTap    : 0x%02x\n", F11_QUERY_HASSINGLETAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasRotate       : 0x%02x\n", F11_QUERY_HASROTATE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPalmDet      : 0x%02x\n", F11_QUERY_HASPALMDET(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("2D MAX X POS    : 0x%04x\n", ts->map.fn11.ctrl.maxXPosition);
				SHTPS_LOG_DBG_PRINT("2D MAX Y POS    : 0x%04x\n", ts->map.fn11.ctrl.maxYPosition);
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
				break;

			case 0x34:
				SHTPS_LOG_DBG_PRINT("Found: Flash memory management\n");
				ts->map.fn34.enable		= 1;
				ts->map.fn34.queryBase = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn34.dataBase  = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn34.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn34.queryBase,
									ts->map.fn34.query.data, sizeof(ts->map.fn34.query.data));
				if(rc){
					goto err_exit;
				}
				SHTPS_LOG_DBG_PRINT("F34 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("bootLoaderID0       : 0x%02x\n", F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("bootLoaderID1       : 0x%02x\n", F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("unlocked            : 0x%02x\n", F34_QUERY_UNLOCKED(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("blockSize           : 0x%04x\n", F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("firmBlockCount      : 0x%04x\n", F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("configBlockCount    : 0x%04x\n", F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data));
				break;

			case 0x54:
				SHTPS_LOG_DBG_PRINT("Found: Specification Addendum\n");
				ts->map.fn54.enable		= 1;
				ts->map.fn54.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn54.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn54.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn54.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn54.queryBase,
									ts->map.fn54.query.data, sizeof(ts->map.fn54.query.data));
				if(rc){
					goto err_exit;
				}

#if defined( SHTPS_SY_REGMAP_BASE3 )
				SHTPS_LOG_DBG_PRINT("F54 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasBaseLine                   : 0x%02x\n", F54_QUERY_HASBASELINE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasImage8                     : 0x%02x\n", F54_QUERY_HAS8BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasImage16                    : 0x%02x\n", F54_QUERY_HAS16BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("ClockRate                     : 0x%02x\n", F54_QUERY_CLOCKRATE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("AnalogHardwareFamily          : 0x%02x\n", F54_QUERY_ANALOGHARDWAREFAMILY(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPixelTouchThresholdTuning  : 0x%02x\n", F54_QUERY_HASPIXELTOUCHTHRESHOLDTUNING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasArbitrarySensorAssignment  : 0x%02x\n", F54_QUERY_HASARBITRARYSENSORASSIGNMENT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasInterferenceMetric         : 0x%02x\n", F54_QUERY_HASINTERFERENCEMETRIC(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasSenseFrequencyControl      : 0x%02x\n", F54_QUERY_HASSENSEFREQCONTROL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasFirmwareNoiseMitigation    : 0x%02x\n", F54_QUERY_HASFWNOISEMITIGATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasLowPowerCtrl               : 0x%02x\n", F54_QUERY_HASLOWPOERCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasTwoByteReportRateReporting : 0x%02x\n", F54_QUERY_HASTWOBYTEREPORTRATEREPORTING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasOneByteReportRateReporting : 0x%02x\n", F54_QUERY_HASONEBYTEREPORTRATEREPORTING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasRelaxationCtrl             : 0x%02x\n", F54_QUERY_HASRELAXATIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("AxisCompensationMode          : 0x%02x\n", F54_QUERY_AXISCOMPENSATIONMODE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasIIRFilter                  : 0x%02x\n", F54_QUERY_HASIIRFILTER(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCMNRemoval                 : 0x%02x\n", F54_QUERY_HASCMNREMOVAL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCMNCapScaleFactor          : 0x%02x\n", F54_QUERY_HASCMNCAPSCALEFACTOR(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPixelThresholdHysteresis   : 0x%02x\n", F54_QUERY_HASPIXCELTHRESHHYSTERESIS(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasEdgeCompensation           : 0x%02x\n", F54_QUERY_HASEDGECOMPENSATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPerFrequencyNoiseControl   : 0x%02x\n", F54_QUERY_HASPERFREQNOISECTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasForceFastRelaxation        : 0x%02x\n", F54_QUERY_HASFORCEFASTRELAXATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasMMStateMitigation          : 0x%02x\n", F54_QUERY_HASMMSTATEMITIGATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCDM4                       : 0x%02x\n", F54_QUERY_HASCDM4(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasVarianceMetric             : 0x%02x\n", F54_QUERY_HASVARIANCEMETRIC(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has0DRelaxation               : 0x%02x\n", F54_QUERY_HAS0DRELAXATIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has0DAcquisitionCtrl          : 0x%02x\n", F54_QUERY_HAS0DACQUISITIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("MaxNumOfSensingFrequencies    : 0x%02x\n", F54_QUERY_MAXNUMOFSENSINGFREQ(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("BurstsPerCluster              : 0x%02x\n", F54_QUERY_BURSTSPERCLUSTER(ts->map.fn54.query.data));
#else
				SHTPS_LOG_DBG_PRINT("F54 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has16bitDelta                 : 0x%02x\n", F54_QUERY_HAS16BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has8bitDelta                  : 0x%02x\n", F54_QUERY_HAS8BIT(ts->map.fn54.query.data));
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
				break;

			case 0x19:
				SHTPS_LOG_DBG_PRINT("Found: 0-D Capacitivve Buttons\n");
				ts->map.fn19.enable		= 1;
				ts->map.fn19.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn19.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn19.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn19.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn19.queryBase,
									ts->map.fn19.query.data, sizeof(ts->map.fn19.query.data));
				if(rc){
					goto err_exit;
				}

				SHTPS_LOG_DBG_PRINT("F19 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("HasSensitivityAdjust   : 0x%02x\n", F19_QUERY_HASSENSITIVITYADJUST(ts->map.fn19.query.data));
				SHTPS_LOG_DBG_PRINT("HasHysteresisThreshold : 0x%02x\n", F19_QUERY_HASHYSTERESISTHRESHOLD(ts->map.fn19.query.data));
				SHTPS_LOG_DBG_PRINT("ButtonCount            : 0x%02x\n", F19_QUERY_BUTTONCOUNT(ts->map.fn19.query.data));
				break;

			case 0x1A:
				SHTPS_LOG_DBG_PRINT("Found: 0-D Capacitivve Buttons\n");
				ts->map.fn1A.enable		= 1;
				ts->map.fn1A.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn1A.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn1A.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn1A.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn1A.queryBase,
									ts->map.fn1A.query.data, sizeof(ts->map.fn1A.query.data));
				if(rc){
					goto err_exit;
				}

				SHTPS_LOG_DBG_PRINT("F1A Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("MaxButtonCount               : 0x%02x\n", F1A_QUERY_MAX_BUTTONCOUNT(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasGeneralControle           : 0x%02x\n", F1A_QUERY_HASGENERALCONTROL(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasInterruptEnable           : 0x%02x\n", F1A_QUERY_HASINTERRUPTENABLE(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasMultiButtonSelect         : 0x%02x\n", F1A_QUERY_HASMULTIBUTTONSELECT(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasTxRxMapping               : 0x%02x\n", F1A_QUERY_HASTXRXMAPPING(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasPerButtonThreshold        : 0x%02x\n", F1A_QUERY_HASPERBUTTONTHRESHOLD(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasReleaseThreshold          : 0x%02x\n", F1A_QUERY_HASRELEASETHRESHOLD(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasStrongestButtonHysteresis : 0x%02x\n", F1A_QUERY_HASSTRONGESTBUTTONHYSTERESIS(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasFilterStrength            : 0x%02x\n", F1A_QUERY_HASFILTERSTRENGTH(ts->map.fn1A.query.data));
				break;

			case 0x51:
				SHTPS_LOG_DBG_PRINT("Found: Custom\n");
				ts->map.fn51.enable		= 1;
				ts->map.fn51.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn51.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn51.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn51.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn51.queryBase,
									ts->map.fn51.query.data, sizeof(ts->map.fn51.query.data));
				if(rc){
					goto err_exit;
				}

				SHTPS_LOG_DBG_PRINT("F51 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("QueryRegisterCount   : 0x%02x\n", F51_QUERY_QUERYREGISTERCOUNT(ts->map.fn51.query.data));
				SHTPS_LOG_DBG_PRINT("DataRegisterCount    : 0x%02x\n", F51_QUERY_DATAREGISTERCOUNT(ts->map.fn51.query.data));
				SHTPS_LOG_DBG_PRINT("ControlRegisterCount : 0x%02x\n", F51_QUERY_CONTROLREGISTERCOUNT(ts->map.fn51.query.data));
				SHTPS_LOG_DBG_PRINT("CommandRegisterCount : 0x%02x\n", F51_QUERY_COMMANDREGISTERCOUNT(ts->map.fn51.query.data));
				break;

			default:
				break;
			}
		}
	}
	shtps_rmi_write(ts, 0xFF, 0x00);

	if(0 == ts->map.fn01.enable){
		SHTPS_LOG_ERR_PRINT("map construct error. fn01=disable\n");
		rc = -1;
		goto err_exit;
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(func_check){
		u8 buf;
		shtps_rmi_read(ts, ts->map.fn01.dataBase, &buf, 1);
		if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
			#ifdef CONFIG_SHTERM
				shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
				SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (first check)\n");
			#endif
			SHTPS_LOG_DBG_PRINT("FW CRC error detect. status = 0x%02X\n", buf & 0x0F);
			if(	ts->map.fn34.enable == 0){
				rc = -1;
				SHTPS_LOG_ERR_PRINT("map construct error. fw status = 0x%02X / fn34=disable\n", buf);
				goto err_exit;
			}
		}else{
			if(	ts->map.fn11.enable == 0 ||
				#if defined(SHTPS_PHYSICAL_KEY_ENABLE)
					ts->map.fn1A.enable == 0 ||
				#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
				ts->map.fn54.enable == 0)
			{
				rc = -1;
				SHTPS_LOG_ERR_PRINT("map construct error. fw status = 0x02%X / fn12=%s, fn1A=%s, fn54=%s\n",
					buf,
					(ts->map.fn11.enable == 1)? "enable" : "disable",
					#if defined(SHTPS_PHYSICAL_KEY_ENABLE)
						(ts->map.fn1A.enable == 1)? "enable" : "disable",
					#else
						"don't care",
					#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
					(ts->map.fn54.enable == 1)? "enable" : "disable");
				goto err_exit;
			}
		}
	}
#endif /* SHTPS_PDT_READ_RETRY_ENABLE) */

	#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
		shtps_rmi_read(ts, ts->map.fn54.ctrlBase + 0x04, &ts->pixel_touch_threshold_def_val, 1);
	#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_get_lpwg_def_settings(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_rmi_read(ts, ts->map.fn11.ctrlBase + SHTPS_REG_OFFSET_SEGMENTATION_AGGRESSIVENESS,
							&ts->pinch_fail_reject.segmentation_aggressiveness_def, 1);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
		shtps_scan_freq_dynamic_change_reg_init(ts);
	#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */
	
	return 0;

err_exit:
	return rc;
}

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
static void shtps_wake_lock_idle(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_cpu_idle_sleep_ctrl_lock);
	if(ts->wake_lock_idle_state == 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle on\n");
		pm_qos_update_request(&ts->qos_cpu_latency, SHTPS_QOS_LATENCY_DEF_VALUE);
		ts->wake_lock_idle_state = 1;
	}
	mutex_unlock(&shtps_cpu_idle_sleep_ctrl_lock);
}

static void shtps_wake_unlock_idle(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_cpu_idle_sleep_ctrl_lock);
	if(ts->wake_lock_idle_state != 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle off\n");
		pm_qos_update_request(&ts->qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
		ts->wake_lock_idle_state = 0;
	}
	mutex_unlock(&shtps_cpu_idle_sleep_ctrl_lock);
}
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
static void shtps_wake_lock_for_fwupdate(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
	if(ts->wake_lock_for_fwupdate_state == 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_for_fwupdate on\n");
		wake_lock(&ts->wake_lock_for_fwupdate);
		pm_qos_update_request(&ts->qos_cpu_latency_for_fwupdate, SHTPS_QOS_LATENCY_DEF_VALUE);
		ts->wake_lock_for_fwupdate_state = 1;
	}
	mutex_unlock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
}

static void shtps_wake_unlock_for_fwupdate(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
	if(ts->wake_lock_for_fwupdate_state != 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_for_fwupdate off\n");
		wake_unlock(&ts->wake_lock_for_fwupdate);
		pm_qos_update_request(&ts->qos_cpu_latency_for_fwupdate, PM_QOS_DEFAULT_VALUE);
		ts->wake_lock_for_fwupdate_state = 0;
	}
	mutex_unlock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
}
#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

static void shtps_reset(struct shtps_rmi_spi *ts)
{
#if 0
	struct pm_gpio param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
	};
	int rc;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__HW_RESET, "");

	param.output_value = 0;
	rc = pm8xxx_gpio_config(ts->rst_pin, &param);
	if(rc != 0){
		SHTPS_LOG_ERR_PRINT("pm8xxx_gpio_config(rst_pin,0) error:%d\n",rc);
	}

	udelay(SHTPS_HWRESET_TIME_US);

	param.output_value = 1;
	pm8xxx_gpio_config(ts->rst_pin, &param);
	if(rc != 0){
		SHTPS_LOG_ERR_PRINT("pm8xxx_gpio_config(rst_pin,1) error:%d\n",rc);
	}

	mdelay(SHTPS_HWRESET_AFTER_TIME_MS);
#else
	shtps_device_reset(ts->rst_pin);
#endif
}

static void shtps_sleep(struct shtps_rmi_spi *ts, int on)
{
	u8 val;

	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	_log_msg_sync( LOGMSG_ID__SET_SLEEP, "%d", on);
	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &val, 1);
	if(on){
		shtps_delayed_rezero_cancel(ts);
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			val &= ~0x04;
			shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
			shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val | 0x01);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_SLEEP);

		#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
			if(SHTPS_SLEEP_IN_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_IN_WAIT_MS);
			}
		#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	}else{
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				if(((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE) ||
				   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_LCD_BRIGHT) ||
				   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_HOVER_OFF)){
					val |= 0x04;
					shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
					shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
				}else{
					val &= ~0x04;
					shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
					shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);
				}
			#else
				val |= 0x04;
				shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
				shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val & 0xFC);

		#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
			if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_OUT_WAIT_MS);
			}
		#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	}

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		SHTPS_LOG_DBG_PRINT("[LPMODE]sleep request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
								ts->lpmode_req_state, ts->lpmode_continuous_req_state);
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}

#if defined( SHTPS_LOW_POWER_MODE_ENABLE ) || defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
static void shtps_set_doze_mode(struct shtps_rmi_spi *ts, int on)
{
	u8 val;
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &val, 1);
	if((val & 0x03) != 0){
		return;
	}
	
	if(on){
		val &= ~0x04;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);
		SHTPS_LOG_DBG_PRINT("doze mode on ([0x%02x] <- 0x%02x)\n", ts->map.fn01.ctrlBase, val);
	}else{
		val |= 0x04;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
		SHTPS_LOG_DBG_PRINT("doze mode off([0x%02x] <- 0x%02x)\n", ts->map.fn01.ctrlBase, val);
	}
}
#endif /* defined( SHTPS_LOW_POWER_MODE_ENABLE ) || defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE ) */

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
static void shtps_set_lpmode_init(struct shtps_rmi_spi *ts)
{
	if(((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE) ||
	   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_LCD_BRIGHT) ||
	   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_HOVER_OFF)){
		shtps_set_doze_mode(ts, 0);
	}else{
		shtps_set_doze_mode(ts, 1);
	}

	SHTPS_LOG_DBG_PRINT("[LPMODE]lpmode init. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
							ts->lpmode_req_state, ts->lpmode_continuous_req_state);
}
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
static int shtps_is_lpmode(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		if(((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE) ||
		   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_LCD_BRIGHT) ||
		   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_HOVER_OFF)){
			return 0;
		}else{
			return 1;
		}
	#else
		return 0;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}
#endif /* #if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE ) */

static void shtps_set_lpmode(struct shtps_rmi_spi *ts, int type, int req, int on)
{
	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		int changed = 0;

		if(on){
			if(type == SHTPS_LPMODE_TYPE_NON_CONTINUOUS){
				if((ts->lpmode_req_state & req) == 0){
					ts->lpmode_req_state |= req;
					changed = 1;
					SHTPS_LOG_DBG_PRINT("[LPMODE]<ON> type = NON CONTINUOUS, req = %s\n",
											(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
											(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
											(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
				}
			}else{
				if((ts->lpmode_continuous_req_state & req) == 0){
					ts->lpmode_continuous_req_state |= req;
					changed = 1;
					SHTPS_LOG_DBG_PRINT("[LPMODE]<ON> type = CONTINUOUS, req = %s\n",
											(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
											(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
											(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
				}
			}
		}
		else{
			if(type == SHTPS_LPMODE_TYPE_NON_CONTINUOUS){
				if((ts->lpmode_req_state & req) != 0){
					ts->lpmode_req_state &= ~req;
					changed = 1;
					SHTPS_LOG_DBG_PRINT("[LPMODE]<OFF> type = NON CONTINUOUS, req = %s\n",
											(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
											(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
											(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
				}
			}else{
				if((ts->lpmode_continuous_req_state & req) != 0){
					ts->lpmode_continuous_req_state &= ~req;
					changed = 1;
					SHTPS_LOG_DBG_PRINT("[LPMODE]<OFF> type = CONTINUOUS, req = %s\n",
											(req == SHTPS_LPMODE_REQ_COMMON)? "common" :
											(req == SHTPS_LPMODE_REQ_ECO)? "eco" :
											(req == SHTPS_LPMODE_REQ_HOVER_OFF)? "hover_off" :
											(req == SHTPS_LPMODE_REQ_LCD_BRIGHT)? "lcd_bright" : "unknown");
				}
			}
		}

		if(changed){
			if(((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_NONE) ||
			   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_LCD_BRIGHT) ||
			   ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) == SHTPS_LPMODE_REQ_HOVER_OFF)){
				shtps_set_doze_mode(ts, 0);
			}else{
				shtps_set_doze_mode(ts, 1);
			}
		}

		SHTPS_LOG_DBG_PRINT("[LPMODE]lpmode request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
								ts->lpmode_req_state, ts->lpmode_continuous_req_state);
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_lpwg_notify_interval_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_lpwg_ctrl *lpwg_p = container_of(dw, struct shtps_lpwg_ctrl, notify_interval_delayed_work);

	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_ctrl_lock);
	lpwg_p->notify_enable = 1;
	mutex_unlock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval end\n");
}

static void shtps_lpwg_notify_interval_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->lpwg.notify_interval_delayed_work);
}

static void shtps_lpwg_notify_interval_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	schedule_delayed_work(&ts->lpwg.notify_interval_delayed_work, msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_INTERVAL));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
static void shtps_lpwg_proximity_check_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, proximity_check_delayed_work);

	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK);
}

static void shtps_lpwg_proximity_check_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->proximity_check_delayed_work);
}

static void shtps_lpwg_proximity_check_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_proximity_check_cancel(ts);
	schedule_delayed_work(&ts->proximity_check_delayed_work, msecs_to_jiffies(SHTPS_LPWG_PROXIMITY_CHECK_PREWAIT));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
static void shtps_touchkey_inproxymity_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, touchkey_inproxymity_delayed_work);

	SHTPS_LOG_FUNC_CALL();
	
	mutex_lock(&shtps_ctrl_lock);
	if(ts->key_down_reserved){
		ts->key_down_ignored |= ts->key_down_reserved;
		ts->key_down_reserved = 0;
		SHTPS_LOG_DBG_PRINT("[key] proximity near. ignore key=0x%02x\n", ts->key_down_ignored);
	}
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_touchkey_inproxymity_delayed_work_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->touchkey_inproxymity_delayed_work);
}

static void shtps_touchkey_inproxymity_delayed_work_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_touchkey_inproxymity_delayed_work_cancel(ts);
	schedule_delayed_work(&ts->touchkey_inproxymity_delayed_work, msecs_to_jiffies(SHTPS_KEY_PROXIMITY_DOWN_HOLD_TIME_MS));
}

static void shtps_touchkey_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, touchkey_delayed_work);
	int prox_data;
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();
	
	prox_data = shtps_proximity_check(ts);


	mutex_lock(&shtps_ctrl_lock);
	
	if (prox_data == SHTPS_PROXIMITY_NEAR) {
		shtps_touchkey_inproxymity_delayed_work_start(ts);
	}
	else{
		if( (ts->key_down_reserved & (1 << SHTPS_PHYSICAL_KEY_DOWN)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], 1);
			input_sync(ts->input_key);
			SHTPS_LOG_EVENT(
				printk(KERN_DEBUG "[shtps][key]Notify event KEY_VOLUMEDOWN:DOWN\n");
			);
			isEvent = 1;

			ts->key_state |= (1 << SHTPS_PHYSICAL_KEY_DOWN);
			ts->key_down_reserved &= ~(1 << SHTPS_PHYSICAL_KEY_DOWN);
		}
		if( (ts->key_down_reserved & (1 << SHTPS_PHYSICAL_KEY_UP)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], 1);
			input_sync(ts->input_key);
			SHTPS_LOG_EVENT(
				printk(KERN_DEBUG "[shtps][key]Notify event KEY_VOLUMEUP:DOWN\n");
			);
			isEvent = 1;

			ts->key_state |= (1 << SHTPS_PHYSICAL_KEY_UP);
			ts->key_down_reserved &= ~(1 << SHTPS_PHYSICAL_KEY_UP);
		}

		if(isEvent){
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	}

	ts->key_proximity_check_state = 0;

	mutex_unlock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_PRINT("[TouchKey] proximity check end\n");
}

static void shtps_touchkey_delayed_work_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->touchkey_delayed_work);
	ts->key_proximity_check_state = 0;
}

static void shtps_touchkey_delayed_work_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[TouchKey] proximity check start\n");

	shtps_touchkey_delayed_work_cancel(ts);
	schedule_delayed_work(&ts->touchkey_delayed_work, msecs_to_jiffies(0));
	ts->key_proximity_check_state = 1;
}
#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

static void shtps_lpwg_wakelock_init(struct shtps_rmi_spi *ts)
{
	memset(&ts->lpwg, 0, sizeof(ts->lpwg));

	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 0;
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;
	ts->lpwg.lpwg_state  = SHTPS_LPWG_STATE_OFF;
	
	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		ts->lpwg_proximity_get_data = -1;
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
	
    wake_lock_init(&ts->lpwg.wake_lock, WAKE_LOCK_SUSPEND, "shtps_lpwg_wake_lock");
	pm_qos_add_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	ts->lpwg.pm_qos_idle_value = SHTPS_LPWG_QOS_LATENCY_DEF_VALUE;
	INIT_DELAYED_WORK(&ts->lpwg.notify_interval_delayed_work, shtps_lpwg_notify_interval_delayed_work_function);
}

static void shtps_lpwg_wakelock_destroy(struct shtps_rmi_spi *ts)
{
   wake_lock_destroy(&ts->lpwg.wake_lock);
   pm_qos_remove_request(&ts->lpwg.pm_qos_lock_idle);
}

static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on)
{
	if(on){
	    wake_lock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, ts->lpwg.pm_qos_idle_value);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock ON\n");
	}else{
		wake_unlock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_DEFAULT_VALUE);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock OFF\n");
	}
}

static void shtps_get_lpwg_def_settings(struct shtps_rmi_spi *ts)
{
	u8 buf[(sizeof(ts->lpwg.fn11_ctrl92_disable_settings) + 2) * 2];
	
	shtps_rmi_read(ts, ts->map.fn01.ctrlBase + 0x03, &ts->lpwg.doze_wakeup_threshold, 1);
	shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x20, &ts->lpwg.clip[0], 2);
	
	shtps_rmi_read_block(ts, ts->map.fn11.ctrlBase + 0x2D, ts->lpwg.fn11_ctrl92_disable_settings, sizeof(ts->lpwg.fn11_ctrl92_disable_settings), buf);
	memcpy(ts->lpwg.fn11_ctrl92_enable_settings, 
				ts->lpwg.fn11_ctrl92_disable_settings, sizeof(ts->lpwg.fn11_ctrl92_enable_settings));

	ts->lpwg.fn11_ctrl92_enable_settings[0x00] = SHTPS_LPWG_ENABLE_GESTURE;
	ts->lpwg.fn11_ctrl92_enable_settings[0x0F] = SHTPS_LPWG_SWIPE_MINIMUM_DISTANCE;
}

#if defined( SHTPS_ES0_HOST_LPWG_ENABLE )
static int shtps_lpwg_get_hwrev(void)
{
	int hwrev;
	
	if(shtps_system_get_hw_type() != SHTPS_HW_TYPE_HANDSET){
		hwrev = SHTPS_GET_HW_VERSION_RET_ES_0;
	}else{
		hwrev = shtps_system_get_hw_revision();
	}
	
	return hwrev;
}
#endif /* SHTPS_ES0_HOST_LPWG_ENABLE */

static void shtps_set_lpwg_mode_on(struct shtps_rmi_spi *ts)
{
	u8 data;

	if(ts->lpwg.lpwg_switch == 1){
		return;
	}

	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		ts->lpwg_proximity_get_data = -1;
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

	shtps_lpwg_notify_interval_stop(ts);
	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 1;
	ts->lpwg.is_notified = 0;
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 1, 0x00);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	
	shtps_rmi_write_block(ts, ts->map.fn11.ctrlBase + 0x2D, ts->lpwg.fn11_ctrl92_enable_settings, 
							sizeof(ts->lpwg.fn11_ctrl92_enable_settings));

	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, ts->lpwg.doze_wakeup_threshold);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x20, SHTPS_LPWG_RX_CLIP_LO);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x21, SHTPS_LPWG_RX_CLIP_HI);

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		#if defined( SHTPS_ES0_HOST_LPWG_ENABLE )
		if(SHTPS_HOST_LPWG_ENABLE != 0 || shtps_lpwg_get_hwrev() == SHTPS_GET_HW_VERSION_RET_ES_0)
		#else
		if(SHTPS_HOST_LPWG_ENABLE != 0)
		#endif /* SHTPS_ES0_HOST_LPWG_ENABLE */
		{
			int i;

			for(i = 0; i < SHTPS_FINGER_MAX; i++){
				ts->lpwg.pre_info.fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->lpwg.pre_info.fingers[i].x     = 0xFFFF;
				ts->lpwg.pre_info.fingers[i].y     = 0xFFFF;
			}
			ts->lpwg.swipe_check_time = 0;
		}
		else{
			shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x04);
		}
	#else
		shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
		shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x04);
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &data, 1);
	
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase, data & ~0x04);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase, data & ~0x07);
	shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);

	#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
		if(data & 0x01){
			if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_OUT_WAIT_MS);
			}
		}
	#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */

	SHTPS_LOG_DBG_PRINT("[LPWG] device set doze [0x%02X]", data & ~0x07);
	SHTPS_LOG_DBG_PRINT("LPWG mode ON\n");
}

static void shtps_set_lpwg_mode_off(struct shtps_rmi_spi *ts)
{
	u8 data;

	if(ts->lpwg.lpwg_switch == 0){
		return;
	}
	
	ts->lpwg.lpwg_switch = 0;

	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &data, 1);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase, (data & ~0x03) | 0x01);
	shtps_set_dev_state(ts, SHTPS_DEV_STATE_SLEEP);

	#if defined( SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE )
		if((data & 0x01) == 0){
			if(SHTPS_SLEEP_IN_WAIT_MS > 0){
				msleep(SHTPS_SLEEP_IN_WAIT_MS);
			}
		}
	#endif /* SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */

	SHTPS_LOG_DBG_PRINT("[LPWG] device set sleep [0x%02X]", (data & ~0x03) | 0x01);
	
	shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x00);

	shtps_rmi_write_block(ts, ts->map.fn11.ctrlBase + 0x2D, ts->lpwg.fn11_ctrl92_disable_settings, 
							sizeof(ts->lpwg.fn11_ctrl92_disable_settings));

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 1, ((1 << SHTPS_PHYSICAL_KEY_NUM) - 1) & 0xFF);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL_DEF);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF_DEF);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x20, ts->lpwg.clip[0]);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x21, ts->lpwg.clip[1]);

	SHTPS_LOG_DBG_PRINT("LPWG mode OFF\n");
}


static void shtps_set_lpwg_mode_cal(struct shtps_rmi_spi *ts)
{
	u8 data[1];
	
	if(SHTPS_LPWG_MODE_ON_AFTER_SLEEP_ENABLE){
		shtps_rmi_read(ts, ts->map.fn01.ctrlBase, data, 1);
		data[0] &= ~0x03;
		data[0] |= 0x01;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, data[0]);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_SLEEP);
		SHTPS_LOG_DBG_PRINT("[LPWG] device set sleep [0x%02X]", data[0]);
		if(SHTPS_SLEEP_IN_WAIT_MS > 0){
			mdelay(SHTPS_SLEEP_IN_WAIT_MS);
		}
		data[0] &= ~0x03;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, data[0]);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);
		SHTPS_LOG_DBG_PRINT("[LPWG] device set doze [0x%02X]", data[0]);
		if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
			mdelay(SHTPS_SLEEP_OUT_WAIT_MS);
		}
	}
	
	if(SHTPS_LPWG_MODE_ON_AFTER_REZERO_ENABLE){
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_REZERO, 0);
	}
}

static int shtps_is_lpwg_active(struct shtps_rmi_spi *ts)
{
	int ret = 0;
	
	if(ts->lpwg.lpwg_state == SHTPS_LPWG_STATE_OFF){
		ret = 0;
	}else if(ts->lpwg.lpwg_state == SHTPS_LPWG_STATE_ON){
		ret = 1;
	}

	return ret;
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

static void shtps_set_charger_armor_init(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		if(ts->charger_armor_state == 0){
			#if defined( SHTPS_JITTER_FILTER_ENABLE )
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x2A, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
			#endif /* #if defined( SHTPS_JITTER_FILTER_ENABLE ) */

			#if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE )
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x02, SHTPS_CHARGER_ARMOR_SATURATION_DEFAULT_LSB);
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x03, SHTPS_CHARGER_ARMOR_SATURATION_DEFAULT_MSB);
				shtps_command_force_update(ts);
				shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
														SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
			#endif /* #if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE ) */

			#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x04, ts->pixel_touch_threshold_def_val);
			#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */
		}
		else{
			#if defined( SHTPS_JITTER_FILTER_ENABLE )
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x2A, SHTPS_CHARGER_ARMOR_STRENGTH);
			#endif /* #if defined( SHTPS_JITTER_FILTER_ENABLE ) */

			#if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE )
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x02, SHTPS_CHARGER_ARMOR_SATURATION_LSB);
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x03, SHTPS_CHARGER_ARMOR_SATURATION_MSB);
				shtps_command_force_update(ts);
				shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
														SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
			#endif /* #if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE ) */

			#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
				shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x04, SHTPS_CHARGER_ARMOR_PIXEL_TOUCH_THRESHOLD);
			#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */
		}
	#endif /* SHTPS_CHARGER_ARMOR_ENABLE */
}

static int shtps_set_charger_armor(struct shtps_rmi_spi *ts, int on)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		if(on != 0){
			if(ts->charger_armor_state == 0){
				ts->charger_armor_state = 1;

				if(shtps_is_uimode(ts)){
					#if defined( SHTPS_JITTER_FILTER_ENABLE )
						shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x2A, SHTPS_CHARGER_ARMOR_STRENGTH);
					#endif /* #if defined( SHTPS_JITTER_FILTER_ENABLE ) */
	
					#if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE )
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x02, SHTPS_CHARGER_ARMOR_SATURATION_LSB);
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x03, SHTPS_CHARGER_ARMOR_SATURATION_MSB);
						shtps_command_force_update(ts);
						shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
																SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
					#endif /* #if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE ) */
	
					#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x04, SHTPS_CHARGER_ARMOR_PIXEL_TOUCH_THRESHOLD);
					#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */
				}
			}
		}
		else{
			if(ts->charger_armor_state != 0){
				ts->charger_armor_state = 0;

				if(shtps_is_uimode(ts)){
					#if defined( SHTPS_JITTER_FILTER_ENABLE )
						shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x2A, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
					#endif /* #if defined( SHTPS_JITTER_FILTER_ENABLE ) */
	
					#if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE )
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x02, SHTPS_CHARGER_ARMOR_SATURATION_DEFAULT_LSB);
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x03, SHTPS_CHARGER_ARMOR_SATURATION_DEFAULT_MSB);
						shtps_command_force_update(ts);
						shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
																SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
					#endif /* #if defined( SHTPS_SATURATION_CAPACITANCE_ENABLE ) */
	
					#if defined( SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE )
						shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x04, ts->pixel_touch_threshold_def_val);
					#endif /* SHTPS_PIXEL_TOUCH_THRESHOLD_ENABLE */
				}
			}
		}
	#endif /* SHTPS_CHARGER_ARMOR_ENABLE */

	return 0;
}

#if defined(SHTPS_HOVER_REJECTION_ENABLE)
static void shtps_init_hover_rejection_threshold(struct shtps_rmi_spi *ts)
{
	if(SHTPS_HOVER_REJECT_ENABLE == 0){
		return;
	}

	if(F11_QUERY_HASCTRL99(ts->map.fn11.query.data) != 0){
		shtps_rmi_write(ts, ts->map.fn11.ctrlBase + SHTPS_REG_ADDR_OFFSET_HOVER_REJECTION_THRESHOLD, SHTPS_REG_PRM_HOVER_REJECTION_THRESHOLD);
	}
}
#endif /* SHTPS_HOVER_REJECTION_ENABLE */

static int shtps_fwdate(struct shtps_rmi_spi *ts, u8 *year, u8 *month)
{
	u8 buf[2] = { 0x00, 0x00 };
	u8 retry = 3;

	do{
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 4, buf, 2);
		if(buf[0] != 0x00 && buf[1] != 0x00){
			break;
		}
	}while(retry-- > 0);

	*year = buf[0] & 0x1F;
#if defined( SHTPS_SY_REGMAP_BASE3 )
	*month= ((buf[0] >> 5) & 0x07) | ((buf[1] << 3) & 0x08 );
#else
	*month= buf[1] & 0x0F;
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

	_log_msg_sync( LOGMSG_ID__GET_FWDATE, "%d|%d", *year, *month);
	return 0;
}

static int shtps_get_serial_number(struct shtps_rmi_spi *ts, u8 *buf)
{
	int ret;
	u8 addr;
	
	ret = shtps_rmi_read(ts, 0xe3, &addr, 1);
	if(!ret){
		ret = shtps_rmi_read(ts, addr + 4, buf, 7);
	}
	
	return ret;
}

static u16 shtps_fwver(struct shtps_rmi_spi *ts)
{
	u8 ver[2] = { 0x00, 0x00 };
	u8 retry = 3;

	do{
		#if defined( SHTPS_SY_REGMAP_BASE3 )
		shtps_rmi_read(ts, ts->map.fn34.ctrlBase + 2, ver, 2);
		#else
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 2, ver, 2);
		#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
		if(ver[0] != 0x00 || ver[1] != 0x00){
			break;
		}
	}while(retry-- > 0);

	_log_msg_sync( LOGMSG_ID__GET_FWVER, "0x%04X", ((ver[0] << 0x08) & 0xff00) | ver[1]);
	return ((ver[0] << 0x08) & 0xff00) | ver[1];
}

static u16 shtps_fwver_builtin(struct shtps_rmi_spi *ts)
{
	u16 ver = 0;

	#if defined(SHTPS_MULTI_FW_ENABLE)
		ver = SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].ver;
		if(ver == 0){
			ver = SHTPS_FWVER_NEWER;
		}
	#else
		ver = (u16)SHTPS_FWVER_NEWER;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return ver;
}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static int shtps_fwsize_builtin(struct shtps_rmi_spi *ts)
{
	int size = 0;

	#if defined(SHTPS_MULTI_FW_ENABLE)
		size = SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].size;
		if(size == 0){
			size = SHTPS_FWSIZE_NEWER;
		}
	#else
		size = SHTPS_FWSIZE_NEWER;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return size;
}
#endif /* SHTPS_FWUPDATE_BUILTIN_ENABLE */

static unsigned char* shtps_fwdata_builtin(struct shtps_rmi_spi *ts)
{
	unsigned char *data = NULL;

	#if defined(SHTPS_MULTI_FW_ENABLE)
		data = (unsigned char *)SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].data;
		if(data == NULL){
			data = (unsigned char *)tps_fw_data;
		}
	#else
		data = (unsigned char *)tps_fw_data;
	#endif /* SHTPS_MULTI_FW_ENABLE */

	return data;
}

#if defined(SHTPS_MULTI_FW_ENABLE)
static void shtps_get_multi_fw_type(struct shtps_rmi_spi *ts)
{
	ts->multi_fw_type = SHTPS_MULTI_FW_INFO_SIZE - 1;

	#if defined(SHTPS_CHECK_HWID_ENABLE)
	{
		u8 i;
		u8 hwrev = 0; // SHTPS_GET_HW_VERSION_RET_MP;

		if(shtps_system_get_hw_type() != SHTPS_HW_TYPE_HANDSET){
			hwrev = SHTPS_GET_HW_VERSION_RET_ES_0;
		}else{
			hwrev = shtps_system_get_hw_revision();
		}
		
		for(i = 0;i < SHTPS_MULTI_FW_INFO_SIZE;i++){
			if(SHTPS_MULTI_FW_INFO_TBL[i].hwrev == hwrev){
				ts->multi_fw_type = i;
				break;
			}
		}
	}
	#endif /* SHTPS_CHECK_HWID_ENABLE */
}
#endif /* SHTPS_MULTI_FW_ENABLE */

static int shtps_init_param(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__FW_INIT, "");
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_MULTI_FW_ENABLE)
	{
		shtps_get_multi_fw_type(ts);
		SHTPS_LOG_DBG_PRINT("multi fw type is %s\n", SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].name);
	}
	#endif /* SHTPS_MULTI_FW_ENABLE */

	rc = shtps_rmi_write(ts, ts->map.fn01.dataBase, 0x00);
	SPI_ERR_CHECK(rc, err_exit);

	if(ts->map.fn11.enable){
		u8 data;
		rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x00);
		SPI_ERR_CHECK(rc, err_exit);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			SPI_ERR_CHECK(rc, err_exit);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
				SPI_ERR_CHECK(rc, err_exit);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
	}
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1, SHTPS_IRQ_ALL);
	SPI_ERR_CHECK(rc, err_exit);

	#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x84);
	shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
	#else
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x80);
	shtps_set_dev_state(ts, SHTPS_DEV_STATE_DOZE);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
	SPI_ERR_CHECK(rc, err_exit);

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		if(ts->map.fn1A.enable){
			int	button_cnt = SHTPS_PHYSICAL_KEY_NUM;

			rc = shtps_rmi_write(ts, 0xFF, ((ts->map.fn1A.ctrlBase >> 8) & 0xFF));
			SPI_ERR_CHECK(rc, err_exit);

			rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase, 0x00);
			SPI_ERR_CHECK(rc, err_exit);

			rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 1, ((1 << button_cnt) - 1) & 0xFF);
			SPI_ERR_CHECK(rc, err_exit);

			rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 2, 0x00);
			SPI_ERR_CHECK(rc, err_exit);

			rc = shtps_rmi_write(ts, 0xFF, 0x00);
			SPI_ERR_CHECK(rc, err_exit);
		}
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL_DEF);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF_DEF);
		SPI_ERR_CHECK(rc, err_exit);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
		{
			u8  val;
			_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
			shtps_rmi_read(ts, 0xF0, &val, 1);
			shtps_rmi_write(ts, 0xF0, val | 0x01);
		}
	#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

	shtps_set_charger_armor_init(ts);

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		shtps_set_lpmode_init(ts);
	#endif /* SHTPS_LOW_POWER_MODE_ENABLE */

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_init_palmthresh(ts);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

	#if defined(SHTPS_SET_MINIMUM_DRUMMING_SEPARATION_ENABLE)
		if(SHTPS_MINIMUM_DRUMMING_SEPARATION_ENABLE){
			rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x24, SHTPS_MINIMUM_DRUMMING_SEPARATION_VALUE);
			SPI_ERR_CHECK(rc, err_exit);
		}
	#endif /* SHTPS_SET_MINIMUM_DRUMMING_SEPARATION_ENABLE */
	
	#if defined(SHTPS_HOVER_REJECTION_ENABLE)
		shtps_init_hover_rejection_threshold(ts);
	#endif /* SHTPS_HOVER_REJECTION_ENABLE */
	
	return 0;

err_exit:
	return -1;
}

static void shtps_standby_param(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__FW_STANDBY, "");
	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_absorption_hold_cancel(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		ts->wakeup_touch_event_inhibit_state = 0;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
	
	shtps_sleep(ts, 1);
}

static void shtps_clr_startup_err(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->state_mgr.starterr = SHTPS_STARTUP_SUCCESS;
}

static int shtps_wait_startup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_COMP_WAIT, "");
	wait_event_interruptible(ts->wait_start,
		ts->state_mgr.state == SHTPS_STATE_ACTIVE          ||
		ts->state_mgr.state == SHTPS_STATE_BOOTLOADER      ||
		ts->state_mgr.state == SHTPS_STATE_FWTESTMODE      ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP           ||
		ts->state_mgr.state == SHTPS_STATE_FACETOUCH       ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH ||
		ts->state_mgr.starterr == SHTPS_STARTUP_FAILED);

	_log_msg_recv( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);

	return ts->state_mgr.starterr;
}

static void shtps_notify_startup(struct shtps_rmi_spi *ts, u8 err)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(err);
	ts->state_mgr.starterr = err;
	_log_msg_send( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);
	wake_up_interruptible(&ts->wait_start);
}

static void shtps_set_startmode(struct shtps_rmi_spi *ts, u8 mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_MODE, "%d", mode);
	ts->state_mgr.mode = mode;
}

static int shtps_get_startmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->state_mgr.mode;
}

static inline int shtps_get_fingermax(struct shtps_rmi_spi *ts)
{
	const int finger_tbl[8]={1, 2, 3, 4, 5, 10, 1, 1};

	if(finger_tbl[F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data)] > SHTPS_FINGER_MAX){
		return SHTPS_FINGER_MAX;
	}
	return finger_tbl[F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data)];
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_get_diff(unsigned short pos1, unsigned short pos2, unsigned long factor)
{
	int diff = (pos1 * factor / 10000) - (pos2 * factor / 10000);
	return (diff >= 0)? diff : -diff;
}

static void shtps_rec_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(ts->touch_state.dragStepReturnTime[index][xy]);
}

static int shtps_chk_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

static int shtps_get_fingerwidth(struct shtps_rmi_spi *ts, int num, struct shtps_touch_info *info)
{
	int w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
	#if defined( SHTPS_FINGER_WIDTH_MODERATION_ENABLE )
	if (!SHTPS_FINGER_WIDTH_MODERATION_DISABLE){
		if (ts->report_info.fingers[num].state == SHTPS_TOUCH_STATE_FINGER &&
			info->fingers[num].state == SHTPS_TOUCH_STATE_FINGER) {
			int dwx = info->fingers[num].wx - ts->report_info.fingers[num].wx;
			int dwy = info->fingers[num].wy - ts->report_info.fingers[num].wy;
			if (dwx > SHTPS_FINGER_WIDTH_GAIN_THRESHOLD ||
				dwy > SHTPS_FINGER_WIDTH_GAIN_THRESHOLD) {
				ts->w_before_gain[num] = (ts->report_info.fingers[num].wx >= ts->report_info.fingers[num].wy) ?
					ts->report_info.fingers[num].wx : ts->report_info.fingers[num].wy;
				SHTPS_LOG_DBG_PRINT("finger width moderation start ([%d] w0=%d)\n", num, ts->w_before_gain[num]);
			}
			if (ts->w_before_gain[num]) {
				if (w > ts->w_before_gain[num]) {
					w = (w - ts->w_before_gain[num]) / SHTPS_FINGER_WIDTH_MODERATION_RATIO + ts->w_before_gain[num];
				} else {
					ts->w_before_gain[num] = 0;
					SHTPS_LOG_DBG_PRINT("finger width moderation end\n");
				}
			}
		} else if (ts->report_info.fingers[num].state == SHTPS_TOUCH_STATE_NO_TOUCH) {
			ts->w_before_gain[num] = 0;
		}
	}
	#endif	/* SHTPS_FINGER_WIDTH_MODERATION_ENABLE */
	return (w < SHTPS_FINGER_WIDTH_MIN)? SHTPS_FINGER_WIDTH_MIN : w;
}

static int shtps_get_dragstep(struct shtps_rmi_spi *ts, int xy, int type, int fingers)
{
	int dragStep;

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_ZERO : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_ZERO : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
		}
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
		}
	}

	return dragStep;
}

static void shtps_set_dragstep(struct shtps_rmi_spi *ts,
		struct shtps_touch_info *info, int type, int xy, int finger)
{
	_log_msg_sync( LOGMSG_ID__SET_DRAG_STEP, "%d|%d|%d", type, xy, finger);

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[finger].x);
			ts->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[finger].y);
			ts->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

static inline void shtps_init_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	ts->drag_hist[finger][xy].pre   = pos;
	ts->drag_hist[finger][xy].count = 0;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		ts->drag_hist[finger][xy].count_up_base = 0;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
}

static void shtps_add_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	int pre = ts->drag_hist[finger][xy].pre;
	u8 dir  = (pos > pre)? SHTPS_DRAG_DIR_PLUS :
			  (pos < pre)? SHTPS_DRAG_DIR_MINUS :
						   SHTPS_DRAG_DIR_NONE;

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int i;
		int drag_smooth_count_limit;
		int drag_smooth_count_limit_new;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

	SHTPS_LOG_DBG_PRINT("add drag hist[%d][%s] pre = %d, cur = %d, dir = %s, cnt = %d, remain time = %d\n",
		finger, (xy == SHTPS_POSTYPE_X)? "X" : "Y",
		pre, pos, 
		(dir == SHTPS_DRAG_DIR_PLUS)? "PLUS" : (dir == SHTPS_DRAG_DIR_MINUS)? "MINUS" : "NONE",
		ts->drag_hist[finger][xy].count,
		time_after(jiffies, ts->touch_state.drag_timeout[finger][xy]));

	if(dir != SHTPS_DRAG_DIR_NONE){
		if(ts->drag_hist[finger][xy].count == 0){
			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(!SHTPS_DRAG_SMOOTH_DISABLE){
					#if defined( SHTPS_DEVELOP_MODE_ENABLE )
						if(SHTPS_DRAG_SMOOTH_COUNT_MIN > SHTPS_DRAG_HISTORY_SIZE_MAX){
							SHTPS_DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
						if(SHTPS_DRAG_SMOOTH_COUNT_MAX > SHTPS_DRAG_HISTORY_SIZE_MAX){
							SHTPS_DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
					#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
					ts->drag_hist[finger][xy].history[ts->drag_hist[finger][xy].count] = pos;
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			ts->drag_hist[finger][xy].dir   = dir;
			ts->drag_hist[finger][xy].count = 1;
		}else{

			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(!SHTPS_DRAG_SMOOTH_DISABLE){
					drag_smooth_count_limit = ts->drag_hist[finger][xy].count;
					drag_smooth_count_limit_new = SHTPS_DRAG_SMOOTH_COUNT_MIN + (ts->drag_hist[finger][xy].count_up_base / SHTPS_DRAG_SMOOTH_COUNT_UP_STEP);
					if(drag_smooth_count_limit < drag_smooth_count_limit_new){
						drag_smooth_count_limit = drag_smooth_count_limit_new;
					}
					if(drag_smooth_count_limit > SHTPS_DRAG_SMOOTH_COUNT_MAX){
						drag_smooth_count_limit = SHTPS_DRAG_SMOOTH_COUNT_MAX;
					}

					if(ts->drag_hist[finger][xy].dir != dir){
						drag_smooth_count_limit = SHTPS_DRAG_SMOOTH_COUNT_MIN;
						if(drag_smooth_count_limit < ts->drag_hist[finger][xy].count){
							for(i= 0; i < drag_smooth_count_limit; i++){
								ts->drag_hist[finger][xy].history[i] = ts->drag_hist[finger][xy].history[ts->drag_hist[finger][xy].count - drag_smooth_count_limit + i];
							}
							ts->drag_hist[finger][xy].count = drag_smooth_count_limit;
						}

						ts->drag_hist[finger][xy].count_up_base = 0;
					}

					if(ts->drag_hist[finger][xy].count < SHTPS_DRAG_SMOOTH_COUNT_MIN-1){
						ts->drag_hist[finger][xy].history[ts->drag_hist[finger][xy].count] = pos;
						ts->drag_hist[finger][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(pos);
						ts->drag_hist[finger][xy].count++;
					}else{
						if(ts->drag_hist[finger][xy].count == drag_smooth_count_limit-1){
							ts->drag_hist[finger][xy].count++;
						}else{
							for(i= 0; i < drag_smooth_count_limit-1; i++){
								ts->drag_hist[finger][xy].history[i] = ts->drag_hist[finger][xy].history[i+1];
							}
						}
						ts->drag_hist[finger][xy].history[drag_smooth_count_limit-1] = pos;

						ts->drag_hist[finger][xy].count_up_base++;
					}
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

			if(ts->drag_hist[finger][xy].dir == dir){
				#ifndef SHTPS_DRAG_SMOOTH_ENABLE
					if(ts->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
						ts->drag_hist[finger][xy].count++;
					}
				#else
					if(SHTPS_DRAG_SMOOTH_DISABLE){
						if(ts->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
							ts->drag_hist[finger][xy].count++;
						}
					}
				#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

				if(ts->drag_hist[finger][xy].count >= SHTPS_DRAG_DIR_FIX_CNT &&
						ts->touch_state.dragStep[finger][xy] == SHTPS_DRAG_THRESHOLD_2ND)
				{
					shtps_rec_notify_time(ts, xy, finger);
					if(xy == SHTPS_POSTYPE_X){
						ts->center_info.fingers[finger].x = pos;
					}else{
						ts->center_info.fingers[finger].y = pos;
					}
					SHTPS_LOG_DBG_PRINT("update center pos(%d, %d) time=%lu\n",
								ts->center_info.fingers[finger].x, ts->center_info.fingers[finger].y,
								ts->touch_state.drag_timeout[finger][xy]);
				}
			}else{
				#ifndef SHTPS_DRAG_SMOOTH_ENABLE
					ts->drag_hist[finger][xy].count = 1;
				#else
					if(SHTPS_DRAG_SMOOTH_DISABLE){
						ts->drag_hist[finger][xy].count = 1;
					}
				#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			}

			ts->drag_hist[finger][xy].dir = dir;
		}

		ts->drag_hist[finger][xy].pre = pos;
	}
}

static inline void shtps_set_eventtype(u8 *event, u8 type)
{
	*event = type;
}

static void shtps_set_touch_info(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info)
{
	int i;
	u8 base;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8* fingerInfo;
	
	if(fingerMax > 8){
		base = 3;
	}else if (fingerMax > 4){
		base = 2;
	}else{
		base = 1;
	}

	memset(&ts->fw_report_info, 0, sizeof(ts->fw_report_info));

	for(i = 0;i < fingerMax;i++){
		fingerInfo = &buf[base + (i * 5)];

		ts->fw_report_info.fingers[i].state	= (buf[i / 4] >> (2 * (i % 4))) & 0x03;
		ts->fw_report_info.fingers[i].x		= F11_DATA_XPOS(fingerInfo);
		ts->fw_report_info.fingers[i].y		= F11_DATA_YPOS(fingerInfo);
//		ts->fw_report_info.fingers[i].y		= ts->map.fn11.ctrl.maxYPosition - F11_DATA_YPOS(fingerInfo);
		ts->fw_report_info.fingers[i].wx	= F11_DATA_WX(fingerInfo);
		ts->fw_report_info.fingers[i].wy	= F11_DATA_WY(fingerInfo);
		ts->fw_report_info.fingers[i].z		= F11_DATA_Z(fingerInfo);

		if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
			ts->fw_report_info.fingers[i].wx |= ((buf[base + (fingerMax * 5) + i + 1] << 0x04) & 0xF0);
			ts->fw_report_info.fingers[i].wy |= (buf[base + (fingerMax * 5) + i + 1] & 0xF0);
		}

		info->fingers[i].state	= ts->fw_report_info.fingers[i].state;
		info->fingers[i].x		= ts->fw_report_info.fingers[i].x;
		info->fingers[i].y		= ts->fw_report_info.fingers[i].y;
		info->fingers[i].wx		= ts->fw_report_info.fingers[i].wx;
		info->fingers[i].wy		= ts->fw_report_info.fingers[i].wy;
		info->fingers[i].z		= ts->fw_report_info.fingers[i].z;

		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			ts->fw_report_info.finger_num++;

			SHTPS_LOG_EVENT(
				printk(KERN_DEBUG "[shtps][%s]Touch Info[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER ? "Finger" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN ? "Pen" :
					ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_HOVER ? "Hover" : "Other" ,
					i,
					ts->fw_report_info.fingers[i].x,
					ts->fw_report_info.fingers[i].y,
					ts->fw_report_info.fingers[i].wx,
					ts->fw_report_info.fingers[i].wy,
					ts->fw_report_info.fingers[i].z
				);
			);
		}
		else if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			SHTPS_LOG_EVENT(
				printk(KERN_DEBUG "[shtps][NoTouch]Touch Info[%d] x=%d, y=%d, wx=%d, wy=%d, z=%d\n",
					i,
					ts->fw_report_info.fingers[i].x,
					ts->fw_report_info.fingers[i].y,
					ts->fw_report_info.fingers[i].wx,
					ts->fw_report_info.fingers[i].wy,
					ts->fw_report_info.fingers[i].z
				);
			);
		}
	}

	#if defined( SHTPS_SY_REGMAP_BASE3 )
		info->gs1 = buf[base + fingerMax * 5] & 0x02;
		info->gs2 = 0;
	#else
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
				info->gs1 		= 0;
				info->gs2 		= buf[base + fingerMax * 5];
			}else{
				info->gs1 		= buf[base + fingerMax * 5];
				info->gs2 		= buf[base + fingerMax * 5 + 1];
			}
		}else{
				info->gs1 		= 0;
				info->gs2 		= 0;
		}
	#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

	#if defined(SHTPS_SHIFT_EDGE_INWARD_ENABLE)
		if(!SHTPS_SHIFT_EDGE_INWARD_DISABLE){
			int i;
			u8 fingerMax = shtps_get_fingermax(ts);

			for(i = 0; i < fingerMax; i++){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if(info->fingers[i].x == 0){
						SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n", i, info->fingers[i].x, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
						info->fingers[i].x = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
					}else if(info->fingers[i].x == (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)){
						SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](x : %d -> %d)\n",
												i, info->fingers[i].x, (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
						info->fingers[i].x = (CONFIG_SHTPS_SY3000_PANEL_SIZE_X-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
					}

					if(info->fingers[i].y == 0){
						SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n", i, info->fingers[i].y, SHTPS_SHIFT_EDGE_INWARD_OFFSET);
						info->fingers[i].y = SHTPS_SHIFT_EDGE_INWARD_OFFSET;
					}else if(info->fingers[i].y == (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)){
						SHTPS_LOG_DBG_PRINT("[SHIFT_EDGE_INWARD][%d](y : %d -> %d)\n",
												i, info->fingers[i].y, (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET);
						info->fingers[i].y = (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y-1)-SHTPS_SHIFT_EDGE_INWARD_OFFSET;
					}
				}
			}
		}
	#endif /* SHTPS_SHIFT_EDGE_INWARD_ENABLE */
}

#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
static void shtps_edge_fail_touch_set_inhibit_finger(struct shtps_rmi_spi *ts, u8 finger, u8 area)
{
	ts->edge_fail_touch_inhibit_id |= (1 << ts->edge_fail_touch_td_info[finger].id);
	
	#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
		if(area == 2){
			ts->right_edge_fail_touch_inhibit_id |= (1 << ts->edge_fail_touch_td_info[finger].id);
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("right edge inhibit finger : 0x%02x\n", ts->right_edge_fail_touch_inhibit_id);
		}
	#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
	
	if(finger == 0){
		memcpy(&ts->edge_fail_touch_td_info[0], &ts->edge_fail_touch_td_info[1], sizeof(struct shtps_edge_fail_touch_info));
	}
	if(ts->edge_fail_touch_td_cnt > 0){
		ts->edge_fail_touch_td_cnt--;
	}
}

static u8 shtps_edge_fail_touch_area_check(unsigned short x, unsigned short y)
{
	u8 area = 0;

	if(y >= SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y){
		if(x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X){
			area = 1;
		}
		else if(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X)){
			area = 2;
		}
	}

	return area;
}

static u8 shtps_edge_fail_touch_side_area_check(unsigned short x, unsigned short y)
{
	u8 is_side_area = 0;

	if( (x <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_X) ||
		(x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_X)) )
	{
		is_side_area = 1;
	}

	return is_side_area;
}

static void shtps_edge_fail_touch_side_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_side_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit clear id : %d\n", i);
			}
		}

		if( (ts->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if( (ts->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_side_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit clear id : %d\n", i);
			}
		}
		else{
			if( (ts->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if( shtps_edge_fail_touch_side_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) == 0 )
					{
						struct shtps_touch_info rep_info;
						u8 event;

						ts->edge_fail_touch_side_inhibit_id &= ~(1 << i);

						memcpy(&rep_info, &ts->report_info, sizeof(struct shtps_touch_info));
						rep_info.fingers[i].state = ts->edge_fail_touch_side_td_info.fingers[i].state;
						rep_info.fingers[i].x  = ts->edge_fail_touch_side_td_info.fingers[i].x;
						rep_info.fingers[i].y  = ts->edge_fail_touch_side_td_info.fingers[i].y;
						rep_info.fingers[i].wx = ts->edge_fail_touch_side_td_info.fingers[i].wx;
						rep_info.fingers[i].wy = ts->edge_fail_touch_side_td_info.fingers[i].wy;
						rep_info.fingers[i].z  = ts->edge_fail_touch_side_td_info.fingers[i].z;

						if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
							rep_info.finger_num++;
						}

						shtps_set_eventtype(&event, 0xff);
						if(ts->touch_state.numOfFingers == 0){
							shtps_set_eventtype(&event, SHTPS_EVENT_TD);
						}else{
							shtps_set_eventtype(&event, SHTPS_EVENT_MTDU);
						}

						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("td report event [%d]<x=%d, y=%d, wx=%d, wy=%d, z=%d>\n",
																i, rep_info.fingers[i].x, rep_info.fingers[i].y,
																rep_info.fingers[i].wx, rep_info.fingers[i].wy, rep_info.fingers[i].z);

						shtps_event_report(ts, &rep_info, event);
					}
				}
			}

			if( (ts->edge_fail_touch_side_inhibit_id & (1 << i)) == 0 ){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
					(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
				{
					if( (shtps_edge_fail_touch_side_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) != 0) )
					{
						u8 w = (ts->fw_report_info.fingers[i].wx > ts->fw_report_info.fingers[i].wy) ?
								ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

						if(w == 0){
							w = 1;
						}

						if( ((ts->fw_report_info.fingers[i].z * 100) / w) <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_ZW_RATIO ){
							ts->edge_fail_touch_side_inhibit_id |= (1 << i);
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit by z/w thresh (id : %d)\n", i);
						}

						if(ts->fw_report_info.fingers[i].z <= SHTPS_EDGE_FAIL_TOUCH_REJECT_SIDE_AREA_THRESH_Z){
							ts->edge_fail_touch_side_inhibit_id |= (1 << i);
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side]inhibit by z thresh (id : %d)\n", i);
						}

						ts->edge_fail_touch_side_td_info.fingers[i].state = info->fingers[i].state;
						ts->edge_fail_touch_side_td_info.fingers[i].x  = ts->fw_report_info.fingers[i].x;
						ts->edge_fail_touch_side_td_info.fingers[i].y  = ts->fw_report_info.fingers[i].y;
						ts->edge_fail_touch_side_td_info.fingers[i].wx = ts->fw_report_info.fingers[i].wx;
						ts->edge_fail_touch_side_td_info.fingers[i].wy = ts->fw_report_info.fingers[i].wy;
						ts->edge_fail_touch_side_td_info.fingers[i].z  = ts->fw_report_info.fingers[i].z;
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_side_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[side][%d] is inhibited\n", i);
		}
	}
}

static void shtps_edge_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i, j;
	u8 fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_EDGE_FAIL_TOUCH_REJECT_ENABLE == 0){
		return;
	}

	if(ts->edge_fail_touch_enable == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
				ts->edge_fail_touch_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("inhibit clear id : %d\n", i);
			}
			#if defined(SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE)
				if( (ts->right_edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
					ts->right_edge_fail_touch_inhibit_id &= ~(1 << i);
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("right edge inhibit finger : 0x%02x\n", ts->right_edge_fail_touch_inhibit_id);
				}
			#endif /* SHTPS_TOUCHKEY_FAIL_TOUCH_REJECTION_ENABLE */
		}

		if( (ts->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if( shtps_edge_fail_touch_area_check(ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y) == 0 ){
					ts->edge_fail_touch_inhibit_id &= ~(1 << i);
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d] inhibit cancel by out of area <x=%d><y=%d>\n",
															i, ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y);
				}
			}
		}

		if( (ts->edge_fail_touch_inhibit_id & (1 << i)) == 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				u8 already_add = 0;

				for(j = 0; j < ts->edge_fail_touch_td_cnt; j++){
					if(i == ts->edge_fail_touch_td_info[j].id){
						already_add = 1;
					}
				}

				if(already_add == 0){
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d]td info add : x=%d, y=%d, z=%d\n", ts->edge_fail_touch_td_cnt,
															ts->fw_report_info.fingers[i].x, ts->fw_report_info.fingers[i].y, ts->fw_report_info.fingers[i].z);

					ts->edge_fail_touch_td_info[ts->edge_fail_touch_td_cnt].x  = ts->fw_report_info.fingers[i].x;
					ts->edge_fail_touch_td_info[ts->edge_fail_touch_td_cnt].y  = ts->fw_report_info.fingers[i].y;
					ts->edge_fail_touch_td_info[ts->edge_fail_touch_td_cnt].z  = ts->fw_report_info.fingers[i].z;
					ts->edge_fail_touch_td_info[ts->edge_fail_touch_td_cnt].id = i;
					ts->edge_fail_touch_td_info[ts->edge_fail_touch_td_cnt].time = jiffies;
					ts->edge_fail_touch_td_cnt++;
				}
			}
			else
			{
				if(ts->edge_fail_touch_td_cnt > 0)
				{
					int find_id = -1;

					for(j = 0; j < ts->edge_fail_touch_td_cnt; j++){
						if(i == ts->edge_fail_touch_td_info[j].id){
							find_id = j;
						}
					}

					if(find_id >= 0){
						for(j = find_id; j < ts->edge_fail_touch_td_cnt; j++){
							if(j < (SHTPS_FINGER_MAX - 1)){
								memcpy(&ts->edge_fail_touch_td_info[j], &ts->edge_fail_touch_td_info[j + 1], sizeof(struct shtps_edge_fail_touch_info));
							}
						}

						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[%d]td info del\n", find_id);
						if(ts->edge_fail_touch_td_cnt > 0){
							ts->edge_fail_touch_td_cnt--;
						}
					}
				}
			}
		}
	}

	if(ts->edge_fail_touch_decide_mt != 0){
		if(ts->edge_fail_touch_td_cnt < 2){
			ts->edge_fail_touch_decide_mt = 0;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide cancel\n");
		}
	}

	if(ts->edge_fail_touch_decide_mt == 0){
		if(ts->edge_fail_touch_td_cnt > 2){
			ts->edge_fail_touch_decide_mt = 1;
			ts->edge_fail_touch_pre_single = 0;
			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide by td num = %d\n", ts->edge_fail_touch_td_cnt);
		}
		else if(ts->edge_fail_touch_td_cnt == 1){
			ts->edge_fail_touch_pre_single = 1;
		}
		else if(ts->edge_fail_touch_td_cnt == 2)
		{
			u8 td_area_1st;
			u8 td_area_2nd;
			int diff_x;
			int diff_y;
			int diff_x_2;
			int diff_y_2;
			u8 now_z_1;
			u8 now_z_2;
			int distance;
			u8 is_check_time_over = 0;

			SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("2nd td detect\n");

			td_area_1st = shtps_edge_fail_touch_area_check(ts->edge_fail_touch_td_info[0].x, ts->edge_fail_touch_td_info[0].y);
			td_area_2nd = shtps_edge_fail_touch_area_check(ts->edge_fail_touch_td_info[1].x, ts->edge_fail_touch_td_info[1].y);

			if(time_after(ts->edge_fail_touch_td_info[1].time,
							ts->edge_fail_touch_td_info[0].time + msecs_to_jiffies(SHTPS_EDGE_FAIL_TOUCH_REJECT_EFFECT_TIME)) == 0)
			{
				is_check_time_over = 0;
			}else{
				is_check_time_over = 1;
			}

			if(td_area_1st != 0){
				if(is_check_time_over != 0){
					ts->edge_fail_touch_decide_mt = 1;
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide by 2nd td check time over\n");
				}
				else{
					diff_x = shtps_get_diff(ts->edge_fail_touch_td_info[0].x, ts->edge_fail_touch_td_info[1].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->edge_fail_touch_td_info[0].y, ts->edge_fail_touch_td_info[1].y, SHTPS_POS_SCALE_Y(ts));
					distance = (diff_x * diff_x) + (diff_y * diff_y);

					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check start\n");

					if(td_area_2nd != 0){
						if(td_area_1st == td_area_2nd){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area same side\n");

							if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
								if(ts->edge_fail_touch_td_info[0].z < ts->edge_fail_touch_td_info[1].z){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
									if( ((ts->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
										if(ts->edge_fail_touch_pre_single != 0){
											if( (ts->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_td_info[0].id)) == 0 ){
												SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
												shtps_event_touch_cancel(ts, ts->edge_fail_touch_td_info[0].id);
											}
										}

										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_td_info[0].id);
										shtps_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
									}
									else{
										ts->edge_fail_touch_decide_mt = 1;
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
									}
								}
								else if(ts->edge_fail_touch_td_info[0].z > ts->edge_fail_touch_td_info[1].z){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
									if( ((ts->edge_fail_touch_td_info[0].z * 100) / ts->edge_fail_touch_td_info[1].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_td_info[1].id);
										shtps_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
									}
									else{
										ts->edge_fail_touch_decide_mt = 1;
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
									}
								}
								else{
									ts->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else{
								ts->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
						else{
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area opposite side\n");

							if(ts->edge_fail_touch_td_info[0].z < ts->edge_fail_touch_td_info[1].z){
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
								if( ((ts->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
									if(ts->edge_fail_touch_pre_single != 0){
										if( (ts->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_td_info[0].id)) == 0 ){
											SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
											shtps_event_touch_cancel(ts, ts->edge_fail_touch_td_info[0].id);
										}
									}

									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_td_info[0].id);
									shtps_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
								}
								else{
									ts->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else if(ts->edge_fail_touch_td_info[0].z > ts->edge_fail_touch_td_info[1].z){
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
								if( ((ts->edge_fail_touch_td_info[0].z * 100) / ts->edge_fail_touch_td_info[1].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_td_info[1].id);
									shtps_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
								}
								else{
									ts->edge_fail_touch_decide_mt = 1;
									SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
								}
							}
							else{
								ts->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
					}
					else{
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]check area out side\n");

						if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 2\n");
							if( ((ts->edge_fail_touch_td_info[1].z * 100) / ts->edge_fail_touch_td_info[0].z) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2 ){
								if(ts->edge_fail_touch_pre_single != 0){
									if( (ts->edge_fail_touch_side_inhibit_id & (1 << ts->edge_fail_touch_td_info[0].id)) == 0 ){
										SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("touch cancel event report\n");
										shtps_event_touch_cancel(ts, ts->edge_fail_touch_td_info[0].id);
									}
								}

								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]inhibit id : %d\n", ts->edge_fail_touch_td_info[0].id);
								shtps_edge_fail_touch_set_inhibit_finger(ts, 0, td_area_1st);
							}
							else{
								ts->edge_fail_touch_decide_mt = 1;
								SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
							}
						}
						else{
							ts->edge_fail_touch_decide_mt = 1;
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-1]mt decide\n");
						}
					}
				}
			}
			else{
				if(td_area_2nd != 0){
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]check start\n");

					diff_x_2 = shtps_get_diff(ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].x,
												ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].x, SHTPS_POS_SCALE_X(ts));
					diff_y_2 = shtps_get_diff(ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].y,
												ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].y, SHTPS_POS_SCALE_Y(ts));
					distance = (diff_x_2 * diff_x_2) + (diff_y_2 * diff_y_2);

					now_z_1 = ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].z;
					now_z_2 = ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].z;

					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("info check [1]<x=%d, y=%d, z=%d> [2]<x=%d, y=%d, z=%d>\n",
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].x,
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].y,
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[0].id].z,
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].x,
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].y,
														ts->fw_report_info.fingers[ts->edge_fail_touch_td_info[1].id].z);

					if(distance >= (SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTPS_EDGE_FAIL_TOUCH_REJECT_DISTANCE)){
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("use z ratio 1\n");
						if( ((now_z_1 * 100) / now_z_2) >= SHTPS_EDGE_FAIL_TOUCH_REJECT_Z_RATIO ){
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]inhibit add id : %d\n", ts->edge_fail_touch_td_info[1].id);
							shtps_edge_fail_touch_set_inhibit_finger(ts, 1, td_area_2nd);
						}
						else{
							ts->edge_fail_touch_decide_mt = 1;
							SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]mt decide\n");
						}
					}
					else{
						ts->edge_fail_touch_decide_mt = 1;
						SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[2-2]mt decide\n");
					}
				}
				else{
					ts->edge_fail_touch_decide_mt = 1;
					SHTPS_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("mt decide by check area no td\n");
				}
			}

			if(ts->edge_fail_touch_td_cnt == 1){
				ts->edge_fail_touch_pre_single = 1;
			}else{
				ts->edge_fail_touch_pre_single = 0;
			}
		}
		else{
			ts->edge_fail_touch_pre_single = 0;
		}
	}

	shtps_edge_fail_touch_side_check(ts, info);

	for(i = 0; i < fingerMax; i++){
		if( (ts->edge_fail_touch_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_PRINT("[edge_fail_reject][%d] is inhibited\n", i);
		}
	}
}
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
static void shtps_grip_fail_touch_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);

	if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data) == 0){
		SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("not supported by 4bit w fw\n");
		return;
	}

	if(SHTPS_GRIP_FAIL_TOUCH_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
				ts->grip_fail_touch_inhibit_id &= ~(1 << i);
			}
		}

		if( (ts->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
			if( (ts->fw_report_info.fingers[i].x > SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA) &&
				(ts->fw_report_info.fingers[i].x < (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA)) )
			{
				ts->grip_fail_touch_inhibit_id &= ~(1 << i);
				SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[%d] inhibit cancel by area over\n", i);
			}
		}

		if( (ts->grip_fail_touch_inhibit_id & (1 << i)) == 0 ){
			if( (info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
				(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
			{
				if( (ts->fw_report_info.fingers[i].x <= SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA) ||
					(ts->fw_report_info.fingers[i].x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_TOUCH_REJECT_AREA)) )
				{
					if(ts->fw_report_info.fingers[i].z < SHTPS_GRIP_FAIL_TOUCH_REJECT_Z_THRESH){
						ts->grip_fail_touch_inhibit_id |= (1 << i);
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->grip_fail_touch_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[td][%d] is inhibited\n", i);
		}
	}
}

static void shtps_grip_fail_flick_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	int diff_x;
	int diff_y;

	if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data) == 0){
		SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("not supported by 4bit w fw\n");
		return;
	}

	if(SHTPS_GRIP_FAIL_FLICK_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			if( (ts->grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
				ts->grip_fail_flick_inhibit_id &= ~(1 << i);
			}
		}

		if( (ts->grip_fail_flick_inhibit_id & (1 << i)) == 0 ){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
					if( (ts->fw_report_info.fingers[i].x <= SHTPS_GRIP_FAIL_FLICK_REJECT_AREA) ||
						(ts->fw_report_info.fingers[i].x >= (CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1 - SHTPS_GRIP_FAIL_FLICK_REJECT_AREA)) )
					{
						if( (ts->fw_report_info.fingers[i].wx <= SHTPS_GRIP_FAIL_FLICK_REJECT_W_THRESH) &&
							(ts->fw_report_info_store.fingers[i].wx > SHTPS_GRIP_FAIL_FLICK_REJECT_W_THRESH) )
						{
							diff_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
							diff_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
							if(((diff_x * diff_x) + (diff_y * diff_y)) > (SHTPS_GRIP_FAIL_FLICK_REJECT_DIST * SHTPS_GRIP_FAIL_FLICK_REJECT_DIST)){
								ts->grip_fail_flick_inhibit_id |= (1 << i);
							}
						}
					}
				}
			}
		}
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_GRIP_FAIL_TOUCH_PRINT("[flick][%d] is inhibited\n", i);
		}
	}
}
#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
static void shtps_absorption_hold_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()", __func__);
	cancel_delayed_work(&ts->absorption_hold_off_delayed_work);
}

inline static void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z);
static void shtps_absorption_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	u8 fingerMax = shtps_get_fingermax(ts);
	int i;
	int td_finger = -1;
	int tu_finger = -1;
	int numOfFingers = 0;
	#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
		unsigned short x[2], y[2];
	#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
	
	if(!SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD || !SHTPS_FINGER_ABSORPTION_HOLD_TIME_MS){
		ts->absorption_hold_enable = 0;
		return;
	}
	
	if(!ts->absorption_hold_enable){
		if(ts->touch_state.numOfFingers == 2){
			for (i = 0; i < fingerMax; i++) {
				if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					numOfFingers++;
					td_finger = i;

					#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
						if(numOfFingers <= 2){
							x[numOfFingers-1] = info->fingers[i].x;
							y[numOfFingers-1] = info->fingers[i].y;
						}
					#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */

				}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH && 
							ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
				{
					tu_finger = i;
				}
			}
			SHTPS_LOG_DBG_PRINT("%s() report fingers = %d, cur fingers = %d", __func__, 
									ts->touch_state.numOfFingers, numOfFingers);

			#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
				if(numOfFingers == 2){
					ts->pre_diff_x = (x[0] >= x[1])? x[0] - x[1] : x[1] - x[0];
					ts->pre_diff_y = (y[0] >= y[1])? y[0] - y[1] : y[1] - y[0];
				}
			#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */

			if( (numOfFingers == 1) && (td_finger >= 0) && (tu_finger >= 0) ){
				int diff_x = info->fingers[td_finger].x - ts->report_info.fingers[tu_finger].x;
				int diff_y = info->fingers[td_finger].y - ts->report_info.fingers[tu_finger].y;
				
				diff_x = (diff_x < 0)? -diff_x : diff_x;
				diff_y = (diff_y < 0)? -diff_y : diff_y;

				SHTPS_LOG_DBG_PRINT("%s() [%d]pos1 = (%d, %d), [%d]pos2 = (%d, %d)", __func__, 
										td_finger, info->fingers[td_finger].x, info->fingers[td_finger].y,
										tu_finger, ts->report_info.fingers[tu_finger].x, ts->report_info.fingers[tu_finger].y);
				SHTPS_LOG_DBG_PRINT("%s() diff_x = %d, diff_y = %d", __func__, diff_x, diff_y);
				
				if(diff_x < SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD &&
					diff_y < SHTPS_FINGER_ABSORPTION_DIST_THRESHOLD)
				{
					SHTPS_LOG_DBG_PRINT("%s() start hold", __func__);
					ts->absorption_hold_enable = 1;
					ts->absorption_hold_tu_finger = tu_finger;
					memcpy(&ts->absorption_hold_finger_info, info, sizeof(ts->absorption_hold_finger_info));
					
					shtps_absorption_hold_cancel(ts);
					schedule_delayed_work(&ts->absorption_hold_off_delayed_work, 
											msecs_to_jiffies(SHTPS_FINGER_ABSORPTION_HOLD_TIME_MS));
				}
				#if defined(SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE)
				else if(!SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_DISABLE){
					if((diff_x > ts->pre_diff_x && (diff_x - ts->pre_diff_x) > SHTPS_PINCHOUT_DIST_THRESHOLD) ||
					   (diff_y > ts->pre_diff_y && (diff_y - ts->pre_diff_y) > SHTPS_PINCHOUT_DIST_THRESHOLD))
					{
						SHTPS_LOG_DBG_PRINT("%s() start hold (caused by pinch out)", __func__);
						ts->absorption_hold_enable = 1;
						ts->absorption_hold_tu_finger = tu_finger;
						memcpy(&ts->absorption_hold_finger_info, info, sizeof(ts->absorption_hold_finger_info));
						
						shtps_absorption_hold_cancel(ts);
						schedule_delayed_work(&ts->absorption_hold_off_delayed_work, 
												msecs_to_jiffies(SHTPS_PINCHOUT_HOLD_TIME_MS));
					}
				}
				#endif /* SHTPS_PINCHOUT_FAIL_FLICK_RESOLV_ENABLE */
			}
		}
	}else{
		for (i = 0; i < fingerMax; i++) {
			if((info->fingers[i].state != ts->report_info.fingers[i].state && i != ts->absorption_hold_tu_finger) ||
			   (info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH && i == ts->absorption_hold_tu_finger))
			{
				shtps_absorption_hold_cancel(ts);
				
				SHTPS_LOG_DBG_PRINT("%s() notify hold tu event", __func__);
				
				if(ts->absorption_hold_tu_finger < SHTPS_FINGER_MAX){
					shtps_report_touch_off(ts, ts->absorption_hold_tu_finger,
								  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].x,
								  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].y,
								  0,
								  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].wx,
								  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].wy,
								  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].z);
					
					input_sync(ts->input);
					ts->touch_state.numOfFingers = 1;
					memcpy(&ts->report_info.fingers[ts->absorption_hold_tu_finger], 
							&ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger], 
							sizeof(ts->report_info.fingers[ts->absorption_hold_tu_finger]));
				}

				ts->absorption_hold_enable = 0;
				ts->absorption_hold_tu_finger = 0;

				break;
			}
		}
	}
	
}

static u8 shtps_absorption_hold_status(struct shtps_rmi_spi *ts)
{
	return ts->absorption_hold_enable;
}

static void shtps_absorption_hold_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, absorption_hold_off_delayed_work);
	
	mutex_lock(&shtps_ctrl_lock);
	if(ts->absorption_hold_enable){
		SHTPS_LOG_DBG_PRINT("%s() notify hold tu event", __func__);
		if(ts->absorption_hold_tu_finger < SHTPS_FINGER_MAX){
			shtps_report_touch_off(ts, ts->absorption_hold_tu_finger,
						  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].x,
						  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].y,
						  0,
						  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].wx,
						  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].wy,
						  ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger].z);
			
			input_sync(ts->input);
			ts->touch_state.numOfFingers = 1;
			memcpy(&ts->report_info.fingers[ts->absorption_hold_tu_finger], 
					&ts->absorption_hold_finger_info.fingers[ts->absorption_hold_tu_finger], 
					sizeof(ts->report_info.fingers[ts->absorption_hold_tu_finger]));
		}

		ts->absorption_hold_enable = 0;
		ts->absorption_hold_tu_finger = 0;
	}
	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_absorption_init(struct shtps_rmi_spi *ts)
{
	ts->absorption_hold_enable = 0;
	ts->absorption_hold_tu_finger = 0;
	memset(&ts->absorption_hold_finger_info, 0, sizeof(ts->absorption_hold_finger_info));
	INIT_DELAYED_WORK(&ts->absorption_hold_off_delayed_work, shtps_absorption_hold_delayed_work_function);
}

#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
static void shtps_pinch_fail_response_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 numOfFingers = 0;
	u16 finger_x[SHTPS_FINGER_MAX], finger_y[SHTPS_FINGER_MAX];
	int diff_x, diff_y;

	if(SHTPS_PINCH_FAIL_RESPONSE_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			finger_x[numOfFingers] = ts->fw_report_info.fingers[i].x;
			finger_y[numOfFingers] = ts->fw_report_info.fingers[i].y;
			numOfFingers++;
		}
	}

	if(numOfFingers == 2){
		diff_x = shtps_get_diff(finger_x[0], finger_x[1], SHTPS_POS_SCALE_X(ts));
		diff_y = shtps_get_diff(finger_y[0], finger_y[1], SHTPS_POS_SCALE_Y(ts));
		ts->pinch_fail_reject.finger_distance = ((diff_x * diff_x) + (diff_y * diff_y));
	}else{
		ts->pinch_fail_reject.finger_distance = 0;
	}

	SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("finger distance : %d\n", ts->pinch_fail_reject.finger_distance);

	if(ts->pinch_fail_reject.segmentation_aggressiveness_set_state != 0){
		if( (numOfFingers != 2) ||
			(ts->pinch_fail_reject.finger_distance > (SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH)) )
		{
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + SHTPS_REG_OFFSET_SEGMENTATION_AGGRESSIVENESS,
								ts->pinch_fail_reject.segmentation_aggressiveness_def);
			ts->pinch_fail_reject.segmentation_aggressiveness_set_state = 0;
			SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set def [%d]\n", ts->pinch_fail_reject.segmentation_aggressiveness_def);
		}
	}

	if(ts->pinch_fail_reject.segmentation_aggressiveness_set_state == 0){
		if(ts->pinch_fail_reject.segmentation_aggressiveness_set_check != 0){
			if(numOfFingers != 2){
				ts->pinch_fail_reject.segmentation_aggressiveness_set_check = 0;
				SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("check end by finger num [%d]", numOfFingers);
			}else{
				if(ts->pinch_fail_reject.finger_distance <= (SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_2ND_DISTANCE_THRESH)){
					shtps_rmi_write(ts, ts->map.fn11.ctrlBase + SHTPS_REG_OFFSET_SEGMENTATION_AGGRESSIVENESS,
										SHTPS_SEGMENTATION_AGGRESSIVENESS_SET_VAL);
					ts->pinch_fail_reject.segmentation_aggressiveness_set_state = 1;
					SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set [%d]\n", SHTPS_SEGMENTATION_AGGRESSIVENESS_SET_VAL);
				}
			}
		}

		if(ts->pinch_fail_reject.segmentation_aggressiveness_set_check == 0){
			if(ts->pinch_fail_reject.finger_distance > (SHTPS_PINCH_FAIL_FINGER_1ST_DISTANCE_THRESH * SHTPS_PINCH_FAIL_FINGER_1ST_DISTANCE_THRESH)){
				ts->pinch_fail_reject.segmentation_aggressiveness_set_check = 1;
				SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("check start\n");
			}
		}
	}
}
#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
static void shtps_pinchout_outset_distort_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 numOfFingers = 0;
	u8 numOfFingers_old = 0;
	int diff_x, diff_y, diff_z;
	u8 id_1st = 0;
	u8 id_2nd = 0;

	if(SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			numOfFingers_old++;
			id_1st = i;
		}
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			numOfFingers++;
			if(id_1st != i){
				id_2nd = i;
			}
		}
	}

	if( (numOfFingers_old == 1) && (numOfFingers == 2) ){
		diff_x = shtps_get_diff(info->fingers[id_1st].x, info->fingers[id_2nd].x, SHTPS_POS_SCALE_X(ts));
		diff_y = shtps_get_diff(info->fingers[id_1st].y, info->fingers[id_2nd].y, SHTPS_POS_SCALE_Y(ts));
		diff_z = ts->report_info.fingers[id_1st].z - info->fingers[id_1st].z;

		SHTPS_LOG_DBG_PINCHOUT_DISTORT_PRINT("[id1:%d][id2:%d][diff_x=%d][diff_y=%d][diff_z=%d][distance = %d]\n",
												id_1st, id_2nd, diff_x, diff_y, diff_z, ((diff_x * diff_x) + (diff_y * diff_y)));

		if( ((diff_x * diff_x) + (diff_y * diff_y)) < (SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_DISTANCE_THRESH * SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_DISTANCE_THRESH) ){
			if(diff_z >= SHTPS_PINCHOUT_OUTSET_DISTORT_REJECT_Z_REDUCE_THRESH){
				SHTPS_LOG_DBG_PINCHOUT_DISTORT_PRINT("report touch cancel\n");

				shtps_report_touch_on(ts, id_1st,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, id_1st, &ts->report_info),
									  ts->report_info.fingers[id_1st].wx,
									  ts->report_info.fingers[id_1st].wy,
									  ts->report_info.fingers[id_1st].z);
				input_sync(ts->input);

				shtps_report_touch_off(ts, id_1st,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, id_1st, &ts->report_info),
									  ts->report_info.fingers[id_1st].wx,
									  ts->report_info.fingers[id_1st].wy,
									  ts->report_info.fingers[id_1st].z);
				input_sync(ts->input);

				ts->report_info.fingers[id_1st].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->report_info.fingers[id_1st].x  = SHTPS_TOUCH_CANCEL_COORDINATES_X;
				ts->report_info.fingers[id_1st].y  = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
			}
		}
	}
}
#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */

#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
static void shtps_swap_finger(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int a, int b)
{
	struct shtps_touch_info info_bak;

	memcpy(info_bak.fingers, info->fingers, sizeof(info_bak.fingers));

	info->fingers[a].state = info->fingers[b].state;
	info->fingers[a].wx    = info->fingers[b].wx;
	info->fingers[a].wy    = info->fingers[b].wy;
	info->fingers[a].z     = info->fingers[b].z;

	info->fingers[b].state = info_bak.fingers[a].state;
	info->fingers[b].wx    = info_bak.fingers[a].wx;
	info->fingers[b].wy    = info_bak.fingers[a].wy;
	info->fingers[b].z     = info_bak.fingers[a].z;

	if(ts->lgm_split_touch_combining.finger_adjust != 0)
	{
		int diff_x_1, diff_y_1;
		int diff_x_2, diff_y_2;

		diff_x_1 = ts->report_info.fingers[a].x - info->fingers[b].x;
		diff_y_1 = ts->report_info.fingers[a].y - info->fingers[b].y;
		diff_x_2 = ts->report_info.fingers[b].x - info->fingers[a].x;
		diff_y_2 = ts->report_info.fingers[b].y - info->fingers[a].y;

		info->fingers[a].x = ts->report_info.fingers[a].x - (diff_x_1 / 2);
		info->fingers[a].y = ts->report_info.fingers[a].y - (diff_y_1 / 2);
		info->fingers[b].x = ts->report_info.fingers[b].x - (diff_x_2 / 2);
		info->fingers[b].y = ts->report_info.fingers[b].y - (diff_y_2 / 2);

		ts->lgm_split_touch_combining.finger_adjust = 0;
	}
	else{
		info->fingers[a].x = info->fingers[b].x;
		info->fingers[a].y = info->fingers[b].y;

		info->fingers[b].x = info_bak.fingers[a].x;
		info->fingers[b].y = info_bak.fingers[a].y;
	}

	SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[swap] [%d](%d, %d), state=%d\n", a,
		info->fingers[a].x, info->fingers[a].y, info->fingers[a].state);
	SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[swap] [%d](%d, %d), state=%d\n", b,
		info->fingers[b].x, info->fingers[b].y, info->fingers[b].state);
}

static void shtps_lgm_split_touch_combining_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	u8 swap = 0;
	int dist = SHTPS_LGM_SPLIT_TOUCH_COMBINE_DIST_THRESH;

	if (SHTPS_LGM_SPLIT_TOUCH_COMBINE_ENABLE == 0) {
		return;
	}

	if( (info->fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
		(info->fingers[1].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
	{
		ts->lgm_split_touch_combining.finger_swap = 0;
	}

	if(ts->lgm_split_touch_combining.finger_swap == 0){
		if( (ts->report_info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) &&
			(info->fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
			(ts->report_info.fingers[1].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
			(info->fingers[1].state == SHTPS_TOUCH_STATE_FINGER) )
		{
			if( (abs(ts->report_info.fingers[0].x - info->fingers[1].x) < dist) &&
				(abs(ts->report_info.fingers[0].y - info->fingers[1].y) < dist) )
			{
				swap = 1;
				SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("need swap status detect\n");
			}
		}
		else if( (ts->report_info.fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
				(info->fingers[0].state == SHTPS_TOUCH_STATE_FINGER) &&
				(ts->report_info.fingers[1].state == SHTPS_TOUCH_STATE_FINGER) &&
				(info->fingers[1].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
		{
			if( (abs(ts->report_info.fingers[1].x - info->fingers[0].x) < dist) &&
				(abs(ts->report_info.fingers[1].y - info->fingers[0].y) < dist) )
			{
				swap = 1;
				SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("need swap status detect\n");
			}
		}
	}
	else{
		if( (ts->report_info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) &&
			(info->fingers[0].state == SHTPS_TOUCH_STATE_FINGER) &&
			(ts->report_info.fingers[1].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
			(info->fingers[1].state == SHTPS_TOUCH_STATE_NO_TOUCH) )
		{
			swap = 2;
			SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("need re-swap status detect\n");
		}
		else if( (ts->report_info.fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
				(info->fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
				(ts->report_info.fingers[1].state == SHTPS_TOUCH_STATE_FINGER) &&
				(info->fingers[1].state == SHTPS_TOUCH_STATE_FINGER) )
		{
			swap = 2;
			SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("need re-swap status detect\n");
		}
	}

	if(swap == 1){
		ts->lgm_split_touch_combining.finger_adjust = 1;
		shtps_swap_finger(ts, info, 0, 1);
		ts->lgm_split_touch_combining.finger_swap = 1;
	}
	else if(swap == 2)
	{
		int diff_x_1, diff_y_1;
		int diff_x_2, diff_y_2;

		diff_x_1 = ts->report_info.fingers[0].x - info->fingers[0].x;
		diff_y_1 = ts->report_info.fingers[0].y - info->fingers[0].y;
		diff_x_2 = ts->report_info.fingers[1].x - info->fingers[1].x;
		diff_y_2 = ts->report_info.fingers[1].y - info->fingers[1].y;

		info->fingers[0].x = ts->report_info.fingers[0].x - (diff_x_1 / 2);
		info->fingers[0].y = ts->report_info.fingers[0].y - (diff_y_1 / 2);
		info->fingers[1].x = ts->report_info.fingers[1].x - (diff_x_2 / 2);
		info->fingers[1].y = ts->report_info.fingers[1].y - (diff_y_2 / 2);

		ts->lgm_split_touch_combining.finger_swap = 0;

		SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[re-swap] [0](%d, %d), state=%d\n",
			info->fingers[0].x, info->fingers[0].y, info->fingers[0].state);
		SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("[re-swap] [1](%d, %d), state=%d\n",
			info->fingers[1].x, info->fingers[1].y, info->fingers[1].state);
	}
	else if(ts->lgm_split_touch_combining.finger_swap != 0){
		shtps_swap_finger(ts, info, 0, 1);
	}

	if (ts->lgm_split_touch_combining.finger_swap) {
		SHTPS_LOG_LGM_SPLIT_TOUCH_COMBINE("finger swapped 0<->1");
	}
}
#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
#if defined( SHTPS_LOG_DEBUG_ENABLE )
int SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(int val)
{
	int i;
	int dec_val = (val) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_FIXED_TO_INT((val)));
	int ret = 0;
	int add_val = 10;
	for(i = 0; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val *= 10;
	}
	for(i = 1; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val /= 2;
		if((dec_val >> (SHTPS_DRAG_SMOOTH_FIXED_SHIFT - i)) & 0x01){
			ret += add_val;
		}
	}
	return ret/1000000;
}
#endif /* SHTPS_LOG_DEBUG_ENABLE */
static void shtps_pos_compensation(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int xy)
{
	int inc_ave_FIXED,temp_FIXED;
	int i;
	int drag_smooth_leave_max_dot_FIXED;
	int last_history;
	int last_history_FIXED;

	if(SHTPS_DRAG_SMOOTH_DISABLE){
		return;
	}

	for(i = 0;i < shtps_get_fingermax(ts);i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER /* info->fingers[i].state == SHTPS_TOUCH_STATE_PEN */){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->drag_hist[i][xy].count >= SHTPS_DRAG_SMOOTH_COUNT_MIN){
					drag_smooth_leave_max_dot_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_LEAVE_MAX_DOT);
					last_history		= ts->drag_hist[i][xy].history[ts->drag_hist[i][xy].count - 1];
					last_history_FIXED	= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(last_history);

					inc_ave_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(last_history- ts->drag_hist[i][xy].history[0]) / (ts->drag_hist[i][xy].count-1);
					SHTPS_LOG_DRAG_SMOOTH("   [X]inc_ave=%d.%03d history[0]=%d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(inc_ave_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(inc_ave_FIXED), ts->drag_hist[i][xy].history[0]);
					if(xy == SHTPS_POSTYPE_X){
						if(ts->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}

							if(temp_FIXED < 0){
								info->fingers[i].x = 0;
								ts->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(CONFIG_SHTPS_SY3000_PANEL_SIZE_X)){
								info->fingers[i].x = CONFIG_SHTPS_SY3000_PANEL_SIZE_X -1;
								ts->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x);
							} else{
								info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist[i][xy].history_old = last_history;
						}
					}else{
						if(ts->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = ts->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}

							if(temp_FIXED < 0){
								info->fingers[i].y = 0;
								ts->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(CONFIG_SHTPS_SY3000_PANEL_SIZE_Y)){
								info->fingers[i].y = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y -1;
								ts->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y);
							} else{
								info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist[i][xy].history_old = last_history;
						}
					}
				}
			}
		}
	}
}
#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
static void shtps_wakeup_calib_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	if(SHTPS_WAKEUP_CALIB_ENABLE_SWITCH){
		int i;
		u8 fingerMax = shtps_get_fingermax(ts);
		unsigned short diff_x;
		unsigned short diff_y;
		u8 numOfFingers = 0;
		u8 is_wakeup_calib_cancel = 0;
		u8 key_state = 0;
		u8 large_object_present = (info->gs1 >> 1) & 0x01;

		#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
			shtps_rmi_read(ts, ts->map.fn1A.dataBase, &key_state, 1);
		#endif

		if(ts->wakeup_calib_cancel_check_enable == 0){
			if (ts->wakeup_calib_lastone_req != 0) {
				if(SHTPS_WAKEUP_CALIB_LAST_ONE_ENABLE != 0){
					for(i = 0; i < fingerMax; i++){
						if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
							numOfFingers++;
						}
					}
					if( (numOfFingers == 0) && (key_state == 0) ){
						if((SHTPS_WAKEUP_CALIB_ONLY_LO_DETECTED == 1 && large_object_present != 0) ||
							(SHTPS_WAKEUP_CALIB_GUARD_LO_DETECTED == 1 && large_object_present == 0))
						{
							if(SHTPS_WAKEUP_CALIB_LAST_ONE_DELAY_MS == 0){
								shtps_forcecal(ts);
								SHTPS_LOG_DBG_PRINT("[wakeup_calib] force cal execute (last one)\n");

								ts->wakeup_calib_enable = 0;
								ts->wakeup_calib_deferment = 0;
								ts->wakeup_calib_lastone = 0;
								ts->wakeup_calib_lastone_req = 0;
								shtps_wakeup_calib_timer_stop(ts);
							}else if(SHTPS_WAKEUP_CALIB_LAST_ONE_DELAY_MS > 0){
								ts->wakeup_calib_enable = 1;
								ts->wakeup_calib_lastone = 1;
								ts->wakeup_calib_lastone_req = 0;
								SHTPS_LOG_DBG_PRINT("[wakeup_calib] last one start\n");
								shtps_wakeup_calib_timer_start(ts, SHTPS_WAKEUP_CALIB_LAST_ONE_DELAY_MS);
							}else{
								ts->wakeup_calib_enable = 0;
								ts->wakeup_calib_deferment = 0;
								ts->wakeup_calib_lastone = 0;
								ts->wakeup_calib_lastone_req = 0;
								shtps_wakeup_calib_timer_stop(ts);
								SHTPS_LOG_DBG_PRINT("[wakeup_calib] last one can't exec by delay value\n");
							}
						}
					}
				}
			}
			return;
		}

		numOfFingers = 0;
		for(i = 0; i < fingerMax; i++){
			if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				numOfFingers++;
			}

			if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->wakeup_calib_event_count[i] = 0;
			}

			if(ts->wakeup_calib_cancel_check[i] == 0){
				if( (info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH) && (ts->wakeup_calib_event_count[i] == 0) ){
					ts->wakeup_calib_td_pos[i][SHTPS_POSTYPE_X] = info->fingers[i].x;
					ts->wakeup_calib_td_pos[i][SHTPS_POSTYPE_Y] = info->fingers[i].y;
					ts->wakeup_calib_cancel_check[i] = 1;
					SHTPS_LOG_DBG_PRINT("[wakeup_calib][%d] cancel check zero pos <x:%d><y:%d>\n",
												i, info->fingers[i].x, info->fingers[i].y);
				}
			}
			else{
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					ts->wakeup_calib_cancel_check[i] = 0;
				}
				else{
					if(ts->wakeup_calib_event_count[i] < SHTPS_WAKEUP_CALIB_STOP_COND_EVENT_NUM){
						ts->wakeup_calib_event_count[i]++;
					}

					diff_x = shtps_get_diff(ts->wakeup_calib_td_pos[i][SHTPS_POSTYPE_X], info->fingers[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(ts->wakeup_calib_td_pos[i][SHTPS_POSTYPE_Y], info->fingers[i].y, SHTPS_POS_SCALE_Y(ts));

					SHTPS_LOG_DBG_PRINT("[wakeup_calib][%d] cancel check <diff_x:%d, diff_y:%d, cnt:%d>\n", i, diff_x, diff_y, ts->wakeup_calib_event_count[i]);

					if( (diff_x >= SHTPS_WAKEUP_CALIB_STOP_COND_MOVE_DIST) ||
						(diff_y >= SHTPS_WAKEUP_CALIB_STOP_COND_MOVE_DIST) )
					{
						if(SHTPS_WAKEUP_CALIB_STOP_COND_EVENT_NUM >= 0 && 
							ts->wakeup_calib_event_count[i] >= SHTPS_WAKEUP_CALIB_STOP_COND_EVENT_NUM)
						{
							ts->wakeup_calib_cancel_check_enable = 0;
							ts->wakeup_calib_lastone_req = 1;
							is_wakeup_calib_cancel = 1;
						}
						else{
							ts->wakeup_calib_cancel_check[i] = 0;
							SHTPS_LOG_DBG_PRINT("[wakeup_calib][%d] cancel check end by fast move\n", i);
						}
					}
				}
			}
		}

		if(is_wakeup_calib_cancel != 0){
			ts->wakeup_calib_enable = 0;
			ts->wakeup_calib_deferment = 0;
			shtps_wakeup_calib_timer_stop(ts);
			SHTPS_LOG_DBG_PRINT("[wakeup_calib] stop\n");
		}
		else{
			if(ts->wakeup_calib_deferment != 0){
				if( (numOfFingers == 0) && (key_state == 0) ){
					ts->wakeup_calib_enable = 1;
					ts->wakeup_calib_deferment = 0;
					SHTPS_LOG_DBG_PRINT("[wakeup_calib] timer restart\n");
					shtps_wakeup_calib_timer_start(ts, SHTPS_WAKEUP_CALIB_DELAY_MS);
				}
			}
		}
	}
}
#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

#if defined( SHTPS_CLING_REJECTION_ENABLE )
static void shtps_cling_reject_init_variables(struct shtps_rmi_spi *ts)
{
	memset(ts->cling_reject.mode0_cling_cnt, 0, sizeof(ts->cling_reject.mode0_cling_cnt));
	memset(ts->cling_reject.mode0_state, 0, sizeof(ts->cling_reject.mode0_state));
	ts->cling_reject.mode0_cling_cnt_max = SHTPS_CLING_REJECT_MODE0_CNT;
	ts->cling_reject.mode0_cling_cnt_change_state = 0;
	memset(ts->cling_reject.mode1_cling_cnt_wx, 0, sizeof(ts->cling_reject.mode1_cling_cnt_wx));
	memset(ts->cling_reject.mode1_cling_cnt_wy, 0, sizeof(ts->cling_reject.mode1_cling_cnt_wy));
}

static int shtps_cling_reject_mode0_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	int cling_detect = 0;

	if(SHTPS_CLING_REJECT_MODE0_ENABLE == 0){
		return 0;
	}

	if(ts->cling_reject.mode0_cling_cnt_change_state != 0){
		if( time_after(jiffies, ts->cling_reject.mode0_cling_cnt_change_time) != 0 ){
			ts->cling_reject.mode0_cling_cnt_max = SHTPS_CLING_REJECT_MODE0_CNT;
			ts->cling_reject.mode0_cling_cnt_change_state = 0;
			SHTPS_LOG_CLING_REJECT("[mode_0] count max change to %d\n", ts->cling_reject.mode0_cling_cnt_max);
		}
	}

	for(i = 0; i < fingerMax; i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
				ts->cling_reject.mode0_tap_x[i] = ts->fw_report_info.fingers[i].x;
				ts->cling_reject.mode0_tap_y[i] = ts->fw_report_info.fingers[i].y;
			}
			
			if(ts->cling_reject.mode0_state[i] == 0){
				if(abs(ts->cling_reject.mode0_tap_x[i] - ts->fw_report_info.fingers[i].x) <= SHTPS_CLING_REJECT_MODE0_MOVE_THRESH &&
				   abs(ts->cling_reject.mode0_tap_y[i] - ts->fw_report_info.fingers[i].y) <= SHTPS_CLING_REJECT_MODE0_MOVE_THRESH)
				{
					if(ts->fw_report_info.fingers[i].z < SHTPS_CLING_REJECT_MODE0_Z_THRESH &&
						abs(ts->fw_report_info.fingers[i].z - ts->fw_report_info_store.fingers[i].z) <= SHTPS_CLING_REJECT_MODE0_OFFSET_Z)
					{
						ts->cling_reject.mode0_cling_cnt[i]++;
						SHTPS_LOG_CLING_REJECT("[mode_0]<%d> cling count up : %d\n", i, ts->cling_reject.mode0_cling_cnt[i]);
					}else{
						ts->cling_reject.mode0_cling_cnt[i] = 0;
					}
				}else{
					ts->cling_reject.mode0_state[i] = 1;
					ts->cling_reject.mode0_cling_cnt[i] = 0;
					SHTPS_LOG_CLING_REJECT("[mode_0]<%d> over the move threshold : (%d -> %d, %d -> %d)\n", i, 
						ts->cling_reject.mode0_tap_x[i], ts->fw_report_info.fingers[i].x,
						ts->cling_reject.mode0_tap_y[i], ts->fw_report_info.fingers[i].y);
				}
			}
		}else{
			ts->cling_reject.mode0_state[i] = 0;
			ts->cling_reject.mode0_cling_cnt[i] = 0;
		}

		if(ts->cling_reject.mode0_cling_cnt[i] > ts->cling_reject.mode0_cling_cnt_max){
			cling_detect = 1;
		}
	}

	return cling_detect;
}

static int shtps_cling_reject_mode1_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	int cling_detect = 0;

	if(SHTPS_CLING_REJECT_MODE1_ENABLE == 0){
		return 0;
	}

	for(i = 0; i < fingerMax; i++){
		if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
			(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
		{
			if(ts->fw_report_info.finger_num == ts->fw_report_info_store.finger_num)
			{
				u8 diff_wx = abs(ts->fw_report_info.fingers[i].wx - ts->fw_report_info_store.fingers[i].wx);
				u8 diff_wy = abs(ts->fw_report_info.fingers[i].wy - ts->fw_report_info_store.fingers[i].wy);

				if(diff_wx >= SHTPS_CLING_REJECT_MODE1_WX_THRESH){
					ts->cling_reject.mode1_cling_cnt_wx[i]++;
					SHTPS_LOG_CLING_REJECT("[mode_1]<%d> cling count up (wx) : %d\n", i, ts->cling_reject.mode1_cling_cnt_wx[i]);
				}else{
					ts->cling_reject.mode1_cling_cnt_wx[i] = 0;
				}

				if(diff_wy >= SHTPS_CLING_REJECT_MODE1_WY_THRESH){
					ts->cling_reject.mode1_cling_cnt_wy[i]++;
					SHTPS_LOG_CLING_REJECT("[mode_1]<%d> cling count up (wy) : %d\n", i, ts->cling_reject.mode1_cling_cnt_wy[i]);
				}else{
					ts->cling_reject.mode1_cling_cnt_wy[i] = 0;
				}
			}else{
				ts->cling_reject.mode1_cling_cnt_wx[i] = 0;
				ts->cling_reject.mode1_cling_cnt_wy[i] = 0;
			}
		}else{
			ts->cling_reject.mode1_cling_cnt_wx[i] = 0;
			ts->cling_reject.mode1_cling_cnt_wy[i] = 0;
		}

		if( (ts->cling_reject.mode1_cling_cnt_wx[i] >= SHTPS_CLING_REJECT_MODE1_CNT) ||
			(ts->cling_reject.mode1_cling_cnt_wy[i] >= SHTPS_CLING_REJECT_MODE1_CNT) )
		{
			cling_detect = 1;
		}
	}

	return cling_detect;
}

static int shtps_cling_reject_mode2_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i, j;
	u8 fingerMax = shtps_get_fingermax(ts);
	int cling_detect = 0;

	if(SHTPS_CLING_REJECT_MODE2_ENABLE == 0){
		return 0;
	}

	for(i = 0; i < fingerMax; i++){
		for(j = (i + 1); j < fingerMax; j++){
			if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) &&
				(ts->fw_report_info.fingers[j].state == SHTPS_TOUCH_STATE_FINGER) )
			{
				if( (ts->fw_report_info.fingers[i].x == ts->fw_report_info.fingers[j].x) &&
					(ts->fw_report_info.fingers[i].y == ts->fw_report_info.fingers[j].y) )
				{
					cling_detect = 1;
					SHTPS_LOG_CLING_REJECT("[mode_2] id %d & %d is same pos detect\n", i, j);
				}
			}
		}
	}

	return cling_detect;
}

static int shtps_cling_reject_mode3_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	int cling_detect = 0;

	if(SHTPS_CLING_REJECT_MODE3_ENABLE == 0){
		return 0;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if(ts->fw_report_info.fingers[i].wx >= SHTPS_CLING_REJECT_MODE3_WX_THRESH){
				if(ts->fw_report_info.fingers[i].z <= SHTPS_CLING_REJECT_MODE3_Z_THRESH){
					cling_detect = 1;
				}
			}
		}
	}

	return cling_detect;
}

static void shtps_cling_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int cling_detect = 0;

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		if(ts->facetouch.state != 0){
			SHTPS_LOG_CLING_REJECT("cling detect check disable by face touch mode state\n");
			return;
		}
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

	if( shtps_cling_reject_mode0_check(ts, info) != 0 ){
		SHTPS_LOG_CLING_REJECT("cling detect by <mode : 0>\n");
		cling_detect = 1;
	}

	if( shtps_cling_reject_mode1_check(ts, info) != 0 ){
		SHTPS_LOG_CLING_REJECT("cling detect by <mode : 1>\n");
		cling_detect = 1;
	}

	if( shtps_cling_reject_mode2_check(ts, info) != 0 ){
		SHTPS_LOG_CLING_REJECT("cling detect by <mode : 2>\n");
		cling_detect = 1;
	}

	if( shtps_cling_reject_mode3_check(ts, info) != 0 ){
		SHTPS_LOG_CLING_REJECT("cling detect by <mode : 3>\n");
		cling_detect = 1;
	}

	if(cling_detect != 0)
	{
		int i;
		u8 fingerMax = shtps_get_fingermax(ts);

		if(SHTPS_CLING_REJECT_REZERO_TYPE == 0){
			SHTPS_LOG_CLING_REJECT("rezero execute\n");
			shtps_rezero(ts);
		}else{
			SHTPS_LOG_CLING_REJECT("force cal execute\n");
			shtps_command_force_cal(ts);
		}

		shtps_event_force_touchup(ts);
		for(i = 0; i < fingerMax; i++){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
		}
		shtps_cling_reject_init_variables(ts);
	}
}
#endif /* SHTPS_CLING_REJECTION_ENABLE */

#if defined(SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE)
static void shtps_z_zero_touch_event_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_Z_ZERO_TOUCH_EVENT_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->fw_report_info.fingers[i].z == 0){
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				SHTPS_LOG_Z_ZERO_TOUCH_EVENT_REJECT("[%d] change NoTouch\n", i);
			}
		}
	}
}
#endif /* SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE */

#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
static void shtps_lgm_fail_touch_reject_chatt_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt roop start req\n");
	cancel_delayed_work(&ts->lgm_fail_touch_up_reject.chatt_work);
	schedule_delayed_work(&ts->lgm_fail_touch_up_reject.chatt_work, msecs_to_jiffies(SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_ROOP_TIME));
}

static void shtps_lgm_fail_touch_reject_chatt_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt roop cancel req\n");
	cancel_delayed_work(&ts->lgm_fail_touch_up_reject.chatt_work);
}

static void shtps_lgm_fail_touch_reject_chatt_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_lgm_fail_touch_up_reject *lgm = container_of(dw, struct shtps_lgm_fail_touch_up_reject, chatt_work);
	struct shtps_rmi_spi *ts = container_of(lgm, struct shtps_rmi_spi, lgm_fail_touch_up_reject);

    mutex_lock(&shtps_ctrl_lock);

	SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt roop start\n");

	if(ts->lgm_fail_touch_up_reject.chatt_work_enable != 0){
		if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
			shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
		}
		else if(ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
			shtps_read_touchevent(ts, SHTPS_STATE_FACETOUCH);
		}
		else{
			ts->lgm_fail_touch_up_reject.chatt_work_enable = 0;
		}
	}

	SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt roop end\n");

	mutex_unlock(&shtps_ctrl_lock);
}

static void shtps_lgm_fail_touch_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 numOfFingers = 0;
	u8 z_max = 0;
	u8 chatt_req = 0;

	if(SHTPS_LGM_FAIL_TOUCH_UP_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if( (ts->lgm_fail_touch_up_reject.chatt_finger & (1 << i)) != 0 ){
			if( time_after(jiffies, ts->lgm_fail_touch_up_reject.chatt_finger_tu_ignore_time[i]) == 0 ){
				if(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					info->fingers[i].state = ts->report_info.fingers[i].state;
					info->fingers[i].x  = ts->report_info.fingers[i].x;
					info->fingers[i].y  = ts->report_info.fingers[i].y;
					info->fingers[i].wx = ts->report_info.fingers[i].wx;
					info->fingers[i].wy = ts->report_info.fingers[i].wy;
					info->fingers[i].z  = ts->report_info.fingers[i].z;
					chatt_req = 1;
					SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay continue\n", i)
				}else{
					ts->lgm_fail_touch_up_reject.chatt_finger &= ~(1 << i);
					SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay end by touch state\n", i);
				}
			}
			else{
				ts->lgm_fail_touch_up_reject.chatt_finger &= ~(1 << i);
				ts->lgm_fail_touch_up_reject.chatt_check_enable_finger &= ~(1 << i);
				SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay end by time over\n", i);
				SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay check disable\n", i);
			}
		}

		if( (ts->lgm_fail_touch_up_reject.chatt_finger & (1 << i)) == 0 ){
			if( (ts->lgm_fail_touch_up_reject.chatt_check_enable_finger & (1 << i)) != 0 ){
				if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
					(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
				{
					if( (ts->lgm_fail_touch_up_reject.pos_move[i] < SHTPS_LGM_FAIL_TOUCH_UP_REJECT_DISABLE_MOVE_THRESH) &&
						(ts->lgm_fail_touch_up_reject.z_max[i] < SHTPS_LGM_FAIL_TOUCH_UP_REJECT_DISABLE_Z_THRESH) )
					{
						if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
							if(ts->lgm_fail_touch_up_reject.chatt_enable != 0){
								ts->lgm_fail_touch_up_reject.chatt_finger_tu_ignore_time[i] = jiffies + msecs_to_jiffies(ts->lgm_fail_touch_up_reject.chatt_time);
								ts->lgm_fail_touch_up_reject.chatt_finger |= (1 << i);
								info->fingers[i].state = ts->report_info.fingers[i].state;
								info->fingers[i].x  = ts->report_info.fingers[i].x;
								info->fingers[i].y  = ts->report_info.fingers[i].y;
								info->fingers[i].wx = ts->report_info.fingers[i].wx;
								info->fingers[i].wy = ts->report_info.fingers[i].wy;
								info->fingers[i].z  = ts->report_info.fingers[i].z;
								chatt_req = 1;
								SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay start <%lu ms>\n", i, ts->lgm_fail_touch_up_reject.chatt_time);
							}
						}
						else{
							ts->lgm_fail_touch_up_reject.chatt_check_enable_finger &= ~(1 << i);
							SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay don't execute by already tu\n", i);
							SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay check disable\n", i);
						}
					}
					else{
						ts->lgm_fail_touch_up_reject.chatt_check_enable_finger &= ~(1 << i);
						SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay don't execute move thresh <%d>\n", i, ts->lgm_fail_touch_up_reject.pos_move[i]);
						SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay don't execute z max <%d>\n", i, ts->lgm_fail_touch_up_reject.z_max[i]);
						SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay check disable\n", i);
					}
				}
			}
		}
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_FINGER){
				ts->lgm_fail_touch_up_reject.pos_move[i] = 0;
				ts->lgm_fail_touch_up_reject.td_pos[i][SHTPS_POSTYPE_X] = ts->fw_report_info.fingers[i].x;
				ts->lgm_fail_touch_up_reject.td_pos[i][SHTPS_POSTYPE_Y] = ts->fw_report_info.fingers[i].y;
				ts->lgm_fail_touch_up_reject.z_max[i] = ts->fw_report_info.fingers[i].z;
			}
			else
			{
				int diff_move_x, diff_move_y;

				diff_move_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->fw_report_info_store.fingers[i].x, SHTPS_POS_SCALE_X(ts));
				diff_move_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->fw_report_info_store.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

				if(diff_move_x > diff_move_y){
					ts->lgm_fail_touch_up_reject.pos_move[i] = diff_move_x;
				}else{
					ts->lgm_fail_touch_up_reject.pos_move[i] = diff_move_y;
				}

				if( (ts->lgm_fail_touch_up_reject.chatt_check_enable_finger & (1 << i)) == 0 )
				{
					int diff_td_move_x, diff_td_move_y;

					diff_td_move_x = shtps_get_diff(ts->fw_report_info.fingers[i].x, ts->lgm_fail_touch_up_reject.td_pos[i][SHTPS_POSTYPE_X], SHTPS_POS_SCALE_X(ts));
					diff_td_move_y = shtps_get_diff(ts->fw_report_info.fingers[i].y, ts->lgm_fail_touch_up_reject.td_pos[i][SHTPS_POSTYPE_Y], SHTPS_POS_SCALE_Y(ts));

					if( (diff_td_move_x >= SHTPS_LGM_FAIL_TOUCH_UP_REJECT_ENABLE_MOVE_THRESH) ||
						(diff_td_move_y >= SHTPS_LGM_FAIL_TOUCH_UP_REJECT_ENABLE_MOVE_THRESH) )
					{
						ts->lgm_fail_touch_up_reject.chatt_check_enable_finger |= (1 << i);
						SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("[%d] tu delay check enable\n", i);
					}
				}

				if(ts->lgm_fail_touch_up_reject.z_max[i] < ts->fw_report_info.fingers[i].z){
					ts->lgm_fail_touch_up_reject.z_max[i] = ts->fw_report_info.fingers[i].z;
				}
			}

			if(z_max < ts->fw_report_info.fingers[i].z){
				z_max = ts->fw_report_info.fingers[i].z;
			}
			numOfFingers++;
		}
	}

	if(numOfFingers == 0){
		if(ts->lgm_fail_touch_up_reject.chatt_enable != 0){
			SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt disable by no touch\n");
		}

		ts->lgm_fail_touch_up_reject.chatt_enable = 0;
		ts->lgm_fail_touch_up_reject.chatt_time = 0;
	}
	else if(z_max < SHTPS_LGM_FAIL_TOUCH_UP_REJECT_Z_THRESH_1){
		if(ts->lgm_fail_touch_up_reject.chatt_enable != SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_1){
			SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt enable <%d ms>", SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_1);
		}

		ts->lgm_fail_touch_up_reject.chatt_enable = 1;
		ts->lgm_fail_touch_up_reject.chatt_time = SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_1;
	}
	else if(z_max < SHTPS_LGM_FAIL_TOUCH_UP_REJECT_Z_THRESH_2){
		if(ts->lgm_fail_touch_up_reject.chatt_enable != SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_2){
			SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt enable <%d ms>", SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_2);
		}

		ts->lgm_fail_touch_up_reject.chatt_enable = 1;
		ts->lgm_fail_touch_up_reject.chatt_time = SHTPS_LGM_FAIL_TOUCH_UP_REJECT_CHATT_TIME_2;
	}
	else{
		if(ts->lgm_fail_touch_up_reject.chatt_enable != 0){
			SHTPS_LOG_LGM_FAIL_TOUCH_UP_REJECT("chatt disable by z\n");
		}

		ts->lgm_fail_touch_up_reject.chatt_enable = 0;
		ts->lgm_fail_touch_up_reject.chatt_time = 0;
	}

	if(numOfFingers == 0){
		if(chatt_req != 0){
			ts->lgm_fail_touch_up_reject.chatt_work_enable = 1;
			shtps_lgm_fail_touch_reject_chatt_start(ts);
		}
		else if(ts->lgm_fail_touch_up_reject.chatt_work_enable != 0){
			ts->lgm_fail_touch_up_reject.chatt_work_enable = 0;
			shtps_lgm_fail_touch_reject_chatt_stop(ts);
		}
	}
	else if(ts->lgm_fail_touch_up_reject.chatt_work_enable != 0){
		ts->lgm_fail_touch_up_reject.chatt_work_enable = 0;
		shtps_lgm_fail_touch_reject_chatt_stop(ts);
	}
}
#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */

#if defined(SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE)
static void shtps_edge_touch_shake_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 fingerMax = shtps_get_fingermax(ts);

	if(SHTPS_EDGE_TOUCH_SHAKE_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0; i < fingerMax; i++){
		if(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			ts->edge_shake_check_state[i] = 0;
		}

		if(ts->edge_shake_check_state[i] != 0){
			if( abs(ts->fw_report_info.fingers[i].y - ts->edge_shake_td_info.fingers[i].y) >= SHTPS_EDGE_TOUCH_SHAKE_ADJUST_CANCEL_MOVE_Y ){
				ts->edge_shake_check_state[i] = 0;
				SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] check end by y move\n", i);
			}

			if( (ts->fw_report_info.fingers[i].x - ts->edge_shake_td_info.fingers[i].x) >= SHTPS_EDGE_TOUCH_SHAKE_ADJUST_CANCEL_MOVE_X ){
				ts->edge_shake_check_state[i] = 0;
				SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] check end by x move\n", i);
			}
		}

		if(ts->edge_shake_check_state[i] == 0){
			if( (ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
				(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) )
			{
				if( (ts->fw_report_info.fingers[i].x <= SHTPS_EDGE_TOUCH_SHAKE_AREA_X) &&
					(ts->fw_report_info.fingers[i].y <= SHTPS_EDGE_TOUCH_SHAKE_AREA_Y) )
				{
					SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] check start\n", i);
					ts->edge_shake_td_info.fingers[i].x = ts->fw_report_info.fingers[i].x;
					ts->edge_shake_td_info.fingers[i].y = ts->fw_report_info.fingers[i].y;
					ts->edge_shake_check_state[i] = 1;
				}
			}
		}

		if(ts->edge_shake_check_state[i] != 0){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER)
			{
				int diff_y = abs(info->fingers[i].y - ts->edge_shake_td_info.fingers[i].y);

				if(diff_y > SHTPS_EDGE_TOUCH_SHAKE_ADJUST_POS){
					if(info->fingers[i].y > ts->edge_shake_td_info.fingers[i].y){
						SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] adjust Y (%d -> %d)\n", i, info->fingers[i].y, (info->fingers[i].y - SHTPS_EDGE_TOUCH_SHAKE_ADJUST_POS));
						info->fingers[i].y -= SHTPS_EDGE_TOUCH_SHAKE_ADJUST_POS;
					}else if(info->fingers[i].y < ts->edge_shake_td_info.fingers[i].y){
						SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] adjust Y (%d -> %d)\n", i, info->fingers[i].y, (info->fingers[i].y + SHTPS_EDGE_TOUCH_SHAKE_ADJUST_POS));
						info->fingers[i].y += SHTPS_EDGE_TOUCH_SHAKE_ADJUST_POS;
					}
				}else{
					if(info->fingers[i].y > ts->edge_shake_td_info.fingers[i].y){
						SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] adjust Y (%d -> %d)\n", i, info->fingers[i].y, (info->fingers[i].y - diff_y));
						info->fingers[i].y -= diff_y;
					}else if(info->fingers[i].y < ts->edge_shake_td_info.fingers[i].y){
						SHTPS_LOG_SHTPS_EDGE_TOUCH_SHAKE_REJECT("[%d] adjust Y (%d -> %d)\n", i, info->fingers[i].y, (info->fingers[i].y + diff_y));
						info->fingers[i].y += diff_y;
					}
				}
			}
		}
	}
}
#endif /* SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE */

#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
static void shtps_quick_flick_fail_resolv_init(struct shtps_rmi_spi *ts)
{
	ts->quick_flick_fail_resolv.enable = 1;
	ts->quick_flick_fail_resolv.count  = 0;
}

static void shtps_quick_flick_fail_resolv_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int finger;
	int numOfFingers = 0;
	int fingerMax = shtps_get_fingermax(ts);
	struct timeval tv;
	unsigned long curtime_ms;

	if(!SHTPS_QUICK_FLICK_FAIL_RESOLV_ENABLE){
		return;
	}
	
	do_gettimeofday(&tv);
	curtime_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;

	for(i = 0; i < fingerMax; i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
			finger = i;
		}
	}

	if(numOfFingers == 0){
		if(ts->quick_flick_fail_resolv.enable == 1 && ts->quick_flick_fail_resolv.count >= SHTPS_QUICK_FLICK_FAIL_RESOLV_COUNT){
			int cur_x, cur_y;
			int x, y;
			int ratio_x = 100, ratio_y = 100;

			ts->quick_flick_fail_resolv.count = 0;
			
			if(ts->report_info.fingers[ts->quick_flick_fail_resolv.finger].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("report dummy event\n");

				finger = ts->quick_flick_fail_resolv.finger;
				cur_x = info->fingers[finger].x;
				cur_y = info->fingers[finger].y;
				x = ts->report_info.fingers[finger].x - (((ts->quick_flick_fail_resolv.pre_posx - ts->report_info.fingers[finger].x) * SHTPS_QUICK_FLICK_FAIL_RESOLV_GAIN) / 100);
				y = ts->report_info.fingers[finger].y - (((ts->quick_flick_fail_resolv.pre_posy - ts->report_info.fingers[finger].y) * SHTPS_QUICK_FLICK_FAIL_RESOLV_GAIN) / 100);

				if(x < 0){
					ratio_x = (cur_x * 100) / (cur_x - x);
				}else if(x >= CONFIG_SHTPS_SY3000_PANEL_SIZE_X){
					ratio_x = ((cur_x - (CONFIG_SHTPS_SY3000_PANEL_SIZE_X - 1)) * 100) / (cur_x - x);
				}
				if(y < 0){
					ratio_y = (cur_y * 100) / (cur_y - y);
				}else if(y >= CONFIG_SHTPS_SY3000_PANEL_SIZE_Y){
					ratio_y = ((cur_y - (CONFIG_SHTPS_SY3000_PANEL_SIZE_Y - 1)) * 100) / (cur_y - y);
				}
				
				if(abs(ratio_x) < abs(ratio_y)){
					x = cur_x + ((x - cur_x) * ratio_x) / 100;
					y = cur_y + ((y - cur_y) * ratio_x) / 100;
				}else if(abs(ratio_x) > abs(ratio_y)){
					x = cur_x + ((x - cur_x) * ratio_y) / 100;
					y = cur_y + ((y - cur_y) * ratio_y) / 100;
				}
				
				if(x < 0){
					x = 0;
				}else if(x >= CONFIG_SHTPS_SY3000_PANEL_SIZE_X){
					x = CONFIG_SHTPS_SY3000_PANEL_SIZE_X - 1;
				}

				if(y < 0){
					y = 0;
				}else if(y >= CONFIG_SHTPS_SY3000_PANEL_SIZE_Y){
					y = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y - 1;
				}

				shtps_report_touch_on(ts, finger,
					  x,
					  y,
					  shtps_get_fingerwidth(ts, finger, &ts->report_info),
					  ts->report_info.fingers[finger].wx,
					  ts->report_info.fingers[finger].wy,
					  ts->report_info.fingers[finger].z);
				input_sync(ts->input);
				info->fingers[finger].x = x;
				info->fingers[finger].y = y;
				ts->report_info.fingers[finger].x = x;
				ts->report_info.fingers[finger].y = y;

				#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
					shtps_wakeup_calib_check(ts, &ts->report_info);
				#endif /* SHTPS_WAKEUP_CALIB_ENABLE */
			}
		}
		SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("re-enable\n");
		ts->quick_flick_fail_resolv.enable = 1;
		return;
		
	}else if(numOfFingers == 1){
		if(ts->report_info.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("start\n");
			ts->quick_flick_fail_resolv.count    = 0;
			ts->quick_flick_fail_resolv.finger   = finger;
			ts->quick_flick_fail_resolv.pre_time = curtime_ms;
			ts->quick_flick_fail_resolv.pre_posx = info->fingers[finger].x;
			ts->quick_flick_fail_resolv.pre_posy = info->fingers[finger].y;
		}else if(ts->quick_flick_fail_resolv.finger == finger){
			unsigned long dist_y = abs(ts->report_info.fingers[finger].y - info->fingers[finger].y);
			unsigned long time_diff = curtime_ms - ts->quick_flick_fail_resolv.pre_time;
			
			ts->quick_flick_fail_resolv.pre_time = curtime_ms;
			ts->quick_flick_fail_resolv.pre_posx = ts->report_info.fingers[finger].x;
			ts->quick_flick_fail_resolv.pre_posy = ts->report_info.fingers[finger].y;

			SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("distance = %lu, time diff = %lums\n", dist_y, time_diff);
			if(dist_y != 0 && time_diff != 0){
				unsigned long speed = (dist_y * 1000) / time_diff;
				if(speed >= SHTPS_QUICK_FLICK_FAIL_RESOLV_SPEED_THRESH){
					ts->quick_flick_fail_resolv.count++;
					ts->quick_flick_fail_resolv.enable = 1;
					SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("enable & count up %d. (speed = %lu)\n", ts->quick_flick_fail_resolv.count, speed);
				}else{
					SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("disable by speed\n");
					ts->quick_flick_fail_resolv.count  = 0;
					ts->quick_flick_fail_resolv.enable = 0;
				}
			}else{
				SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("disable by stop\n");
				ts->quick_flick_fail_resolv.enable = 0;
			}
		}else{
			SHTPS_LOG_QUICK_FLICK_FAIL_RESOLV("disable by touch-state\n");
			ts->quick_flick_fail_resolv.enable = 0;
		}
	}else{
		ts->quick_flick_fail_resolv.enable = 0;
	}
}
#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */

#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
static void shtps_scan_freq_dynamic_change_init(struct shtps_rmi_spi *ts)
{
	ts->scan_freq_change.count    = 0;
	ts->scan_freq_change.detect   = 0;
}

static void shtps_scan_freq_dynamic_change_reg_init(struct shtps_rmi_spi *ts)
{
	shtps_rmi_read(ts, ts->map.fn54.ctrlBase + SHTPS_SCAN_FREQ_ADDRESS_OFFSET + SHTPS_SCAN_FREQ_DYNAMIC_1ST_FREQ_INDEX,
						&ts->scan_freq_change.reg_freq[0], 1);
	shtps_rmi_read(ts, ts->map.fn54.ctrlBase + SHTPS_SCAN_FREQ_ADDRESS_OFFSET + SHTPS_SCAN_FREQ_DYNAMIC_2ND_FREQ_INDEX,
						&ts->scan_freq_change.reg_freq[1], 1);
}

static int shtps_scan_freq_dynamic_change_get_curfreq(struct shtps_rmi_spi *ts)
{
	u8 buf = 0;
	
	shtps_rmi_read(ts, ts->map.fn54.dataBase + 0x04, &buf, 1);
	
	SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Current freq index = %d\n", (buf & 0x0F));
	if((buf & 0x0F) == SHTPS_SCAN_FREQ_DYNAMIC_1ST_FREQ_INDEX){
		return 0;
	}else{
		return 1;
	}
}

static void shtps_scan_freq_dynamic_change_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int preNumOfFingers = 0;
	int numOfFingers    = 0;
	struct timeval tv;
	unsigned long curtime_ms;

	if(!SHTPS_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE){
		return;
	}
	
	do_gettimeofday(&tv);
	curtime_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	
	for(i = 0;i < fingerMax;i++){
		if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
		if(ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			preNumOfFingers++;
		}
	}
	
	if(numOfFingers > 0){
		if(preNumOfFingers == 0){
			if(ts->scan_freq_change.count > 0){
				if((curtime_ms - ts->scan_freq_change.pre_time) > SHTPS_SCAN_FREQ_DYNAMIC_TU_TIMEOUT){
					SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Reset count. tu time = %lums\n", curtime_ms - ts->scan_freq_change.pre_time);
					ts->scan_freq_change.count = 0;
				}
			}else{
				ts->scan_freq_change.count = 0;
			}
			ts->scan_freq_change.pre_time = curtime_ms;
		}else{
			if(ts->scan_freq_change.detect == 0 && 
				(curtime_ms - ts->scan_freq_change.pre_time) > SHTPS_SCAN_FREQ_DYNAMIC_CHANGE_INTERVAL_THRESH)
			{
				ts->scan_freq_change.count++;
				SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Count up: %d /%d. interval = %lums\n", 
						ts->scan_freq_change.count, SHTPS_SCAN_FREQ_DYNAMIC_CHANGE_COUNT, curtime_ms - ts->scan_freq_change.pre_time);

				if(ts->scan_freq_change.count >= SHTPS_SCAN_FREQ_DYNAMIC_CHANGE_COUNT){
					SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Detect noise state.\n");
					ts->scan_freq_change.count = 0;
					ts->scan_freq_change.detect= 1;
				}
			}
			ts->scan_freq_change.pre_time = curtime_ms;
		}
	}else{
		if(ts->scan_freq_change.detect){
			u8 offset;
			u8 reg;
			
			ts->scan_freq_change.count = 0;
			ts->scan_freq_change.detect = 0;
			
			if(shtps_scan_freq_dynamic_change_get_curfreq(ts) == 0){
				SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Change freq 1st(%d) -> 2nd(%d)\n",
					SHTPS_SCAN_FREQ_DYNAMIC_1ST_FREQ_INDEX, SHTPS_SCAN_FREQ_DYNAMIC_2ND_FREQ_INDEX);
				offset = SHTPS_SCAN_FREQ_ADDRESS_OFFSET + SHTPS_SCAN_FREQ_DYNAMIC_1ST_FREQ_INDEX;
				reg    = ts->scan_freq_change.reg_freq[0];
			}else{
				SHTPS_LOG_SCAN_FREQ_DYNAMIC_CHANGE("Change freq 2nd(%d) -> 1st(%d)\n",
					SHTPS_SCAN_FREQ_DYNAMIC_2ND_FREQ_INDEX, SHTPS_SCAN_FREQ_DYNAMIC_1ST_FREQ_INDEX);
				offset = SHTPS_SCAN_FREQ_ADDRESS_OFFSET + SHTPS_SCAN_FREQ_DYNAMIC_2ND_FREQ_INDEX;
				reg    = ts->scan_freq_change.reg_freq[1];
			}
			
			shtps_rmi_write(ts, ts->map.fn54.ctrlBase + offset, reg | 0x08);
			shtps_command_force_update(ts);
			shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
													SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
			shtps_rmi_write(ts, ts->map.fn54.ctrlBase + offset, reg & ~0x08);
		}
	}
}
#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
static int shtps_filter_topofscreen_ghost_reject_ghost_check(struct shtps_rmi_spi *ts, u8 finger)
{
	int ret = TOP_OF_SCREEN_GHOST_NOT_DETECTED;
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfSamelineFingers = 0;
	int numOfBottomAreaFingers = 0;
	int bottomAreaZMax = 0;
	unsigned long diff;
	
	if(ts->topofscreen_ghost_reject.is_ghost[finger] != 0){
		return TOP_OF_SCREEN_GHOST_DETECTED;
	}
	
	if(ts->fw_report_info_store.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH &&
		ts->fw_report_info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH)
	{
		if(ts->fw_report_info.fingers[finger].y <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH &&
		   ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_THRESH)
		{
			for(i = 0;i < fingerMax;i++){
				if(i == finger || ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
					continue;
				}
				
				diff = abs(ts->fw_report_info.fingers[i].x - ts->fw_report_info.fingers[finger].x);
				
				if(diff <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH &&
				   ts->fw_report_info.fingers[i].y  >  ts->fw_report_info.fingers[finger].y  &&
				   ts->fw_report_info.fingers[i].z  >  ts->fw_report_info.fingers[finger].z  &&
				   ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[finger].wx &&
				   ((ts->fw_report_info.fingers[finger].z * 100) / ts->fw_report_info.fingers[i].z) < SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_RATIO_THRESH)
				{
					numOfSamelineFingers++;
					if(numOfSamelineFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_FINGER_NUM_THRESH ||
					   ts->fw_report_info.fingers[i].wy >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_FINGER_WY_THRESH)
					{
						ret = TOP_OF_SCREEN_GHOST_DETECTED;
						SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] detect ghost\n", finger);
						break;
					}
				}
				
				if(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_ENABLE){
					if(ts->fw_report_info.fingers[i].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_AREA){
						numOfBottomAreaFingers++;
						bottomAreaZMax = (bottomAreaZMax < ts->fw_report_info.fingers[i].z)? ts->fw_report_info.fingers[i].z : bottomAreaZMax;
					}
				}
			}
		}
		
		if(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_ENABLE){
			if(ret != TOP_OF_SCREEN_GHOST_DETECTED){
				if(bottomAreaZMax == 255 ||
				   (numOfBottomAreaFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_NUM_THRESH &&
				    bottomAreaZMax >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_Z_THRESH))
				{
					if(ts->fw_report_info.fingers[finger].y <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH &&
					   ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_Z_THRESH)
					{
						ret = TOP_OF_SCREEN_GHOST_DETECTED;
						SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] detect ghost by bottom palm touch\n", finger);
					}
				}
			}
		}
	}
	
	return ret;
}

static int shtps_filter_topofscreen_ghost_reject_ghost_clear_check(struct shtps_rmi_spi *ts, u8 finger)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	int numOfBottomAreaFingers = 0;
	int bottomAreaZMax = 0;
	unsigned long diff;

	if(ts->topofscreen_ghost_reject.is_ghost[finger] == 0){
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(ts->fw_report_info.fingers[finger].state == SHTPS_TOUCH_STATE_NO_TOUCH){
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by touch-state\n", finger);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(time_after(jiffies, ts->topofscreen_ghost_reject.timeout[finger])){
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by time\n", finger);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(ts->fw_report_info.fingers[finger].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Y_THRESH ||
	   ts->fw_report_info.fingers[finger].z > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_Z_THRESH)
	{
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by ypos(%d) or z(%d)\n", finger,
			ts->fw_report_info.fingers[finger].y, ts->fw_report_info.fingers[finger].z);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(i == finger || ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
			continue;
		}
		
		diff = abs(ts->fw_report_info.fingers[i].x - ts->fw_report_info.fingers[finger].x);
		
		if(diff <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH){
			return TOP_OF_SCREEN_GHOST_NOT_CLEARED;
		}
		
		if(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_ENABLE){
			if(ts->fw_report_info.fingers[i].y > SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_AREA){
				numOfBottomAreaFingers++;
				bottomAreaZMax = (bottomAreaZMax < ts->fw_report_info.fingers[i].z)? ts->fw_report_info.fingers[i].z : bottomAreaZMax;
			}
		}
	}
	
	if(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_CHECK_ENABLE){
		if(bottomAreaZMax == 255 ||
		   (numOfBottomAreaFingers >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_NUM_THRESH &&
		    bottomAreaZMax >= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_FINGER_Z_THRESH))
		{
			if(ts->fw_report_info.fingers[finger].z <= SHTPS_TOP_OF_SCREEN_GHOST_REJECT_BOTTOM_Z_THRESH){
				return TOP_OF_SCREEN_GHOST_NOT_CLEARED;
			}
		}
	}

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by other finger\n", finger);
	return TOP_OF_SCREEN_GHOST_CLEARED;
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_topofscreen_ghost_reject_clear_check_timer_start(struct shtps_rmi_spi *ts)
{
	int i;
	int fingerMax   = shtps_get_fingermax(ts);
	u8 isNeedWork   = 0;
	unsigned long timeoutTime = 0;
	unsigned long tempTime;

	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject.is_ghost[i] != 0){
			isNeedWork = 1;
			tempTime   = ts->topofscreen_ghost_reject.timeout[i] - jiffies;
			
			SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] timeout = %lu\n", i, tempTime);
			if(timeoutTime == 0 || timeoutTime < tempTime){
				timeoutTime = tempTime;
			}
		}
	}
	
	if(isNeedWork){
		if(timeoutTime == 0){
			timeoutTime = 1;
		}
		SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("start timer (%d ms)\n", jiffies_to_msecs(timeoutTime));
		cancel_delayed_work(&ts->topofscreen_ghost_reject.ghost_clear_check_delayed_work);
		schedule_delayed_work(&ts->topofscreen_ghost_reject.ghost_clear_check_delayed_work, timeoutTime);
	}
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_topofscreen_ghost_reject_clear_check_timer_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("cancel ghost check work\n");
	cancel_delayed_work(&ts->topofscreen_ghost_reject.ghost_clear_check_delayed_work);
}

/* -------------------------------------------------------------------------- */
static void shtps_filter_topofscreen_ghost_reject_clear_check_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_filter_topofscreen_ghost_reject_info *tgri = container_of(dw, struct shtps_filter_topofscreen_ghost_reject_info, ghost_clear_check_delayed_work);
	struct shtps_rmi_spi *ts = container_of(tgri, struct shtps_rmi_spi, topofscreen_ghost_reject);

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("start ghost check work\n");

	mutex_lock(&shtps_ctrl_lock);
	if(ts->state_mgr.state == SHTPS_STATE_ACTIVE || ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
		int i;
		int fingerMax = shtps_get_fingermax(ts);
		u8 isStateChange = 0;
		
		for(i = 0;i < fingerMax;i++){
			if(ts->topofscreen_ghost_reject.is_ghost[i] != 0){
				if(shtps_filter_topofscreen_ghost_reject_ghost_clear_check(ts, i) == TOP_OF_SCREEN_GHOST_CLEARED){
					isStateChange = 1;
					ts->topofscreen_ghost_reject.is_ghost[i] = 0;
				}
			}
		}
		
		if(isStateChange){
			shtps_read_touchevent(ts, ts->state_mgr.state);
		}
		
		shtps_filter_topofscreen_ghost_reject_clear_check_timer_start(ts);
	}
	mutex_unlock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("end ghost check work\n");
	
	return;
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(!SHTPS_TOP_OF_SCREEN_GHOST_REJECT_ENABLE){
		return;
	}
	
	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject.is_ghost[i] == 0){
			if(shtps_filter_topofscreen_ghost_reject_ghost_check(ts, i) == TOP_OF_SCREEN_GHOST_DETECTED){
				ts->topofscreen_ghost_reject.is_ghost[i] = 1;
				ts->topofscreen_ghost_reject.timeout[i]  = jiffies + msecs_to_jiffies(SHTPS_TOP_OF_SCREEN_GHOST_REJECT_CLEAR_TIME);
			}
		}else{
			if(shtps_filter_topofscreen_ghost_reject_ghost_clear_check(ts, i) == TOP_OF_SCREEN_GHOST_CLEARED){
				ts->topofscreen_ghost_reject.is_ghost[i] = 0;
			}
		}
	}

	for(i = 0;i < fingerMax;i++){
		if(ts->topofscreen_ghost_reject.is_ghost[i] != 0){
			info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			SHTPS_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] reject ghost\n", i);
		}
	}
	
	shtps_filter_topofscreen_ghost_reject_clear_check_timer_start(ts);
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	shtps_filter_topofscreen_ghost_reject_clear_check_timer_cancel(ts);
	memset(ts->topofscreen_ghost_reject.is_ghost, 0, sizeof(ts->topofscreen_ghost_reject.is_ghost));
}

/* -------------------------------------------------------------------------- */
void shtps_filter_topofscreen_ghost_reject_init(struct shtps_rmi_spi *ts)
{
	memset(ts->topofscreen_ghost_reject.is_ghost, 0, sizeof(ts->topofscreen_ghost_reject.is_ghost));
	INIT_DELAYED_WORK(&ts->topofscreen_ghost_reject.ghost_clear_check_delayed_work, 
		shtps_filter_topofscreen_ghost_reject_clear_check_delayed_work_function);
}
#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

static void shtps_calc_notify(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info, u8 *event)
{
	int		i;
	u8		fingerMax = shtps_get_fingermax(ts);
	u8		numOfFingers = 0;
	u8		base;
	int 	diff_x;
	int 	diff_y;
	int 	diff_cx;
	int 	diff_cy;
	int		dragStep1stX;
	int		dragStep1stY;
	int		dragStepCurX;
	int		dragStepCurY;

	SHTPS_LOG_FUNC_CALL();

	if(fingerMax > 8){
		base = 3;
	}else if (fingerMax > 4){
		base = 2;
	}else{
		base = 1;
	}

	shtps_set_eventtype(event, 0xff);
	shtps_set_touch_info(ts, buf, info);

	#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
		shtps_scan_freq_dynamic_change_check(ts, info);
	#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_check(ts, info);
	#endif /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	if(ts->state_mgr.state == SHTPS_STATE_FACETOUCH){
		#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_grip_fail_touch_check(ts, info);
		#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

		#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_edge_fail_touch_check(ts, info);
		#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

		#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_grip_fail_flick_check(ts, info);
		#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

		#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			shtps_check_facetouch(ts, info);
		#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */

		#if defined(SHTPS_CLING_REJECTION_ENABLE)
			shtps_cling_reject_check(ts, info);
		#endif /* SHTPS_CLING_REJECTION_ENABLE */

		#if defined(SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE)
			shtps_z_zero_touch_event_reject_check(ts, info);
		#endif /* SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE */
	}
	else{
		#if defined(SHTPS_CLING_REJECTION_ENABLE)
			shtps_cling_reject_check(ts, info);
		#endif /* SHTPS_CLING_REJECTION_ENABLE */

		#if defined(SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE)
			shtps_z_zero_touch_event_reject_check(ts, info);
		#endif /* SHTPS_Z_ZERO_TOUCH_EVENT_REJECTION_ENABLE */

		#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_grip_fail_touch_check(ts, info);
		#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

		#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_edge_fail_touch_check(ts, info);
		#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

		#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
			shtps_grip_fail_flick_check(ts, info);
		#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */
	}

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_absorption_check(ts, info);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		shtps_pinch_fail_response_check(ts, info);
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		shtps_lgm_split_touch_combining_check(ts, info);
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE )
		shtps_pinchout_outset_distort_check(ts, info);
	#endif /* SHTPS_PINCHOUT_OUTSET_DISTORT_REJECTION_ENABLE */
	
	#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
		shtps_lgm_fail_touch_reject_check(ts, info);
	#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */
	
	#if defined(SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE)
		shtps_edge_touch_shake_reject_check(ts, info);
	#endif /* SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE */

	#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
		shtps_quick_flick_fail_resolv_check(ts, info);
	#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	info->finger_num = numOfFingers;

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
			}
		}else{
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
		}
	}

	#if defined(SHTPS_DRAG_SMOOTH_ENABLE)
		shtps_pos_compensation(ts, info, SHTPS_POSTYPE_X);
		shtps_pos_compensation(ts, info, SHTPS_POSTYPE_Y);
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

	dragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	dragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	for(i = 0;i < fingerMax;i++){
		_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
							info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[i].x,
							info->fingers[i].y,
							info->fingers[i].wx,
							info->fingers[i].wy,
							info->fingers[i].z);

		dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
		dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);

		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){

			diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			diff_cx= shtps_get_diff(info->fingers[i].x, ts->center_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_cy= shtps_get_diff(info->fingers[i].y, ts->center_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
											ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
					}
				}

				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						if(ts->touch_state.dragStep[i][0] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_X, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}

				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_Y, i) == 0 ||
								ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else{
				ts->center_info.fingers[i].x = info->fingers[i].x;
				ts->center_info.fingers[i].y = info->fingers[i].y;
			}
		}else{
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_X, i);
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_Y, i);
		}

		if(info->fingers[i].state != ts->report_info.fingers[i].state){
			shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
		}
	}

	if(numOfFingers > 0){
		ts->poll_info.stop_count = 0;
		if(ts->touch_state.numOfFingers == 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TD);
		}
		if(numOfFingers >= 2 && ts->touch_state.numOfFingers < 2){
			shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_MTD, info->gs2);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCH, info->gs2);
	}else{
		if(ts->touch_state.numOfFingers != 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TU);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCHUP, info->gs2);
	}

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_set_palmthresh(ts, numOfFingers);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */	
}


inline static void shtps_report_touch_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x;
	int lcd_y;

	if( (x == SHTPS_TOUCH_CANCEL_COORDINATES_X) && (y == SHTPS_TOUCH_CANCEL_COORDINATES_Y) ){
		lcd_x = x;
		lcd_y = y;
	}else{
		lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
		lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	shtps_offset_pos(ts, &lcd_x, &lcd_y);

	#if defined( SHTPS_MAGNIFICATION_CORRECTION_SIZE_W_ENABLE )
		if(SHTPS_MAGNIFICATION_CORRECT_SIZE_W_ENABLE){
			w = (w * SHTPS_MAGNIFICATION_CORRECT_SIZE_SCALE) / 100;
			if(w <= 0){
				w = 1;
			}
		}
	#endif /* SHTPS_MAGNIFICATION_CORRECTION_SIZE_W_ENABLE */

	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    z);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps][finger]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, lcd_x, x, lcd_y, y, w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_FINGER, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

inline static void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps][finger]Notify event[%d] touch=0(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|0|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy, z);

	#if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE )
		if(shtps_touch_eventlog_is_recording() != 0){
			shtps_touch_eventlog_rec_event_info(SHTPS_TOUCH_STATE_NO_TOUCH, finger, x, y, w, wx, wy, z);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EVENTLOG_ENABLE ) */
}

static void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event)
{
	int	i;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		if(shtps_absorption_hold_status(ts)){
			for(i = 0;i < fingerMax;i++){
				SHTPS_LOG_EVENT(
					printk(KERN_DEBUG "[shtps][%s]Drop event[%d] touch=%d(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
									(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN)?    "pen" :
									(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER)?  "hover" : "finger",
									(info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH)? 0 : 100,
									i, info->fingers[i].z, info->fingers[i].x, info->fingers[i].x, 
									info->fingers[i].y, info->fingers[i].y, shtps_get_fingerwidth(ts, i, info), 
									info->fingers[i].wx, info->fingers[i].wy);
				);
			}
			return;
		}
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			shtps_report_touch_on(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);
		}else if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			shtps_report_touch_off(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);
		}
	}
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);
	input_sync(ts->input);
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	ts->touch_state.numOfFingers = info->finger_num;

	ts->diag.event = 1;
	memcpy(&ts->report_info, info, sizeof(ts->report_info));

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event = event;
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	wake_up_interruptible(&ts->diag.wait);
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_event_update(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int	i;
	u8	numOfFingers = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	ts->touch_state.numOfFingers = numOfFingers;
}
#endif /* #if defined(SHTPS_LPWG_MODE_ENABLE) */

static void shtps_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int	i;
	int isEvent = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_absorption_hold_cancel(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
		ts->lgm_split_touch_combining.finger_swap = 0;
		ts->lgm_split_touch_combining.finger_adjust = 0;
	#endif  /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_forcetouchup(ts);
	#endif  /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_TOUCHCANCEL_BEFORE_FORCE_TOUCHUP_ENABLE )
		for(i = 0;i < fingerMax;i++){
			if(ts->report_info.fingers[i].state != 0x00){
				isEvent = 1;
				shtps_report_touch_on(ts, i,
									  SHTPS_TOUCH_CANCEL_COORDINATES_X,
									  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
									  shtps_get_fingerwidth(ts, i, &ts->report_info),
									  ts->report_info.fingers[i].wx,
									  ts->report_info.fingers[i].wy,
									  ts->report_info.fingers[i].z);
			}
		}
		if(isEvent){
			input_sync(ts->input);
			isEvent = 0;
		}
	#endif /* SHTPS_TOUCHCANCEL_BEFORE_FORCE_TOUCHUP_ENABLE */

	for(i = 0;i < fingerMax;i++){
		if(ts->report_info.fingers[i].state != 0x00){
			isEvent = 1;
			shtps_report_touch_off(ts, i,
								  ts->report_info.fingers[i].x,
								  ts->report_info.fingers[i].y,
								  0,
								  0,
								  0,
								  0);
		}
	}
	if(isEvent){
		input_sync(ts->input);
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )

#if defined(SHTPS_LOG_DEBUG_ENABLE) || defined(SHTPS_LOG_EVENT_ENABLE)
static char* shtps_get_key_name(int key){
	switch(key){
		case SHTPS_PHYSICAL_KEY_UP:
			return "KEY_VOLUMEUP";
		case SHTPS_PHYSICAL_KEY_DOWN:
			return "KEY_VOLUMEDOWN";
	}
	return "KEY_UNKNOWN";
}
#endif /* SHTPS_LOG_DEBUG_ENABLE || SHTPS_LOG_EVENT_ENABLE */

static int shtps_key_event_report_each_key(struct shtps_rmi_spi *ts, u8 state, int key)
{
	int isEvent = 0;
	int isReportedKeyDownState = (ts->key_state & (1 << key) ? 1 : 0);
	int isReserveKeyDownState = (ts->key_down_reserved & (1 << key) ? 1 : 0);
	int isKeyDownState = (state & (1 << key) ? 1 : 0);
	
	if((ts->key_down_ignored & (1 << key)) != 0){
		if(isKeyDownState == 0){
			ts->key_down_ignored &= ~(1 << key);
		}
	
		SHTPS_LOG_DBG_PRINT("[TouchKey] %s ignored", shtps_get_key_name(key));

		return isEvent;
	}

	if((isReportedKeyDownState == 0) && (isKeyDownState == 1)){
		if(ts->key_proximity_check_state == 0){
			shtps_touchkey_delayed_work_start(ts);
		}
		ts->key_down_reserved |= (1 << key);
	}
	else if((isReportedKeyDownState == 1) && (isKeyDownState == 0)){
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 0);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			printk(KERN_DEBUG "[shtps][key]Notify event %s:UP\n", shtps_get_key_name(key));
		);
		isEvent = 1;

		ts->key_down_reserved &= ~(1 << key);
		ts->key_state &= ~(1 << key);
	}
	else if((isReserveKeyDownState == 1) && (isKeyDownState == 0)){
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 1);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			printk(KERN_DEBUG "[shtps][key]Notify event %s:DOWN\n", shtps_get_key_name(key));
		);
		input_event(ts->input_key, EV_MSC, MSC_SCAN, key);
		input_report_key(ts->input_key, ts->keycodes[key], 0);
		input_sync(ts->input_key);
		SHTPS_LOG_EVENT(
			printk(KERN_DEBUG "[shtps][key]Notify event %s:UP\n", shtps_get_key_name(key));
		);
		isEvent = 1;

		ts->key_down_reserved &= ~(1 << key);
		ts->key_state &= ~(1 << key);
	}

	return isEvent;
}
#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

static void shtps_key_event_report(struct shtps_rmi_spi *ts, u8 state)
{
	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		int isEvent = 0;

		SHTPS_LOG_FUNC_CALL();

		isEvent |= shtps_key_event_report_each_key(ts, state, SHTPS_PHYSICAL_KEY_DOWN);
		isEvent |= shtps_key_event_report_each_key(ts, state, SHTPS_PHYSICAL_KEY_UP);

		if(isEvent){
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	#else /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
		int isEvent = 0;

		SHTPS_LOG_FUNC_CALL();

		if(ts->key_state != state){
			if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_DOWN))) != 0 ){
				input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
				input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], ((state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01));
				isEvent = 1;
			}
			if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_UP))) != 0 ){
				input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
				input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], ((state >> SHTPS_PHYSICAL_KEY_UP) & 0x01));
				isEvent = 1;
			}
		}

		if(isEvent){
			input_sync(ts->input_key);
			ts->key_state = state;
			ts->diag.event_touchkey = 1;
			wake_up_interruptible(&ts->diag.wait);
		}
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
}

static void shtps_key_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();

	if(ts->key_state != 0){
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], 0);
			isEvent = 1;
		}
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], 0);
			isEvent = 1;
		}
	}

	if(isEvent){
		input_sync(ts->input_key);
	}

	ts->key_state = 0;

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		ts->key_down_reserved = 0;
		ts->key_down_ignored = 0;
		shtps_touchkey_delayed_work_cancel(ts);
		shtps_touchkey_inproxymity_delayed_work_cancel(ts);
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_notify_wakeup_event(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("wakeup touch blocked until rezero\n");

	ts->lpwg.block_touchevent = 1;
	ts->lpwg.is_notified = 1;
	ts->lpwg.notify_time = jiffies;
	ts->lpwg.notify_enable = 0;

	input_report_key(ts->input_key, KEY_SWEEPON, 1);
	input_sync(ts->input_key);

	input_report_key(ts->input_key, KEY_SWEEPON, 0);
	input_sync(ts->input_key);
	
	shtps_lpwg_notify_interval_start(ts);
}

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
static void shtps_notify_cancel_wakeup_event(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	input_report_key(ts->input_key, KEY_SWEEPON, 1);
	input_sync(ts->input_key);

	input_report_key(ts->input_key, KEY_SWEEPON, 0);
	input_sync(ts->input_key);
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
static void shtps_event_touch_cancel(struct shtps_rmi_spi *ts, u8 id)
{
	if(ts->report_info.fingers[id].state == SHTPS_TOUCH_STATE_FINGER){
		shtps_report_touch_on(ts, id,
							  SHTPS_TOUCH_CANCEL_COORDINATES_X,
							  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
							  shtps_get_fingerwidth(ts, id, &ts->report_info),
							  ts->report_info.fingers[id].wx,
							  ts->report_info.fingers[id].wy,
							  ts->report_info.fingers[id].z);
		input_sync(ts->input);

		ts->report_info.fingers[id].x = SHTPS_TOUCH_CANCEL_COORDINATES_X;
		ts->report_info.fingers[id].y = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
	}

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		if(ts->report_info.fingers[id].state == SHTPS_TOUCH_STATE_PEN){
			shtps_report_touch_pen_on(ts, id,
								  SHTPS_TOUCH_CANCEL_COORDINATES_X,
								  SHTPS_TOUCH_CANCEL_COORDINATES_Y,
								  shtps_get_fingerwidth(ts, id, &ts->report_info),
								  ts->report_info.fingers[id].wx,
								  ts->report_info.fingers[id].wy,
								  ts->report_info.fingers[id].z);
			input_sync(ts->input);

			ts->report_info.fingers[id].x = SHTPS_TOUCH_CANCEL_COORDINATES_X;
			ts->report_info.fingers[id].y = SHTPS_TOUCH_CANCEL_COORDINATES_Y;
		}
	#endif /* SHTPS_PEN_DETECT_ENABLE */
}
#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static int shtps_tm_irqcheck(struct shtps_rmi_spi *ts)
{
	u8 buf[2];

	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);

	if((buf[1] & SHTPS_IRQ_ANALOG) != 0x00){
		return 1;
	}
	return 0;
}

static int shtps_tm_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->diag.tm_wait_ack,
			ts->diag.tm_ack == 1 || ts->diag.tm_stop == 1,
			msecs_to_jiffies(SHTPS_FWTESTMODE_ACK_TMO));
	_log_msg_recv( LOGMSG_ID__FW_TESTMODE_ATTN, "");

#if defined( SHTPS_LOG_SEQ_ENABLE )
	if(rc == 0){
		_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERR_PRINT("shtps_tm_wait_attn() :ATTN_TIMEOUT\n");
	}
#endif /* #if defined( SHTPS_LOG_SEQ_ENABLE ) */

	if(ts->diag.tm_ack == 0 || ts->diag.tm_stop == 1){
		SHTPS_LOG_ERR_PRINT("shtps_tm_wait_attn() tm_ack:%d, tm_stop:%d\n", ts->diag.tm_ack, ts->diag.tm_stop);
		return -1;
	}

	ts->diag.tm_ack = 0;
	return 0;
}

static void shtps_tm_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_ack = 1;
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static void shtps_tm_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_stop = 1;
	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_CANCEL, "");
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static int shtps_get_tm_rxsize(struct shtps_rmi_spi *ts)
{
	int receive_num = 0;

	if(ts->map.fn05.enable != 0){
		receive_num = F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data);
	}else{
		receive_num = F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data);
	}

	return (receive_num > SHTPS_TM_TXNUM_MAX)? SHTPS_TM_TXNUM_MAX : receive_num;
}

static int shtps_get_tm_txsize(struct shtps_rmi_spi *ts)
{
	int trans_num = 0;

	if(ts->map.fn05.enable != 0){
		trans_num   = F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data);
	}else{
		trans_num   = F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data);
	}

	return (trans_num > SHTPS_TM_RXNUM_MAX)? SHTPS_TM_RXNUM_MAX : trans_num;
}

static int shtps_start_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_START, "%d", ts->diag.tm_mode);
	ts->diag.tm_stop = 0;
	ts->diag.tm_ack = 0;
	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  0x00);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0x42);
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0xe1);

		shtps_rmi_write(ts, 0xff, 0x80);
		shtps_rmi_write(ts, 0x00, 0x01);

	}else{
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_set_dev_state(ts, SHTPS_DEV_STATE_ACTIVE);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  SHTPS_IRQ_ANALOG);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
		shtps_rmi_write(ts, 0xff, 0x01);
	}
	return 0;
}

static void shtps_baseline_offset_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x08, 0x00);
	shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x54, 0x00);
	shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x51, 0x01);

	shtps_command_force_update(ts);
	shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
											SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);

	shtps_command_force_cal(ts);
	shtps_f54_command_completion_wait(ts, SHTPS_F54_COMMAND_WAIT_POLL_COUNT,
											SHTPS_F54_COMMAND_WAIT_POLL_INTERVAL);
}

static void shtps_stop_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_STOP, "%d", ts->diag.tm_mode);
	shtps_tm_cancel(ts);
	shtps_rmi_write(ts, 0xff, 0x00);
	shtps_init_param(ts);
}

static void shtps_read_tmdata(struct shtps_rmi_spi *ts, u8 mode)
{
	int i;
	int j;
	int receive_num = shtps_get_tm_rxsize(ts);
	int trans_num   = shtps_get_tm_txsize(ts);
	u8  tmp[SHTPS_TM_TXNUM_MAX * 2] = {0};
	u16 index = 0;

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_GETDATA, "%d", ts->diag.tm_mode);

	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		mutex_lock(&shtps_ctrl_lock);
		if(mode == SHTPS_TMMODE_FRAMELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
		}else if(mode == SHTPS_TMMODE_BASELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num * 2);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
		}
		mutex_unlock(&shtps_ctrl_lock);
	}else if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V02){
		if(mode == SHTPS_TMMODE_FRAMELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x80);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x40);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num * 2);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}else if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V03){
		if(mode == SHTPS_TMMODE_FRAMELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn54.dataBase, 0x02);
			shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x01);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 1, (index & 0xFF));
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 2, ((index >> 0x08) & 0xFF));

				shtps_rmi_read(ts, ts->map.fn54.dataBase + 3, tmp, receive_num * 2);
				index += (receive_num * 2);

				#if defined( SHTPS_TMDATA_NOTCHANGE_TXRX )
					memcpy(&ts->diag.tm_data[(i * receive_num * 2)], &tmp[0], (receive_num * 2));
				#else /* SHTPS_TMDATA_NOTCHANGE_TXRX */
					for(j = 0;j < receive_num;j++){
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[(j * 2) + 1];
					}
				#endif /* SHTPS_TMDATA_NOTCHANGE_TXRX */
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn54.dataBase, 0x03);
			shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x01);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 1, (index & 0xFF));
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 2, ((index >> 0x08) & 0xFF));

				shtps_rmi_read(ts, ts->map.fn54.dataBase + 3, tmp, receive_num * 2);
				index += (receive_num * 2);

				#if defined( SHTPS_TMDATA_NOTCHANGE_TXRX )
					memcpy(&ts->diag.tm_data[(i * receive_num * 2)], &tmp[0], (receive_num * 2));
				#else /* SHTPS_TMDATA_NOTCHANGE_TXRX */
					for(j = 0;j < receive_num;j++){
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[(j * 2) + 1];
					}
				#endif /* SHTPS_TMDATA_NOTCHANGE_TXRX */
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE_RAW){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn54.dataBase, 0x14);
			shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x01);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 1, (index & 0xFF));
				shtps_rmi_write(ts, ts->map.fn54.dataBase + 2, ((index >> 0x08) & 0xFF));

				shtps_rmi_read(ts, ts->map.fn54.dataBase + 3, tmp, receive_num * 2);
				index += (receive_num * 2);

				#if defined( SHTPS_TMDATA_NOTCHANGE_TXRX )
					memcpy(&ts->diag.tm_data[(i * receive_num * 2)], &tmp[0], (receive_num * 2));
				#else /* SHTPS_TMDATA_NOTCHANGE_TXRX */
					for(j = 0;j < receive_num;j++){
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
						ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[(j * 2) + 1];
					}
				#endif /* SHTPS_TMDATA_NOTCHANGE_TXRX */
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}else{
		memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	}

	return;

tm_read_cancel:
	memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	return;
}

/* -----------------------------------------------------------------------------------
 */
#if !defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_DISABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
		disable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}

static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_ENABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
		enable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}
#endif /* !SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

static void shtps_irq_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_DISABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_DISABLE){
		disable_irq_nosync(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_DISABLE;
	}
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
			disable_irq_wake(ts->irq_mgr.irq);
			ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

			#if defined( SHTPS_MODULE_PARAM_ENABLE )
				shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
			#endif /* SHTPS_MODULE_PARAM_ENABLE */
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
}

static void shtps_irq_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_ENABLE, "%d", ts->irq_mgr.state);
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
			enable_irq_wake(ts->irq_mgr.irq);
			ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

			#if defined( SHTPS_MODULE_PARAM_ENABLE )
				shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
			#endif /* SHTPS_MODULE_PARAM_ENABLE */
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_ENABLE){
		enable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	}
}

static int shtps_irq_resuest(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__IRQ_REQUEST, "%d", ts->irq_mgr.irq);

	#if defined(SHTPS_IRQ_LEVEL_ENABLE)
	rc = request_threaded_irq(ts->irq_mgr.irq,
							  shtps_irq_handler,
							  shtps_irq,
							  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
	#else
		rc = request_threaded_irq(ts->irq_mgr.irq,
							  shtps_irq_handler,
							  shtps_irq,
							  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
	#endif /* SHTPS_IRQ_LEVEL_ENABLE */

	if(rc){
		_log_msg_sync( LOGMSG_ID__IRQ_REQUEST_NACK, "");
		SHTPS_LOG_ERR_PRINT("request_threaded_irq error:%d\n",rc);
		return -1;
	}

	ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	ts->irq_mgr.wake  = SHTPS_IRQ_WAKE_DISABLE;
	shtps_irq_disable(ts);
	return 0;
}

static void shtps_irqtimer_start(struct shtps_rmi_spi *ts, long time_ms)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_START, "%lu", time_ms);
	schedule_delayed_work(&ts->tmo_check, msecs_to_jiffies(time_ms));
}

static void shtps_irqtimer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_CANCEL, "");
	cancel_delayed_work(&ts->tmo_check);
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_read_touchevent_insleep(struct shtps_rmi_spi *ts, int state)
{
	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		struct shtps_touch_info info;
		u8 buf[2 + 3 + 5];
	#else
		u8 buf[2];
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */
	u8 val = SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE;

	memset(buf, 0, sizeof(buf));

	shtps_device_access_setup(ts);

	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
	{
		u8 fingerMax = shtps_get_fingermax(ts);
		u8 base;
		u8 readsize = 0;
		u8* fingerInfo;
		unsigned short diff_x;
		unsigned short diff_y;

		#if defined( SHTPS_ES0_HOST_LPWG_ENABLE )
		if(SHTPS_HOST_LPWG_ENABLE != 0 || shtps_lpwg_get_hwrev() == SHTPS_GET_HW_VERSION_RET_ES_0)
		#else
		if(SHTPS_HOST_LPWG_ENABLE != 0)
		#endif /* SHTPS_ES0_HOST_LPWG_ENABLE */
		{
			if(fingerMax > 8){
				base = 3;
				readsize = 3 + 5;
			}else if (fingerMax > 4){
				base = 2;
				readsize = 2 + 5;
			}else{
				base = 1;
				readsize = 1 + 5;
			}

			shtps_rmi_read(ts, ts->map.fn11.dataBase, &buf[2], readsize);

			fingerInfo = &buf[2 + base];
			info.fingers[0].state	= (buf[2] & 0x03);
			info.fingers[0].x		= (F11_DATA_XPOS(fingerInfo) * SHTPS_POS_SCALE_X(ts) / 10000);
			info.fingers[0].y		= (F11_DATA_YPOS(fingerInfo) * SHTPS_POS_SCALE_X(ts) / 10000);

			SHTPS_LOG_DBG_PRINT("[LPWG] TouchInfo <state: %d><x: %d><y: %d>\n", info.fingers[0].state, info.fingers[0].x, info.fingers[0].y);

			if( (info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) && (ts->lpwg.pre_info.fingers[0].state == SHTPS_TOUCH_STATE_NO_TOUCH) ){
				ts->lpwg.pre_info.fingers[0].state = info.fingers[0].state;
				ts->lpwg.pre_info.fingers[0].x     = info.fingers[0].x;
				ts->lpwg.pre_info.fingers[0].y     = info.fingers[0].y;
				ts->lpwg.swipe_check_time = jiffies + msecs_to_jiffies(SHTPS_LPWG_SWIPE_CHECK_TIME_MS);

				SHTPS_LOG_DBG_PRINT("[LPWG] swipe check zero pos <x: %d><y: %d>\n", ts->lpwg.pre_info.fingers[0].x, ts->lpwg.pre_info.fingers[0].y);
			}
			else if( (info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) && (ts->lpwg.pre_info.fingers[0].state == SHTPS_TOUCH_STATE_FINGER) ){
				if( time_after(jiffies, ts->lpwg.swipe_check_time) == 0 ){
					diff_x = (info.fingers[0].x >= ts->lpwg.pre_info.fingers[0].x) ?
								(info.fingers[0].x - ts->lpwg.pre_info.fingers[0].x) :
								(ts->lpwg.pre_info.fingers[0].x - info.fingers[0].x);

					diff_y = (info.fingers[0].y >= ts->lpwg.pre_info.fingers[0].y) ?
								(info.fingers[0].y - ts->lpwg.pre_info.fingers[0].y) :
								(ts->lpwg.pre_info.fingers[0].y - info.fingers[0].y);

					SHTPS_LOG_DBG_PRINT("[LPWG] swipe distance (%d, %d)\n", diff_x, diff_y);

					if( (diff_x > SHTPS_LPWG_SWIPE_DIST_THRESHOLD) || (diff_y > SHTPS_LPWG_SWIPE_DIST_THRESHOLD) ){
						val = SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE;
					}
				}
				else{
					ts->lpwg.pre_info.fingers[0].x = info.fingers[0].x;
					ts->lpwg.pre_info.fingers[0].y = info.fingers[0].y;
					ts->lpwg.swipe_check_time = jiffies + msecs_to_jiffies(SHTPS_LPWG_SWIPE_CHECK_TIME_MS);

					SHTPS_LOG_DBG_PRINT("[LPWG] swipe check zero pos update<x: %d><y: %d>\n", ts->lpwg.pre_info.fingers[0].x, ts->lpwg.pre_info.fingers[0].y);
				}
			}
		}
		else{
			if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
				shtps_rmi_read(ts, ts->map.fn11.dataBase + 0x21, &val, 1);
			}else{
				shtps_rmi_read(ts, ts->map.fn11.dataBase + 0x1C, &val, 1);
			}
			SHTPS_LOG_DBG_PRINT("LPWG detect <%s><val=0x%02X>\n",
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE) ? "None" :
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) ? "Swipe" :
									(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP) ? "Double Tap" : "Unkown",
									val);
		}
	}
	#else
		if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
			shtps_rmi_read(ts, ts->map.fn11.dataBase + 0x21, &val, 1);
		}else{
			shtps_rmi_read(ts, ts->map.fn11.dataBase + 0x1C, &val, 1);
		}
		SHTPS_LOG_DBG_PRINT("LPWG detect <%s><val=0x%02X>\n",
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_NONE) ? "None" :
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) ? "Swipe" :
								(val == SHTPS_LPWG_DETECT_GESTURE_TYPE_DOUBLE_TAP) ? "Double Tap" : "Unkown",
								val);
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */

	if((val & SHTPS_LPWG_DETECT_GESTURE_TYPE_SWIPE) != 0){
		if(ts->lpwg.notify_enable != 0)
		{
			#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
			{
				if(SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE != 0){
					if (shtps_proximity_state_check(ts) == 0) {
					#if defined(SHTPS_ASYNC_OPEN_ENABLE)
						shtps_notify_wakeup_event(ts);
						SHTPS_LOG_DBG_PRINT("[LPWG] proximity check async req\n");
						shtps_lpwg_proximity_check_start(ts);
					#else
						ts->lpwg_proximity_get_data = shtps_proximity_check(ts);
						if(ts->lpwg_proximity_get_data != SHTPS_PROXIMITY_NEAR){
							shtps_notify_wakeup_event(ts);
						}else{
							SHTPS_LOG_DBG_PRINT("[LPWG] proximity near\n");
						}
					#endif /* SHTPS_ASYNC_OPEN_ENABL */
					}
				}
				else{
					shtps_notify_wakeup_event(ts);
				}
			}
			#else
				shtps_notify_wakeup_event(ts);
			#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
		}
		else{
			SHTPS_LOG_DBG_PRINT("[LPWG] notify event blocked\n");
		}
	}else{
		SHTPS_LOG_DBG_PRINT("LPWG: Not Support Gesture Type Detect\n");\
	}

	shtps_device_access_teardown(ts);

	#if defined( SHTPS_HOST_LPWG_MODE_ENABLE )
		#if defined( SHTPS_ES0_HOST_LPWG_ENABLE )
		if(SHTPS_HOST_LPWG_ENABLE != 0 || shtps_lpwg_get_hwrev() == SHTPS_GET_HW_VERSION_RET_ES_0)
		#else
		if(SHTPS_HOST_LPWG_ENABLE != 0)
		#endif /* SHTPS_ES0_HOST_LPWG_ENABLE */
		{
			if(info.fingers[0].state != SHTPS_TOUCH_STATE_FINGER){
				ts->lpwg.pre_info.fingers[0].state = SHTPS_TOUCH_STATE_NO_TOUCH;
				ts->lpwg.pre_info.fingers[0].x     = 0xFFFF;
				ts->lpwg.pre_info.fingers[0].y     = 0xFFFF;
				ts->lpwg.swipe_check_time = 0;
			}
		}
	#endif /* SHTPS_HOST_LPWG_MODE_ENABLE */
}
#endif /*  SHTPS_LPWG_MODE_ENABLE */

static void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 5 + (SHTPS_FINGER_MAX / 4) + 1 + 5 + 5];
	u8 event;
	u8 finger;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 offset, readsize;
	u8 *pFingerInfoBuf;
	struct shtps_touch_info info;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__READ_EVENT, "%d", state);

	shtps_device_access_setup(ts);

	if(fingerMax > 8){
		offset = 5;
		readsize = 3 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}else if (fingerMax > 4){
		offset = 4;
		readsize = 2 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}else{
		offset = 3;
		readsize = 1 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}

	memset(buf, 0, sizeof(buf));

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	if(shtps_touch_emu_is_running() != 0){
		shtps_touch_emu_set_finger_info(buf, sizeof(buf));
		ts->finger_state[0] = buf[2];
		if (fingerMax > 4){
			ts->finger_state[1] = buf[3];
		}
		if(fingerMax > 8){
			ts->finger_state[2] = buf[4];
		}

		pFingerInfoBuf = &buf[offset];
	}
	else {
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
	#if !defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
		shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 4);
	#else
		#if (SHTPS_SY3X00_FINGER_SPIREAD_CNT == 5)
			shtps_rmi_read_block(ts, ts->map.fn01.dataBase, buf, 2, ts->purposebuf_p);
			shtps_rmi_read_block(ts, ts->map.fn11.dataBase, &buf[2], readsize, ts->purposebuf_p);
		#else
			shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
			shtps_rmi_read(ts, ts->map.fn11.dataBase, &buf[2], readsize);
		#endif

		if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
			shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + (fingerMax * 5) + 1,
											&buf[offset + (fingerMax * 5) + 1], SHTPS_SY3X00_FINGER_SPIREAD_CNT);
		}
	#endif

	ts->finger_state[0] = buf[2];
	if (fingerMax > 4){
		ts->finger_state[1] = buf[3];
	}
	if(fingerMax > 8){
		ts->finger_state[2] = buf[4];
	}

	pFingerInfoBuf = &buf[offset];
	for(finger = 0;finger < fingerMax;finger++){
		if(((buf[2 + finger / 4] >> (2 * (finger % 4))) & 0x03) != SHTPS_TOUCH_STATE_NO_TOUCH ||
		   (ts->report_info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH))
		{
			#if	defined( SHTPS_SPI_BLOCKACCESS_ENABLE )
				#if	(SHTPS_SY3X00_FINGER_SPIREAD_CNT != 5)
					if(finger >= SHTPS_SY3X00_FINGER_SPIREAD_CNT){
						shtps_rmi_read(ts,
							   ts->map.fn01.dataBase + offset + (finger * 5),
							   &pFingerInfoBuf[finger * 5], 5);

						if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
							shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + (fingerMax * 5) + finger + 1,
														   &pFingerInfoBuf[(fingerMax * 5) + finger + 1], 1);
						}
					}
				#endif	/* (SHTPS_SY3X00_FINGER_SPIREAD_CNT != 5) */
			#else
				shtps_rmi_read(ts,
						   ts->map.fn01.dataBase + offset + (finger * 5),
						   &pFingerInfoBuf[finger * 5], 5);

				if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
					shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + (fingerMax * 5) + finger + 1,
												   &pFingerInfoBuf[(fingerMax * 5) + finger + 1], 1);
				}
			#endif
		}
	}

#if defined( SHTPS_SY_REGMAP_BASE3 )
	shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
								   &pFingerInfoBuf[fingerMax * 5], 1);
#else
	if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
			shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
						   &pFingerInfoBuf[fingerMax * 5], 1);
		}else{
			shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
						   &pFingerInfoBuf[fingerMax * 5], 2);
		}
	}
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
#if defined ( SHTPS_TOUCH_EMURATOR_ENABLE )
	if(shtps_touch_emu_is_recording() != 0){
		shtps_touch_emu_rec_finger_info(&buf[2]);
	}
	}
#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		if(ts->wakeup_touch_event_inhibit_state != 0){
			memset(&buf[2], 0, sizeof(buf) - 2);
			SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Cleared by not rezero exec yet\n");
		}
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	switch(state){
	case SHTPS_STATE_SLEEP:
	case SHTPS_STATE_IDLE:
		shtps_device_access_teardown(ts);
		break;

	case SHTPS_STATE_SLEEP_FACETOUCH:
		break;

	case SHTPS_STATE_FACETOUCH:
	case SHTPS_STATE_ACTIVE:
	default:
		#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
			if((buf[1] & SHTPS_IRQ_BUTTON) != 0){
				u8 present_key_state = 0;

				shtps_rmi_read(ts, ts->map.fn1A.dataBase, &present_key_state, 1);
				shtps_key_event_report(ts, present_key_state);
			}
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

		#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			if(SHTPS_STATE_FACETOUCH == state && (pFingerInfoBuf[fingerMax * 5] & 0x02) != 0){
				shtps_event_all_cancel(ts);
			}
		#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

		shtps_calc_notify(ts, &buf[2], &info, &event);

		if(event != 0xff){
			#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
				if(event == SHTPS_EVENT_TD){
					shtps_perf_lock_enable(ts);
					shtps_perf_lock_disable_timer_start(ts, ts->perf_lock_enable_time_ms);
					SHTPS_LOG_DBG_PRINT("perf_lock start by TouchDown\n");
				}else if(event == SHTPS_EVENT_DRAG){
					if(ts->report_event == SHTPS_EVENT_TD){
						shtps_perf_lock_enable(ts);
						shtps_perf_lock_disable_timer_start(ts, ts->perf_lock_enable_time_ms);
						SHTPS_LOG_DBG_PRINT("perf_lock start by Drag\n");
					}
				}else if(event == SHTPS_EVENT_TU){
					shtps_perf_lock_disable(ts);
					SHTPS_LOG_DBG_PRINT("perf_lock end by TouchUp\n");
				}
			#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

			#if defined(SHTPS_LPWG_MODE_ENABLE)
				if(ts->lpwg.block_touchevent == 0){
					shtps_event_report(ts, &info, event);
				}else{
					SHTPS_LOG_DBG_PRINT("LPWG touch event blocked\n");
					shtps_event_update(ts, &info);
				}
			#else
				shtps_event_report(ts, &info, event);
			#endif /* SHTPS_LPWG_MODE_ENABLE */
		}

		#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
			shtps_wakeup_calib_check(ts, &info);
		#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

		#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			if(SHTPS_STATE_FACETOUCH == state){
				shtps_check_facetouchoff(ts);
			}
		#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

		
		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if(ts->lpwg.block_touchevent != 0){
				u8 rezero_req = 0;

				if(time_after(jiffies,
					ts->lpwg.wakeup_time + msecs_to_jiffies(SHTPS_LPWG_BLOCK_TIME_MAX_MS)))
				{
					rezero_req = 1;
					SHTPS_LOG_DBG_PRINT("LPWG force rezero\n");
				}else if((ts->lpwg.tu_rezero_req != 0) && 
							(ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0)
				{
					rezero_req = 1;
					SHTPS_LOG_DBG_PRINT("LPWG rezero by touch up\n");
				}

				if(rezero_req != 0){
					ts->touch_state.numOfFingers = 0;
					ts->lpwg.block_touchevent = 0;
					ts->lpwg.tu_rezero_req = 0;
					shtps_rezero_request(ts,
										 SHTPS_REZERO_REQUEST_REZERO,
										 SHTPS_REZERO_TRIGGER_WAKEUP);

					#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
						ts->wakeup_touch_event_inhibit_state = 0;
						SHTPS_LOG_WAKEUP_FAIL_TOUCH_EVENT_REJECT("TouchEvent Inhibit end by rezero exec\n");
					#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */
				}
			}
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		if(ts->touch_state.numOfFingers == 0){
			if((ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0){
				shtps_device_access_teardown(ts);
			}
		}

		memcpy(&ts->fw_report_info_store, &ts->fw_report_info, sizeof(ts->fw_report_info));

		break;
	}
}

#if defined(SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE)
static int shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	u8 buf[2]={0,0};

	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	_log_msg_sync( LOGMSG_ID__BL_IRQCLR, "0x%02X", buf[1]);

	return (int)buf[1];
}
#else
static void shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	u8 buf[2];

	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	_log_msg_sync( LOGMSG_ID__BL_IRQCLR, "0x%02X", buf[1]);
}
#endif

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
static int shtps_wakeup_calib_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms)
{
	SHTPS_LOG_FUNC_CALL_INPARAM((int)delay_ms);

	cancel_delayed_work(&ts->wakeup_calib_delayed_work);
	schedule_delayed_work(&ts->wakeup_calib_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

static int shtps_wakeup_calib_timer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->wakeup_calib_delayed_work);

	return 0;
}

static void shtps_wakeup_calib_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, wakeup_calib_delayed_work);

	mutex_lock(&shtps_ctrl_lock);

	if( (ts->state_mgr.state != SHTPS_STATE_ACTIVE) && (ts->state_mgr.state != SHTPS_STATE_FACETOUCH) ){
		ts->wakeup_calib_cancel_check_enable = 0;
		ts->wakeup_calib_enable = 0;
		ts->wakeup_calib_deferment = 0;
		ts->wakeup_calib_lastone = 0;
		ts->wakeup_calib_lastone_req = 0;
		SHTPS_LOG_DBG_PRINT("[wakeup_calib] work stop by drv state\n");
	}

	if(ts->wakeup_calib_enable != 0)
	{
		int i;
		u8 fingerMax = shtps_get_fingermax(ts);
		u8 numOfFingers = 0;
		u8 large_object_present = 0;
		u8 key_state = 0;
		u8 fn11_finger_state_size = 0;
		u8 fn11_finter_state_buf[3];

		if(ts->map.fn11.enable != 0){
			if(fingerMax > 8){
				fn11_finger_state_size = 3;
			}else if (fingerMax > 4){
				fn11_finger_state_size = 2;
			}else{
				fn11_finger_state_size = 1;
			}

			if(fn11_finger_state_size > sizeof(fn11_finter_state_buf)){
				fn11_finger_state_size = sizeof(fn11_finter_state_buf);
			}

			memset(fn11_finter_state_buf, 0, sizeof(fn11_finter_state_buf));
			shtps_rmi_read(ts, ts->map.fn11.dataBase, fn11_finter_state_buf, fn11_finger_state_size);

			shtps_rmi_read(ts, ts->map.fn11.dataBase + fn11_finger_state_size + fingerMax * 5, &large_object_present, 1);
			large_object_present = ((large_object_present >> 1) & 0x01);

			for(i = 0; i < fingerMax; i++){
				if( ((fn11_finter_state_buf[i / 4] >> (2 * (i % 4))) & 0x03) != SHTPS_TOUCH_STATE_NO_TOUCH ){
					numOfFingers++;
				}
			}
		}else{
			for(i = 0; i < fingerMax; i++){
				if(ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					numOfFingers++;
				}
			}
		}

		#if defined(SHTPS_PHYSICAL_KEY_ENABLE)
			shtps_rmi_read(ts, ts->map.fn1A.dataBase, &key_state, 1);
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

		if(ts->wakeup_calib_lastone != 0){
			if( (numOfFingers == 0) && (key_state == 0) ){
				if((SHTPS_WAKEUP_CALIB_ONLY_LO_DETECTED == 1 && large_object_present != 0) ||
					(SHTPS_WAKEUP_CALIB_GUARD_LO_DETECTED == 1 && large_object_present == 0))
				{
					shtps_forcecal(ts);
					SHTPS_LOG_DBG_PRINT("[wakeup_calib] force cal execute (last one)\n");

					ts->wakeup_calib_cancel_check_enable = 0;
					ts->wakeup_calib_enable = 0;
					ts->wakeup_calib_deferment = 0;
					ts->wakeup_calib_lastone = 0;
					ts->wakeup_calib_lastone_req = 0;
					shtps_wakeup_calib_timer_stop(ts);
					SHTPS_LOG_DBG_PRINT("[wakeup_calib] stop by last one rezero exec\n");
				}
			}
			else{
				SHTPS_LOG_DBG_PRINT("[wakeup_calib] last one can't force cal by touch down <f_num=%d><key=%d><palm=%d>\n",
										numOfFingers, key_state, large_object_present);

				ts->wakeup_calib_cancel_check_enable = 0;
				ts->wakeup_calib_enable = 0;
				ts->wakeup_calib_deferment = 0;
				ts->wakeup_calib_lastone = 0;
				ts->wakeup_calib_lastone_req = 1;
			}
		}
		else{
			if( (numOfFingers == 0) && (key_state == 0) ){
				if((SHTPS_WAKEUP_CALIB_ONLY_LO_DETECTED == 1 && large_object_present != 0) ||
					(SHTPS_WAKEUP_CALIB_GUARD_LO_DETECTED == 1 && large_object_present == 0))
				{
					shtps_forcecal(ts);
					ts->wakeup_calib_rezero_exec_num++;
					SHTPS_LOG_DBG_PRINT("[wakeup_calib] force cal execute <%d>\n", ts->wakeup_calib_rezero_exec_num);

					#if 0
					#if defined(SHTPS_CLING_REJECTION_ENABLE)
						if(large_object_present != 0){
							ts->cling_reject.forcecal_in_touch = 1;
						}
					#endif /* SHTPS_CLING_REJECTION_ENABLE */
					#endif
				}

				if(ts->wakeup_calib_rezero_exec_num >= SHTPS_WAKEUP_CALIB_STOP_COND_REZERO_EXEC_NUM){
					ts->wakeup_calib_cancel_check_enable = 0;
					ts->wakeup_calib_enable = 0;
					ts->wakeup_calib_deferment = 0;
					ts->wakeup_calib_lastone = 0;
					ts->wakeup_calib_lastone_req = 0;
					shtps_wakeup_calib_timer_stop(ts);
					SHTPS_LOG_DBG_PRINT("[wakeup_calib] stop by rezero exec count\n");
				}else{
					ts->wakeup_calib_enable = 1;
					ts->wakeup_calib_deferment = 0;
					shtps_wakeup_calib_timer_start(ts, SHTPS_WAKEUP_CALIB_INTERVAL_MS);
				}
			}
			else{
				ts->wakeup_calib_enable = 0;
				ts->wakeup_calib_deferment = 1;
				SHTPS_LOG_DBG_PRINT("[wakeup_calib] can't force cal by touch down <f_num=%d><key=%d><palm=%d>\n",
										numOfFingers, key_state, large_object_present);
			}
		}
	}

	mutex_unlock(&shtps_ctrl_lock);
}
#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_TPIN_CHECK_ENABLE )
static int shtps_tpin_enable_check(struct shtps_rmi_spi *ts)
{
	int ret;
	int val;

	ret = gpio_request(SHTPS_GPIO_TPIN_NO, "tpin");
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_request() error[%d]\n", __func__, ret);
	}

	ret = gpio_tlmm_config(GPIO_CFG(SHTPS_GPIO_TPIN_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_tlmm_config(set) error[%d]\n", __func__, ret);
	}
	udelay(50);
	val = gpio_get_value(SHTPS_GPIO_TPIN_NO);
	SHTPS_LOG_DBG_PRINT("%s() gpio_get_value() val = %d\n", __func__, val);
	
	ret = gpio_tlmm_config(GPIO_CFG(SHTPS_GPIO_TPIN_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() gpio_tlmm_config(reset) error[%d]\n", __func__, ret);
	}

	gpio_free(SHTPS_GPIO_TPIN_NO);

	if(!val) {
		SHTPS_LOG_ERR_PRINT("Upper unit does not exist.\n");
		return -1;
	}

	return 0;
}
#endif /* SHTPS_TPIN_CHECK_ENABLE */

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_boot_fwupdate_enable_check(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET)
	{
		sharp_smem_common_type *sharp_smem;
		unsigned char handset;

		sharp_smem = sh_smem_get_common_address();
		handset = sharp_smem->sh_hw_handset;

		if(handset == 0){
			return 0;
		}
	}
	#endif /* if defined(SHTPS_BOOT_FWUPDATE_ONLY_ON_HANDSET) */

	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return 0;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	return 1;
}

#if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
static int shtps_fwup_flag_check(void)
{
	sharp_smem_common_type *smemdata = NULL;

	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		SHTPS_LOG_DBG_PRINT("shtps_fwup_flag : %s\n", smemdata->shtps_fwup_flag == 0 ? "off" : "on");
		if(smemdata->shtps_fwup_flag == 0){
			return 0;
		}else{
			return 1;
		}
	}

	return -1;
}
#endif /* #if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE) */

static void shtps_fwup_flag_clear(void)
{
	sharp_smem_common_type *smemdata = NULL;

	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		smemdata->shtps_fwup_flag = 0;
	}
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts)
{
#if defined( SHTPS_FWUPDATE_DISABLE )
	return 0;
#else
	return 1;
#endif /* #if defined( SHTPS_FWUPDATE_DISABLE ) */
}

static int shtps_loader_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__BL_ATTN_WAIT, "");
	SHTPS_LOG_DBG_PRINT("shtps_loader_wait_attn() start:%d\n", ts->loader.ack);

	rc = wait_event_interruptible_timeout(ts->loader.wait_ack,
			ts->loader.ack == 1,
			msecs_to_jiffies(SHTPS_BOOTLOADER_ACK_TMO));

	_log_msg_recv( LOGMSG_ID__BL_ATTN, "");

	if(0 == rc && 0 == ts->loader.ack){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_ERROR, "");
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() ACK:%d\n", ts->loader.ack);
		return -1;
	}

	if(0 == rc){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() warning rc = %d\n", rc);
	}

	ts->loader.ack = 0;

	SHTPS_LOG_DBG_PRINT("shtps_loader_wait_attn() end:%d\n", ts->loader.ack);
	return 0;
}

static void shtps_loader_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->loader.ack = 1;
	_log_msg_send( LOGMSG_ID__BL_ATTN, "");
	wake_up_interruptible(&ts->loader.wait_ack);
}

static int shtps_loader_cmd(struct shtps_rmi_spi *ts, u8 cmd, u8 isLockdown)
{
	int rc;
	u8  buf;
	u16 blockSize;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__BL_COMMAND, "0x%02X|%d", cmd, isLockdown);
	if(isLockdown){
		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2,
						 	 F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 3,
				 			 F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);
	}
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize, &buf, 1);
	SPI_ERR_CHECK(rc, err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + blockSize, (buf & 0xF0) | (cmd & 0x0F));
	SPI_ERR_CHECK(rc, err_exit);

	return 0;

err_exit:
	return rc;
}

static int shtps_enter_bootloader(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__BL_ENTER, "");

	mutex_lock(&shtps_loader_lock);

	request_event(ts, SHTPS_EVENT_STOP, 0);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	if(request_event(ts, SHTPS_EVENT_STARTLOADER, 0) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_enter_bootloader() start loader error\n");
		goto err_exit;
	}
	shtps_wait_startup(ts);

	shtps_sleep(ts, 1);
	#if !defined(SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE)
		msleep(SHTPS_SLEEP_IN_WAIT_MS);
	#endif /* !SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */

	ts->loader.ack = 0;
	shtps_loader_cmd(ts, 0x0F, 1);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERR_PRINT("shtps_enter_bootloader() mode change error\n");
		goto err_exit;
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(shtps_map_construct(ts, 0) != 0){
#else
	if(shtps_map_construct(ts) != 0){
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
	}

	_log_msg_sync( LOGMSG_ID__BL_ENTER_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_enter_bootloader() done\n");
	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ENTER_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_exit_bootloader(struct shtps_rmi_spi *ts)
{
	u8  status;

	_log_msg_sync( LOGMSG_ID__BL_EXIT, "");
	shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01);
	msleep(SHTPS_RESET_BOOTLOADER_WAIT_MS);

	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	request_event(ts, SHTPS_EVENT_STOP, 0);

	if((status & 0x40) != 0 || (status & 0x0F) == 4 || (status & 0x0F) == 5 || (status & 0x0F) == 6){
		SHTPS_LOG_DBG_PRINT("shtps_exit_bootloader() error status = 0x%02x\n", status);
		_log_msg_sync( LOGMSG_ID__BL_EXIT_FAIL, "0x%02X", status);
		return -1;
	}

	SHTPS_LOG_DBG_PRINT("shtps_exit_bootloader() done\n");
	_log_msg_sync( LOGMSG_ID__BL_EXIT_DONE, "");
	return 0;
}

static int shtps_lockdown_bootloader(struct shtps_rmi_spi *ts, u8* fwdata)
{
	return 0;
}

static int shtps_flash_erase(struct shtps_rmi_spi *ts)
{
	int rc;
	u8  status;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_ERASE, "");

	mutex_lock(&shtps_loader_lock);

	ts->loader.ack = 0;
	shtps_loader_cmd(ts, 0x03, 1);
	msleep(SHTPS_FLASH_ERASE_WAIT_MS);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() err %d\n", rc);
		goto err_exit;
	}

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
						&status, 1);
	SPI_ERR_CHECK((rc || status != 0x80), err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);

	SHTPS_LOG_DBG_PRINT("shtps_flash_erase() done\n");
	_log_msg_sync( LOGMSG_ID__BL_ERASE_DONE, "");
	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ERASE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeImage(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	u16 blockNum;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE, "");

	mutex_lock(&shtps_loader_lock);

	blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
		rc = shtps_rmi_write_fw_data(ts, ts->map.fn34.dataBase + 2, fwdata);
		SPI_ERR_CHECK(rc, err_exit);
	#else
		{
			int i;
			for(i = 0;i < blockSize;i++){
				rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i, fwdata[i]);
				SPI_ERR_CHECK(rc, err_exit);
			}
		}
	#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

//	ts->loader.ack = 0;
	rc = shtps_loader_cmd(ts, 0x02, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		goto err_exit;
	}

//	msleep(10);

	SHTPS_LOG_DBG_PRINT("shtps_flash_writeImage() done\n");
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_DONE, "");

	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	SHTPS_LOG_DBG_PRINT("shtps_flash_writeImage:err_exit\n");
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeConfig(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	int block;
	u16 blockNum;
	u16 blockSize;
	u8  status;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG, "");

	mutex_lock(&shtps_loader_lock);

	blockNum  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	status=0;
	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);

	for(block = 0;block < blockNum;block++){
		#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
			rc = shtps_rmi_write_fw_data(ts, ts->map.fn34.dataBase + 2, &fwdata[block * blockSize]);
			SPI_ERR_CHECK(rc, err_exit);
		#else
			{
				int i;
				for(i = 0;i < blockSize;i++){
					rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i,
										 fwdata[(block * blockSize) + i]);
					SPI_ERR_CHECK(rc, err_exit);
				}
			}
		#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

//		ts->loader.ack = 0;
		rc = shtps_loader_cmd(ts, 0x06, 0);
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_loader_wait_attn(ts);
		SPI_ERR_CHECK(rc, err_exit);

//		msleep(10);

		rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
							&status, 1);
		SPI_ERR_CHECK((rc || status != 0x80), err_exit);
	}

	SHTPS_LOG_DBG_PRINT("shtps_flash_writeConfig() done\n");
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_DONE, "");
	rc = shtps_exit_bootloader(ts);
	SPI_ERR_CHECK(rc, err_exit);

	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	SHTPS_LOG_DBG_PRINT("shtps_flash_writeConfig:err_exit:%d\n",status);
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_ASYNC_OPEN_ENABLE )
static void shtps_func_open(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
		if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
			u8 buf;
			const unsigned char* fw_data = NULL;
			int ver;
			u8 update = 0;

			if(shtps_start(ts) == 0){
				shtps_wait_startup(ts);
			}

			shtps_rmi_read(ts, 0x0013, &buf, 1);

			#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
				ver = shtps_fwver(ts);
				DBG_PRINTK("fw version = 0x%04x\n", ver);
				if(ver != shtps_fwver_builtin(ts)){
					update = 1;
				}
			#else
				if(shtps_fwup_flag_check() > 0){
					ver = shtps_fwver(ts);
					if(ver != shtps_fwver_builtin(ts)){
						update = 1;
					}
				}
			#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

			if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
				#ifdef CONFIG_SHTERM
					shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
					SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (first check)\n");
				#endif
				SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
				update = 1;
			}

			if(update != 0){
				fw_data = shtps_fwdata_builtin(ts);;
				if(fw_data){
					int ret;
					int retry = 5;
					do{
						ret = shtps_fw_update(ts, fw_data);
						request_event(ts, SHTPS_EVENT_STOP, 0);
						#ifdef CONFIG_SHTERM
							if (ret != 0) {
								shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
								SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (retry=%d)\n", retry);
							}
						#endif
					}while(ret != 0 && (retry-- > 0));
					
					#ifdef CONFIG_SHTERM
						if (ret != 0 && retry < 0) {
							shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX);
							SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX\n");
						} else {
							shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX);
                                                        SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX\n");
						}
					#endif
				}
			}
		}
		shtps_fwup_flag_clear();
	#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}
}

static void shtps_func_close(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}

static int shtps_func_enable(struct shtps_rmi_spi *ts)
{
	if(shtps_start(ts) != 0){
		return -EFAULT;
	}
	if(shtps_wait_startup(ts) != 0){
		return -EFAULT;
	}

	return 0;
}

static void shtps_func_disable(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
static void shtps_func_proximity_check(struct shtps_rmi_spi *ts)
{
	int proximity_data = -1;

	wake_lock(&ts->wake_lock_proximity);
    pm_qos_update_request(&ts->pm_qos_lock_idle_proximity, SHTPS_QOS_LATENCY_DEF_VALUE);

	proximity_data = shtps_proximity_check(ts);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
	{
		unsigned long time = 0;

		ts->lpwg_proximity_get_data = proximity_data;
		if(ts->lpwg_proximity_get_data == SHTPS_PROXIMITY_NEAR){
			SHTPS_LOG_DBG_PRINT("[LPWG][Func] proximity near");

			mutex_lock(&shtps_ctrl_lock);
			if(ts->lpwg.is_notified != 0){
				if(time_after(jiffies, ts->lpwg.notify_time + msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL))){
					time = 0;
				}else{
					time = jiffies_to_msecs(((ts->lpwg.notify_time + msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL)) - jiffies)
												% SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL);
				}
			}
			mutex_unlock(&shtps_ctrl_lock);

			if(time > 0){
				SHTPS_LOG_DBG_PRINT("[LPWG] cancel notify wait <%lums>\n", time);
				msleep(time);
			}

			mutex_lock(&shtps_ctrl_lock);
			if(ts->lpwg.is_notified != 0){
				shtps_notify_cancel_wakeup_event(ts);
				ts->lpwg.is_notified = 0;
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}
	#endif /* SHTPS_LPWG_MODE_ENABLE */

    pm_qos_update_request(&ts->pm_qos_lock_idle_proximity, PM_QOS_DEFAULT_VALUE);
	wake_unlock(&ts->wake_lock_proximity);
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

static void shtps_func_request_async_complete(void *arg_p)
{
	kfree( arg_p );
}

static void shtps_func_request_async( struct shtps_rmi_spi *ts, int event)
{
	struct shtps_req_msg		*msg_p;
	unsigned long	flags;

	msg_p = (struct shtps_req_msg *)kzalloc( sizeof( struct shtps_req_msg ), GFP_KERNEL );
	if ( msg_p == NULL ){
		SHTPS_LOG_ERR_PRINT("Out of memory [event:%d]\n", event);
		return;
	}

	msg_p->complete = shtps_func_request_async_complete;
	msg_p->event = event;
	msg_p->context = msg_p;
	msg_p->status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg_p->queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );
}

static void shtps_func_request_sync_complete(void *arg_p)
{
	complete( arg_p );
}

static int shtps_func_request_sync( struct shtps_rmi_spi *ts, int event)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct shtps_req_msg msg;
	unsigned long	flags;

	msg.complete = shtps_func_request_sync_complete;
	msg.event = event;
	msg.context = &done;
	msg.status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg.queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );

	wait_for_completion(&done);

	return msg.status;
}

static void shtps_func_workq( struct work_struct *work_p )
{
	struct shtps_rmi_spi	*ts;
	unsigned long			flags;

	ts = container_of(work_p, struct shtps_rmi_spi, work_data);

	spin_lock_irqsave( &(ts->queue_lock), flags );
	while( list_empty( &(ts->queue) ) == 0 ){
		ts->cur_msg_p = list_entry( ts->queue.next, struct shtps_req_msg, queue);
		list_del_init( &(ts->cur_msg_p->queue) );
		spin_unlock_irqrestore( &(ts->queue_lock), flags );

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] start\n", ts->cur_msg_p->event);

		switch(ts->cur_msg_p->event){
			case SHTPS_FUNC_REQ_EVEMT_OPEN:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_OPEN, 0) == 0){
						shtps_func_open(ts);
					}
				#else
					shtps_func_open(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_CLOSE:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE, 0) == 0){
						shtps_func_close(ts);
					}
				#else
					shtps_func_close(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_ENABLE:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE, 0) == 0){
						ts->cur_msg_p->status = shtps_func_enable(ts);
					}else{
						ts->cur_msg_p->status = 0;
					}
				#else
					ts->cur_msg_p->status = shtps_func_enable(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				break;

			case SHTPS_FUNC_REQ_EVEMT_DISABLE:
				shtps_func_disable(ts);
				ts->cur_msg_p->status = 0;
				break;

			#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
				case SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK:
					shtps_func_proximity_check(ts);
					ts->cur_msg_p->status = 0;
					break;
			#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

			default:
				ts->cur_msg_p->status = -1;
				break;
		}

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] end\n", ts->cur_msg_p->event);

		if( ts->cur_msg_p->complete ){
			ts->cur_msg_p->complete( ts->cur_msg_p->context );
		}
		spin_lock_irqsave( &(ts->queue_lock), flags );
	}
	spin_unlock_irqrestore( &(ts->queue_lock), flags );
}
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param)
{
	int ret;

	_log_msg_sync( LOGMSG_ID__REQUEST_EVENT, "%d|%d", event, ts->state_mgr.state);

	SHTPS_LOG_DBG_PRINT("event %d in state %d\n", event, ts->state_mgr.state);

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	switch(event){
	case SHTPS_EVENT_START:
		ret = state_func_tbl[ts->state_mgr.state]->start(ts, param);
		break;
	case SHTPS_EVENT_STOP:
		ret = state_func_tbl[ts->state_mgr.state]->stop(ts, param);
		break;
	case SHTPS_EVENT_SLEEP:
		ret = state_func_tbl[ts->state_mgr.state]->sleep(ts, param);
		break;
	case SHTPS_EVENT_WAKEUP:
		ret = state_func_tbl[ts->state_mgr.state]->wakeup(ts, param);
		break;
	case SHTPS_EVENT_STARTLOADER:
		ret = state_func_tbl[ts->state_mgr.state]->start_ldr(ts, param);
		break;
	case SHTPS_EVENT_STARTTM:
		ret = state_func_tbl[ts->state_mgr.state]->start_tm(ts, param);
		break;
	case SHTPS_EVENT_STOPTM:
		ret = state_func_tbl[ts->state_mgr.state]->stop_tm(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_ON:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_on(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_OFF:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_off(ts, param);
		break;
	case SHTPS_EVENT_INTERRUPT:
		ret = state_func_tbl[ts->state_mgr.state]->interrupt(ts, param);
		break;
	case SHTPS_EVENT_TIMEOUT:
		ret = state_func_tbl[ts->state_mgr.state]->timeout(ts, param);
		break;
	default:
		ret = -1;
		break;
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return ret;
}

static int state_change(struct shtps_rmi_spi *ts, int state)
{
	int ret = 0;
	int old_state = ts->state_mgr.state;

	_log_msg_sync( LOGMSG_ID__STATE_CHANGE, "%d|%d", ts->state_mgr.state, state);
	SHTPS_LOG_DBG_PRINT("state %d -> %d\n", ts->state_mgr.state, state);

	if(ts->state_mgr.state != state){
		ts->state_mgr.state = state;
		ret = state_func_tbl[ts->state_mgr.state]->enter(ts, old_state);
	}
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_NOP, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return 0;
}

static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_ERROR, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return -1;
}

static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_STOP, "%d", ts->state_mgr.state);
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_system_set_sleep(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_set_facetouchmode(ts, 1);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_set_facetouchmode(ts, 0);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return -1;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	shtps_system_set_wakeup(ts);
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_NORMAL);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined( SHTPS_TPIN_CHECK_ENABLE )
		if(shtps_tpin_enable_check(ts) != 0){
			return -1;
		}
	#endif /* SHTPS_TPIN_CHECK_ENABLE */

	shtps_system_set_wakeup(ts);
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_LOADER);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irqtimer_stop(ts);
	shtps_statef_cmn_stop(ts, param);
	return 0;
}

static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_reset_startuptime(ts);
	shtps_irqtimer_stop(ts);
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param)
{
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irqtimer_stop(ts);
	shtps_statef_cmn_stop(ts, param);
	return 0;
}

static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param)
{
#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	int rc;
	int retry = SHTPS_PDT_READ_RETRY_COUNT;
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	unsigned long time;
	if((time = shtps_check_startuptime(ts)) != 0){
		SHTPS_LOG_DBG_PRINT("startup wait time : %lu\n", time);
		msleep(time);
	}

	shtps_irqtimer_stop(ts);

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	do{
		rc = shtps_map_construct(ts, 1);
		if(rc){
			msleep(SHTPS_PDT_READ_RETRY_INTERVAL);
		}
	}while(rc != 0 && retry-- > 0);
#else
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(rc != 0){
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
	}else
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param)
{
#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	int rc;
	int retry = SHTPS_PDT_READ_RETRY_COUNT;

	do{
		rc = shtps_map_construct(ts, 1);
		if(rc){
			msleep(SHTPS_PDT_READ_RETRY_INTERVAL);
		}
	}while(rc != 0 && retry-- > 0);
#else
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}

#if defined(SHTPS_PDT_READ_RETRY_ENABLE)
	if(rc != 0){
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
	}else
#endif /* SHTPS_PDT_READ_RETRY_ENABLE */
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param)
{
	if(param == SHTPS_STATE_WAIT_READY){
		shtps_init_param(ts);
		if(ts->poll_info.boot_rezero_flag == 0){
			ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
		}
		shtps_read_touchevent(ts, ts->state_mgr.state);
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			shtps_irq_enable(ts);
		#else
			shtps_irq_wake_enable(ts);
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	}

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		if(1 == shtps_get_facetouchmode(ts)){
			shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
			state_change(ts, SHTPS_STATE_FACETOUCH);
		}
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	
	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			state_change(ts, SHTPS_STATE_SLEEP);
		}
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	return 0;
}

static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param)
{
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		shtps_set_facetouchmode(ts, 1);
		state_change(ts, SHTPS_STATE_FACETOUCH);
	#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT */
	return 0;
}

static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();

	#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
		shtps_wake_lock_for_fwupdate(ts);
	#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_loader_irqclr(ts);
	return 0;
}

static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
		shtps_wake_unlock_for_fwupdate(ts);
	#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
#if defined(SHTPS_IRQ_LOADER_CHECK_INT_STATUS_ENABLE)
	if(shtps_loader_irqclr(ts) != 0){
		shtps_loader_wakeup(ts);
	}
#else
	shtps_loader_irqclr(ts);
	shtps_loader_wakeup(ts);
#endif
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
			shtps_set_doze_mode(ts, 1);
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_rezero_request(ts,
							 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
							 SHTPS_REZERO_TRIGGER_ENDCALL);
		shtps_set_facetouchmode(ts, 0);
		state_change(ts, SHTPS_STATE_ACTIVE);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_tm_irqcheck(ts);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_start_tm(ts);
	return 0;
}

static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_stop_tm(ts);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param)
{
	SHTPS_LOG_FUNC_CALL();
	if(shtps_tm_irqcheck(ts)){
		shtps_tm_wakeup(ts);
	}
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param)
{
	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
		shtps_debug_sleep_enter();
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event = SHTPS_EVENT_TU;
		shtps_perf_lock_disable(ts);
		SHTPS_LOG_DBG_PRINT("perf_lock end by ForceTouchUp\n");
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		ts->edge_fail_touch_inhibit_id = 0;
		ts->edge_fail_touch_decide_mt = 0;
		ts->edge_fail_touch_td_cnt = 0;
		ts->edge_fail_touch_side_inhibit_id = 0;
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		ts->grip_fail_touch_inhibit_id = 0;
		ts->grip_fail_flick_inhibit_id = 0;
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		if(ts->pinch_fail_reject.segmentation_aggressiveness_set_state != 0){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + SHTPS_REG_OFFSET_SEGMENTATION_AGGRESSIVENESS,
								ts->pinch_fail_reject.segmentation_aggressiveness_def);
			SHTPS_LOG_DBG_SHTPS_PINCH_FAIL_RESPONSE_PRINT("segmentation aggressiveness set def [%d]\n", ts->pinch_fail_reject.segmentation_aggressiveness_def);
		}
		ts->pinch_fail_reject.segmentation_aggressiveness_set_state = 0;
		ts->pinch_fail_reject.segmentation_aggressiveness_set_check = 0;
		ts->pinch_fail_reject.finger_distance = 0;
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	#if defined( SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE )
		ts->lgm_split_touch_combining.finger_swap = 0;
		ts->lgm_split_touch_combining.finger_adjust = 0;
	#endif  /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
		ts->wakeup_calib_enable = 0;
		ts->wakeup_calib_cancel_check_enable = 0;
		ts->wakeup_calib_deferment = 0;
		ts->wakeup_calib_lastone = 0;
		ts->wakeup_calib_lastone_req = 0;
		ts->wakeup_calib_rezero_exec_num = 0;
		memset(ts->wakeup_calib_td_pos, 0, sizeof(ts->wakeup_calib_td_pos));
		memset(ts->wakeup_calib_cancel_check, 0, sizeof(ts->wakeup_calib_cancel_check));
		memset(ts->wakeup_calib_event_count, 0, sizeof(ts->wakeup_calib_event_count));
		shtps_wakeup_calib_timer_stop(ts);
	#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

	#if defined( SHTPS_CLING_REJECTION_ENABLE )
		shtps_cling_reject_init_variables(ts);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */
	
	#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
		ts->lgm_fail_touch_up_reject.chatt_enable = 0;
		ts->lgm_fail_touch_up_reject.chatt_time = 0;
		ts->lgm_fail_touch_up_reject.chatt_finger = 0;
		ts->lgm_fail_touch_up_reject.chatt_check_enable_finger = 0;
		ts->lgm_fail_touch_up_reject.chatt_work_enable = 0;
		memset(ts->lgm_fail_touch_up_reject.pos_move, 0, sizeof(ts->lgm_fail_touch_up_reject.pos_move));
		memset(ts->lgm_fail_touch_up_reject.z_max, 0, sizeof(ts->lgm_fail_touch_up_reject.z_max));
		shtps_lgm_fail_touch_reject_chatt_stop(ts);
	#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE)
		memset(ts->edge_shake_check_state, 0, sizeof(ts->edge_shake_check_state));
	#endif /* SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE */

	#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
		shtps_quick_flick_fail_resolv_init(ts);
	#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */
	
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_key_event_force_touchup(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	shtps_event_force_touchup(ts);
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined( SHTPS_FAILSAFE_ENABLE )
		{
			u8 buf[2];
			shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
		}
	#endif /* #if defined( SHTPS_FAILSAFE_ENABLE ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(shtps_is_lpwg_active(ts)){
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_enable(ts);
			#else
				shtps_irq_wake_enable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
			
			shtps_set_lpwg_mode_on(ts);
			shtps_set_lpwg_mode_cal(ts);
		}else{
			shtps_sleep(ts, 1);
			shtps_system_set_sleep(ts);
		}
	#else
		shtps_sleep(ts, 1);
		shtps_system_set_sleep(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;
}

static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param)
{
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if (ts->lpwg.lpwg_switch){
			ts->lpwg.wakeup_time = jiffies;
			shtps_set_lpwg_mode_off(ts);
			shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
		}else{
			shtps_system_set_wakeup(ts);
		}
	#else
		shtps_system_set_wakeup(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	shtps_sleep(ts, 0);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		{
			u8 buf;
			shtps_rmi_read(ts, ts->map.fn01.dataBase + 1, &buf, 1);
		}
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined( SHTPS_CLING_REJECTION_ENABLE )
		ts->cling_reject.mode0_cling_cnt_max = (SHTPS_CLING_REJECT_MODE0_CNT / 2);
		ts->cling_reject.mode0_cling_cnt_change_time = jiffies + msecs_to_jiffies(SHTPS_CLING_REJECT_MODE0_CNT_CHANGE_TIME);
		ts->cling_reject.mode0_cling_cnt_change_state = 1;
		SHTPS_LOG_CLING_REJECT("[mode_0] count max change to %d\n", ts->cling_reject.mode0_cling_cnt_max);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_WAKEUP_REZERO |
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_WAKEUP);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param)
{
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
	#if !defined(SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE)
		if(SHTPS_SLEEP_OUT_WAIT_MS > 0){
			msleep(SHTPS_SLEEP_OUT_WAIT_MS);
		}
	#endif /* !SHTPS_ACTIVE_SLEEP_WAIT_ALWAYS_ENABLE */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
			shtps_irq_enable(ts);
		#else
			shtps_irq_wake_enable(ts);
		#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
		
		#if defined(SHTPS_LPWG_MODE_ENABLE)
			if (ts->lpwg.lpwg_switch){
				ts->lpwg.wakeup_time = jiffies;
				shtps_set_lpwg_mode_off(ts);
				shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
			}else{
				shtps_system_set_wakeup(ts);
			}
		#else
			shtps_system_set_wakeup(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		shtps_sleep(ts, 0);

		#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
			shtps_set_doze_mode(ts, 1);
		#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */

		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		shtps_set_facetouchmode(ts, 1);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param)
{
	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if (ts->lpwg.lpwg_switch){
			shtps_read_touchevent_insleep(ts, SHTPS_STATE_SLEEP);
		}else{
			shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
		}
	#else
		shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param)
{
	#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
		ts->wakeup_calib_enable = 0;
		ts->wakeup_calib_cancel_check_enable = 0;
		ts->wakeup_calib_deferment = 0;
		ts->wakeup_calib_lastone = 0;
		ts->wakeup_calib_lastone_req = 0;
		ts->wakeup_calib_rezero_exec_num = 0;
		memset(ts->wakeup_calib_td_pos, 0, sizeof(ts->wakeup_calib_td_pos));
		memset(ts->wakeup_calib_cancel_check, 0, sizeof(ts->wakeup_calib_cancel_check));
		memset(ts->wakeup_calib_event_count, 0, sizeof(ts->wakeup_calib_event_count));
		shtps_wakeup_calib_timer_stop(ts);
	#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	ts->facetouch.touch_num = ts->touch_state.numOfFingers;
	
	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event = SHTPS_EVENT_TU;
		shtps_perf_lock_disable(ts);
		SHTPS_LOG_DBG_PRINT("perf_lock end by sleep\n");
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_absorption_hold_cancel(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_key_event_force_touchup(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	
	shtps_event_all_cancel(ts);

#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE )
	shtps_set_reporting_mode(ts, SHTPS_FACETOUCH_DETECT_REPORTING_MODE);
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE */

	return 0;
#else
	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	shtps_sleep(ts, 1);
	shtps_system_set_sleep(ts);
	return 0;
#endif /* #if !defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);
	
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);
	
#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE )
	shtps_set_reporting_mode(ts, SHTPS_FACETOUCH_DETECT_REPORTING_MODE_DEFAULT);
#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHG_REPORTMODE_ENABLE */

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE )
		if(shtps_is_lpmode(ts) == 0){
			shtps_set_doze_mode(ts, 0);
		}
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_DOZE_ENABLE */

	#if defined( SHTPS_FAILSAFE_ENABLE )
		{
			u8 buf[2];
			shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
		}
	#endif /* #if defined( SHTPS_FAILSAFE_ENABLE ) */
#else
	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_enable(ts);
	#else
		shtps_irq_wake_enable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
	
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

#if defined( SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE )
	if(ts->facetouch.rezero_disable == 1 && SHTPS_FACETOUCHOFF_DETECT_SKIP_REZERO_ENABLE == 1){
		ts->facetouch.rezero_disable = 0;
		SHTPS_LOG_DBG_PRINT("Detect face touch off. skip rezero...\n");
	}else{
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
	}
#else
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
#endif /* SHTPS_FACETOUCH_OFF_DETECT_SKIP_REZERO_ENABLE */


	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);
	shtps_sleep(ts, 0);
#else
	shtps_system_set_wakeup(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_facetouch_wakelock(ts, 0);
	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);

	#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
		shtps_irq_disable(ts);
	#else
		shtps_irq_wake_disable(ts);
	#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(shtps_is_lpwg_active(ts)){
			#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
				shtps_irq_enable(ts);
			#else
				shtps_irq_wake_enable(ts);
			#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */
			
			shtps_set_lpwg_mode_on(ts);
			shtps_set_lpwg_mode_cal(ts);
		}else{
			shtps_sleep(ts, 1);
			shtps_system_set_sleep(ts);
		}
	#else
		shtps_sleep(ts, 1);
		shtps_system_set_sleep(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	shtps_set_facetouchmode(ts, 0);

#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_read_touchevent_infacetouchmode(ts);
#else
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CREATE_KOBJ_ENABLE)
#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
static int shtps_ioctl_set_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_lpwg_enable(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_lcd_bright_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);

static ssize_t show_suspend_spi_test_state(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "suspend spi test cmd\n"
									"  1: set suspend state\n"
									"  2: clr suspend state\n"
									"  3: [test request] charger\n"
									"  4: [test request] set sleep\n"
									"  5: [test request] set lpwg\n"
									"  6: [test request] set lpmode\n"
									"  7: [test request] set continuous lpmode\n"
									"  8: [test request] set lcd bright lpmode\n"
									"  9: [test request] tps open\n"
									" 10: [test request] tps enable\n"
									" 11: [test request] tps close\n");
}

static ssize_t store_suspend_spi_test_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int test_no;
	int param;
	
	sscanf(buf,"%d %d", &test_no, &param);
	
	switch(test_no){
		case 1: shtps_set_suspend_state(ts);
			break;
		case 2: shtps_clr_suspend_state(ts);
			break;
		case 3: shtps_ioctl_set_charger_armor(ts, param);
			break;
		case 4: msm_tps_setsleep(param);
			break;
		case 5: shtps_ioctl_lpwg_enable(ts, param);
			break;
		case 6: shtps_ioctl_set_low_power_mode(ts, param);
			break;
		case 7: shtps_ioctl_set_continuous_low_power_mode(ts, param);
			break;
		case 8: shtps_ioctl_set_lcd_bright_low_power_mode(ts, param);
			break;
		case 9: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
			break;
		case 10: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
			break;
		case 11: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
			break;
		default:
			break;
	}
	
	return count;
}

static struct kobj_attribute suspend_spi_test = 
	__ATTR(suspend_spi_test, S_IRUSR | S_IWUSR, show_suspend_spi_test_state, store_suspend_spi_test_state);

#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

static struct attribute *attrs_ctrl[] = {
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		&suspend_spi_test.attr,
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	
	NULL
};

static struct attribute_group attr_grp_ctrl = {
	.name = "ctrl",
	.attrs = attrs_ctrl,
};
#endif /* SHTPS_CREATE_KOBJ_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static int shtps_ioctl_enable(struct shtps_rmi_spi *ts)
{
	int ret;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
	#else
		ret = shtps_start(ts);
		shtps_wait_startup(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return ret;
}

static int shtps_ioctl_disable(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);
	#else
		shtps_shutdown(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return 0;
}

static int shtps_ioctl_reset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	if(power == 0){
		shtps_irq_enable(ts);
	}

	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_softreset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	if(power == 0){
		shtps_irq_enable(ts);
	}

	if(shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_getver(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8  tps_stop_request = 0;
	u16 ver;

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(ts->state_mgr.state == SHTPS_STATE_IDLE){
		tps_stop_request = 1;
	}

	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERR_PRINT("[%s] error - shtps_start()\n", __func__);
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ver = shtps_fwver(ts);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	if(tps_stop_request){
		shtps_shutdown(ts);
	}

	if(copy_to_user((u16*)arg, &ver, sizeof(ver))){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] version = 0x%04x\n", __func__, ver);
	return 0;
}

static int shtps_ioctl_enter_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	struct shtps_bootloader_info info;

	if(0 == arg){
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();
	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);

	if(0 == rc){
		info.block_size        = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
		info.program_block_num = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
		info.config_block_num  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);

		if(copy_to_user((u8*)arg, (u8*)&info, sizeof(struct shtps_bootloader_info))){
			return -EFAULT;
		}
	}

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_lockdown_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > shtps_fwsize_builtin(ts)) {
			return -EINVAL;
		}
		fw_data = shtps_fwdata_builtin(ts);
		if(fw_data == NULL){
			SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
			return -EFAULT;
		}
		rc = shtps_lockdown_bootloader(ts, (u8*)&(fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal.\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			kfree(data);
			return -EINVAL;
		}
		rc = shtps_lockdown_bootloader(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		kfree(data);
		return -EINVAL;
	}
	rc = shtps_lockdown_bootloader(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_erase_flash(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return shtps_flash_erase(ts);
}

static int shtps_ioctl_write_image(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > shtps_fwsize_builtin(ts)) {
			return -EINVAL;
		}

		fw_data = shtps_fwdata_builtin(ts);
		if(fw_data == NULL){
			SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
			return -EFAULT;
		}
		rc = shtps_flash_writeImage(ts, (u8*)&(fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			kfree(data);
			return -EINVAL;
		}
		rc = shtps_flash_writeImage(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		kfree(data);
		return -EINVAL;
	}
	rc = shtps_flash_writeImage(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_write_config(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > shtps_fwsize_builtin(ts)) {
			return -EINVAL;
		}

		fw_data = shtps_fwdata_builtin(ts);
		if(fw_data == NULL){
			SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
			return -EFAULT;
		}
		rc = shtps_flash_writeConfig(ts, (u8*)&(fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			kfree(data);
			return -EINVAL;
		}
		rc = shtps_flash_writeConfig(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		kfree(data);
		return -EINVAL;
	}
	rc = shtps_flash_writeConfig(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int i;
	u8 	fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans(struct shtps_rmi_spi *ts, unsigned long arg)
{
	if(copy_to_user((u8*)arg, (u8*)&ts->report_info, sizeof(ts->report_info))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_set_touchinfo_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.pos_mode = arg;
	return 0;
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static int shtps_ioctl_get_touchkeyinfo(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret;
	struct shtps_touch_key_info info;

	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event_touchkey == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(ret != 0){
		info.up_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_UP) & 0x01);
		info.down_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01);

		ts->diag.event_touchkey = 0;

		if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
			return -EFAULT;
		}

		return 0;
	}

	return -EFAULT;
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static int shtps_ioctl_reg_read(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 buf;
	u8 data[2];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size == 1){
		if(0 != copy_from_user(data, param.data, 1)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0], &buf, 1)){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0] << 0x08 | data[1], &buf, 1)){
			return -EFAULT;
		}
	}
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)&buf, 1)){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_reg_allread(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 dbbuf[0x100];
	u8 data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(&data, param.data, 1)){
		return -EINVAL;
	}
	if(shtps_rmi_read(ts, data << 0x08, dbbuf, 0x100)){
		return -EFAULT;
	}

	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)dbbuf, 0x100)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_reg_write(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 data[3];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size == 2){
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0], data[1])){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 3)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0] << 0x08 | data[1], data[2])){
			return -EFAULT;
		}
	}
	return 0;
}

#if defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
static int shtps_ioctl_reg_write_block(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 data[0x100 + 2];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	if(shtps_rmi_write_block(ts, data[0] << 0x08 | data[1], &data[2], (param.size - 2))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_reg_read_block(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 buf[0x100];
	u8 data[2];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(data, param.data, 2)){
		return -EINVAL;
	}
	if(shtps_rmi_read(ts, data[0] << 0x08 | data[1], buf, param.size)){
		return -EFAULT;
	}

	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)&buf, param.size)){
		return -EFAULT;
	}

	return 0;
}
#endif /* SHTPS_SPI_BLOCKACCESS_ENABLE */

static int shtps_ioctl_tm_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.tm_mode = SHTPS_FWTESTMODE_V01;
	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_tm_stop(struct shtps_rmi_spi *ts)
{
	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline_raw(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE_RAW);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_frameline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data,
#if defined( SHTPS_SY_REGMAP_BASE3 )
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
#else
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts))){
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_start_facetouchmode(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);
}

static int shtps_ioctl_stop_facetouchmode(struct shtps_rmi_spi *ts)
{
	int ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	ts->facetouch.wake_sig = 1;
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_facetouch_wakelock(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return ret;
}

static int shtps_ioctl_poll_facetouchoff(struct shtps_rmi_spi *ts)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int rc;

	SHTPS_LOG_DBG_PRINT("wait for face touch off detect\n");
	rc = wait_event_interruptible(ts->facetouch.wait_off,
		(ts->facetouch.detect == 1) || (ts->facetouch.mode == 0) ||
		(ts->facetouch.wake_sig == 1));

	ts->facetouch.wake_sig = 0;

	if(ts->facetouch.detect){
		SHTPS_LOG_DBG_PRINT("face touch %s detect\n", (ts->facetouch.state == 0)? "off" : "on");
		rc = (ts->facetouch.state == 0)? TPSDEV_FACETOUCH_OFF_DETECT : TPSDEV_FACETOUCH_DETECT;
		ts->facetouch.detect = 0;
	}else{
		SHTPS_LOG_DBG_PRINT("wait for facetouch was cancelled\n");
		rc = TPSDEV_FACETOUCH_NOCHG;
	}

	return rc;
#else
	return TPSDEV_FACETOUCH_NOCHG;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_ioctl_get_fwstatus(struct shtps_rmi_spi *ts, unsigned long arg)
{
	unsigned char status;

	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	status = status & 0x0F;

	if(copy_to_user((u8*)arg, (u8*)&status, sizeof(status))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_fwdate(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 year;
	u8 month;
	unsigned short date;

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_fwdate(ts, &year, &month);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	date = (year << 0x08) | month;

	if(copy_to_user((u16*)arg, (u16*)&date, sizeof(date))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_calibration_param(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 *data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > sizeof(struct shtps_offset_info)){
		return -EINVAL;
	}
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		kfree(data);
		return -EINVAL;
	}
	memcpy(ts->offset.base, data, sizeof(u16) * 5);
	ts->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);
	kfree(data);

	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}

	return 0;
}

static int shtps_ioctl_debug_reqevent(struct shtps_rmi_spi *ts, unsigned long arg)
{
	request_event(ts, (int)arg, 0);
	return 0;
}

static int shtps_ioctl_set_dragstep_x(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_X_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_X_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep_y(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_Y_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_Y_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_set_fingerfixtime(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg < 0xFFFF){
		SHTPS_DRAG_THRESH_RETURN_TIME =	(int)arg;
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_RETURN_TIME;
}

static int shtps_ioctl_rezero(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return 0;
}

static int shtps_ioctl_ack_facetouchoff(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	mutex_lock(&shtps_ctrl_lock);
	shtps_facetouch_wakelock(ts, 0);
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_ioctl_tmf05_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	if(ts->map.fn05.enable != 0){
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V02;
	}else{
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V03;
	}

	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static int shtps_ioctl_log_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	gLogOutputEnable = (int)arg;
	return 0;
}
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static int shtps_ioctl_getver_builtin(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u16 ver = shtps_fwver_builtin(ts);

	if(shtps_fwdata_builtin(ts) == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}
	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(copy_to_user((u16*)arg, &ver, sizeof(ver))){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in version = 0x%04x\n", __func__, ver);
	return 0;
}
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

#if defined( SHTPS_SMEM_BASELINE_ENABLE )
static int shtps_ioctl_get_smem_baseline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();

	if(!smemdata){
		return -EFAULT;
	}
	
	if(copy_to_user((u8*)arg, (u8*)(smemdata->shdiag_TpsBaseLineTbl),
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */

static int shtps_ioctl_set_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE, (u8)arg) == 0){
			shtps_ioctl_setlpmode_proc(ts, (u8)arg);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, (int)arg);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE, (u8)arg) == 0){
			shtps_ioctl_setconlpmode_proc(ts, (u8)arg);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, (int)arg);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_lcd_bright_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE, (u8)arg) == 0){
			shtps_ioctl_setlcdbrightlpmode_proc(ts, (u8)arg);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, (int)arg);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, (u8)arg) == 0){
			shtps_charger_armor_proc(ts, (u8)arg);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		ret = shtps_set_charger_armor(ts, (int)arg);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return ret;
}

static int shtps_ioctl_set_wireless_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, (u8)arg) == 0){
			shtps_charger_armor_proc(ts, (u8)arg);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		ret = shtps_set_charger_armor(ts, (int)arg);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return ret;
}

#if defined( SHTPS_LPWG_MODE_ENABLE )
static int shtps_ioctl_lpwg_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG, (u8)arg) == 0){
			shtps_ioctl_setlpwg_proc(ts, (u8)arg);
		}
	#else
	    mutex_lock(&shtps_ctrl_lock);
		ts->lpwg.lpwg_state = arg;
		ts->lpwg.notify_enable = 1;
	    SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_state = %d\n", ts->lpwg.lpwg_state);
	    if (SHTPS_STATE_SLEEP == ts->state_mgr.state) {
			u8 new_setting = shtps_is_lpwg_active(ts);
			
			if(new_setting != ts->lpwg.lpwg_switch){
				if(new_setting){
					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
						shtps_irq_enable(ts);
					#else
						shtps_irq_wake_enable(ts);
					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_system_set_wakeup(ts);
					shtps_set_lpwg_mode_on(ts);
				}else{
					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
						shtps_irq_disable(ts);
					#else
						shtps_irq_wake_disable(ts);
					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_set_lpwg_mode_off(ts);
					shtps_sleep(ts, 1);
					shtps_system_set_sleep(ts);
				}
			}
		}
	    mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return 0;
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

static int shtps_ioctl_set_veilview_state(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_get_veilview_pattern(struct shtps_rmi_spi *ts)
{
	return SHTPS_VEILVIEW_PATTERN;
}

static int shtps_ioctl_baseline_offset_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_baseline_offset_disable(ts);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return 0;
}

static int shtps_ioctl_set_narrow_frame_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
	{
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		if(arg == 0){
			ts->edge_fail_touch_enable = 0;
		}else{
			ts->edge_fail_touch_enable = 1;
		}

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	}
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	return 0;
}

static int shtps_ioctl_get_serial_number(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret;
	u8 serial_number[7];

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ret = shtps_get_serial_number(ts, serial_number);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	if(!ret){
		if(copy_to_user((u8*)arg, serial_number, sizeof(serial_number))){
			SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
			return -EFAULT;
		}
	}
	
	return (ret == 0)? 0 : -EFAULT;
}

/* -----------------------------------------------------------------------------------
 */
static int shtpsif_open(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_OPEN, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"Open(PID:%ld)\n", sys_getpid());
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_RELEASE, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"Close(PID:%ld)\n", sys_getpid());
	return 0;
}

static ssize_t shtpsif_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int i;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 	fingerMax;
	struct shtps_touch_info info;

	_log_msg_sync( LOGMSG_ID__DEVICE_READ, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}
	fingerMax = shtps_get_fingermax(ts);

	wait_event_interruptible(ts->diag.wait, ts->diag.event == 1);

	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	if(copy_to_user((u8*)buf, (u8*)&info, sizeof(info))){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}

	ts->diag.event = 0;
	_log_msg_sync( LOGMSG_ID__DEVICE_READ_DONE, "");
	return sizeof(ts->report_info);
}

static unsigned int shtpsif_poll(struct file *file, poll_table *wait)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_FAIL, "");
		return POLLERR;
	}

	ret = wait_event_interruptible_timeout(ts->diag.wait,
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 != ret){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "%d", POLLIN | POLLRDNORM);
		return POLLIN | POLLRDNORM;
	}

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "0");
	return 0;
}

static long shtpsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL, "%ld|0x%08X|0x%lX", sys_getpid(), cmd, arg);
	SHTPS_LOG_DBG_PRINT(TPS_ID1"ioctl(PID:%ld,CMD:0x%08X(%d),ARG:0x%lX)\n",
											sys_getpid(), cmd, cmd & 0xff, arg);

	if(ts == NULL){
		SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_ioctl ts == NULL\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		return -EFAULT;
	}

	switch(cmd){
	case TPSDEV_ENABLE: 				rc = shtps_ioctl_enable(ts);					break;
	case TPSDEV_DISABLE:				rc = shtps_ioctl_disable(ts);					break;
	case TPSDEV_RESET:					rc = shtps_ioctl_reset(ts);						break;
	case TPSDEV_SOFT_RESET:				rc = shtps_ioctl_softreset(ts);					break;
	case TPSDEV_GET_FW_VERSION:			rc = shtps_ioctl_getver(ts, arg);				break;
	case TPSDEV_ENTER_BOOTLOADER:		rc = shtps_ioctl_enter_bootloader(ts, arg);		break;
	case TPSDEV_LOCKDOWN_BOOTLOADER:	rc = shtps_ioctl_lockdown_bootloader(ts, arg);	break;
	case TPSDEV_ERASE_FLASE:			rc = shtps_ioctl_erase_flash(ts, arg);			break;
	case TPSDEV_WRITE_IMAGE:			rc = shtps_ioctl_write_image(ts, arg);			break;
	case TPSDEV_WRITE_CONFIG:			rc = shtps_ioctl_write_config(ts, arg);			break;
	case TPSDEV_GET_TOUCHINFO:			rc = shtps_ioctl_get_touchinfo(ts, arg);		break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS:	rc = shtps_ioctl_get_touchinfo_untrans(ts, arg);break;
	case TPSDEV_SET_TOUCHMONITOR_MODE:	rc = shtps_ioctl_set_touchinfo_mode(ts, arg);	break;
	case TPSDEV_READ_REG:				rc = shtps_ioctl_reg_read(ts, arg);				break;
	case TPSDEV_READ_ALL_REG:			rc = shtps_ioctl_reg_allread(ts, arg);			break;
	case TPSDEV_WRITE_REG:				rc = shtps_ioctl_reg_write(ts, arg);			break;
	case TPSDEV_START_TM:				rc = shtps_ioctl_tm_start(ts, arg);				break;
	case TPSDEV_STOP_TM:				rc = shtps_ioctl_tm_stop(ts);					break;
	case TPSDEV_GET_BASELINE:			rc = shtps_ioctl_get_baseline(ts, arg);			break;
	case TPSDEV_GET_FRAMELINE:			rc = shtps_ioctl_get_frameline(ts, arg);		break;
	case TPSDEV_START_FACETOUCHMODE:	rc = shtps_ioctl_start_facetouchmode(ts);		break;
	case TPSDEV_STOP_FACETOUCHMODE:		rc = shtps_ioctl_stop_facetouchmode(ts);		break;
	case TPSDEV_POLL_FACETOUCHOFF:		rc = shtps_ioctl_poll_facetouchoff(ts);			break;
	case TPSDEV_GET_FWSTATUS:			rc = shtps_ioctl_get_fwstatus(ts, arg);			break;
	case TPSDEV_GET_FWDATE:				rc = shtps_ioctl_get_fwdate(ts, arg);			break;
	case TPSDEV_CALIBRATION_PARAM:		rc = shtps_ioctl_calibration_param(ts, arg);	break;
	case TPSDEV_DEBUG_REQEVENT:			rc = shtps_ioctl_debug_reqevent(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP:			rc = shtps_ioctl_set_dragstep(ts, arg);			break;
	case TPSDEV_SET_FINGERFIXTIME:		rc = shtps_ioctl_set_fingerfixtime(ts, arg);	break;
	case TPSDEV_REZERO:					rc = shtps_ioctl_rezero(ts, arg);				break;
	case TPSDEV_ACK_FACETOUCHOFF:		rc = shtps_ioctl_ack_facetouchoff(ts, arg);		break;
	case TPSDEV_START_TM_F05:			rc = shtps_ioctl_tmf05_start(ts, arg);			break;
	case TPSDEV_SET_DRAGSTEP_X:			rc = shtps_ioctl_set_dragstep_x(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP_Y:			rc = shtps_ioctl_set_dragstep_y(ts, arg);		break;
#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	case TPSDEV_LOGOUTPUT_ENABLE:		rc = shtps_ioctl_log_enable(ts, arg);			break;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	case TPSDEV_GET_TOUCHKEYINFO:		rc = shtps_ioctl_get_touchkeyinfo(ts, arg);		break;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	case TPSDEV_GET_FW_VERSION_BUILTIN:	rc = shtps_ioctl_getver_builtin(ts, arg);		break;
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */
#if defined( SHTPS_SMEM_BASELINE_ENABLE )
	case TPSDEV_GET_SMEM_BASELINE:		rc = shtps_ioctl_get_smem_baseline(ts, arg);	break;
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */
	case TPSDEV_SET_LOWPOWER_MODE:		rc = shtps_ioctl_set_low_power_mode(ts, arg);	break;
	case TPSDEV_SET_CONT_LOWPOWER_MODE:	rc = shtps_ioctl_set_continuous_low_power_mode(ts, arg); break;
	case TPSDEV_SET_LCD_LOWPOWER_MODE:	rc = shtps_ioctl_set_lcd_bright_low_power_mode(ts, arg);	break;
	case TPSDEV_SET_CHARGER_ARMOR:		rc = shtps_ioctl_set_charger_armor(ts, arg); break;
	case TPSDEV_SET_WIRELESS_CHARGER_ARMOR:	rc = shtps_ioctl_set_wireless_charger_armor(ts, arg); break;
#if defined( SHTPS_LPWG_MODE_ENABLE )
	case TPSDEV_LPWG_ENABLE:			rc = shtps_ioctl_lpwg_enable(ts, arg); 			break;
#endif /* SHTPS_LPWG_MODE_ENABLE */
	case TPSDEV_SET_VEILVIEW_STATE:		rc = shtps_ioctl_set_veilview_state(ts, arg);	break;
#if defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
	case TPSDEV_READ_REG_BLOCK:			rc = shtps_ioctl_reg_read_block(ts, arg);		break;
	case TPSDEV_WRITE_REG_BLOCK:		rc = shtps_ioctl_reg_write_block(ts, arg);		break;
#endif /* SHTPS_SPI_BLOCKACCESS_ENABLE */
	case TPSDEV_GET_VEILVIEW_PATTERN:	rc = shtps_ioctl_get_veilview_pattern(ts);		break;
#if defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
	case TPSDEV_READ_REG_PACKET:		rc = shtps_ioctl_reg_read_block(ts, arg);		break;
	case TPSDEV_WRITE_REG_PACKET:		rc = shtps_ioctl_reg_write_block(ts, arg);		break;
#endif /* SHTPS_SPI_BLOCKACCESS_ENABLE */
	case TPSDEV_GET_BASELINE_RAW:		rc = shtps_ioctl_get_baseline_raw(ts, arg);		break;
	case TPSDEV_BASELINE_OFFSET_DISABLE:rc = shtps_ioctl_baseline_offset_disable(ts); 	break;
	case TPSDEV_SET_NARROW_FRAME_MODE:	rc = shtps_ioctl_set_narrow_frame_mode(ts, arg);	break;
	case TPSDEV_GET_SERIAL_NUMBER:		rc = shtps_ioctl_get_serial_number(ts, arg);	break;
	default:
		SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_ioctl switch(cmd) default\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		rc = -ENOIOCTLCMD;
		break;
	}
	
	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_DONE, "%d", rc);
	return rc;
}

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.read    = shtpsif_read,
	.poll    = shtpsif_poll,
	.unlocked_ioctl   = shtpsif_ioctl,
};

int __init shtpsif_init(void)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__DEVICE_INIT, "");
	rc = alloc_chrdev_region(&shtpsif_devid, 0, 1, SH_TOUCH_IF_DEVNAME);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:alloc_chrdev_region error\n");
		return rc;
	}

	shtpsif_class = class_create(THIS_MODULE, SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_class)) {
		rc = PTR_ERR(shtpsif_class);
		SHTPS_LOG_ERR_PRINT("shtpsif:class_create error\n");
		goto error_vid_class_create;
	}

	shtpsif_device = device_create(shtpsif_class, NULL,
								shtpsif_devid, &shtpsif_cdev,
								SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_device)) {
		rc = PTR_ERR(shtpsif_device);
		SHTPS_LOG_ERR_PRINT("shtpsif:device_create error\n");
		goto error_vid_class_device_create;
	}

	cdev_init(&shtpsif_cdev, &shtpsif_fileops);
	shtpsif_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shtpsif_cdev, shtpsif_devid, 1);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:cdev_add error\n");
		goto err_via_cdev_add;
	}

	SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_init() done\n");
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_DONE, "");

	return 0;

err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_FAIL, "");

	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);

	_log_msg_sync( LOGMSG_ID__DEVICE_EXIT_DONE, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_exit() done\n");
}
module_exit(shtpsif_exit);

/* -----------------------------------------------------------------------------------
   -----------------------------------------------------------------------------------
 */
static int shtps_numStrToList(
	const char	*numStr,		/* [I  ] num strings */
	int			*numList,		/* [I/O] num list */
	int			numListMaxSize	/* [I/ ] num list size */
)
{
	int			i;
	int			numListNum;
	int			isParam;
	char		buf[10];
	int			buf_current;
	int			num;
	int			rc;

	if((numStr == NULL) || (numList == NULL) || (numListMaxSize < 1)){
		return 0;
	}

	numListNum = 0;
	isParam = 0;
	buf_current = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if((numStr[i] == '\0') || (numStr[i] == '\n') || (numStr[i] == ',') || (numStr[i] == ' ') || (numStr[i] == '\t')){
			if(isParam == 1){
				buf[buf_current] = '\0';
				rc = kstrtoint(buf, 0, &num);
				if(rc == 0){
					numList[numListNum] = num;
				}
				else{
					/* Conversion failure */
					return 0;
				}
				numListNum++;
			}
			isParam = 0;
			if(numListNum >= numListMaxSize){
				break;
			}
			if( (numStr[i] == '\0') ){
				break;
			}
		}
		else{
			if(isParam == 0){
				isParam = 1;
				buf_current = 0;
			}
			buf[buf_current] = numStr[i];
			buf_current++;
			if(buf_current >= sizeof(buf)){
				/* Conversion failure */
				return 0;
			}
		}
	}

	return numListNum;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_common(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return -EINVAL;
}

static ssize_t shtpsif_store_common(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gShtpsIfLog);
}
static ssize_t shtpsif_store_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc >= 1){
		gShtpsIfLog = argv[0];
	}
	else{
		return -EINVAL;
	}

	return count;
}
SHTPSIF_DEFINE(shtpsiflog, shtpsif_show_shtpsiflog, shtpsif_store_shtpsiflog);

static ssize_t shtpsif_store_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
	#else
		ret = shtps_start(ts);
		shtps_wait_startup(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(enable, SHTPSIF_SHOW_COMMON, shtpsif_store_enable);

static ssize_t shtpsif_store_disable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);
	#else
		shtps_shutdown(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(disable, SHTPSIF_SHOW_COMMON, shtpsif_store_disable);

static ssize_t shtpsif_store_reset(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPSIF_LOG_FUNC_CALL();

	if(power == 0){
		shtps_irq_enable(ts);
	}

	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return count;
}
SHTPSIF_DEFINE(reset, SHTPSIF_SHOW_COMMON, shtpsif_store_reset);

static ssize_t shtpsif_store_softreset(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPSIF_LOG_FUNC_CALL();

	if(power == 0){
		shtps_irq_enable(ts);
	}

	if(shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return count;
}
SHTPSIF_DEFINE(softreset, SHTPSIF_SHOW_COMMON, shtpsif_store_softreset);

static ssize_t shtpsif_show_fwver(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8  tps_stop_request = 0;
	u16 ver;

	SHTPSIF_LOG_FUNC_CALL();

	if(ts->state_mgr.state == SHTPS_STATE_IDLE){
		tps_stop_request = 1;
	}

	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERR_PRINT("[%s] error - shtps_start()\n", __func__);
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ver = shtps_fwver(ts);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	if(tps_stop_request){
		shtps_shutdown(ts);
	}

	SHTPS_LOG_DBG_PRINT("[%s] version = 0x%04x\n", __func__, ver);

	return snprintf(buf, PAGE_SIZE, "%04X\n", ver);
}
SHTPSIF_DEFINE(fwver, shtpsif_show_fwver, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", shtps_fwupdate_datasize);
}
static ssize_t shtpsif_store_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	if((shtps_fwupdate_datasize + count) > sizeof(shtps_fwupdate_buffer)){
		return -ENOMEM;
	}
	else{
		memcpy(&shtps_fwupdate_buffer[shtps_fwupdate_datasize], buf, count);
		shtps_fwupdate_datasize += count;
	}

	return count;
}
SHTPSIF_DEFINE(fwupdate_buffer, shtpsif_show_fwupdate_buffer, shtpsif_store_fwupdate_buffer);

static ssize_t shtpsif_store_clear_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	shtps_fwupdate_datasize = 0;

	return count;
}
SHTPSIF_DEFINE(clear_fwupdate_buffer, SHTPSIF_SHOW_COMMON, shtpsif_store_clear_fwupdate_buffer);

static ssize_t shtpsif_show_enter_bootloader(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;
	struct shtps_bootloader_info info;

	SHTPSIF_LOG_FUNC_CALL();

	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);

	if(0 == rc){
		info.block_size        = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
		info.program_block_num = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
		info.config_block_num  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);

		memcpy(buf, (u8*)&info, sizeof(struct shtps_bootloader_info));
	}

	if(rc){
		return -EFAULT;
	}

	return sizeof(struct shtps_bootloader_info);
}
SHTPSIF_DEFINE(enter_bootloader, shtpsif_show_enter_bootloader, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_store_lockdown_bootloader(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {

			if(count == 4){
				int position = 0;
				memcpy(&position, buf, sizeof(position));
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_lockdown_bootloader(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal.\n", __func__);
			rc = shtps_lockdown_bootloader(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_lockdown_bootloader(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(lockdown_bootloader, SHTPSIF_SHOW_COMMON, shtpsif_store_lockdown_bootloader);

static ssize_t shtpsif_store_erase_flash(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_flash_erase(ts);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(erase_flash, SHTPSIF_SHOW_COMMON, shtpsif_store_erase_flash);

static ssize_t shtpsif_store_write_image(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {
			if(count == 4){
				int position = 0;
				memcpy(&position, buf, sizeof(position));
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_flash_writeImage(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
			rc = shtps_flash_writeImage(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_flash_writeImage(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(write_image, SHTPSIF_SHOW_COMMON, shtpsif_store_write_image);

static ssize_t shtpsif_store_write_config(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {
			if(count == 4){
				int position = 0;
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
				memcpy(&position, buf, sizeof(position));

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_flash_writeConfig(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
			rc = shtps_flash_writeConfig(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_flash_writeConfig(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(write_config, SHTPSIF_SHOW_COMMON, shtpsif_store_write_config);

static ssize_t shtpsif_show_touchinfo(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int i;
	u8 	fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	SHTPSIF_LOG_FUNC_CALL();

	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	memcpy(buf, (u8*)&info, sizeof(info));

	return sizeof(info);
}
SHTPSIF_DEFINE(touchinfo, shtpsif_show_touchinfo, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_touchinfo_untrans(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	memcpy(buf, (u8*)&ts->report_info, sizeof(ts->report_info));

	return sizeof(ts->report_info);
}
SHTPSIF_DEFINE(touchinfo_untrans, shtpsif_show_touchinfo_untrans, SHTPSIF_STORE_COMMON);


static ssize_t shtpsif_store_touchinfo_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	ts->diag.pos_mode = argv[0];

	return count;
}
SHTPSIF_DEFINE(touchinfo_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_touchinfo_mode);

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static ssize_t shtpsif_show_touchkeyinfo(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;
	struct shtps_touch_key_info info;

	SHTPSIF_LOG_FUNC_CALL();

	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event_touchkey == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(ret != 0){
		info.up_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_UP) & 0x01);
		info.down_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01);

		ts->diag.event_touchkey = 0;

		memcpy(buf, (u8*)&info, sizeof(info));

		return sizeof(info);
	}

	return -EFAULT;
}
SHTPSIF_DEFINE(touchkeyinfo, shtpsif_show_touchkeyinfo, SHTPSIF_STORE_COMMON);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static ssize_t shtpsif_show_reg_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 data[SHTPSIF_REG_READ_SIZE_MAX];
	char word[10];
	int i;
	int read_addr = shtpsif_reg_read_addr;
	int read_size = shtpsif_reg_read_size;

	SHTPSIF_LOG_FUNC_CALL();

	if(read_size > sizeof(data)){
		read_size = sizeof(data);
	}
	if(shtps_rmi_read_packet(ts, read_addr, (u8*)&data, read_size)){
		return -EFAULT;
	}
	else{
		snprintf(buf, PAGE_SIZE, "0x%04X", read_addr);
		for(i = 0; i < read_size; i++){
			snprintf(word, sizeof(word), ",0x%02X", data[i]);
			strcat(buf, word);
		}
		strcat(buf, "\n");
	}

	return strlen(buf);
}
static ssize_t shtpsif_store_reg_read(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[2];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtpsif_reg_read_addr = argv[0];
	if(argc >= 2){
		shtpsif_reg_read_size = argv[1];
		if(shtpsif_reg_read_size > SHTPSIF_REG_READ_SIZE_MAX){
			shtpsif_reg_read_size = SHTPSIF_REG_READ_SIZE_MAX;
		}
	}
	else{
		shtpsif_reg_read_size = 1;
	}

	return count;
}
SHTPSIF_DEFINE(reg_read, shtpsif_show_reg_read, shtpsif_store_reg_read);

static ssize_t shtpsif_store_reg_write(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	unsigned short addr;
	u8 data[SHTPSIF_REG_WRITE_SIZE_MAX];
	int argc;
	int argv[1 + SHTPSIF_REG_WRITE_SIZE_MAX];
	int datanum;
	int i;

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	datanum = argc - 1;

	if(datanum <= 0){
		return -EINVAL;
	}

	addr = argv[0];

	for(i = 0; i < datanum; i++){
		data[i] = argv[1 + i];
	}

	if(shtps_rmi_write_packet(ts, addr, data, datanum)){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(reg_write, SHTPSIF_SHOW_COMMON, shtpsif_store_reg_write);

static ssize_t shtpsif_store_tm_start(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	if(ts->map.fn05.enable != 0){
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V02;
	}else{
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V03;
	}

	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, argv[0])){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(tm_start, SHTPSIF_SHOW_COMMON, shtpsif_store_tm_start);

static ssize_t shtpsif_store_tm_stop(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(tm_stop, SHTPSIF_SHOW_COMMON, shtpsif_store_tm_stop);

static ssize_t shtpsif_show_baseline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(baseline, shtpsif_show_baseline, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_baseline_raw(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE_RAW);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(baseline_raw, shtpsif_show_baseline_raw, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_frameline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	#if defined( SHTPS_SY_REGMAP_BASE3 )
		size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	#else
		size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts);
	#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(frameline, shtpsif_show_frameline, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_poll_touch_info(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int i;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 	fingerMax;
	struct shtps_touch_info info;

//	SHTPSIF_LOG_FUNC_CALL();

	if(NULL == ts){
		return POLLERR;
	}

	ret = wait_event_interruptible_timeout(ts->diag.wait,
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 == ret){
		return 0;
	}

	fingerMax = shtps_get_fingermax(ts);

	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	memcpy((u8*)buf, (u8*)&info, sizeof(info));

	ts->diag.event = 0;
	return sizeof(ts->report_info);
}
SHTPSIF_DEFINE(poll_touch_info, shtpsif_show_poll_touch_info, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_store_start_facetouchmode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(start_facetouchmode, SHTPSIF_SHOW_COMMON, shtpsif_store_start_facetouchmode);

static ssize_t shtpsif_store_stop_facetouchmode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	ts->facetouch.wake_sig = 1;
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_facetouch_wakelock(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(stop_facetouchmode, SHTPSIF_SHOW_COMMON, shtpsif_store_stop_facetouchmode);

static ssize_t shtpsif_show_poll_facetouchoff(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;

		SHTPS_LOG_DBG_PRINT("wait for face touch off detect\n");
		rc = wait_event_interruptible(ts->facetouch.wait_off,
			(ts->facetouch.detect == 1) || (ts->facetouch.mode == 0) ||
			(ts->facetouch.wake_sig == 1));

		ts->facetouch.wake_sig = 0;

		if(ts->facetouch.detect){
			SHTPS_LOG_DBG_PRINT("face touch %s detect\n", (ts->facetouch.state == 0)? "off" : "on");
			rc = (ts->facetouch.state == 0)? TPSDEV_FACETOUCH_OFF_DETECT : TPSDEV_FACETOUCH_DETECT;
			ts->facetouch.detect = 0;
		}else{
			SHTPS_LOG_DBG_PRINT("wait for facetouch was cancelled\n");
			rc = TPSDEV_FACETOUCH_NOCHG;
		}
	}
	#else
		rc = TPSDEV_FACETOUCH_NOCHG;
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return snprintf(buf, PAGE_SIZE, "%d\n", rc);
}
SHTPSIF_DEFINE(poll_facetouchoff, shtpsif_show_poll_facetouchoff, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_store_calibration_param(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	const u8 *data;

	SHTPSIF_LOG_FUNC_CALL();

	if(count > sizeof(struct shtps_offset_info)){
		return -EINVAL;
	}
	data = buf;

	memcpy(ts->offset.base, data, sizeof(u16) * 5);
	ts->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);

	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}

	return count;
}
SHTPSIF_DEFINE(calibration_param, SHTPSIF_SHOW_COMMON, shtpsif_store_calibration_param);

static ssize_t shtpsif_store_rezero(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return count;
}
SHTPSIF_DEFINE(rezero, SHTPSIF_SHOW_COMMON, shtpsif_store_rezero);

static ssize_t shtpsif_store_ack_facetouchoff(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;

		mutex_lock(&shtps_ctrl_lock);
		shtps_facetouch_wakelock(ts, 0);
		mutex_unlock(&shtps_ctrl_lock);
	}
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return count;
}
SHTPSIF_DEFINE(ack_facetouchoff, SHTPSIF_SHOW_COMMON, shtpsif_store_ack_facetouchoff);

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static ssize_t shtpsif_show_log_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", gLogOutputEnable);
}
static ssize_t shtpsif_store_log_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	gLogOutputEnable = argv[0];

	return count;
}
SHTPSIF_DEFINE(log_enable, shtpsif_show_log_enable, shtpsif_store_log_enable);
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static ssize_t shtpsif_show_fwver_builtin(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u16 ver = shtps_fwver_builtin(ts);

	SHTPSIF_LOG_FUNC_CALL();

	if(shtps_fwdata_builtin(ts) == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in version = 0x%04x\n", __func__, ver);

	return snprintf(buf, PAGE_SIZE, "%04X\n", ver);
}
SHTPSIF_DEFINE(fwver_builtin, shtpsif_show_fwver_builtin, SHTPSIF_STORE_COMMON);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

#if defined( SHTPS_SMEM_BASELINE_ENABLE )
static ssize_t shtpsif_show_smem_baseline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();
	int size = 0;

	SHTPSIF_LOG_FUNC_CALL();

	if(!smemdata){
		return -EFAULT;
	}

	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy((u8*)buf, (u8*)(smemdata->shdiag_TpsBaseLineTbl), size);

	return size;
}
SHTPSIF_DEFINE(smem_baseline, shtpsif_show_smem_baseline, SHTPSIF_STORE_COMMON);
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */

static ssize_t shtpsif_store_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE, argv[0]) == 0){
			shtps_ioctl_setlpmode_proc(ts, argv[0]);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, argv[0]);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_low_power_mode);

static ssize_t shtpsif_store_continuous_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE, argv[0]) == 0){
			shtps_ioctl_setconlpmode_proc(ts, argv[0]);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, argv[0]);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(continuous_low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_continuous_low_power_mode);

static ssize_t shtpsif_store_lcd_bright_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE, argv[0]) == 0){
			shtps_ioctl_setlcdbrightlpmode_proc(ts, argv[0]);
		}
	#else
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, argv[0]);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(lcd_bright_low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_lcd_bright_low_power_mode);

static ssize_t shtpsif_store_charger_armor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, argv[0]) == 0){
			shtps_charger_armor_proc(ts, argv[0]);
		}
	#else
	{
		int ret = -1;
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		ret = shtps_set_charger_armor(ts, argv[0]);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);

		if(ret < 0){
			return ret;
		}
	}
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(charger_armor, SHTPSIF_SHOW_COMMON, shtpsif_store_charger_armor);

static ssize_t shtpsif_store_wireless_charger_armor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, argv[0]) == 0){
			shtps_charger_armor_proc(ts, argv[0]);
		}
	#else
	{
		int ret = -1;
		SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
		mutex_lock(&shtps_ctrl_lock);

		ret = shtps_set_charger_armor(ts, argv[0]);

		SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
		mutex_unlock(&shtps_ctrl_lock);

		if(ret < 0){
			return ret;
		}
	}
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(wireless_charger_armor, SHTPSIF_SHOW_COMMON, shtpsif_store_wireless_charger_armor);

#if defined( SHTPS_LPWG_MODE_ENABLE )
static ssize_t shtpsif_show_lpwg_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->lpwg.lpwg_state);
}
static ssize_t shtpsif_store_lpwg_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

 	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG, argv[0]) == 0){
			shtps_ioctl_setlpwg_proc(ts, argv[0]);
		}
	#else
		mutex_lock(&shtps_ctrl_lock);
		ts->lpwg.lpwg_state = argv[0];
		ts->lpwg.notify_enable = 1;
		SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_state = %d\n", ts->lpwg.lpwg_state);
		if (SHTPS_STATE_SLEEP == ts->state_mgr.state) {
			u8 new_setting = shtps_is_lpwg_active(ts);
			
			if(new_setting != ts->lpwg.lpwg_switch){
				if(new_setting){
					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
						shtps_irq_enable(ts);
					#else
						shtps_irq_wake_enable(ts);
					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_system_set_wakeup(ts);
					shtps_set_lpwg_mode_on(ts);
				}else{
					#if defined(SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE)
						shtps_irq_disable(ts);
					#else
						shtps_irq_wake_disable(ts);
					#endif /* SHTPS_IRQ_LINKED_WITH_IRQWAKE_ENABLE */

					shtps_set_lpwg_mode_off(ts);
					shtps_sleep(ts, 1);
					shtps_system_set_sleep(ts);
				}
			}
		}
		mutex_unlock(&shtps_ctrl_lock);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(lpwg_enable, shtpsif_show_lpwg_enable, shtpsif_store_lpwg_enable);
#endif /* SHTPS_LPWG_MODE_ENABLE */

static ssize_t shtpsif_store_veilview_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	return count;
}
SHTPSIF_DEFINE(veilview_state, SHTPSIF_SHOW_COMMON, shtpsif_store_veilview_state);

static ssize_t shtpsif_show_veilview_pattern(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", SHTPS_VEILVIEW_PATTERN);
}
static ssize_t shtpsif_store_veilview_pattern(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	SHTPS_VEILVIEW_PATTERN = argv[0];
#else /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return -EFAULT;
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return count;
}
SHTPSIF_DEFINE(veilview_pattern, shtpsif_show_veilview_pattern, shtpsif_store_veilview_pattern);

static ssize_t shtpsif_store_baseline_offset_disable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_baseline_offset_disable(ts);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return count;
}
SHTPSIF_DEFINE(baseline_offset_disable, SHTPSIF_SHOW_COMMON, shtpsif_store_baseline_offset_disable);

/* -----------------------------------------------------------------------------------
 */

static struct attribute *attrs_shtpsif[] = {
	&shtpsif_shtpsiflog.attr,
	&shtpsif_enable.attr,
	&shtpsif_disable.attr,
	&shtpsif_reset.attr,
	&shtpsif_softreset.attr,
	&shtpsif_fwver.attr,
	&shtpsif_fwupdate_buffer.attr,
	&shtpsif_clear_fwupdate_buffer.attr,
	&shtpsif_enter_bootloader.attr,
	&shtpsif_lockdown_bootloader.attr,
	&shtpsif_erase_flash.attr,
	&shtpsif_write_image.attr,
	&shtpsif_write_config.attr,
	&shtpsif_touchinfo.attr,
	&shtpsif_touchinfo_untrans.attr,
	&shtpsif_touchinfo_mode.attr,
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		&shtpsif_touchkeyinfo.attr,
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	&shtpsif_reg_read.attr,
	&shtpsif_reg_write.attr,
	&shtpsif_tm_start.attr,
	&shtpsif_tm_stop.attr,
	&shtpsif_baseline.attr,
	&shtpsif_baseline_raw.attr,
	&shtpsif_frameline.attr,
	&shtpsif_poll_touch_info.attr,
	&shtpsif_start_facetouchmode.attr,
	&shtpsif_stop_facetouchmode.attr,
	&shtpsif_poll_facetouchoff.attr,
	&shtpsif_calibration_param.attr,
	&shtpsif_rezero.attr,
	&shtpsif_ack_facetouchoff.attr,
	#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
		&shtpsif_log_enable.attr,
	#endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */
	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
		&shtpsif_fwver_builtin.attr,
	#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */
	#if defined( SHTPS_SMEM_BASELINE_ENABLE )
		&shtpsif_smem_baseline.attr,
	#endif /* SHTPS_SMEM_BASELINE_ENABLE */
	&shtpsif_low_power_mode.attr,
	&shtpsif_continuous_low_power_mode.attr,
	&shtpsif_lcd_bright_low_power_mode.attr,
	&shtpsif_charger_armor.attr,
	&shtpsif_wireless_charger_armor.attr,
	#if defined( SHTPS_LPWG_MODE_ENABLE )
		&shtpsif_lpwg_enable.attr,
	#endif /* SHTPS_LPWG_MODE_ENABLE */
	&shtpsif_veilview_state.attr,
	&shtpsif_veilview_pattern.attr,
	&shtpsif_baseline_offset_disable.attr,
	NULL
};

struct attribute_group shtps_attr_grp_shtpsif = {
	.name = "shtpsif",
	.attrs = attrs_shtpsif,
};
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_open(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	_log_msg_sync( LOGMSG_ID__OPEN, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("Open(PID:%ld)\n", sys_getpid());

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
#else
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
		u8 buf;
		const unsigned char* fw_data = NULL;
		int ver;
		u8 update = 0;

		if(shtps_start(ts) == 0){
			shtps_wait_startup(ts);
		}

		shtps_rmi_read(ts, 0x0013, &buf, 1);

		#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
			ver = shtps_fwver(ts);
			DBG_PRINTK("fw version = 0x%04x\n", ver);
			if(ver != shtps_fwver_builtin(ts)){
				update = 1;
			}
		#else
			if(shtps_fwup_flag_check() > 0){
				ver = shtps_fwver(ts);
				if(ver != shtps_fwver_builtin(ts)){
					update = 1;
				}
			}
		#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

		if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
			#ifdef CONFIG_SHTERM
				shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
				SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (first check)\n");
			#endif
			SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
			update = 1;
		}

		if(update != 0){
			fw_data = shtps_fwdata_builtin(ts);
			if(fw_data){
				int ret;
				int retry = 5;
				do{
					ret = shtps_fw_update(ts, fw_data);
					request_event(ts, SHTPS_EVENT_STOP, 0);
					#ifdef CONFIG_SHTERM
						if (ret != 0) {
							shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
							SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (retry=%d)\n", retry);
						}
					#endif
				}while(ret != 0 && (retry-- > 0));

				#ifdef CONFIG_SHTERM
					if (ret != 0 && retry < 0) {
						shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX);
						SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX\n");
					} else {
						shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX);
                                                    SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX\n");
					}
				#endif
			}
		}
	}
	shtps_fwup_flag_clear();

#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */
	shtps_start(ts);
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return 0;
}

static void shtps_rmi_close(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	_log_msg_sync( LOGMSG_ID__CLOSE, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("Close(PID:%ld)\n", sys_getpid());

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
	#else
		shtps_shutdown(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */
}

static int shtps_init_internal_variables(struct shtps_rmi_spi *ts)
{
	int i;
	int result = 0;
	
	INIT_DELAYED_WORK(&ts->tmo_check, shtps_work_tmof);

	init_waitqueue_head(&ts->wait_start);
	init_waitqueue_head(&ts->loader.wait_ack);
	init_waitqueue_head(&ts->diag.wait);
	init_waitqueue_head(&ts->diag.tm_wait_ack);

	hrtimer_init(&ts->rezero_delayed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->rezero_delayed_timer.function = shtps_delayed_rezero_timer_function;
	INIT_WORK(&ts->rezero_delayed_work, shtps_rezero_delayed_work_function);

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ts->workqueue_p = alloc_workqueue("TPS_WORK", WQ_UNBOUND, 1);
		if(ts->workqueue_p == NULL){
			result = -ENOMEM;
			goto fail_init;
		}
		INIT_WORK( &(ts->work_data), shtps_func_workq );
		INIT_LIST_HEAD( &(ts->queue) );
		spin_lock_init( &(ts->queue_lock) );
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	ts->map.fn01.ctrlBase = SHTPS_F01_RMI_CTRL_ADR;
	ts->state_mgr.state = SHTPS_STATE_IDLE;
	ts->loader.ack = 0;
	ts->diag.event = 0;
	ts->offset.enabled = 0;
	ts->bt_ver = 0;

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_facetouch_init(ts);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		ts->lpmode_req_state = SHTPS_LPMODE_REQ_NONE;
		ts->lpmode_continuous_req_state = SHTPS_LPMODE_REQ_HOVER_OFF;
	#endif /*` #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		ts->charger_armor_state = 0;
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	memset(&ts->poll_info,   0, sizeof(ts->poll_info));
	memset(&ts->fw_report_info, 0, sizeof(ts->fw_report_info));
	memset(&ts->fw_report_info_store, 0, sizeof(ts->fw_report_info_store));
	memset(&ts->report_info, 0, sizeof(ts->report_info));
	memset(&ts->center_info, 0, sizeof(ts->center_info));
	memset(&ts->touch_state, 0, sizeof(ts->touch_state));
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_ZERO;
		ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_ZERO;
		ts->touch_state.dragStepReturnTime[i][0] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
		ts->touch_state.dragStepReturnTime[i][1] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}
	memset(ts->drag_hist, 0, sizeof(ts->drag_hist));

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event             = SHTPS_EVENT_TU;
		ts->perf_lock_enable_time_ms = SHTPS_PERF_LOCK_ENABLE_TIME_MS;
		perf_lock_init(&ts->perf_lock, PERF_LOCK_1190400KHz, "shtps_perf_lock");
		INIT_DELAYED_WORK(&ts->perf_lock_disable_delayed_work, shtps_perf_lock_disable_delayed_work_function);
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		ts->wake_lock_idle_state = 0;
		pm_qos_add_request(&ts->qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
		ts->wake_lock_for_fwupdate_state = 0;
		wake_lock_init(&ts->wake_lock_for_fwupdate, WAKE_LOCK_SUSPEND, "shtps_wake_lock_for_fwupdate");
		pm_qos_add_request(&ts->qos_cpu_latency_for_fwupdate, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	#if defined( SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE )
		shtps_absorption_init(ts);
	#endif /* SHTPS_FINGER_ABSORPTION_PROVISION_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		shtps_lpwg_wakelock_init(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		ts->key_state  = 0;
		ts->diag.event_touchkey = 0;
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		ts->system_boot_mode = sh_boot_get_bootmode();
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		wake_lock_init(&ts->wake_lock_proximity, WAKE_LOCK_SUSPEND, "shtps_wake_lock_proximity");
		pm_qos_add_request(&ts->pm_qos_lock_idle_proximity, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
		INIT_DELAYED_WORK(&ts->proximity_check_delayed_work, shtps_lpwg_proximity_check_delayed_work_function);
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

	#if defined(SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE)
		ts->wakeup_touch_event_inhibit_state = 0;
	#endif /* SHTPS_WAKEUP_FAIL_TOUCH_EVENT_REJECTION_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
	    ts->deter_suspend_spi.wake_lock_state = 0;
		memset(&ts->deter_suspend_spi, 0, sizeof(ts->deter_suspend_spi));
	    wake_lock_init(&ts->deter_suspend_spi.wake_lock, WAKE_LOCK_SUSPEND, "shtps_resume_wake_lock");
		pm_qos_add_request(&ts->deter_suspend_spi.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
		INIT_WORK(&ts->deter_suspend_spi.pending_proc_work, shtps_deter_suspend_spi_pending_proc_work_function);
		#ifdef SHTPS_DEVELOP_MODE_ENABLE
			INIT_DELAYED_WORK(&ts->deter_suspend_spi.pending_proc_work_delay, shtps_deter_suspend_spi_pending_proc_delayed_work_function);
		#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
		ts->edge_fail_touch_enable = 1;
		ts->edge_fail_touch_inhibit_id = 0;
		ts->edge_fail_touch_decide_mt = 0;
		ts->edge_fail_touch_td_cnt = 0;
		ts->edge_fail_touch_side_inhibit_id = 0;
		memset(ts->edge_fail_touch_td_info, 0, sizeof(ts->edge_fail_touch_td_info));
		memset(&ts->edge_fail_touch_side_td_info, 0, sizeof(ts->edge_fail_touch_side_td_info));
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE)
		ts->grip_fail_touch_inhibit_id = 0;
		ts->grip_fail_flick_inhibit_id = 0;
	#endif /* SHTPS_GRIP_FAIL_TOUCH_REJECTION_ENABLE */

	#if defined(SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE)
		memset(&(ts->pinch_fail_reject), 0, sizeof(struct shtps_pinch_fail_reject));
	#endif /* SHTPS_PINCH_FAIL_RESPONSE_REJECTION_ENABLE */

	ts->dev_state = SHTPS_DEV_STATE_SLEEP;

	#if defined(SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE)
		memset(&ts->lgm_split_touch_combining, 0, sizeof(ts->lgm_split_touch_combining));
	#endif /* SHTPS_LGM_SPLIT_TOUCH_COMBINING_ENABLE */

	#if defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE )
		ts->key_down_reserved = 0;
		ts->key_down_ignored = 0;
		ts->key_proximity_check_state = 0;
		INIT_DELAYED_WORK(&ts->touchkey_delayed_work, shtps_touchkey_delayed_work_function);
		INIT_DELAYED_WORK(&ts->touchkey_inproxymity_delayed_work, shtps_touchkey_inproxymity_delayed_work_function);
	#endif /* defined( SHTPS_KEY_PROXIMITY_ASYNC_CHECK_ENABLE ) */

	#if defined(SHTPS_WAKEUP_CALIB_ENABLE)
		ts->wakeup_calib_enable = 0;
		ts->wakeup_calib_cancel_check_enable = 0;
		ts->wakeup_calib_deferment = 0;
		ts->wakeup_calib_lastone = 0;
		ts->wakeup_calib_lastone_req = 0;
		ts->wakeup_calib_rezero_exec_num = 0;
		memset(ts->wakeup_calib_td_pos, 0, sizeof(ts->wakeup_calib_td_pos));
		memset(ts->wakeup_calib_cancel_check, 0, sizeof(ts->wakeup_calib_cancel_check));
		memset(ts->wakeup_calib_event_count, 0, sizeof(ts->wakeup_calib_event_count));
		INIT_DELAYED_WORK(&ts->wakeup_calib_delayed_work, shtps_wakeup_calib_delayed_work_function);
	#endif /* SHTPS_WAKEUP_CALIB_ENABLE */

	#if defined( SHTPS_CLING_REJECTION_ENABLE )
		shtps_cling_reject_init_variables(ts);
	#endif /* SHTPS_CLING_REJECTION_ENABLE */

	#if defined(SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE)
		memset(&ts->lgm_fail_touch_up_reject, 0, sizeof(ts->lgm_fail_touch_up_reject));
		INIT_DELAYED_WORK(&ts->lgm_fail_touch_up_reject.chatt_work, shtps_lgm_fail_touch_reject_chatt_delayed_work_function);
	#endif /* SHTPS_LGM_FAIL_TOUCH_UP_REJECTION_ENABLE */

	#if defined(SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE)
		memset(&ts->edge_shake_td_info, 0, sizeof(ts->edge_shake_td_info));
		memset(ts->edge_shake_check_state, 0, sizeof(ts->edge_shake_check_state));
	#endif /* SHTPS_EDGE_TOUCH_SHAKE_REJECTION_ENABLE */

	#if defined(SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE)
		shtps_quick_flick_fail_resolv_init(ts);
	#endif /* SHTPS_DEF_QUICK_FLICK_FAIL_RESOLV_ENABLE */

	#if defined(SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE)
		shtps_scan_freq_dynamic_change_init(ts);
	#endif /* SHTPS_DEF_SCAN_FREQ_DYNAMIC_CHANGE_ENABLE */

	#if defined(SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
		shtps_filter_topofscreen_ghost_reject_init(ts);
	#endif  /* SHTPS_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

	#if defined( SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE )
		INIT_DELAYED_WORK(&ts->facetouch.notify_interval_delayed_work, shtps_delayed_facetouch_off_notify_function);
	    wake_lock_init(&ts->facetouch.facetouch_off_delayed_wake_lock, WAKE_LOCK_SUSPEND, "shtps_faceoff_delayed_wake_lock");
		pm_qos_add_request(&ts->facetouch.facetouch_off_delayed_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	#endif /* SHTPS_FACETOUCH_OFF_DETECT_CHATTER_CHK_ENABLE */

	shtps_performance_check_init();
	return 0;

fail_init:
	if(ts->workqueue_p){
		destroy_workqueue(ts->workqueue_p);
	}
	return result;
}


static void shtps_deinit_internal_variables(struct shtps_rmi_spi *ts)
{
	if(ts){
		hrtimer_cancel(&ts->rezero_delayed_timer);

		#if defined( SHTPS_ASYNC_OPEN_ENABLE )
			if(ts->workqueue_p)
				destroy_workqueue(ts->workqueue_p);
		#endif /* SHTPS_ASYNC_OPEN_ENABLE */

		#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
			pm_qos_remove_request(&ts->qos_cpu_latency);
		#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

		#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
			pm_qos_remove_request(&ts->qos_cpu_latency_for_fwupdate);
			wake_lock_destroy(&ts->wake_lock_for_fwupdate);
		#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

		#if defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_lpwg_wakelock_destroy(ts);
		#endif /* SHTPS_LPWG_MODE_ENABLE */

		#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
			wake_lock_destroy(&ts->wake_lock_proximity);
		#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
	}
}

static int shtps_init_inputdev(struct shtps_rmi_spi *ts)
{
	ts->input = input_allocate_device();
	if (!ts->input){
		SHTPS_LOG_ERR_PRINT("Failed input_allocate_device\n");
		return -ENOMEM;
	}

	ts->input->name 		= ts->spi->modalias;
	ts->input->phys         = ts->phys;
	ts->input->id.vendor	= 0x0001;
	ts->input->id.product	= 0x0002;
	ts->input->id.version	= 0x0100;
	ts->input->dev.parent	= &ts->spi->dev;
	ts->input->open			= shtps_rmi_open;
	ts->input->close		= shtps_rmi_close;

	/** set properties */
	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);
	
	#if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY )
		__set_bit(KEY_PROG1, ts->input->keybit);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY ) */

	input_set_drvdata(ts->input, ts);
	input_mt_init_slots(ts->input, SHTPS_FINGER_MAX);

	if(ts->input->mt == NULL){
		input_free_device(ts->input);
		return -ENOMEM;
	}

	/** set parameters */
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 50, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE,    0, 255, 0, 0);

	/** register input device */
	if(input_register_device(ts->input) != 0){
		input_free_device(ts->input);
		return -EFAULT;
	}
	
	return 0;
}

static void shtps_deinit_inputdev(struct shtps_rmi_spi *ts)
{
	if(ts && ts->input){
		if(ts->input->mt){
			input_mt_destroy_slots(ts->input);
		}
		input_free_device(ts->input);
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
static int shtps_init_inputdev_key(struct shtps_rmi_spi *ts)
{
	ts->input_key = input_allocate_device();
	if(!ts->input_key){
		return -ENOMEM;
	}

	ts->input_key->name 		= "shtps_key";
	ts->input_key->phys         = ts->phys;
	ts->input_key->id.vendor	= 0x0000;
	ts->input_key->id.product	= 0x0000;
	ts->input_key->id.version	= 0x0000;
	ts->input_key->dev.parent	= &ts->spi->dev;

	__set_bit(EV_KEY, ts->input_key->evbit);

	input_set_drvdata(ts->input_key, ts);

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		ts->input_key->keycode = ts->keycodes;
		ts->input_key->keycodemax = SHTPS_PHYSICAL_KEY_NUM;
		ts->input_key->keycodesize = sizeof(ts->keycodes);
		ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN] = KEY_VOLUMEDOWN;
		ts->keycodes[SHTPS_PHYSICAL_KEY_UP] = KEY_VOLUMEUP;

		__set_bit(KEY_VOLUMEDOWN, ts->input_key->keybit);
		__set_bit(KEY_VOLUMEUP, ts->input_key->keybit);

		input_set_capability(ts->input_key, EV_MSC, MSC_SCAN);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		__set_bit(KEY_SWEEPON, ts->input_key->keybit);
	#endif /*  SHTPS_LPWG_MODE_ENABLE */
	
	__clear_bit(KEY_RESERVED, ts->input_key->keybit);

	if(input_register_device(ts->input_key)){
		input_free_device(ts->input_key);
		return -EFAULT;
	}
	
	return 0;
}

static void shtps_deinit_inputdev_key(struct shtps_rmi_spi *ts)
{
	if(ts && ts->input_key){
		input_free_device(ts->input_key);
	}
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

static void shtps_init_debugfs(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_CREATE_KOBJ_ENABLE)
		if(ts->kobj != NULL){
			if(sysfs_create_group(ts->kobj, &attr_grp_ctrl)){
				SHTPS_LOG_ERR_PRINT("kobj create failed : attr_grp_ctrl\n");
			}
		}
	#endif /* SHTPS_CREATE_KOBJ_ENABLE */
}

static void shtps_deinit_debugfs(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_CREATE_KOBJ_ENABLE)
		if(ts->kobj != NULL){
			sysfs_remove_group(ts->kobj, &attr_grp_ctrl);
			kobject_put(ts->kobj);
		}
	#endif /* SHTPS_CREATE_KOBJ_ENABLE */
}

static int __devinit shtps_rmi_probe(struct spi_device *spi)
{
	int result = 0;
	struct shtps_rmi_spi *ts;
	#ifndef CONFIG_OF
		struct shtps_platform_data *pdata = spi->dev.platform_data;
	#endif /* !CONFIG_OF */

	mutex_lock(&shtps_ctrl_lock);
	
	ts = kzalloc(sizeof(struct shtps_rmi_spi), GFP_KERNEL);
	if(!ts){
		SHTPS_LOG_ERR_PRINT("memory allocation error\n");
		result = -ENOMEM;
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}
	spi_set_drvdata(spi, ts);

	if(shtps_init_internal_variables(ts)){
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_init_internal_variables;
	}

	/** set device info */
	gShtps_rmi_spi	= ts;
	ts->spi			= spi;

	#ifdef CONFIG_OF
		ts->irq_mgr.irq	= irq_of_parse_and_map(spi->dev.of_node, 0);
		ts->rst_pin		= of_get_named_gpio(spi->dev.of_node, "shtps_rmi,rst_pin", 0);
	#else
		ts->rst_pin		= pdata->gpio_rst;
		ts->irq_mgr.irq	= spi->irq;
	#endif /* CONFIG_OF */

    if(!gpio_is_valid(ts->rst_pin)){
		SHTPS_LOG_ERR_PRINT("gpio resource error\n");
		result = -EFAULT;
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_get_dtsinfo;
	}

	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(&spi->dev));
	
	/** setup device */
	#ifdef CONFIG_OF
		result = shtps_device_setup(ts->irq_mgr.irq, ts->rst_pin);
		if(result){
			SHTPS_LOG_ERR_PRINT("Filed shtps_device_setup\n");
			mutex_unlock(&shtps_ctrl_lock);
			goto fail_device_setup;
		}
	#else
		if (pdata && pdata->setup) {
			result = pdata->setup(&spi->dev);
			if (result){
				mutex_unlock(&shtps_ctrl_lock);
				goto fail_alloc_mem;
			}
		}
	#endif /* !CONFIG_OF */
	
	if(shtps_irq_resuest(ts)){
		result = -EFAULT;
		SHTPS_LOG_ERR_PRINT("shtps:request_irq error\n");
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_irq_request;
	}
	
	mutex_unlock(&shtps_ctrl_lock);


	/** init device info */
	result = spi_setup(ts->spi);
	if(result < 0){
		SHTPS_LOG_DBG_PRINT("spi_setup fail\n");
	}
	
	result = shtps_init_inputdev(ts);
	if(result != 0){
		SHTPS_LOG_DBG_PRINT("Failed init input device\n");
		goto fail_init_inputdev;
	}
	
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
		result = shtps_init_inputdev_key(ts);
		if(result != 0){
			SHTPS_LOG_DBG_PRINT("Failed init input key-device\n");
			goto fail_init_inputdev_key;
		}
	#endif /* defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

	/** init sysfs I/F */
	ts->kobj = kobject_create_and_add("shtps", kernel_kobj);
	if(ts->kobj == NULL){
		SHTPS_LOG_ERR_PRINT("kobj create failed : shtps\n");
	}else{
		if(sysfs_create_group(ts->kobj, &shtps_attr_grp_shtpsif)){
			SHTPS_LOG_ERR_PRINT("kobj create failed : shtps_attr_grp_shtpsif\n");
		}
	}

	/** init debug fs */
	shtps_init_debugfs(ts);

	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
	{
		struct shtps_debug_init_param param;
		param.shtps_root_kobj = ts->kobj;

		/** init debug function data */
		shtps_debug_init(&param);
	}
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		#if defined(SHTPS_ASYNC_OPEN_ENABLE)
		    shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
		#else
			shtps_start(ts);
			shtps_wait_startup(ts);
		#endif /* SHTPS_ASYNC_OPEN_ENABLE */
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	SHTPS_LOG_DBG_PRINT("shtps_rmi_probe() done\n");
	return 0;

#if defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE)
fail_init_inputdev_key:
	input_free_device(ts->input);
#endif /* defined( SHTPS_PHYSICAL_KEY_ENABLE ) || defined(SHTPS_LPWG_MODE_ENABLE) */

fail_init_inputdev:
fail_irq_request:
fail_get_dtsinfo:
	shtps_deinit_internal_variables(ts);

fail_init_internal_variables:
fail_device_setup:
	kfree(ts);
	
fail_alloc_mem:
	return result;
}

static int __devexit shtps_rmi_remove(struct spi_device *spi)
{
	struct shtps_rmi_spi *ts = spi_get_drvdata(spi);
	#ifndef CONFIG_OF
		struct shtps_platform_data *pdata = spi->dev.platform_data;
	#endif /* !CONFIG_OF */

	gShtps_rmi_spi = NULL;
	
	
	if(ts){
		free_irq(ts->irq_mgr.irq, ts);

		#ifdef CONFIG_OF
			shtps_device_teardown(ts->irq_mgr.irq, ts->rst_pin);
		#else
			if (pdata && pdata->teardown){
				pdata->teardown(&spi->dev);
			}
		#endif /* CONFIG_OF */

		shtps_deinit_internal_variables(ts);
		#if defined( SHTPS_DEVELOP_MODE_ENABLE )
			shtps_debug_deinit();
		#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
		shtps_deinit_debugfs(ts);

		if(ts->kobj != NULL){
			sysfs_remove_group(ts->kobj, &shtps_attr_grp_shtpsif);
			kobject_put(ts->kobj);
		}

		shtps_deinit_inputdev(ts);
		#if defined(SHTPS_PHYSICAL_KEY_ENABLE) || defined(SHTPS_LPWG_MODE_ENABLE)
			shtps_deinit_inputdev_key(ts);
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE || SHTPS_LPWG_MODE_ENABLE */

		kfree(ts);
	}
	
	_log_msg_sync( LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_rmi_remove() done\n");
	return 0;
}

static int shtps_rmi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		shtps_set_suspend_state(gShtps_rmi_spi);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	#if defined(SHTPS_SYSTEM_HOT_STANDBY_ENABLE)
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;

		_log_msg_sync( LOGMSG_ID__SUSPEND, "");
		SHTPS_LOG_FUNC_CALL();

		shtps_device_access_teardown(ts);
		request_event(ts, SHTPS_EVENT_SLEEP, 0);
	}
	#else
		_log_msg_sync( LOGMSG_ID__SUSPEND, "");
	#endif /* SHTPS_SYSTEM_HOT_STANDBY_ENABLE */

	return 0;
}

static int shtps_rmi_resume(struct spi_device *spi)
{
	#if defined(SHTPS_SYSTEM_HOT_STANDBY_ENABLE)
		_log_msg_sync( LOGMSG_ID__RESUME, "");
		SHTPS_LOG_FUNC_CALL();
	#else
		_log_msg_sync( LOGMSG_ID__RESUME, "");
	#endif /* SHTPS_SYSTEM_HOT_STANDBY_ENABLE */

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		shtps_clr_suspend_state(gShtps_rmi_spi);
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return 0;
}

#ifdef CONFIG_OF // Open firmware must be defined for dts useage
static struct of_device_id shtps_rmi_table [] = {
	{ . compatible = "sharp,shtps_rmi" ,}, // Compatible node must match dts
	{ },
};
#else
#define	shtps_rmi_table	NULL
#endif

static struct spi_driver shtps_rmi_driver = {
	.probe = shtps_rmi_probe,
	.remove = __devexit_p(shtps_rmi_remove),
	.suspend = shtps_rmi_suspend,
	.resume = shtps_rmi_resume,
	.driver = {
		.of_match_table = shtps_rmi_table,
		.name = "shtps_rmi",
		.owner = THIS_MODULE,
	},
};

static int __init shtps_rmi_init(void)
{
	SHTPS_LOG_DBG_PRINT("shtps_rmi_init() start\n");
	_log_msg_sync( LOGMSG_ID__INIT, "");

	return spi_register_driver(&shtps_rmi_driver);
}
module_init(shtps_rmi_init);

static void __exit shtps_rmi_exit(void)
{
	spi_unregister_driver(&shtps_rmi_driver);

	SHTPS_LOG_DBG_PRINT("shtps_rmi_exit() done\n");
	_log_msg_sync( LOGMSG_ID__EXIT, "");
}
module_exit(shtps_rmi_exit);

/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int on)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__API_SLEEP, "%d", on);
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	if(ts){
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_SETSLEEP, (u8)on) == 0){
				shtps_setsleep_proc(ts, (u8)on);
			}
		#else
			if(on){
				shtps_device_access_teardown(ts);
				request_event(ts, SHTPS_EVENT_SLEEP, 0);
			}else{
				#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
					ts->system_boot_mode = SH_BOOT_NORMAL;
				#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

				request_event(ts, SHTPS_EVENT_WAKEUP, 0);
			}
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	}
	_log_msg_sync( LOGMSG_ID__API_SLEEP_DONE, "");
}
EXPORT_SYMBOL(msm_tps_setsleep);

void shtps_setFlipInformation(int state)
{
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	request_event(ts, SHTPS_EVENT_FORMCHANGE, state);
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
}
EXPORT_SYMBOL(shtps_setFlipInformation);

int msm_tps_set_veilview_state_on(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_ioctl_set_veilview_state(ts, 1);
	return rc;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_on);

int msm_tps_set_veilview_state_off(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_ioctl_set_veilview_state(ts, 0);
	return rc;
}
EXPORT_SYMBOL(msm_tps_set_veilview_state_off);

int msm_tps_get_veilview_pattern(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPS_LOG_FUNC_CALL();

	rc = shtps_ioctl_get_veilview_pattern(ts);
	return rc;
}
EXPORT_SYMBOL(msm_tps_get_veilview_pattern);

void msm_tps_set_grip_state(int on)
{
}
EXPORT_SYMBOL(msm_tps_set_grip_state);

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data)
{
	int i;
	unsigned long blockSize;
	unsigned long blockNum;

	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE, "");

	if(0 != shtps_enter_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_enter_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "0");
		return -1;
	}

	if(0 != shtps_lockdown_bootloader(ts, (u8*)&fw_data[0x00d0])){
		SHTPS_LOG_ERR_PRINT("error - shtps_lockdown_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "1");
		return -1;
	}

	if(0 != shtps_flash_erase(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_erase()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "2");
		return -1;
	}

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);

	for(i = 0;i < blockNum;i++){
		if(0 != shtps_flash_writeImage(ts, (u8*)&fw_data[0x0100 + i * blockSize])){
			SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeImage(%d)\n", i);
			_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "3");
			return -1;
		}
	}

	if(0 != shtps_flash_writeConfig(ts, (u8*)&fw_data[0x0100 + (blockNum * blockSize)])){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeConfig()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "4");
		return -1;
	}
	if(0 != shtps_exit_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_exit_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "5");
		return -1;
	}
	DBG_PRINTK("fw update done\n");
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_DONE, "");

	return 0;
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

int shtps_get_logflag(void)
{
	#if defined( SHTPS_DEVELOP_MODE_ENABLE )
		return gLogOutputEnable;
	#else
		return 0;
	#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
}

#if defined( SHTPS_DEVELOP_MODE_ENABLE )
int shtps_read_touchevent_from_outside(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		if(SHTPS_STATE_ACTIVE == ts->state_mgr.state){
			shtps_wake_lock_idle(ts);
		}
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	mutex_lock(&shtps_ctrl_lock);

	if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
		shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
		ret = 0;
	}
	else{
		ret = -1;
	}

	mutex_unlock(&shtps_ctrl_lock);

	#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
		shtps_wake_unlock_idle(ts);
	#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

	return ret;
}

int shtps_get_fingermax_from_outside(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int fingerMax = shtps_get_fingermax(ts);
	return fingerMax;
}

int shtps_has_8bitW_from_outside(void)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret = 0;
	if(F11_QUERY_HAS8BITW(ts->map.fn11.query.data)){
		ret = 1;
	}
	return ret;
}
#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */


MODULE_DESCRIPTION("SHARP TOUCHPANEL DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
