/* drivers/sharp/shtps/sy3000/shtps_rmi_spi.h
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
#ifndef __SHTPS_RMI_SPI_H__
#define __SHTPS_RMI_SPI_H__

#if 0 /** For build test */
	#undef CONFIG_SHTPS_SY3000_TM2949_001

	#define CONFIG_SHTPS_SY3000_TM2949_001
#endif

#if defined( CONFIG_SHTPS_SY3000_TM2949_001 )
	#include "tm2949-001/shtps_cfg_tm2949-001.h"
#else
	#include "tm2949-001/shtps_cfg_tm2949-001.h"
#endif

/* ===================================================================================
 * Debug
 */
#define	TPS_ID0	"[shtps]"
#define	TPS_ID1	"[shtpsif]"
#define	DBG_PRINTK(...)	printk(KERN_DEBUG TPS_ID0 " "__VA_ARGS__)
#define	ERR_PRINTK(...)	printk(KERN_ERR TPS_ID0 " "__VA_ARGS__)
#define	PR_DEBUG(...)	pr_debug(TPS_ID0 " "__VA_ARGS__)
#define	PR_ERROR(...)	pr_err(TPS_ID0 " "__VA_ARGS__)

#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_ERR_PRINT(...)	DBG_PRINTK( __VA_ARGS__)
#else
	#define SHTPS_LOG_ERR_PRINT(...)
#endif /* defined( SHTPS_LOG_ERROR_ENABLE ) */

#if defined( SHTPS_LOG_DEBUG_ENABLE )
	#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
		#define	SHTPS_LOG_DBG_PRINT(...)			\
			if((gLogOutputEnable & 0x02) != 0){		\
				DBG_PRINTK( __VA_ARGS__);			\
			}
		#define SHTPS_LOG_DEBUG(p)					\
			if((gLogOutputEnable & 0x02) != 0){		\
				p									\
			}
		#define SHTPS_LOG_FUNC_CALL()				\
			if((gLogOutputEnable & 0x02) != 0){		\
				DBG_PRINTK("%s()\n", __func__);		\
			}
		#define SHTPS_LOG_FUNC_CALL_INPARAM(param)			\
			if((gLogOutputEnable & 0x02) != 0){				\
				DBG_PRINTK("%s(%d)\n", __func__, param);	\
			}
	#else
		#define	SHTPS_LOG_DBG_PRINT(...)	DBG_PRINTK(__VA_ARGS__)
		#define SHTPS_LOG_DEBUG(p)	p
		#define SHTPS_LOG_FUNC_CALL()	DBG_PRINTK("%s()\n", __func__)
		#define SHTPS_LOG_FUNC_CALL_INPARAM(param)	\
										DBG_PRINTK("%s(%d)\n", __func__, param)
	#endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */
#else
	#define	SHTPS_LOG_DBG_PRINT(...)
	#define SHTPS_LOG_DEBUG(p)
	#define SHTPS_LOG_FUNC_CALL()
	#define SHTPS_LOG_FUNC_CALL_INPARAM(param)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_EVENT_ENABLE ) && defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	#define SHTPS_LOG_EVENT(p)				\
		if((gLogOutputEnable & 0x01) != 0){	\
			p								\
		}
#elif defined( SHTPS_LOG_EVENT_ENABLE )
	#define SHTPS_LOG_EVENT(p)	p
#else
	#define SHTPS_LOG_EVENT(p)
#endif /* defined( SHTPS_LOG_EVENT_ENABLE ) */

#define _log_msg_sync(id, fmt, ...)
#define _log_msg_send(id, fmt, ...)
#define _log_msg_recv(id, fmt, ...)

/* ===================================================================================
 * Common
 */
#define SPI_ERR_CHECK(check, label) \
	if((check)) goto label

#define SHTPS_POSTYPE_X (0)
#define SHTPS_POSTYPE_Y (1)

#define SHTPS_TOUCH_CANCEL_COORDINATES_X (0)
#define SHTPS_TOUCH_CANCEL_COORDINATES_Y (9999)


int shtps_get_logflag(void);
int shtps_read_touchevent_from_outside(void);
int shtps_get_fingermax_from_outside(void);
int shtps_has_8bitW_from_outside(void);
#endif /* __SHTPS_RMI_SPI_H__ */
