/***************************************************************************************/
/* 
	@file		shdtv_spi.c
	@brief		SPI I/F driver for NMI326
	@author		[SC] miya
	@version	1.0
	@note		2014/06/01

	Copyright (c) 2014 Sharp Corporation
*/
/***************************************************************************************/

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
#include <mach/cpuidle.h>
#include <linux/pm_qos.h>
#include "shdtv_spi_ext.h"
#include <linux/semaphore.h>

/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
//#define __LOGOUT__
#define __SKIPMODE__

#define SHDTV_SPI_NAME				"shdtvspi"
#define SHDTV_SPI_BUFFER			(20)
//#define SHDTV_SPI_CLK_SPEED		(10000000)
#define SHDTV_SPI_CLK_SPEED			(4800000)
#define SHDTV_SPI_BIT_WORD			(8)
#define SHDTV_SPI_TSP_READ_NUM		(15)
#define SHDTV_SPI_SEM_VAL			(1)

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int __devinit shdtv_dev_spi_probe(struct spi_device *spi);
static int __devexit shdtv_dev_spi_remove(struct spi_device *spi);

static int shdtvspi_open(struct inode *inode, struct file *file);
static int shdtvspi_close(struct inode *inode, struct file *file);
static int shdtvspi_read(struct file* filp, char* buf, size_t count, loff_t* offset);

static int shdtvspi_sem_init(void);
struct semaphore *spi_sem = NULL;

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
/* --------------------------------------------------------- */
/* Serial Interface Parameter                                */
/* --------------------------------------------------------- */
static struct spi_device *spi_dev = NULL;

#ifdef CONFIG_OF						// Open firmware must be defined for dts usage
static struct of_device_id  shdtv_dev_spi_table [] = {
	{ . compatible = "sharp,shdtv_spi" ,},			// Compatible node must match dts
	{ },
};
#else
#define shdtv_dev_spi_table  NULL
#endif

static struct spi_driver  shdtv_dev_driver = {
	.probe = shdtv_dev_spi_probe,
	.remove = __devexit_p(shdtv_dev_spi_remove),
	.driver = {
		.of_match_table = shdtv_dev_spi_table,
		.name = "shdtv_spi",
		.owner = THIS_MODULE,
	},
};


/* --------------------------------------------------------- */
/* file_operations                                           */
/* --------------------------------------------------------- */
static struct file_operations  shdtvspi_fops = {
	.owner          = THIS_MODULE,
	.open           = shdtvspi_open,
	.release        = shdtvspi_close,
	.read           = shdtvspi_read,
};


/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
struct shdtv_spi_drv {
	struct input_dev	*input;
	struct wake_lock	wake_lock;
};

static dev_t				shdtvspi_dev;
static struct cdev 			shdtvspi_cdev;
static struct class* 		shdtvspi_class;
static struct device*		shdtvspi_device;
static struct shdtv_spi_drv	dtv_ctrl;

static struct mutex 		shdtvspi_io_lock;
static unsigned char		*gRbuf;


static int shdtv_drv_init(struct shdtv_spi_drv *ctrl)
{
	int ret;

	ctrl->input = input_allocate_device();
	if (!(ctrl->input)) {
		printk("%s:%d input_allocate_device() failed \n", __FILE__, __LINE__);
		return -ENOMEM;
	}

	ctrl->input->name         = SHDTV_SPI_NAME;
	ctrl->input->phys         = "shdtvspi/input0";
	ctrl->input->id.vendor    = 0x0001;
	ctrl->input->id.product   = 0x0001;
	ctrl->input->id.version   = 0x0001;
	ctrl->input->dev.parent   = &spi_dev->dev;

	ret = input_register_device(ctrl->input);
	if (ret) {
		printk("%s:%d input_register_device() failed \n", __FILE__, __LINE__);
		input_free_device(ctrl->input);
		return ret;
	}

	wake_lock_init(&(ctrl->wake_lock), WAKE_LOCK_SUSPEND, "shdtvspi_wake_lock");

	mutex_init(&shdtvspi_io_lock);
	return 0;
}


static void shdtv_drv_exit(struct shdtv_spi_drv *ctrl)
{
	mutex_destroy(&shdtvspi_io_lock);

	wake_lock_destroy(&(ctrl->wake_lock));

	input_unregister_device(ctrl->input);

	return;
}


static int __devinit shdtv_dev_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;
	spi_set_drvdata(spi, ctrl);
	spi_dev = spi;

	ret = spi_setup( spi_dev );
	if ( ret < 0 ) {
		printk("%s:%d spi_setup() failed \n", __FILE__, __LINE__);
		return ret;
	}

	ret = shdtv_drv_init(ctrl);
	if (ret != 0) {
		printk("%s:%d shdtv_drv_init() failed \n", __FILE__, __LINE__);
		return ret;
	}
	shdtvspi_sem_init();
//	printk("%s:%d shdtv_dev_spi_probe() OK \n", __FILE__, __LINE__);
	return ret;
}


static int __devexit shdtv_dev_spi_remove(struct spi_device *spi)
{
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;
	spi_set_drvdata(spi, ctrl);
	spi_dev = spi;

	shdtv_drv_exit(ctrl);

	/* semaphore free */
	kfree(spi_sem);

	return 0;
}


static int __init shdtv_dev_init(void)
{
	int ret;

	/* spi_register_driver */
	ret = spi_register_driver(&shdtv_dev_driver);
	if (ret) {
		printk("%s:%d spi_register_driver() failed \n", __FILE__, __LINE__);
	}
	else {
//		printk("%s:%d spi_register_driver() OK \n", __FILE__, __LINE__);
	}

	return ret;
}
module_init(shdtv_dev_init);


static void __exit shdtv_dev_exit(void)
{
	spi_unregister_driver(&shdtv_dev_driver);
	return;
}
module_exit(shdtv_dev_exit);


static int __init shdtv_spi_init(void)
{
	int ret;
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;

	ret = alloc_chrdev_region(&shdtvspi_dev, 0, 1, SHDTV_SPI_NAME);
	if (ret < 0) {
		printk("%s:%d alloc_chrdev_region() failed \n", __FILE__, __LINE__);
		return ret;
	}

	shdtvspi_class = class_create(THIS_MODULE, SHDTV_SPI_NAME);
	if (IS_ERR(shdtvspi_class)) {
		ret = PTR_ERR(shdtvspi_class);
		printk("%s:%d class_create() failed \n", __FILE__, __LINE__);
		goto error_class_create;
	}

	shdtvspi_device = device_create(shdtvspi_class, NULL, 
									shdtvspi_dev, &shdtvspi_cdev, SHDTV_SPI_NAME);
	if (IS_ERR(shdtvspi_device)) {
		ret = PTR_ERR(shdtvspi_device);
		printk("%s:%d device_create() failed \n", __FILE__, __LINE__);
		goto error_device_create;
	}

	cdev_init(&shdtvspi_cdev, &shdtvspi_fops);
	shdtvspi_cdev.owner = THIS_MODULE;
	shdtvspi_cdev.ops   = &shdtvspi_fops;

	ret = cdev_add(&shdtvspi_cdev, shdtvspi_dev, 1);
	if (ret) {
		printk("%s:%d cdev_add() failed \n", __FILE__, __LINE__);
		goto error_cdev_add;
	}

//	printk("%s:%d shdtv_spi_init() OK \n", __FILE__, __LINE__);
	return 0;

error_cdev_add:
	cdev_del(&shdtvspi_cdev);
error_device_create:
	class_destroy(shdtvspi_class);
error_class_create:
	unregister_chrdev_region(shdtvspi_dev, 1);

	printk("%s:%d shdtv_spi_init() failed \n", __FILE__, __LINE__);
	return ret;
}
module_init(shdtv_spi_init);


static void __exit shdtv_spi_exit(void)
{
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;

	cdev_del(&shdtvspi_cdev);
	class_destroy(shdtvspi_class);
	unregister_chrdev_region(shdtvspi_dev, 1);

	return;
}
module_exit(shdtv_spi_exit);

#if 0
#define BIT_NUM (8)
static unsigned char bitflip8(unsigned char base)
{
	unsigned char temp;
	unsigned char val;
	int i;
	
	temp = 0xFF & base;
	val = 0;
	
	for (i = 0; i < BIT_NUM; i++) {
		val |= ((temp >> i) & 0x01) << (BIT_NUM - 1 - i);
	}
	
	return val;
}
#endif

#if 0
static int shdtv_spi_write8(struct shdtv_spi_drv *ctrl, unsigned long adr, unsigned char data)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	unsigned char senddata[SHDTV_SPI_BUFFER];
	int ret = 0;


	if (!spi_dev) {
		printk("%s:%d spi_dev is NULL \n", __FILE__, __LINE__);
		return -EINVAL;
	}

	/**************************************/
	/* Write 6 config bytes + 1 byte data */
	/**************************************/
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 7;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;
		
	senddata[0] = 0x90;
	senddata[1] = 0x00;
	senddata[2] = 0x01;
	senddata[3] = (unsigned char)(adr >> 16);
	senddata[4] = (unsigned char)(adr >> 8);
	senddata[5] = (unsigned char)(adr);
	senddata[6] = data;

	spi_message_init(&msg);
	x->tx_buf           = senddata;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
	}

	return ret;
}
#endif

static int shdtv_spi_write32(struct shdtv_spi_drv *ctrl, unsigned long adr, unsigned long data)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	unsigned char senddata[SHDTV_SPI_BUFFER];
	int ret = 0;


	if (!spi_dev) {
		printk("%s:%d spi_dev is NULL \n", __FILE__, __LINE__);
		return -EINVAL;
	}

	/**************************************/
	/* Write 6 config bytes + 1 word data */
	/**************************************/
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 10;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;
		
	senddata[0] = 0xB0;		/* word access */
	senddata[1] = 0x00;
	senddata[2] = 0x04;		/* 4 bytes */
	senddata[3] = (unsigned char)(adr >> 16);
	senddata[4] = (unsigned char)(adr >> 8);
	senddata[5] = (unsigned char)(adr);
	senddata[6] = (unsigned char)(data);
	senddata[7] = (unsigned char)(data >> 8);
	senddata[8] = (unsigned char)(data >> 16);
	senddata[9] = (unsigned char)(data >> 24);

	spi_message_init(&msg);
	x->tx_buf           = senddata;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
	}

	return ret;
}

#if 0
static int shdtv_spi_read8(struct shdtv_spi_drv *ctrl, unsigned long adr, unsigned char *data)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int ret = 0;
	unsigned char readdata[SHDTV_SPI_BUFFER];
	unsigned char senddata[SHDTV_SPI_BUFFER];
	unsigned char temp_rx, temp_rx_dma_count[3];
	unsigned char temp_tx, temp_tx_dma_count[3];
	int count;


	if (!spi_dev) {
		printk("%s:%d spi_dev is NULL \n", __FILE__, __LINE__);
		return -EINVAL;
	}

	/************************/
	/* Write 6 config bytes */
	/************************/
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 6;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	senddata[0] = 0x50;		/* byte access */
	senddata[1] = 0x00;
	senddata[2] = 0x01;
	senddata[3] = (unsigned char)(adr >> 16);
	senddata[4] = (unsigned char)(adr >> 8);
	senddata[5] = (unsigned char)(adr);

	spi_message_init(&msg);
	x->tx_buf           = senddata;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	/************************/
	/* Wait for complete    */
	/************************/
	/* M sends 0x00 to S */
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 1;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	temp_tx = 0x00;
	spi_message_init(&msg);
	x->tx_buf           = &temp_tx;
	x->rx_buf           = NULL;
	spi_message_add_tail(x, &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	/* M receives 0xFF from S */
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 1;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	spi_message_init(&msg);
	x->tx_buf           = NULL;
	x->rx_buf           = &temp_rx;
	spi_message_add_tail(x, &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	/************************/
	/* Read DMA count       */
	/************************/
	if ( temp_rx == 0xFF ) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SHDTV_SPI_BIT_WORD;
		x->len              = 3;
		x->speed_hz         = SHDTV_SPI_CLK_SPEED;
		x->deassert_wait    = 0;

		temp_tx_dma_count[0] = temp_tx_dma_count[1] = temp_tx_dma_count[2] = 0x00;
		spi_message_init(&msg);
		x->tx_buf           = temp_tx_dma_count;
		x->rx_buf           = temp_rx_dma_count;
		spi_message_add_tail(x, &msg);

		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
			return ret;
		}

		count = ((int)(temp_rx_dma_count[0]) << 16) |
		        ((int)(temp_rx_dma_count[1]) << 8)  |
		        (int)(temp_rx_dma_count[2]);

		/************************/
		/* Read DATA            */
		/************************/
		if ( count == 1 ) {
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SHDTV_SPI_BIT_WORD;
			x->len              = 1;
			x->speed_hz         = SHDTV_SPI_CLK_SPEED;
			x->deassert_wait    = 0;

			temp_tx = 0x00;
			spi_message_init(&msg);
			x->tx_buf           = &temp_tx;
			x->rx_buf           = readdata;
			spi_message_add_tail(x, &msg);

			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
				return ret;
			}
			*data = readdata[0];

			/************************/
			/* Terminate read seq.  */
			/************************/
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SHDTV_SPI_BIT_WORD;
			x->len              = 1;
			x->speed_hz         = SHDTV_SPI_CLK_SPEED;
			x->deassert_wait    = 0;

			temp_tx = 0xFF;
			spi_message_init(&msg);
			x->tx_buf           = &temp_tx;
			x->rx_buf           = NULL;
			spi_message_add_tail(x, &msg);

			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
				return ret;
			}
		}
		else {
			printk("%s:%d spi bus error, bad count. [count=%d] \n", __FILE__, __LINE__, count);
			return -EFAULT;
		}
	}
	else {
		printk("%s:%d spi bus error, not complete. [temp_rx=0x%02x] \n", __FILE__, __LINE__, temp_rx);
		return -EFAULT;
	}

	return ret;
}
#endif

static int shdtv_spi_read32(struct shdtv_spi_drv *ctrl, unsigned long adr, unsigned long *data)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int ret = 0;
	unsigned char readdata[SHDTV_SPI_BUFFER];
	unsigned char senddata[SHDTV_SPI_BUFFER];
	int count;
#ifndef __SKIPMODE__
	unsigned char temp_rx[8], temp_rx_dma_count[3];
	unsigned char temp_tx_dma_count[5];
#endif
	

	if (!spi_dev) {
		printk("%s:%d spi_dev is NULL \n", __FILE__, __LINE__);
		return -EINVAL;
	}

#ifndef __SKIPMODE__
	/************************/
	/* Write 6 config bytes */
	/************************/
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 8;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	senddata[0] = 0x70;		/* word access */
	senddata[1] = 0x00;
	senddata[2] = 0x04;		/* 4 bytes */
	senddata[3] = (unsigned char)(adr >> 16);
	senddata[4] = (unsigned char)(adr >> 8);
	senddata[5] = (unsigned char)(adr);

	senddata[6] = 0x00;		/* M sends 0x00 to S */
	senddata[7] = 0x00;		/* dummy to receive 0xFF from S */

	spi_message_init(&msg);
	x->tx_buf           = senddata;
	x->rx_buf           = temp_rx;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}
//	printk("%s:%d [temp_rx=0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ] \n",
//		__FILE__, __LINE__, temp_rx[0], temp_rx[1], temp_rx[2], temp_rx[3], temp_rx[4], temp_rx[5], temp_rx[6], temp_rx[7]);

	/************************/
	/* Wait for complete    */
	/************************/


	/************************/
	/* Read DMA count       */
	/************************/
	if ( temp_rx[7] == 0xFF ) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SHDTV_SPI_BIT_WORD;
		x->len              = 3;
		x->speed_hz         = SHDTV_SPI_CLK_SPEED;
		x->deassert_wait    = 0;

		temp_tx_dma_count[0] = temp_tx_dma_count[1] = temp_tx_dma_count[2] = 0x00;
		spi_message_init(&msg);
		x->tx_buf           = temp_tx_dma_count;
		x->rx_buf           = temp_rx_dma_count;
		spi_message_add_tail(x, &msg);

		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
			return ret;
		}

		count = ((int)(temp_rx_dma_count[0]) << 16) |
		        ((int)(temp_rx_dma_count[1]) << 8)  |
		        (int)(temp_rx_dma_count[2]);

	//	printk("count = %x \n", count);

		/************************/
		/* Read DATA            */
		/************************/
		if ( count == 4 ) {
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SHDTV_SPI_BIT_WORD;
			x->len              = 5;
			x->speed_hz         = SHDTV_SPI_CLK_SPEED;
			x->deassert_wait    = 0;

			temp_tx_dma_count[0] = temp_tx_dma_count[1] = temp_tx_dma_count[2] = temp_tx_dma_count[3] = 0x00;
			temp_tx_dma_count[4] = 0xFF;	/* Terminate read seq.  */
			spi_message_init(&msg);
			x->tx_buf           = temp_tx_dma_count;
			x->rx_buf           = readdata;
			spi_message_add_tail(x, &msg);

			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
				return ret;
			}

			*data = ((unsigned long)readdata[3]) << 24 |
			        ((unsigned long)readdata[2]) << 16 |
			        ((unsigned long)readdata[1]) << 8  |
			        (unsigned long)readdata[0];

		//	printk("data = %lx \n", *data);

			/************************/
			/* Terminate read seq.  */
			/************************/
		}
		else {
			printk("%s:%d spi bus error, bad count. [count=%d] \n", __FILE__, __LINE__, count);
			return -EFAULT;
		}
	}
	else {
		printk("%s:%d spi bus error, not complete. [temp_rx=0x%02x] \n", __FILE__, __LINE__, temp_rx[7]);
		return -EFAULT;
	}

#else
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 8 + 3 + 5;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	senddata[0] = 0x70;		/* word access */
	senddata[1] = 0x00;
	senddata[2] = 0x04;		/* 4 bytes */
	senddata[3] = (unsigned char)(adr >> 16);
	senddata[4] = (unsigned char)(adr >> 8);
	senddata[5] = (unsigned char)(adr);

	senddata[6] = 0x00;		/* M sends 0x00 to S */
	senddata[7] = 0x00;		/* dummy to receive 0xFF from S */

	senddata[8] = 0x00;		/* M sends three 0x00s to receive DMA count from S */
	senddata[9] = 0x00;	
	senddata[10] = 0x00;

	senddata[11] = 0x00;	/* M sends four 0x00s to receive Read DATA from S */
	senddata[12] = 0x00;
	senddata[13] = 0x00;
	senddata[14] = 0x00;
	senddata[15] = 0xFF;	/* 0xFF to terminate read seq.  */
	
	spi_message_init(&msg);
	x->tx_buf           = senddata;
	x->rx_buf           = readdata;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	count = ((int)(readdata[8]) << 16) |
	        ((int)(readdata[9]) << 8)  |
	        (int)(readdata[10]);

	*data = ((unsigned long)readdata[14]) << 24 |
	        ((unsigned long)readdata[13]) << 16 |
	        ((unsigned long)readdata[12]) << 8  |
	        (unsigned long)readdata[11];

	if ( readdata[7] != 0xFF || count != 4 ) {
		printk("shdtv_spi_read32() error \n");
		printk("                   [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x] \n", 
			readdata[0], readdata[1], readdata[2], readdata[3], readdata[4], readdata[5], readdata[6], readdata[7]);
		printk("                   [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x] \n", 
			readdata[8], readdata[9], readdata[10], readdata[11], readdata[12], readdata[13], readdata[14], readdata[15]);

		return -EFAULT;
	}

#endif	// __SKIPMODE__

	return ret;
}

#if 1
static int shdtv_spi_block_read(struct shdtv_spi_drv *ctrl, unsigned char *data, int *size)
{
	/**********************************************/
	/* [NOTE] the polling configuration byte 0xC0 */
	/**********************************************/
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int ret = 0;
	unsigned char senddata[SHDTV_SPI_BUFFER];
	unsigned char temp_rx[6], temp_tx;
	int count;
#ifndef __SKIPMODE__
	unsigned char temp_tx_dma_count[3];
#endif


	if (!spi_dev) {
		printk("%s:%d spi_dev is NULL \n", __FILE__, __LINE__);
		return -EINVAL;
	}

#ifndef __SKIPMODE__
	/************************/
	/* Write config byte */
	/************************/
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 3;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	senddata[0] = 0xC0;
	senddata[1] = 0x00;		/* M sends 0x00 to S */
	senddata[2] = 0x00;		/* dummy to receive 0xFF from S */
	spi_message_init(&msg);
	x->tx_buf           = senddata;
	x->rx_buf           = temp_rx;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	/************************/
	/* Wait for complete    */
	/************************/


	/************************/
	/* Read DMA count       */
	/************************/
	if ( temp_rx[2] == 0xFF ) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SHDTV_SPI_BIT_WORD;
		x->len              = 3;
		x->speed_hz         = SHDTV_SPI_CLK_SPEED;
		x->deassert_wait    = 0;

		temp_tx_dma_count[0] = temp_tx_dma_count[1] = temp_tx_dma_count[2] = 0x00;
		spi_message_init(&msg);
		x->tx_buf           = temp_tx_dma_count;
		x->rx_buf           = temp_rx_dma_count;
		spi_message_add_tail(x, &msg);

		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
			return ret;
		}

		count = ((int)(temp_rx_dma_count[0]) << 16) |
		        ((int)(temp_rx_dma_count[1]) << 8)  |
		        (int)(temp_rx_dma_count[2]);

	//	printk("count = %d \n", count);

		/************************/
		/* Read DATA            */
		/************************/
		if ( (count > 0) && (count <= (40 * 188)) ) {
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SHDTV_SPI_BIT_WORD;
			x->len              = count;
			x->speed_hz         = SHDTV_SPI_CLK_SPEED;
			x->deassert_wait    = 0;

			spi_message_init(&msg);
			x->tx_buf           = NULL;
			x->rx_buf           = data;
			spi_message_add_tail(x, &msg);

			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
				return ret;
			}

			*size = count;

			/************************/
			/* Terminate read seq.  */
			/************************/
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SHDTV_SPI_BIT_WORD;
			x->len              = 1;
			x->speed_hz         = SHDTV_SPI_CLK_SPEED;
			x->deassert_wait    = 0;

			temp_tx             = 0xFF;
			spi_message_init(&msg);
			x->tx_buf           = &temp_tx;
			x->rx_buf           = NULL;
			spi_message_add_tail(x, &msg);

			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
				return ret;
			}
		}
		else {
			printk("%s:%d spi bus error, bad count. [count=%d] \n", __FILE__, __LINE__, count);
			return -EFAULT;
		}
	}
	else {
		printk("%s:%d spi bus error, not complete. \n", __FILE__, __LINE__);
		return -EFAULT;
	}

#else
	memset(&xfer, 0, sizeof(xfer));
	x = &xfer;
	x->bits_per_word    = SHDTV_SPI_BIT_WORD;
	x->len              = 3 + 3;
	x->speed_hz         = SHDTV_SPI_CLK_SPEED;
	x->deassert_wait    = 0;

	senddata[0] = 0xC0;
	senddata[1] = 0x00;		/* M sends 0x00 to S */
	senddata[2] = 0x00;		/* dummy to receive 0xFF from S */
	
	senddata[3] = 0x00;		/* M sends three 0x00s to receive DMA count from S */
	senddata[4] = 0x00;
	senddata[5] = 0x00;
	
	spi_message_init(&msg);
	x->tx_buf           = senddata;
	x->rx_buf           = temp_rx;
	spi_message_add_tail(x, &msg);

	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	count = ((int)(temp_rx[3]) << 16) |
	        ((int)(temp_rx[4]) << 8)  |
	        (int)(temp_rx[5]);

	if ( temp_rx[2] != 0xFF ) {
		printk("shdtv_spi_block_read() error \n");
		printk("                   [0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x] \n", 
						temp_rx[0], temp_rx[1], temp_rx[2], temp_rx[3], temp_rx[4], temp_rx[5]);
		return -EFAULT;
	}

	/************************/
	/* Read DATA            */
	/************************/
	if ( (count > 0) && (count <= (40 * 188)) ) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SHDTV_SPI_BIT_WORD;
		x->len              = count;
		x->speed_hz         = SHDTV_SPI_CLK_SPEED;
		x->deassert_wait    = 0;

		spi_message_init(&msg);
		x->tx_buf           = NULL;
		x->rx_buf           = data;
		spi_message_add_tail(x, &msg);

		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
			return ret;
		}

		*size = count;

		/************************/
		/* Terminate read seq.  */
		/************************/
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SHDTV_SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SHDTV_SPI_CLK_SPEED;
		x->deassert_wait    = 0;

		temp_tx             = 0xFF;
		spi_message_init(&msg);
		x->tx_buf           = &temp_tx;
		x->rx_buf           = NULL;
		spi_message_add_tail(x, &msg);

		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			printk("%s:%d spi_sync err. [ret=%d] \n", __FILE__, __LINE__, ret);
			return ret;
		}
	}
	else {
		printk("%s:%d spi bus error, bad count. [count=%d] \n", __FILE__, __LINE__, count);
		return -EFAULT;
	}

#endif	// __SKIPMODE__

	return ret;
}
#endif

static int shdtv_spi_transfer(struct shdtv_spi_drv *ctrl, unsigned char *rbuf, int rsize)
{
#if 1
	unsigned long status, irqsts, adr, tsz, esz, fsm, tsoctl;
	int readsize, readA, readB;
	int ret;
#else
	int ret, i;
	unsigned long adr, data32;
	unsigned char data, rdata;
#endif
	
#if 1
	readsize = 0;
	
	/*** TSP Transfer Protocol ***/
	/* check interrupt_STS */
	ret = shdtv_spi_read32(ctrl, 0x6120, &status);
	if ( ret != 0 ) {
		printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
	}
#ifdef  __LOGOUT__
	printk("%s:%d [status=0x%08lx] \n", __FILE__, __LINE__,  status);
#endif

	/* Read and clear the core interrupt status */
	if ( (status >> 4) & 0x1 ) {
	//	shdtv_spi_read32(ctrl, 0xA008, &irqsts);
	//	printk("%s:%d [adr=0xA008][irqsts=0x%08lx] \n", __FILE__, __LINE__,  irqsts);
		ret = shdtv_spi_read32(ctrl, 0xA00C, &irqsts);
		if ( ret != 0 ) {
			printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
		}
#ifdef  __LOGOUT__
		printk("%s:%d [adr=0xA00C][irqsts=0x%08lx] \n", __FILE__, __LINE__,  irqsts);
#endif
		shdtv_spi_write32(ctrl, 0xA00C, irqsts);
	//	shdtv_spi_write32(ctrl, 0xA008, irqsts);		/* <-- 0xA00C should be cleared */
	}

	/* Read and clear the decoder interrupt status */
	if ( (status >> 5) & 0x1 ) {
	
		/* check FSM_STS */
	//	shdtv_spi_read32(ctrl, 0xA80C, &irqsts);
	//	printk("%s:%d [fsmsts=0x%08lx] \n", __FILE__, __LINE__,  irqsts);

		/* check DATA_READY flag */
		ret = shdtv_spi_read32(ctrl, 0xA818, &irqsts);
		if ( ret != 0 ) {
			printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
		}
#ifdef  __LOGOUT__
		printk("%s:%d [adr=0xA818][irqsts=0x%08lx] \n", __FILE__, __LINE__,  irqsts);
#endif

		irqsts = irqsts & 0x00FFFFFF;		/* NOT clear for GetSpStatus() to check IRQ flag */
		shdtv_spi_write32(ctrl, 0xA818, irqsts);
		
		if ( (irqsts >> 4) & 0x1 ) {		/* when DMA data is ready, ... */
			/* check if DMA enable... */
			ret = shdtv_spi_read32(ctrl, 0xA804, &tsoctl);
			if ( ret != 0 ) {
				printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
			}
		//	printk("%s:%d [adr=0xA804][tsoctl=0x%lx] \n", __FILE__, __LINE__,  tsoctl);

			if ( (tsoctl >> 8) & 0x1 ) {	/* DMA output : enabled */
				/* get the size */
				tsz = (tsoctl >> 10) & 0x7F;
				tsz *= 188;													/* tsz = <TSO FIFO block size> × 188bytes */

				/* read and calculate the starting address from the decoder */
				do {
					ret = shdtv_spi_read32(ctrl, 0xA83C, &adr);				/* reading the <DMA block_starting_adr> */
					if ( ret != 0 ) {
						printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
					}
				//	printk("%s:%d [adr=0xA83C][adr=0x%lx] \n", __FILE__, __LINE__,  adr);
					adr *= 4;
					adr += 0x60050;											/* It comes to the <physical DMA stating adr> = [read pointer] */
				//	printk("%s:%d [adr=0x%lx] <physical DMA stating adr> \n", __FILE__, __LINE__,  adr);

					/* calculate the size */
					esz = 0x6204C - adr + 4;

					if ( (irqsts >> 11) & 0x1 ) {	/* overflow */
						printk("%s:%d TSO FIFO overflow... \n", __FILE__, __LINE__);
					
						/* disable DMA */
						tsoctl &= ~(1 << 8);
						shdtv_spi_write32(ctrl, 0xA804, tsoctl);

						/* read the last 4 bytes */
						if ( esz < tsz ) {
							tsz -= esz;
							adr = 0x60050 + tsz - 4;
						}
						else {
							adr += tsz - 4;					
						} 

						shdtv_spi_write32(ctrl, 0x7864, adr);
						shdtv_spi_write32(ctrl, 0x7868, 4);

						/* read the last 4 bytes */
						ret = shdtv_spi_block_read(ctrl, rbuf, &readsize);
						if ( ret != 0 ) {
							printk("%s:%d shdtv_spi_block_read error \n", __FILE__, __LINE__);
						}

						/* enable DMA */
						tsoctl |= (1 << 8);
						shdtv_spi_write32(ctrl, 0xA804, tsoctl);
					}
					else {
					//	printk("%s:%d [esz=0x%lx][tsz=0x%lx] \n", __FILE__, __LINE__,  esz, tsz);

						shdtv_spi_write32(ctrl, 0x7864, adr);				/* set the <physical DMA stating adr> */
						if ( esz < tsz ) {			/* wrap around */
							/* 1st transfer */
							shdtv_spi_write32(ctrl, 0x7868, esz);

							/* read the data */
							ret = shdtv_spi_block_read(ctrl, rbuf, &readA);
							if ( ret != 0 ) {
								printk("%s:%d shdtv_spi_block_read error \n", __FILE__, __LINE__);
							}

							/* 2nd transfer */
							shdtv_spi_write32(ctrl, 0x7864, 0x60050);		/* set [read pointer] to the first adr of DMA block area */
							shdtv_spi_write32(ctrl, 0x7868, (tsz - esz));	/* set the 2nd transfer count to the remain size */

							/* read the data */
							ret = shdtv_spi_block_read(ctrl, (rbuf+readA), &readB);
							if ( ret != 0 ) {
								printk("%s:%d shdtv_spi_block_read error \n", __FILE__, __LINE__);
							}

							readsize = readA + readB;
						}
						else {
							shdtv_spi_write32(ctrl, 0x7868, tsz);

							/* read the data */
							ret = shdtv_spi_block_read(ctrl, rbuf, &readsize);
							if ( ret != 0 ) {
								printk("%s:%d shdtv_spi_block_read error \n", __FILE__, __LINE__);
							}
						}
					}

					/* read and clear status */
					ret = shdtv_spi_read32(ctrl, 0xA818, &irqsts);
					if ( ret != 0 ) {
						printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
					}
#ifdef  __LOGOUT__
					printk("[adr=0xA818][irqsts=0x%08lx] \n", irqsts);
#endif
					shdtv_spi_write32(ctrl, 0xA818, 0x00000020);

					if ( (irqsts >> 5) & 0x1 ) {
#ifdef  __LOGOUT__
						printk("%s:%d dma_done break here \n", __FILE__, __LINE__);
#endif
						break;
					}

					/* read the FSM state */
					ret = shdtv_spi_read32(ctrl, 0xA80C, &fsm);
					if ( ret != 0 ) {
						printk("%s:%d temp_rx No respnse \n", __FILE__, __LINE__);
					}
#ifdef  __LOGOUT__
					printk("[fsm=0x%08lx] \n", fsm);
#endif
					if ( ((fsm >> 12) & 0x3) != 0x2 ) {
						break;
					}

				} while (1);
			}
		}

	}
	/*** TSP Transfer Protocol END ***/
#else
	/* Test R/W */

	ret = shdtv_spi_block_read(ctrl, rbuf, rsize);
	printk("%s:%d shdtv_spi_block_read success  [ret=%d][rsize=%d] \n", __FILE__, __LINE__, ret, *rsize);

	return 0;

#if 1
	/* word read */
	for ( i=0; i<1; i++ ) {
	adr = 0x00006448;
	ret = shdtv_spi_read32(ctrl, adr, &data32);
	if (ret) {
		printk("%s:%d shdtv_spi_read32 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_read32 success  [ret=%d][data32=0x%lx] \n", __FILE__, __LINE__, ret, data32);
	}

	/* word write */
	adr = 0x00006448;
	data32 = 0x0000ABAB;
	ret = shdtv_spi_write32(ctrl, adr, data32);
	if (ret) {
		printk("%s:%d shdtv_spi_write32 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_write32 success  [ret=%d] \n", __FILE__, __LINE__, ret);

	/* word read */
	for ( i=0; i<1; i++ ) {
	adr = 0x00006448;
	ret = shdtv_spi_read32(ctrl, adr, &data32);
	if (ret) {
		printk("%s:%d shdtv_spi_read32 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_read32 success  [ret=%d][data32=0x%lx] \n", __FILE__, __LINE__, ret, data32);
	}


	/* byte read */
	for ( i=0; i<1; i++ ) {
	adr = 0x00007888;
	ret = shdtv_spi_read8(ctrl, adr, &rdata);
	if (ret) {
		printk("%s:%d shdtv_spi_read8 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_read8 success  [ret=%d][rdata=0x%02x] \n", __FILE__, __LINE__, ret, rdata);
	}

	/* byte write */
	adr = 0x00007888;
	data = 0x03;
	ret = shdtv_spi_write8(ctrl, adr, data);
	if (ret) {
		printk("%s:%d shdtv_spi_write8 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_write8 success  [ret=%d] \n", __FILE__, __LINE__, ret);

	/* byte read */
	for ( i=0; i<1; i++ ) {
	adr = 0x00007888;
	ret = shdtv_spi_read8(ctrl, adr, &rdata);
	if (ret) {
		printk("%s:%d shdtv_spi_read8 err.  [ret=%d] \n", __FILE__, __LINE__, ret);
		return ret;
	}

	printk("%s:%d shdtv_spi_read8 success  [ret=%d][rdata=0x%02x] \n", __FILE__, __LINE__, ret, rdata);
	}
#endif
#endif

	return readsize;
}


static int shdtv_spi_renew(struct shdtv_spi_drv *ctrl)
{
	int ret;
	unsigned long tsoctl, adr, tsz, esz, fsm;
	unsigned char rbuf[10];
	int readsize;


//	printk("shdtv_spi_renew start \n");

	/* disable DMA */
	ret = shdtv_spi_read32(ctrl, 0xA804, &tsoctl);
	if ( ret != 0 ) {
		printk("%s:%d shdtv_spi_read32 error \n", __FILE__, __LINE__);
	}
//	printk("[adr=0xA804][tsoctl=0x%lx] \n", tsoctl);

	tsoctl &= ~(1 << 8);
	shdtv_spi_write32(ctrl, 0xA804, tsoctl);

	/* clear DMA interrupts */
	shdtv_spi_write32(ctrl, 0xA818, 0x00000830 );					/* Write 1s to clear bit4, 5 and 11 */

	do {
		ret = shdtv_spi_read32(ctrl, 0xA80C, &fsm);
		if ( ret != 0 ) {
			printk("%s:%d shdtv_spi_read32 error \n", __FILE__, __LINE__);
		}
	//	printk("[fsm=0x%08lx] \n", fsm);
		
		if ( ((fsm >> 12) & 0x3) == 0x2 ) {
			tsz = (tsoctl >> 10) & 0x7F;
			tsz *= 188;

			/* read and calculate the starting address from the decoder */
			ret = shdtv_spi_read32(ctrl, 0xA83C, &adr);				/* reading the <DMA block_starting_adr> */
			if ( ret != 0 ) {
				printk("%s:%d shdtv_spi_read32 error \n", __FILE__, __LINE__);
			}
			adr *= 4;
			adr += 0x60050;

			/* claculate the size */
			esz = 0x6204C - adr + 4;

			/* read the last 4 bytes */
			if ( esz < tsz ) {
				tsz -= esz;
				adr = 0x60050 + tsz - 4;
			}
			else {
				adr += tsz - 4;					
			} 

			shdtv_spi_write32(ctrl, 0x7864, adr);
			shdtv_spi_write32(ctrl, 0x7868, 4);

			ret = shdtv_spi_block_read(ctrl, rbuf, &readsize);
			if ( ret != 0 ) {
				printk("%s:%d shdtv_spi_block_read error \n", __FILE__, __LINE__);
			}
		}
		else {
			break;
		}
	} while (1);

//	printk("shdtv_spi_renew end \n");
	return ret;
}


static int shdtvspi_open(struct inode *inode, struct file *file)
{
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;
	wake_lock(&(ctrl->wake_lock));

	gRbuf = kmalloc( 188*SHDTV_SPI_TSP_READ_NUM, GFP_KERNEL );
	if ( gRbuf == NULL ) {
		wake_unlock(&(ctrl->wake_lock));
		printk("%s:%d kmalloc() failed \n", __FILE__, __LINE__);
		return -EFAULT;
	}

//	printk("%s:%d shdtvspi_open() called \n", __FILE__, __LINE__);
	return 0;
}


static int shdtvspi_close(struct inode *inode, struct file *file)
{
	struct shdtv_spi_drv *ctrl;

	ctrl = &dtv_ctrl;
	wake_unlock(&(ctrl->wake_lock));

	kfree( gRbuf );

//	printk("%s:%d shdtvspi_close() called \n", __FILE__, __LINE__);
	return 0;
}


static int shdtvspi_read(struct file* filp, char* buf, size_t count, loff_t* offset)
{
	int ret = 0;
	struct shdtv_spi_drv *ctrl;


//	printk("shdtvspi_read() start \n");

	shdtvspi_sem_lock();
	
	if ( count < 188*SHDTV_SPI_TSP_READ_NUM ) {
		ctrl = &dtv_ctrl;
		mutex_lock(&shdtvspi_io_lock);

	//	printk("shdtvspi_read()  ch = %d \n", count);
		ret = shdtv_spi_renew(ctrl);

		mutex_unlock(&shdtvspi_io_lock);
	}
	else {
		ctrl = &dtv_ctrl;
		memset(gRbuf, 0, 188*SHDTV_SPI_TSP_READ_NUM);
		mutex_lock(&shdtvspi_io_lock);

		ret = shdtv_spi_transfer(ctrl, gRbuf, 188*SHDTV_SPI_TSP_READ_NUM);
		if ( ret > 0 ) {
			if ( copy_to_user( (unsigned char*)buf, gRbuf, 188*SHDTV_SPI_TSP_READ_NUM ) ) {
				printk("%s:%d copy_to_user failed \n", __FILE__, __LINE__);
				mutex_unlock(&shdtvspi_io_lock);

				shdtvspi_sem_unlock();

				return -EFAULT;
			}
		}

		mutex_unlock(&shdtvspi_io_lock);
	}

	shdtvspi_sem_unlock();
//	printk("shdtvspi_read() end  ret = %d \n", ret);
	return ret;
}

int shdtvspi_sem_init(void)
{

	printk("shdtvspi_sem_init() start \n");
	spi_sem = kmalloc(sizeof(struct semaphore),GFP_KERNEL);
	sema_init( spi_sem, SHDTV_SPI_SEM_VAL );

	printk("shdtvspi_sem_init() end  \n");
	return 0;
}

int shdtvspi_sem_lock(void)
{

	printk("shdtvspi_sem_lock() start \n");

	/* sem wait */
	down( spi_sem );

	printk("shdtvspi_sem_lock() end  \n");
	return 0;
}

int shdtvspi_sem_unlock(void)
{

	printk("shdtvspi_sem_unlock() start \n");

	/* sem post */
	up( spi_sem );

	printk("shdtvspi_sem_unlock() end  \n");
	return 0;
}

