/* drivers/sharp/shaudio/shsmartamp_ssm4329.c
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <sharp/shsmartamp_ssm4329.h>
#include <sharp/shsmartamp_ssm4329_fw.h>
#include <linux/of_gpio.h>

static struct i2c_client *shsmartamp_client;

static DEFINE_MUTEX(smartamp_lock);
static int shsmartamp_opened;
static int shsmartamp_en = -1;

static int shsmartamp_i2c_write(const uint8_t* data, int data_sz, int package_sz)
{
	int ret = 0;
	int retry_max = 20;
	int try_count = 0;
	int tail, front, buf_sz;
	unsigned short addr;
	uint8_t buf[package_sz + SSM4329_ADDR_LEN];

	struct i2c_msg msgs[] =
	{
		{
			.addr = shsmartamp_client->addr,
			.flags = 0,
			.buf = buf,
		},
	};

	pr_debug("%s(): Writing to address 0x%02x%02x data_sz = %d\n",
		__func__, data[0], data[1], data_sz);

	addr = ((data[0] << 8) & 0xff00) + (data[1] & 0xff);
	tail = data_sz;
	front = SSM4329_ADDR_LEN;

	for (; tail != front;) {
		if ((tail - front) >= package_sz) {
			buf_sz = package_sz;
		} else {
			buf_sz = tail - front;
		}

		buf[0] = (addr >> 8) & 0xff;
		buf[1] = addr & 0xff;
		memcpy(&buf[SSM4329_ADDR_LEN], &data[front], buf_sz);
		msgs[0].len = buf_sz + SSM4329_ADDR_LEN;
		front += buf_sz;

		for (try_count = 0; try_count <= retry_max; try_count++) {
			ret = i2c_transfer(shsmartamp_client->adapter, msgs, 1);
			if(ret < 0 && ret != -EAGAIN) {
				pr_err("%s(): FAILED. writing to address 0x%02x%02x\n",
					__func__, buf[0], buf[1]);
				return ret;
			} else if (ret > 0) {
				break;
			}
		}

		addr += buf_sz / package_sz * SSM4329_PACKAGE;
	}

	return (ret > 0)? 0 : ret;
}

static int shsmartamp_i2c_read(const uint8_t *data)
{
	int ret = 0;
	int retry_max = 20;
	int try_count = 0;
	uint8_t write_buf[SSM4329_ADDR_LEN];
	uint8_t read_buf;
	struct i2c_msg msg[] =
	{
		{
			.addr = shsmartamp_client->addr,
			.flags= 0,
			.len  = 2,
			.buf  = write_buf,
		},
		{
			.addr = shsmartamp_client->addr,
			.flags= I2C_M_RD,
			.len  = 1,
			.buf  = &read_buf,
		},
	};

	memcpy(write_buf, data, SSM4329_ADDR_LEN);

	pr_debug("%s()\n", __func__);

	for (try_count = 0; try_count <= retry_max; try_count++) {
		ret = i2c_transfer(shsmartamp_client->adapter, &msg[0], 1);
		if (ret == -EAGAIN)
			continue;
		if(ret < 0 && ret != -EAGAIN) {
			pr_err("%s(): FAILED. writing to address 0x%02x%02x\n",
				__func__, write_buf[0], write_buf[1]);
			return ret;
		}
		ret = i2c_transfer(shsmartamp_client->adapter, &msg[1], 1);
		if (ret == -EAGAIN)
			continue;
		if(ret < 0 && ret != -EAGAIN) {
			pr_err("%s(): FAILED. read from address 0x%02x%02x\n",
				__func__, write_buf[0], write_buf[1]);
			return ret;
		}
		pr_debug("%s(): read 0x%02x from address 0x%02x%02x\n",
			__func__, read_buf, write_buf[0], write_buf[1]);
		break;
	}

	return (ret > 0)? 0 : ret;
}

static int shsmartamp_fw_init(void)
{
	int ret;
	int i, j;

	pr_debug("%s() SSM4329_PACKAGE = %d\n", __func__, SSM4329_PACKAGE);

	gpio_direction_output(shsmartamp_en, 1);
	for (i = 0; i < SSM4329_FW_SIGNEL_X; i++) {
		ret = shsmartamp_i2c_write(ssm4329_fw_signel[i],
			SSM4329_FW_SIGNEL_Y, SSM4329_DATA);
		if (ret != 0)
			break;
		if (i == 29) {
			ret = shsmartamp_i2c_write(ssm4329_fw_large_0800,
				sizeof(ssm4329_fw_large_0800)/sizeof(uint8_t),
				SSM4329_PROGRAM);
			if (ret != 0)
				break;
			ret = shsmartamp_i2c_write(ssm4329_fw_large_0000,
				sizeof(ssm4329_fw_large_0000)/sizeof(uint8_t),
				SSM4329_DATA);
			if (ret != 0)
				return ret;
			ret = shsmartamp_i2c_write(ssm4329_fw_large_177f,
				sizeof(ssm4329_fw_large_177f)/sizeof(uint8_t),
				SSM4329_DATA);
			if (ret != 0)
				return ret;
		}
		if (i == 31) {
			for (j = 0; j < SSM4329_FW_MEDIUM_X; j++) {
				ret = shsmartamp_i2c_write(ssm4329_fw_medium[j],
					SSM4329_FW_MEDIUM_Y,
					SSM4329_PROGRAM);
				if (ret != 0)
					break;
			}
		}
	}

	ret = shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_OFF],
		SSM4329_SW_CONTROL_Y, SSM4329_DATA);

	return ret;
}

static int shsmartamp_fw_check(void)
{
	int ret;
	int i;

	pr_debug("%s()\n", __func__);

	for (i = 0; i < SSM4329_FW_SIGNEL_X; i++) {
		ret = shsmartamp_i2c_read(ssm4329_fw_signel[i]);
		if (ret != 0)
			break;
	}

	return ret;
}

static int shsmartamp_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&smartamp_lock);

	if (shsmartamp_opened) {
		pr_err("%s(): busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	shsmartamp_opened = 1;
done:
	mutex_unlock(&smartamp_lock);
	return rc;
}

static int shsmartamp_release(struct inode *inode, struct file *file)
{


	mutex_lock(&smartamp_lock);
	shsmartamp_opened = 0;
	mutex_unlock(&smartamp_lock);
	return 0;
}

static struct file_operations shsmartamp_fops = {
	.owner   = THIS_MODULE,
	.open    = shsmartamp_open,
	.release = shsmartamp_release,
};

static struct miscdevice shsmartamp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "shsmartamp",
	.fops = &shsmartamp_fops,
};

void shsmartamp_control(int on)
{
	static int is_on;


	pr_debug("%s()\n", __func__);

	mutex_lock(&smartamp_lock);
	if (on && !is_on) {
		shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_ON],
			SSM4329_SW_CONTROL_Y, SSM4329_DATA);
		is_on = 1;
	} else if (!on && is_on){
		shsmartamp_i2c_write(ssm4329_sw_control[SSM4329_SW_OFF],
			SSM4329_SW_CONTROL_Y, SSM4329_DATA);
		is_on = 0;
		msleep(30);
	}
	mutex_unlock(&smartamp_lock);

	return ;
}

static int shsmartamp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	shsmartamp_client = client;

	ret = misc_register(&shsmartamp_device);
	if (ret) {
		pr_err("%s(): shsmartamp_device register failed\n", __func__);
	}

	shsmartamp_en = of_get_named_gpio(client->dev.of_node,
		"sharp,shsmartamp-en-gpio", 0);

	if (shsmartamp_en < -1) {
		pr_err("%s(): Get gpio info failed shsmartamp_en = %d\n",
			__func__, shsmartamp_en);
	} else {
		pr_debug("%s(): shsmartamp_en = %d\n", __func__, shsmartamp_en);
	}

	ret = gpio_request(shsmartamp_en, "shsmartamp");
	if (ret) {
		pr_err("%s(): Failed to request gpio %d\n", __func__,
			shsmartamp_en);
		goto err_free_gpio;
	}

	if (0 != shsmartamp_fw_init()) {
		pr_err("%s(): firmware init failed\n", __func__);
		shsmartamp_fw_check();
		goto err_free_gpio;
	} else {
		pr_err("%s(): firmware init successful\n", __func__);
	}

	return 0;

err_free_gpio:
	gpio_free(shsmartamp_en);
	shsmartamp_en = -1;
	return ret;
}

static int shsmartamp_remove(struct i2c_client *client)
{
	shsmartamp_client = i2c_get_clientdata(client);
	return 0;
}

static const struct i2c_device_id shsmartamp_id[] = {
	{ SHSMARTAMP_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id mdss_dsi_ctrl_dt_match[] = {
	{.compatible = "sharp,shsmartamp_i2c"},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_dsi_ctrl_dt_match);

static struct i2c_driver shsmartamp_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SHSMARTAMP_I2C_NAME,
		.of_match_table = mdss_dsi_ctrl_dt_match,
	},
	.class    = I2C_CLASS_HWMON,
	.probe    = shsmartamp_probe,
	.id_table = shsmartamp_id,
	.remove   = shsmartamp_remove,
};

static int __init shsmartamp_init(void)
{
	pr_debug("%s()\n", __func__);
	return i2c_add_driver(&shsmartamp_driver);
}

module_init(shsmartamp_init);

MODULE_DESCRIPTION("shsmartamp driver");
MODULE_LICENSE("GPL");
