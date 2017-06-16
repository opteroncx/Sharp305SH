#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/adnc-sensorhub-api.h>

struct dummy_driver_priv {
	int state;
	int i2chid;
};

static struct dummy_driver_priv sensor_priv;
void *sensorhub_aud_probe(void *priv)
{
	pr_info("Sensor-driver: Probe\n");
	return NULL;
}

void sensorhub_aud_intr(void *priv)
{
	pr_info("Sensor-driver: INTR for sensor-hub\n");
}

void sensorhub_aud_status(void *priv, u8 state)
{
	pr_info("Sensor-driver: MQ100 reset state %d\n", state);
}

static struct mq100_extension_cb sensorhub_cb = {
	.probe = sensorhub_aud_probe,
	.intr = sensorhub_aud_intr,
	.status = sensorhub_aud_status,
};

static int sensorhub_aud_init(void)
{
	int status;
	pr_info("MQ100: %s(): Module init\n", __func__);
	status = mq100_register_extensions(&sensorhub_cb, &sensor_priv);
	return status;
}

static void sensorhub_aud_exit(void)
{
	return;
}


module_init(sensorhub_aud_init);
module_exit(sensorhub_aud_exit);

MODULE_DESCRIPTION("Dummy sensorhub driver");
MODULE_AUTHOR("Pratik Khade <pkhade@audience.com>");
MODULE_VERSION("0.2.0");
MODULE_LICENSE("GPL");
