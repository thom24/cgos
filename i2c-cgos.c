// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Author: Thomas Richard <thomas.richard@bootlin.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include "cgos.h"

#define CGOS_I2C_CMD_START	0x40
#define CGOS_I2C_CMD_STAT	0x48
#define CGOS_I2C_CMD_DATA	0x50
#define CGOS_I2C_CMD_SPEED	0x58

#define CGOS_I2C_STAT_IDL	0x00
#define CGOS_I2C_STAT_DAT	0x01
#define CGOS_I2C_STAT_BUSY	0x02

#define CGOS_I2C_START	0x80
#define CGOS_I2C_STOP	0x40


#define CGOS_I2C_FREQ_UNIT_100KHZ	0xC0

enum {
	STATE_DONE = 0,
	STATE_INIT,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_ERROR,
};

struct cgos_i2c_data {
	struct cgos_device_data	*cgos;
	struct device		*dev;
	struct i2c_adapter	adap;
	struct i2c_msg		*msg;
	int			nmsgs;
	int			state;
};

static int cgos_i2c_get_status(struct i2c_adapter *adap)
{
	struct cgos_i2c_data *i2c = i2c_get_adapdata(adap);
	u8 cmd = CGOS_I2C_CMD_STAT | 4;// | (u8) (0x00010000 & ~0x07); /* bus 0 */
	u8 status;
	int ret;

	printk("%s: %d\n", __func__, __LINE__);
	ret = cgos_command(i2c->cgos, &cmd, 1, NULL, 0, &status);
	printk("%s: %d: ret= %d\n", __func__, __LINE__, ret);
	if (ret)
		return ret;

	printk("%s: %d: status=%x\n", __func__, __LINE__, status);
	return status;
}

#if 0
static int cgos_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct cgos_i2c_data *i2c = i2c_get_adapdata(adap);
	struct cgos_device_data *cgos = i2c->cgos;
	u8 cmd[4], status;
	int ret;

	printk("%s: %d: addr = %x\n", __func__, __LINE__, msg->addr);
	cmd[0] = CGOS_I2C_CMD_START |4;//| 0x00010000; /* bus 0 */
	cmd[1] = CGOS_I2C_START | 1;
	cmd[2] = 0;
	cmd[3] = 0xA0 & 0xFE;

	ret = cgos_command(cgos, &cmd[0], 4, NULL, 0, &status);
	if (ret)
		return ret;

	printk("%s: %d\n", __func__, __LINE__);
	do {
		ret = cgos_i2c_get_status(adap);
		if (ret < 0)
			return ret;
	} while(ret != CGOS_I2C_STAT_IDL);

	printk("%s: %d\n", __func__, __LINE__);
	cmd[0] = CGOS_I2C_CMD_START | 4;// | 0x00010000; /* bus 0 */
	cmd[1] = CGOS_I2C_START | 1;
	cmd[1] |= CGOS_I2C_STOP;
	cmd[2] = msg->len;
	cmd[3] = msg->addr | 0x01;

	printk("%s: %d\n", __func__, __LINE__);
	cgos_command(cgos, &cmd[0], 4, NULL, 0, &status);
	if (ret)
		return ret;

	do {
		ret = cgos_i2c_get_status(adap);
		if (ret < 0)
			return ret;
	} while(ret != CGOS_I2C_STAT_IDL);

	printk("%s: %d\n", __func__, __LINE__);
	if (ret == CGOS_I2C_STAT_DAT) {
		cmd[0] = CGOS_I2C_CMD_DATA | 4;
		ret = cgos_command(cgos, &cmd[0], 1, msg->buf, msg->len & 0x3F, &status);
		if (ret)
			return ret;

	printk("%s: %d: msg=%x\n", __func__, __LINE__, msg->buf[0]);
	}

	printk("%s: %d\n", __func__, __LINE__);
	return 0;
}
#endif

static int cgos_i2c_xfer_msg(struct i2c_adapter *adap)
{
	struct cgos_i2c_data *i2c = i2c_get_adapdata(adap);
	struct cgos_device_data *cgos = i2c->cgos;
	struct i2c_msg *msg = i2c->msg;
	u8 cmd[5], status;
	int ret = 0;

	printk("CgosI2CReadRegisterRaw_BC\n");
	printk("%s: %d: addr = %x\n", __func__, __LINE__, msg->addr);
	printk("%s: %d: msg->buf[0] = %x msg->len = %d flag_read=%d\n", __func__, __LINE__, msg->buf[0], msg->len, (msg->flags & I2C_M_RD));

	if (i2c->state == STATE_DONE)
		return ret;

	cmd[0] = CGOS_I2C_CMD_START | 4;//| 0x00010000; /* bus 0 */

	if (i2c->state == STATE_INIT || i2c->state == STATE_WRITE) {
		cmd[1] = CGOS_I2C_START;
		i2c->state = (msg->flags & I2C_M_RD) ? STATE_READ : STATE_WRITE;
		printk("STATE = %s\n", (msg->flags & I2C_M_RD) ? "STATE_READ" : "STATE_WRITE");
	} else
		cmd[1] = 0x00;

	cmd[3] = i2c_8bit_addr_from_msg(msg);

	if (i2c->nmsgs == 1)
		cmd[1] |= CGOS_I2C_STOP;

	if (i2c->state == STATE_WRITE) {
		cmd[1] |= 2;
		cmd[2] = 0x00; //size read
		cmd[4] = msg->buf[0];

		while(cgos_i2c_get_status(adap) == CGOS_I2C_STAT_BUSY){}

		ret =  cgos_command(cgos, &cmd[0], 5, NULL, 0, &status);
		if (!ret) {
			i2c->msg++;
			i2c->nmsgs--;
		}
	} else if (i2c->state == STATE_READ) {
		cmd[1] |= 1;
		cmd[2] = msg->len;//0x01; //size read

		while(cgos_i2c_get_status(adap) == CGOS_I2C_STAT_BUSY){}

		ret = cgos_command(cgos, &cmd[0], 4, NULL, 0, &status);
		if (ret)
			goto end;

		cmd[0] = CGOS_I2C_CMD_DATA | 4;
		while(cgos_i2c_get_status(adap) == CGOS_I2C_STAT_BUSY){}

		ret = cgos_command(cgos, &cmd[0], 1, msg->buf, msg->len, &status);
		if (!ret) {
			i2c->msg++;
			i2c->nmsgs--;
		}
	}

	if (i2c->nmsgs == 0)
		i2c->state = STATE_DONE;
end:
	return ret;
}


static int cgos_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct cgos_i2c_data *i2c = i2c_get_adapdata(adap);
	unsigned long timeout = jiffies + HZ;
	int ret;

	i2c->state = STATE_INIT;
	i2c->msg = msgs;
	i2c->nmsgs = num;

	printk("%s: %d: msg=%d\n", __func__, __LINE__, num);
	while (time_before(jiffies, timeout)) {
		ret = cgos_i2c_xfer_msg(adap);

		if (i2c->state == STATE_DONE || i2c->state == STATE_ERROR)
			return (i2c->state == STATE_DONE) ? num : ret;

		if (ret == 0)
			timeout = jiffies + HZ;
	}

	i2c->state = STATE_ERROR;

	return -ETIMEDOUT;
}

static int cgos_i2c_device_init(struct i2c_adapter *adap)
{
	struct cgos_i2c_data *i2c = i2c_get_adapdata(adap);
	struct cgos_device_data *cgos = i2c->cgos;
	u8 cmd[2], data, status;

	cmd[0] = CGOS_I2C_CMD_SPEED | 0;
	cmd[1] = CGOS_I2C_FREQ_UNIT_100KHZ | 4;

	return cgos_command(cgos, &cmd[0], 2, &data, 1, &status);
}

static u32 cgos_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR;
}

static const struct i2c_algorithm cgos_i2c_algorithm = {
	.master_xfer	= cgos_i2c_xfer,
	.functionality	= cgos_i2c_func,
};

static const struct i2c_adapter cgos_i2c_adapter = {
	.owner		= THIS_MODULE,
	.name		= "i2c-cgos",
	.class		= I2C_CLASS_DEPRECATED,
	.algo		= &cgos_i2c_algorithm,
};

static int cgos_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cgos_device_data *cgos = dev_get_drvdata(dev->parent);
	struct cgos_i2c_data *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->cgos = cgos;
	i2c->dev = &pdev->dev;
	i2c->adap = cgos_i2c_adapter;
	i2c->adap.dev.parent = &pdev->dev;
	i2c_set_adapdata(&i2c->adap, i2c);
	platform_set_drvdata(pdev, i2c);

	ret = cgos_i2c_device_init(&i2c->adap);
	if (ret)
		return ret;

	i2c->adap.nr = -1;
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret)
		return ret;

	dev_info(i2c->dev, "I2C bus initialized at %dkHz\n",
		 1000000);

	return 0;
}

static void cgos_i2c_remove(struct platform_device *pdev)
{
	struct cgos_i2c_data *i2c = platform_get_drvdata(pdev);

	/* remove adapter & data */
	i2c_del_adapter(&i2c->adap);
}

static struct platform_driver cgos_i2c_driver = {
	.driver = {
		.name = "cgos-i2c",
	},
	.probe		= cgos_i2c_probe,
	.remove_new	= cgos_i2c_remove,
};

module_platform_driver(cgos_i2c_driver);

MODULE_DESCRIPTION("CGOS I2C Driver");
MODULE_AUTHOR("Thomas Richard <thomas.richard@bootlin.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cgos_i2c");
