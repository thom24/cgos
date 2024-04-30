// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Author: Thomas Richard <thomas.richard@bootlin.com>
 */
#include <linux/dmi.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include "cgos.h"

#define GEN5_HCC_ACCESS 0x0C
#define GEN5_HCC_STROBE 0x00
#define GEN5_HCC_INDEX  0x02
#define     GEN5_HCC_INDEX_CBI_MSK    0xFC
#define     GEN5_HCC_INDEX_CBM_MSK    0x03
#define     GEN5_HCC_INDEX_CBM_MAN8   0x00
#define     GEN5_HCC_INDEX_CBM_AUTO32 0x03
#define GEN5_HCC_DATA   0x04
#define GEN5_HCC_ACCESS 0x0C

#define CGOS_GEN5_HCNM_IO_BASE	0x0E20
#define CGOS_GEN5_HCNM_IO_SIZE	0x0010

#define CGOS_GEN5_HCC_IO_BASE	0x0E00
#define CGOS_GEN5_HCC_IO_SIZE	0x0010

#define CGOS_GEN5_HCNM_CMD	0x00
#define		CGOS_GEN5_HCNM_CMD_IDLE		0x00
#define		CGOS_GEN5_HCNM_CMD_REQUEST	0x01
#define CGOS_GEN5_HCNM_DATA	0x01
#define CGOS_GEN5_HCNM_STATUS	0x02
#define		CGOS_GEN5_HCNM_STATUS_FREE	0x03
#define CGOS_GEN5_HCNM_ACCESS	0x04
#define		CGOS_GEN5_HCMN_ACCESS_GAINED	0x00

#define CGOS_GEN5_HCC_STROBE	0x00
#define CGOS_GEN5_HCC_INDEX	0x02
#define		CGOS_GEN5_HCC_INDEX_CBI_MSK	0xFC
#define		CGOS_GEN5_HCC_INDEX_CBM_MSK	0x03
#define		CGOS_GEN5_HCC_INDEX_CBM_MAN8	0x00
#define		CGOS_GEN5_HCC_INDEX_CBM_AUTO32	0x03
#define CGOS_GEN5_HCC_DATA	0x04
#define CGOS_GEN5_HCC_ACCESS	0x0C

#define CGOS_STATUS_DATA_READY	0x00
#define CGOS_STATUS_CMD_READY	BIT(6)
#define CGOS_STATUS_ERROR	BIT(7) | BIT(6)
#define CGOS_STATUS_MASK	(CGOS_STATUS_CMD_READY | CGOS_STATUS_ERROR)

#define CGOS_DATA_COUNT_MASK	0x1F
#define CGOS_ERROR_CODE_MASK	0x1F

static struct platform_device *cgos_pdev;

static int cgos_hcnm_detect_device(struct cgos_device_data *cgos)
{
	int ret = 0;
	int i;

	for (i = 0; i < 100000 ; i++) {
		if (ioread16(cgos->io_hcnm + CGOS_GEN5_HCNM_STATUS) == CGOS_GEN5_HCNM_STATUS_FREE) {
			if (ioread32(cgos->io_hcnm + CGOS_GEN5_HCNM_ACCESS))
				ret = -ENODEV;

			break;
		}
	}

	return ret;
}

static u8 cgos_hcnm_command(struct cgos_device_data *cgos, u8 cmd)
{
	u8 ret;

	while (ioread8(cgos->io_hcnm + CGOS_GEN5_HCNM_CMD) != CGOS_GEN5_HCNM_CMD_IDLE)
		;

	iowrite8(cmd, cgos->io_hcnm + CGOS_GEN5_HCNM_CMD);

	while (ioread8(cgos->io_hcnm + CGOS_GEN5_HCNM_CMD) != CGOS_GEN5_HCNM_CMD_IDLE)
		;

	ret = ioread8(cgos->io_hcnm + CGOS_GEN5_HCNM_DATA);

	iowrite8(CGOS_GEN5_HCNM_STATUS_FREE, cgos->io_hcnm + CGOS_GEN5_HCNM_STATUS);

	return ret;
}

static int cgos_hcnm_create_session(struct cgos_device_data *cgos)
{
	unsigned int ret = cgos_hcnm_detect_device(cgos);

	if (ret)
		return dev_err_probe(cgos->dev, ret, "device not found\n");

	cgos->session_id = cgos_hcnm_command(cgos, CGOS_GEN5_HCNM_CMD_REQUEST);

	/* we got a bad session id for the controller, we cannot communicate
	 * with it.
	 */
	if ((cgos->session_id < 0x02) || (cgos->session_id > 0xFE)) {
		cgos->session_id = 0;
		return dev_err_probe(cgos->dev, -ENODEV, "failed to create a hcnm session\n");
	}

	return 0;
}

static void cgos_hcnm_release_session(struct cgos_device_data *cgos)
{
	cgos_hcnm_detect_device(cgos);

	if (cgos_hcnm_command(cgos, cgos->session_id) != cgos->session_id)
		dev_err(cgos->dev, "failed to release hcnm session\n");
}

#define exec_until(x, y)		\
({					\
	int __loop, __ret;		\
	__loop = 0x2000;		\
	do {				\
		x;			\
		__loop--;		\
	} while (y && __loop != 0);	\
	if (!__loop)			\
		__ret = -ETIMEDOUT;	\
	else				\
		__ret = 0;		\
	__ret;				\
})

int cgos_command_gen5(struct cgos_device_data *cgos,
		      u8 *cmd, u8 cmd_size, u8 *data, u8 data_size, u8 *status)
{
	u8 checksum = 0, data_checksum = 0;
	int mode_change = -1;
	int ret, i;

	mutex_lock(&cgos->lock);

	/* request access */
	ret = exec_until(iowrite8(cgos->session_id, cgos->io_hcc + CGOS_GEN5_HCC_ACCESS),
			 ioread8(cgos->io_hcc + CGOS_GEN5_HCC_ACCESS) != cgos->session_id);
	if (ret)
		goto out;

	/* write command packet */
	ret = exec_until(, ioread8(cgos->io_hcc + CGOS_GEN5_HCC_STROBE) != 0);

	if (cmd_size <= 2) {
		iowrite8(CGOS_GEN5_HCC_INDEX_CBM_MAN8, cgos->io_hcc + CGOS_GEN5_HCC_INDEX);
	} else {
		iowrite8(CGOS_GEN5_HCC_INDEX_CBM_AUTO32, cgos->io_hcc + CGOS_GEN5_HCC_INDEX);
		if ((cmd_size % 4) != 0x03)
			mode_change = (cmd_size & 0xFFFC) - 1;
	}

	for (i = 0; i < cmd_size; i++) {
		iowrite8(cmd[i], cgos->io_hcc + CGOS_GEN5_HCC_DATA + (i % 4));
		checksum ^= cmd[i];
		if (mode_change == i){
			iowrite8((i + 1) | CGOS_GEN5_HCC_INDEX_CBM_MAN8, cgos->io_hcc + CGOS_GEN5_HCC_INDEX);
		}
	}

	/* append checksum byte */
	iowrite8(checksum, cgos->io_hcc + CGOS_GEN5_HCC_DATA + (i % 4));

	/* perform command strobe */
	iowrite8(cgos->session_id, cgos->io_hcc + CGOS_GEN5_HCC_STROBE);

	/* rewind hcc buffer index */
	iowrite8(CGOS_GEN5_HCC_INDEX_CBM_AUTO32, cgos->io_hcc + CGOS_GEN5_HCC_INDEX);

	/* wait command completion */
	ret = exec_until(, ioread8(cgos->io_hcc + CGOS_GEN5_HCC_STROBE) != 0);
	if (ret)
		goto release;

	/* check command status */
	checksum = *status = ioread8(cgos->io_hcc + CGOS_GEN5_HCC_DATA);
	switch (*status & CGOS_STATUS_MASK) {
	case CGOS_STATUS_DATA_READY:
		if (*status > data_size)
			*status = data_size;
		for (i = 0; i < *status; i++) {
			data[i] = ioread8(cgos->io_hcc + CGOS_GEN5_HCC_DATA + ((i + 1) % 4));
			checksum ^= data[i];
		}
		data_checksum = ioread8(cgos->io_hcc + CGOS_GEN5_HCC_DATA + ((i + 1) % 4));
		*status = *status & CGOS_DATA_COUNT_MASK;
		break;
	case CGOS_STATUS_ERROR:
	case CGOS_STATUS_CMD_READY:
		data_checksum = ioread8(cgos->io_hcc + CGOS_GEN5_HCC_DATA + 1);
		*status = *status & CGOS_ERROR_CODE_MASK;
		if ((*status & CGOS_STATUS_MASK) == (CGOS_STATUS_ERROR))
			ret = -EIO;
		break;
	default:
		data_checksum = ioread8(cgos->io_hcc + CGOS_GEN5_HCC_DATA + 1);
		*status = *status & CGOS_ERROR_CODE_MASK;
		ret = -EIO;
		break;
	}

	/* checksum verification */
	if (ret == 0 && data_checksum != checksum)
		ret = -EIO;

release:
	/* release */
	iowrite8(cgos->session_id, cgos->io_hcc + CGOS_GEN5_HCC_ACCESS);

out:
	mutex_unlock(&cgos->lock);

	return ret;
}

static struct mfd_cell cgos_devs[] = {
	{
		.name = "cgos-wdt",
	},
	{
		.name = "cgos-gpio",
	},
	{
		.name = "cgos-i2c",
	},
};

static int cgos_register_cells_gen5(struct cgos_device_data *cgos)
{
	return mfd_add_devices(cgos->dev, -1, cgos_devs, ARRAY_SIZE(cgos_devs), NULL, 0, NULL);
}

static int cgos_map_gen5(struct platform_device *pdev, struct cgos_device_data *cgos)
{
	struct device *dev = &pdev->dev;
	struct resource *ioport;

	ioport = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!ioport)
		return -EINVAL;

	cgos->io_hcnm = devm_ioport_map(dev, ioport->start,
					resource_size(ioport));
	if (!cgos->io_hcnm)
		return -ENOMEM;

	ioport = platform_get_resource(pdev, IORESOURCE_IO, 1);
	if (!ioport)
		return -EINVAL;

	cgos->io_hcc = devm_ioport_map(dev, ioport->start,
				       resource_size(ioport));
	if (!cgos->io_hcc)
		return -ENOMEM;

	return 0;
}

static int cgos_create_platform_device_gen5(const struct dmi_system_id *id)
{
	const struct cgos_platform_data *pdata = id->driver_data;
	int ret;

	cgos_pdev = platform_device_alloc("cgos", -1);
	if (!cgos_pdev)
		return -ENOMEM;

	ret = platform_device_add_data(cgos_pdev, pdata, sizeof(*pdata));
	if (ret)
		goto err;

	ret = platform_device_add_resources(cgos_pdev, &pdata->ioresource_hcnm, 2);
	if (ret)
		goto err;

	ret = platform_device_add(cgos_pdev);
	if (ret)
		platform_device_put(cgos_pdev);

	return 0;

err:
	return ret;
}

static const struct cgos_platform_data cgos_platform_data_gen5 = {
	.ioresource_hcnm = {
		.start  = CGOS_GEN5_HCNM_IO_BASE,
		.end    = CGOS_GEN5_HCNM_IO_BASE + CGOS_GEN5_HCNM_IO_SIZE,
		.flags  = IORESOURCE_IO,
	},
	.ioresource_hcc	= {
		.start  = CGOS_GEN5_HCC_IO_BASE,
		.end    = CGOS_GEN5_HCC_IO_BASE + CGOS_GEN5_HCC_IO_SIZE,
		.flags  = IORESOURCE_IO,
	},
	.command = cgos_command_gen5,
	.register_cells = cgos_register_cells_gen5,
	.map = cgos_map_gen5,
	.init = cgos_hcnm_create_session,
	.close = cgos_hcnm_release_session,
};

static ssize_t cgos_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cgos_device_data *cgos = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", cgos->info.version);
}

static DEVICE_ATTR_RO(cgos_version);

static struct attribute *cgos_attributes[] = {
	&dev_attr_cgos_version.attr,
	NULL
};

static const struct attribute_group cgos_attr_group = {
	.attrs = cgos_attributes,
};

static int cgos_get_info(struct cgos_device_data *cgos)
{
	u8 cmd = CGOS_CGBC_CMD_GET_FW_REV;
	u8 data[4], status;
	int ret;

	/* struct for the CGOS_CGBC_CMD_GET_FW_REV command is 4 bytes long
	 * but in the code only 3 bytes are read
	 */
	ret = cgos_command(cgos, &cmd, 1, &data[0], 4, &status);
	if (ret)
		return ret;

	cgos->info.feature = data[0];
	cgos->info.major = data[1];
	cgos->info.minor = data[2];
	ret = snprintf(cgos->info.version, sizeof(cgos->info.version), "CGBCP%c%c%c",
		       cgos->info.feature, cgos->info.major,
		       cgos->info.minor);

	if (ret < 0)
		return ret;

	return 0;
}

static int cgos_register_cells(struct cgos_device_data *pld)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(pld->dev);

	return pdata->register_cells(pld);
}

static int cgos_detect_device(struct cgos_device_data *cgos)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);
	int ret;

	if (pdata->init) {
		ret = pdata->init(cgos);
		if (ret)
			return ret;
	}

	ret = cgos_get_info(cgos);
	if (ret)
		return ret;

	dev_info(cgos->dev, "Found Congatec Board Controller - %s\n",
		 cgos->info.version);

	ret = sysfs_create_group(&cgos->dev->kobj, &cgos_attr_group);
	if (ret)
		return ret;

	ret = cgos_register_cells(cgos);
	if (ret)
		sysfs_remove_group(&cgos->dev->kobj, &cgos_attr_group);

	return ret;
}

int cgos_command(struct cgos_device_data *cgos, u8 *cmd, u8 cmd_size, u8 *data, u8 data_size, u8 *status)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	return pdata->command(cgos, cmd, cmd_size, data, data_size, status);
}
EXPORT_SYMBOL_GPL(cgos_command);

static int cgos_probe(struct platform_device *pdev)
{
	const struct cgos_platform_data *pdata;
	struct device *dev = &pdev->dev;
	struct cgos_device_data *cgos;
	int ret;

	pdata = dev_get_platdata(dev);

	cgos = devm_kzalloc(dev, sizeof(*cgos), GFP_KERNEL);
	if (!cgos)
		return -ENOMEM;

	ret = pdata->map(pdev, cgos);
	if (ret)
		return ret;

	mutex_init(&cgos->lock);

	cgos->dev = dev;

	platform_set_drvdata(pdev, cgos);

	return cgos_detect_device(cgos);
}

static void cgos_remove(struct platform_device *pdev)
{
	struct cgos_device_data *cgos = platform_get_drvdata(pdev);
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	if (pdata->close) {
		/* it fails, but it seems not to be called in the original driver */
		pdata->close(cgos);
	}

	sysfs_remove_group(&cgos->dev->kobj, &cgos_attr_group);

	mfd_remove_devices(&pdev->dev);
}

static struct platform_driver cgos_driver = {
	.driver		= {
		.name	= "cgos",
	},
	.probe		= cgos_probe,
	.remove_new	= cgos_remove,
};

static const struct dmi_system_id cgos_dmi_table[] __initconst = {
	{
		.ident = "SA7",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "congatec"),
			DMI_MATCH(DMI_BOARD_NAME, "conga-SA7"),
		},
		.callback = cgos_create_platform_device_gen5,
		.driver_data = (void *)&cgos_platform_data_gen5,
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, cgos_dmi_table);

static int __init cgos_init(void)
{
	if (!dmi_check_system(cgos_dmi_table))
		return -ENODEV;

	return platform_driver_register(&cgos_driver);
}

static void __exit cgos_exit(void)
{
	platform_device_unregister(cgos_pdev);
	platform_driver_unregister(&cgos_driver);
}

module_init(cgos_init);
module_exit(cgos_exit);

MODULE_DESCRIPTION("CGOS Core Driver");
MODULE_AUTHOR("Thomas Richard <thomas.richard@bootlin.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cgos-core");
