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

#define GEN5_HCNM_STATUS  0x02
#define GEN5_HCNM_ACCESS  0x04
#define GEN5_HCNM_FREE    0x0003
#define GEN5_HCNM_GAINED  0x00000000
#define GEN5_HCNM_COMMAND 0x00
#define GEN5_HCNM_IDLE    0x00
#define GEN5_HCNM_REQUEST 0x01
#define GEN5_HCNM_DATA    0x01

#define GEN5_HCC_ACCESS 0x0C
#define GEN5_HCC_STROBE 0x00
#define GEN5_HCC_INDEX  0x02
#define     GEN5_HCC_INDEX_CBI_MSK    0xFC
#define     GEN5_HCC_INDEX_CBM_MSK    0x03
#define     GEN5_HCC_INDEX_CBM_MAN8   0x00
#define     GEN5_HCC_INDEX_CBM_AUTO32 0x03
#define GEN5_HCC_DATA   0x04
#define GEN5_HCC_ACCESS 0x0C


#define HCNM_BASE 0x0E20
#define HCNM_SIZE 0x0010

#define HCC0_BASE 0x0E00
#define HCC0_SIZE 0x0010

#define CGOS_HCNM_IO_BASE HCNM_BASE
#define CGOS_HCNM_IO_END  HCNM_BASE + HCNM_SIZE

#define CGOS_HCC_IO_BASE HCC0_BASE
#define CGOS_HCC_IO_END  HCC0_BASE + HCC0_SIZE

#define CGOS_HCNM_COMMAND	GEN5_HCNM_COMMAND
#define CGOS_HCNM_DATA		GEN5_HCNM_DATA
#define CGOS_HCNM_STATUS	GEN5_HCNM_STATUS
#define CGOS_HCNM_ACCESS	GEN5_HCNM_ACCESS

#define CGOS_HCNM_IDLE		GEN5_HCNM_IDLE
#define CGOS_HCNM_REQUEST	GEN5_HCNM_REQUEST

#define CGOS_HCNM_FREE		GEN5_HCNM_FREE
#define CGOS_HCNM_GAINED	GEN5_HCNM_GAINED

#define CGOS_HCC_ACCESS			GEN5_HCC_ACCESS
#define CGOS_HCC_STROBE			GEN5_HCC_STROBE
#define CGOS_HCC_INDEX			GEN5_HCC_INDEX
#define CGOS_HCC_INDEX_CBI_MSK		GEN5_HCC_INDEX_CBI_MSK
#define CGOS_HCC_INDEX_CBM_MSK		GEN5_HCC_INDEX_CBM_MSK
#define CGOS_HCC_INDEX_CBM_MAN8		GEN5_HCC_INDEX_CBM_MAN8
#define CGOS_HCC_INDEX_CBM_AUTO32	GEN5_HCC_INDEX_CBM_AUTO32


#define CGOS_HCC_DATA   0x04


#define CGOS_CGBC_ERR_BIT     7                                     /* error flag */

#define CGOS_CGBC_BSY_BIT     7                             /* busy flag         */
#define CGOS_CGBC_RDY_BIT     6                             /* ready flag        */
#define CGOS_CGBC_STAT_MSK    ((1<<CGOS_CGBC_BSY_BIT)|(1<<CGOS_CGBC_RDY_BIT)) /* state msk */
#define CGOS_CGBC_IDL_STAT    ((0<<CGOS_CGBC_BSY_BIT)|(0<<CGOS_CGBC_RDY_BIT)) /* IDLE      */
#define CGOS_CGBC_BSY_STAT    ((1<<CGOS_CGBC_BSY_BIT)|(0<<CGOS_CGBC_RDY_BIT)) /* BUSY      */
#define CGOS_CGBC_RDY_STAT    ((0<<CGOS_CGBC_BSY_BIT)|(1<<CGOS_CGBC_RDY_BIT)) /* READY     */
#define CGOS_CGBC_ERR_STAT    ((1<<CGOS_CGBC_BSY_BIT)|(1<<CGOS_CGBC_RDY_BIT)) /* ERROR     */

#define CGOS_CGBC_DAT_STAT    ((0<<CGOS_CGBC_ERR_BIT)|(0<<CGOS_CGBC_RDY_BIT)) /* DATA READY */

#define CGOS_CGBC_DAT_CNT_MSK 0x1F                          /* data count        */
#define CGOS_CGBC_ERR_COD_MSK 0x1F                          /* error code        */

static int cgos_hcnm_detect_device(struct cgos_device_data *cgos)
{
	int i;

	for (i = 0; i < 100000 ; i++) {
		if (ioread16(cgos->io_hcnm + CGOS_HCNM_STATUS) == CGOS_HCNM_FREE) {
			if (!ioread32(cgos->io_hcnm + GEN5_HCNM_ACCESS))
				return 0;
			else
				break;
		}
	}

	return -ENODEV;
}

static u8 cgos_hcnm_command(struct cgos_device_data *cgos, u8 cmd)
{
	u8 ret;

	while (ioread8(cgos->io_hcnm + CGOS_HCNM_COMMAND) != CGOS_HCNM_IDLE)

	iowrite8(cmd, cgos->io_hcnm + CGOS_HCNM_COMMAND);

	while (ioread8(cgos->io_hcnm + CGOS_HCNM_COMMAND) != CGOS_HCNM_IDLE)

	ret = ioread8(cgos->io_hcnm + GEN5_HCNM_DATA);

	iowrite8(CGOS_HCNM_FREE, cgos->io_hcnm + CGOS_HCNM_STATUS);

	return ret;
}

static int cgos_hcnm_create_session(struct cgos_device_data *cgos)
{
	unsigned int ret = cgos_hcnm_detect_device(cgos);

	if (ret)
		return dev_err_probe(cgos->dev, ret, "device not found\n");

	cgos->session_id = cgos_hcnm_command(cgos, CGOS_HCNM_REQUEST);

	/* we got a bad session id for the controller, we cannot communicate
	 * with it.
	 */
	if ((cgos->session_id < 0x02) || ( cgos->session_id > 0xFE)) {
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
	if (cgos_command(cgos, &cmd, 1, &data[0], 4, &status))
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

static int cgos_detect_device(struct cgos_device_data *cgos)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);
	int ret;

	if (pdata->hcnm) {
		ret = cgos_hcnm_create_session(cgos);
		if (ret)
			return ret;
	}

	ret = cgos_get_info(cgos);
	if (ret)
		return ret;

	dev_info(cgos->dev, "Found Congatec Board Controller - %s\n",
		 cgos->info.version);

	return sysfs_create_group(&cgos->dev->kobj, &cgos_attr_group);
}


#define macro(x,y)			\
({					\
	int __loop, __ret;		\
	__loop = 0x2000;			\
	do {				\
		x;			\
		__loop--;		\
	} while(y && __loop != 0);	\
	if (!__loop)			\
		__ret = -ETIMEDOUT;	\
	else				\
		__ret = 0;		\
	__ret;				\
})

unsigned int cgos_command_generic(struct cgos_device_data *cgos,
				  u8 *cmd, u8 cmd_size, u8 *data, u8 data_size, u8 *status)
{
	u8 checksum = 0, data_checksum = 0;
	int mode_change = -1;
	int ret, i;

	mutex_lock(&cgos->lock);

	/* request access */
	ret = macro(iowrite8(cgos->session_id, cgos->io_hcc + CGOS_HCC_ACCESS),
		    ioread8(cgos->io_hcc + CGOS_HCC_ACCESS) != cgos->session_id);
	if (ret)
		return ret;

	/* write command packet */
	ret = macro(, ioread8(cgos->io_hcc + CGOS_HCC_STROBE) != 0);

	if (cmd_size <= 2) {
		iowrite8(CGOS_HCC_INDEX_CBM_MAN8, cgos->io_hcc + CGOS_HCC_INDEX);
	} else {
		iowrite8(CGOS_HCC_INDEX_CBM_AUTO32, cgos->io_hcc + CGOS_HCC_INDEX);
		if((cmd_size % 4) != 0x03)
			mode_change = (cmd_size & 0xFFFC) - 1;
	}

	for (i = 0; i < cmd_size; i++) {
		iowrite8(cmd[i] , cgos->io_hcc + GEN5_HCC_DATA + (i % 4));
		checksum ^= cmd[i];
		if (mode_change == i)
			iowrite8((i + 1) | CGOS_HCC_INDEX_CBM_MAN8, cgos->io_hcc + CGOS_HCC_INDEX );
	}

	/* append checksum byte */
	iowrite8(checksum, cgos->io_hcc + GEN5_HCC_DATA + (i % 4));

	/* perform command strobe */
	iowrite8(cgos->session_id, cgos->io_hcc + CGOS_HCC_STROBE);

	/* rewind hcc buffer index */
	iowrite8(CGOS_HCC_INDEX_CBM_AUTO32, cgos->io_hcc + CGOS_HCC_INDEX);

	/* wait command completion */
	ret = macro(,ioread8(cgos->io_hcc + CGOS_HCC_STROBE) != 0);
	if (ret)
		return ret;

	/* check command status */
	checksum = *status = ioread8(cgos->io_hcc + CGOS_HCC_DATA);
	switch (*status & CGOS_CGBC_STAT_MSK)
	{
		case CGOS_CGBC_DAT_STAT:
			if(*status > data_size)
				*status = data_size;
			for (i = 0; i < *status; i++) {
				data[i] = ioread8(cgos->io_hcc + CGOS_HCC_DATA + ((i + 1) % 4));
				checksum ^= data[i];
			}
			data_checksum = ioread8(cgos->io_hcc + CGOS_HCC_DATA + ((i + 1) % 4));
			*status = *status & CGOS_CGBC_DAT_CNT_MSK;
			break;
		case CGOS_CGBC_ERR_STAT:
		case CGOS_CGBC_RDY_STAT:
			data_checksum = ioread8(cgos->io_hcc + CGOS_HCC_DATA + 1);
			*status = *status & CGOS_CGBC_ERR_COD_MSK;
			if ((*status & CGOS_CGBC_STAT_MSK) == CGOS_CGBC_ERR_STAT)
				ret = -1;
			break;
		default:
			data_checksum = ioread8(cgos->io_hcc + CGOS_HCC_DATA + 1);
			*status = *status & CGOS_CGBC_ERR_COD_MSK;
			ret = -1;
			break;
	}

	/* release */
	iowrite8(cgos->session_id, cgos->io_hcc + CGOS_HCC_ACCESS);

	/* checksum verification */
	if (ret == 0 && data_checksum != checksum)
		ret = -1;

	mutex_unlock(&cgos->lock);

	return ret;
}

int gcos_command(struct cgos_device_data *cgos, u8 *cmd, u8 cmd_size, u8 *data, u8 data_size, u8 *status)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	return pdata->command(cgos, cmd, cmd_size, data, data_size, status);
}
EXPORT_SYMBOL_GPL(cgos_command);

static struct mfd_cell cgos_devs[] = {
	{
		.name = "cgos-wdt",
	},
};

static int cgos_register_cells_generic(struct cgos_device_data *cgos)
{
	return mfd_add_devices(cgos->dev, -1, cgos_devs, ARRAY_SIZE(cgos_devs), NULL, 0, NULL);
}

static const struct cgos_platform_data cgos_platform_data_generic = {
	.ioresource_hcnm = {
		.start  = CGOS_HCNM_IO_BASE,
		.end    = CGOS_HCNM_IO_END,
		.flags  = IORESOURCE_IO,
	},
	.ioresource_hcc	= {
		.start  = CGOS_HCC_IO_BASE,
		.end    = CGOS_HCC_IO_END,
		.flags  = IORESOURCE_IO,
	},
	.command = cgos_command_generic,
	.register_cells = cgos_register_cells_generic,
};

static struct platform_device *cgos_pdev;

static int cgos_create_platform_device_generic(const struct dmi_system_id *id)
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

static int cgos_probe(struct platform_device *pdev)
{
	const struct cgos_platform_data *pdata;
	struct device *dev = &pdev->dev;
	struct cgos_device_data *cgos;
	struct resource *ioport;
	int ret;
	printk("%s: %d\n", __func__, __LINE__);

	pdata = dev_get_platdata(dev);

	cgos = devm_kzalloc(dev, sizeof(*cgos), GFP_KERNEL);
	if (!cgos)
		return -ENOMEM;

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

	cgos->dev = dev;

	platform_set_drvdata(pdev, cgos);

	return cgos_detect_device(cgos);
}

static void cgos_remove(struct platform_device *pdev)
{
	struct cgos_device_data *cgos = platform_get_drvdata(pdev);
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	if (pdata->hcnm) {
		/* it fails, but it seems not to be called in the original driver */
		cgos_hcnm_release_session(cgos);
	}

	sysfs_remove_group(&cgos->dev->kobj, &cgos_attr_group);

	printk("%s: %d\n", __func__, __LINE__);
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
		.callback = cgos_create_platform_device_generic,
		.driver_data = (void *)&cgos_platform_data_generic,
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
