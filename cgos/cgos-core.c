// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Author: Thomas Richard <thomas.richard@bootlin.com>
 */
#include <linux/dmi.h>
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

#define CGOS_HCC_ACCESS		GEN5_HCC_ACCESS

static void cgos_get_hardware_mutex(struct cgos_device_data *cgos)
{
	do {
		iowrite8(cgos->session_id, cgos->io_hcc + CGOS_HCC_ACCESS);
	} while (ioread8(cgos->io_hcc + CGOS_HCC_ACCESS) != cgos->session_id);
}

static void cgos_release_hardware_mutex(struct cgos_device_data *cgos)
{
	iowrite8(cgos->session_id, cgos->io_hcc + CGOS_HCC_ACCESS);
}

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

	u8 ret = cgos_hcnm_command(cgos, cgos->session_id);

	if (cgos_hcnm_command(cgos, cgos->session_id) != cgos->session_id)
		dev_err(cgos->dev, "failed to release hcnm session\n");
}

static void cgos_get_mutex(struct cgos_device_data *cgos)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	mutex_lock(&cgos->lock);
	pdata->get_hardware_mutex(cgos);
}

static void cgos_release_mutex(struct cgos_device_data *cgos)
{
	const struct cgos_platform_data *pdata = dev_get_platdata(cgos->dev);

	pdata->release_hardware_mutex(cgos);
	mutex_unlock(&cgos->lock);
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
	.get_hardware_mutex = cgos_get_hardware_mutex,
	.release_hardware_mutex = cgos_release_hardware_mutex,
};

static struct platform_device *cgos_pdev;

static int cgos_create_platform_device(const struct dmi_system_id *id)
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

	return cgos_hcnm_create_session(cgos);
}

static void cgos_remove(struct platform_device *pdev)
{
	struct cgos_device_data *cgos = platform_get_drvdata(pdev);

	/* it fails, but it seems not to be called in the original driver */
	if (cgos->session_id)
		cgos_hcnm_release_session(cgos);

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
		.callback = cgos_create_platform_device,
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
