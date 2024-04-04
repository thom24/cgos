// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Author: Thomas Richard <thomas.richard@bootlin.com>
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>

#include "cgos.h"

#define CGOS_GPIO_NGPIO	8

#define CGOS_GPIO_CMD_GET	0x64
#define CGOS_GPIO_CMD_SET	0x65
#define CGOS_GPIO_CMD_DIR_GET	0x66
#define CGOS_GPIO_CMD_DIR_SET	0x67

struct cgos_gpio_data {
	struct gpio_chip	chip;
	struct cgos_device_data	*cgos;
	struct mutex lock;
};

static int cgos_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct cgos_gpio_data *gpio = gpiochip_get_data(chip);
	struct cgos_device_data *cgos = gpio->cgos;
	u8 cmd[3], val, status;
	int ret;

	cmd[0] = CGOS_GPIO_CMD_GET;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);

	if (ret)
		return ret;
	else
		return (int)((val & (u8)BIT(offset)) >> offset);
}

static void cgos_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct cgos_gpio_data *gpio = gpiochip_get_data(chip);
	struct cgos_device_data *cgos = gpio->cgos;
	u8 cmd[3], val, status;
	int ret;

	mutex_lock(&gpio->lock);

	cmd[0] = CGOS_GPIO_CMD_GET;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);
	if (ret) {
		mutex_unlock(&gpio->lock);
		return;
	}

	cmd[0] = CGOS_GPIO_CMD_SET;
	if (value)
		cmd[2] = val | BIT(offset);
	else
		cmd[2] = val & ~((u8) BIT(offset));

	cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);

	mutex_unlock(&gpio->lock);
}

static int cgos_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct cgos_gpio_data *gpio = gpiochip_get_data(chip);
	struct cgos_device_data *cgos = gpio->cgos;
	u8 cmd[3], val, status;
	int ret;

	mutex_lock(&gpio->lock);

	cmd[0] = CGOS_GPIO_CMD_DIR_GET;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);
	if (ret) {
		mutex_unlock(&gpio->lock);
		return ret;
	}

	cmd[0] = CGOS_GPIO_CMD_DIR_SET;
	cmd[2] = val & ~(BIT(offset));

	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);

	mutex_unlock(&gpio->lock);

	return ret;
}

static int cgos_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct cgos_gpio_data *gpio = gpiochip_get_data(chip);
	struct cgos_device_data *cgos = gpio->cgos;
	u8 cmd[3], val, status;
	int ret;

	mutex_lock(&gpio->lock);

	cmd[0] = CGOS_GPIO_CMD_DIR_GET;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);
	if (ret) {
		mutex_unlock(&gpio->lock);
		return ret;
	}

	cmd[0] = CGOS_GPIO_CMD_DIR_SET;
	cmd[2] = val | BIT(offset);
	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);

	mutex_unlock(&gpio->lock);

	if (!ret)
		cgos_gpio_set(chip, offset, value);

	return ret;
}

static int cgos_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct cgos_gpio_data *gpio = gpiochip_get_data(chip);
	struct cgos_device_data *cgos = gpio->cgos;
	u8 cmd[3], val, status;
	int ret;

	cmd[0] = CGOS_GPIO_CMD_DIR_GET;
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	ret = cgos_command(cgos, &cmd[0], sizeof(cmd), &val, 1, &status);
	if (ret)
		return ret;

	if ((val & BIT(offset)) >> offset)
		return GPIO_LINE_DIRECTION_OUT;
	else
		return GPIO_LINE_DIRECTION_IN;
}

static int cgos_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cgos_device_data *cgos = dev_get_drvdata(dev->parent);
	struct cgos_gpio_data *gpio;
	struct gpio_chip *chip;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->cgos = cgos;

	platform_set_drvdata(pdev, gpio);

	chip = &gpio->chip;
	chip->label = "gpio-cgos";
	chip->owner = THIS_MODULE;
	chip->parent = dev;
	chip->base = -1;
	chip->direction_input = cgos_gpio_direction_input;
	chip->direction_output = cgos_gpio_direction_output;
	chip->get_direction = cgos_gpio_get_direction;
	chip->get = cgos_gpio_get;
	chip->set = cgos_gpio_set;
	chip->ngpio = CGOS_GPIO_NGPIO;

	mutex_init(&gpio->lock);

	ret = devm_gpiochip_add_data(dev, chip, gpio);
	if (ret)
		return dev_err_probe(dev, ret, "Could not register GPIO chip\n");

	dev_info(dev, "GPIO functionality initialized with %d pins\n",
		 chip->ngpio);

	return 0;
}

static struct platform_driver cgos_gpio_driver = {
	.driver = {
		.name = "cgos-gpio",
	},
	.probe		= cgos_gpio_probe,
};

module_platform_driver(cgos_gpio_driver);

MODULE_DESCRIPTION("CGOS GPIO Driver");
MODULE_AUTHOR("Thomas Richard <thomas.richard@bootlin.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cgos-gpio");
