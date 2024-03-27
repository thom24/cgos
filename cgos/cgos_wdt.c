#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

#include "cgos.h"

#define CGOS_WATCHDOG_TIMEOUT 30

static unsigned int timeout = CGOS_WATCHDOG_TIMEOUT;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds. (>=0, default="
		 __MODULE_STRING(CGOS_WATCHDOG_TIMEOUT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define CGOS_CMD_WDT_TRIGGER	0x27
#define CGOS_CMD_WDT_INIT	0x28
#define		CGOS_WDT_DISABLED	0x00

#define CGOS_WDT_MODE_SINGLE_EVENT 0x02
#define CGOS_WDT_EVENT_RESET 0x02 /* system reset */
#define CGOS_WDT_EVENT_SMI   0x01

enum {
	CGOS_WDT_ACTION_SMI = 1,
	CGOS_WDT_ACTION_RESET,

};

struct cgos_wdt_data {
	struct cgos_device_data	*cgos;
	struct watchdog_device	wdd;
	bool running;
};

static int cgos_wdt_start(struct watchdog_device *wdd)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	struct cgos_device_data *cgos = wdt_data->cgos;
	u8 cmd[15], status;
	int ret;

	printk("%s: start with timeout=%d\n", __func__, wdd->timeout);
	cmd[0]	= CGOS_CMD_WDT_INIT;
	cmd[1]	= CGOS_WDT_MODE_SINGLE_EVENT;
	cmd[2]	= 1;
	cmd[2]	|= (CGOS_WDT_EVENT_RESET << 2);
//	cmd[2]  |= (CGOS_WDT_EVENT_SMI << 2);
	cmd[3]	= (wdd->timeout * 1000) & 0xFF;
	cmd[4]	= ((wdd->timeout * 1000) & 0xFF00) >> 8;
	cmd[5]	= ((wdd->timeout * 1000) & 0xFF0000) >> 8;
	cmd[12] = 0x00; /* delay = 0 */
	cmd[13] = 0x00; /* delay = 0 */
	cmd[14] = 0x00; /* delay = 0 */

	ret = cgos_command(cgos, &cmd[0], 15, NULL, 0, &status);
	if (ret)
		return ret;

	wdt_data->running = true;

	return 0;
}

static int cgos_wdt_stop(struct watchdog_device *wdd)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	struct cgos_device_data *cgos = wdt_data->cgos;
	u8 cmd[15], status;
	int ret;

	cmd[0] = CGOS_CMD_WDT_INIT;
	cmd[1] = CGOS_WDT_DISABLED;

	ret = cgos_command(cgos, &cmd[0], 15, NULL, 0, &status);
	if (ret)
		return ret;

	wdt_data->running = false;

	return 0;
}

static int cgos_wdt_keepalive(struct watchdog_device *wdd)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	struct cgos_device_data *cgos = wdt_data->cgos;
	u8 cmd = CGOS_CMD_WDT_TRIGGER;
	u8 status;

	return cgos_command(cgos, &cmd, 1, NULL, 0, &status);
}

static int cgos_wdt_set_timeout(struct watchdog_device *wdd,
				unsigned int timeout)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);

	wdd->timeout = timeout;

	if (wdt_data->running)
		return cgos_wdt_start(wdd);

	return 0;
}

static const struct watchdog_info cgos_wdt_info = {
	.identity	= "CGOS Watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE
};

static long cgos_wdt_ioctl(struct watchdog_device *wdd,
			   unsigned int cmd, unsigned long arg)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	void __user *argp = (void __user *)arg;
	int ret = -ENOIOCTLCMD;
	int __user *p = argp;

	switch (cmd) {
		case WDIOC_GETSUPPORT:
			return copy_to_user(argp, &cgos_wdt_info, sizeof(cgos_wdt_info)) ? -EFAULT : 0;
		default:
	}

	return ret;
}

static const struct watchdog_ops cgos_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= cgos_wdt_start,
	.stop		= cgos_wdt_stop,
	.ping		= cgos_wdt_keepalive,
	.set_timeout	= cgos_wdt_set_timeout,
	.ioctl		= cgos_wdt_ioctl,
};

static int cgos_wdt_probe(struct platform_device *pdev)
{
	struct cgos_device_data *cgos = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct cgos_wdt_data *wdt_data;
	struct watchdog_device *wdd;
	int ret;

	wdt_data = devm_kzalloc(dev, sizeof(*wdt_data), GFP_KERNEL);
	if (!wdt_data)
		return -ENOMEM;

	wdt_data->cgos = cgos;
	wdd = &wdt_data->wdd;
	wdd->parent = dev;

	wdd->info = &cgos_wdt_info;
	wdd->ops = &cgos_wdt_ops;

	watchdog_set_drvdata(wdd, wdt_data);
	watchdog_set_nowayout(wdd, nowayout);

	cgos_wdt_set_timeout(wdd, timeout);

	platform_set_drvdata(pdev, wdt_data);
	watchdog_stop_on_reboot(wdd);
	watchdog_stop_on_unregister(wdd);
	printk("%s: %d\n", __func__, __LINE__);

	ret = devm_watchdog_register_device(dev, wdd);
	if (ret)
		return ret;

	printk("%s: %d\n", __func__, __LINE__);
	dev_info(dev, "Watchdog registered with %ds timeout\n", wdd->timeout);

	return 0;
}

static struct platform_driver cgos_wdt_driver = {
	.driver		= {
		.name	= "cgos-wdt",
	},
	.probe		= cgos_wdt_probe,
};

module_platform_driver(cgos_wdt_driver);

MODULE_DESCRIPTION("CGOS Watchdog Driver");
MODULE_AUTHOR("Thomas Richard <thomas.richard@bootlin.com>");
MODULE_LICENSE("GPL");
