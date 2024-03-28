#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

#include "cgos.h"

#define CGOS_WDT_DEFAULT_TIMEOUT	30
#define CGOS_WDT_DEFAULT_PRETIMEOUT	0

#define CGOS_WDT_CMD_TRIGGER	0x27
#define CGOS_WDT_CMD_INIT	0x28
#define	CGOS_WDT_DISABLE	0x00

#define CGOS_WDT_MODE_SINGLE_EVENT 0x02

enum {
	ACTION_INT = 0,
	ACTION_SMI,
	ACTION_RESET,
	ACTION_BUTTON,
};

static unsigned int timeout = CGOS_WDT_DEFAULT_TIMEOUT;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds. (>=0, default="
		 __MODULE_STRING(CGOS_WDT_DEFAULT_TIMEOUT) ")");

static unsigned int pretimeout = CGOS_WDT_DEFAULT_PRETIMEOUT;
module_param(pretimeout, uint, 0);
MODULE_PARM_DESC(pretimeout, "Watchdog pretimeout in seconds. (>=0, default="
		 __MODULE_STRING(CGOS_WDT_DEFAULT_PRETIMEOUT) ")");

static unsigned int timeout_action = ACTION_RESET;
module_param(timeout_action, uint, 0);
MODULE_PARM_DESC(timeout_action, "Watchdog timeout action"
		 "(irq=0, smi=1, reset=2, button=3, default=2");

static unsigned int pretimeout_action = ACTION_SMI;
module_param(pretimeout_action, uint, 0);
MODULE_PARM_DESC(pretimeout_action, "Watchdog pretimeout action"
		 "(irq=0, smi=1, reset=2, button=3, default=1");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct cgos_wdt_data {
	struct cgos_device_data	*cgos;
	struct watchdog_device	wdd;
	bool running;
	unsigned int timeout_action;
	unsigned int pretimeout_action;
};

static int cgos_wdt_start(struct watchdog_device *wdd)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	struct cgos_device_data *cgos = wdt_data->cgos;
	u8 cmd[15], status;
	int ret;

	printk("%s: start with timeout=%d pretimeout=%d actiont=%d actionpt=%d\n",
	       __func__, wdd->timeout, wdd->pretimeout, wdt_data->timeout_action, wdt_data->pretimeout_action);
	cmd[0]	= CGOS_WDT_CMD_INIT;
	cmd[1]	= CGOS_WDT_MODE_SINGLE_EVENT;
	if (wdd->pretimeout > 0) {
		cmd[2] = 2;
		cmd[2] |= (wdt_data->pretimeout_action << 2);
		cmd[2] |= (wdt_data->timeout_action << 4);
		cmd[3] = ((wdd->timeout - wdd->pretimeout) * 1000) & 0xFF;
		cmd[4] = (((wdd->timeout - wdd->pretimeout) * 1000) & 0xFF00) >> 8;
		cmd[5] = (((wdd->timeout - wdd->pretimeout) * 1000) & 0xFF0000) >> 16;
		cmd[6] = (wdd->timeout * 1000) & 0xFF;
		cmd[7] = ((wdd->timeout * 1000) & 0xFF00) >> 8;
		cmd[8] = ((wdd->timeout * 1000) & 0xFF0000) >> 8;
	} else {
		cmd[2] = 1 | (wdt_data->timeout_action << 2);
		cmd[3] = (wdd->timeout * 1000) & 0xFF;
		cmd[4] = ((wdd->timeout * 1000) & 0xFF00) >> 8;
		cmd[5] = ((wdd->timeout * 1000) & 0xFF0000) >> 16;
	}
	/* set delay to 0 */
	cmd[12] = 0x00;
	cmd[13] = 0x00;
	cmd[14] = 0x00;

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

	cmd[0] = CGOS_WDT_CMD_INIT;
	cmd[1] = CGOS_WDT_DISABLE;

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
	u8 cmd = CGOS_WDT_CMD_TRIGGER;
	u8 status;

	return cgos_command(cgos, &cmd, 1, NULL, 0, &status);
}

static int cgos_wdt_set_pretimeout(struct watchdog_device *wdd,
				   unsigned int pretimeout)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);

	if (pretimeout > wdd->timeout)
		return -EINVAL;

	wdd->pretimeout = pretimeout;

	if (wdt_data->running)
		return cgos_wdt_start(wdd);

	return 0;
}

static int cgos_wdt_set_timeout(struct watchdog_device *wdd,
				unsigned int timeout)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);

	if (timeout < wdd->timeout) {
		dev_warn(wdd->parent, "pretimeout < timeout. Setting to zero\n");
		wdd->pretimeout = 0;
	}
	wdd->timeout = timeout;

	if (wdt_data->running)
		return cgos_wdt_start(wdd);

	return 0;
}

static void cgos_wdt_set_actions(struct watchdog_device *wdd,
				 unsigned int timeout_action, unsigned int pretimeout_action)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	if (timeout_action < 0 || timeout_action > ACTION_BUTTON) {
		dev_warn(wdd->parent, 
			 "invalid timeout action. Set default action (%d)",
			 ACTION_RESET);
		wdt_data->timeout_action = ACTION_RESET;
	}
	else
		wdt_data->timeout_action = timeout_action;

	if (pretimeout_action < 0 || pretimeout_action > ACTION_BUTTON) {
		dev_warn(wdd->parent,
			 "invalid pretimeout action. Set default action (%d)",
			 ACTION_SMI);
		wdt_data->pretimeout_action = ACTION_RESET;
	}
	else
		wdt_data->pretimeout_action = pretimeout_action;
}

static const struct watchdog_info cgos_wdt_info = {
	.identity	= "CGOS Watchdog",
	.options	= WDIOF_SETTIMEOUT |
			WDIOF_KEEPALIVEPING |
			WDIOF_MAGICCLOSE |
			WDIOF_PRETIMEOUT
};

static long cgos_wdt_ioctl(struct watchdog_device *wdd,
			   unsigned int cmd, unsigned long arg)
{
	printk("%s: %d\n", __func__, __LINE__);
	struct cgos_wdt_data *wdt_data = watchdog_get_drvdata(wdd);
	void __user *argp = (void __user *)arg;
	unsigned int pretimeout;
	int ret = -ENOIOCTLCMD;
	int __user *p = argp;

	switch (cmd) {
		case WDIOC_GETSUPPORT:
			return copy_to_user(argp, &cgos_wdt_info, sizeof(cgos_wdt_info)) ? -EFAULT : 0;
		case WDIOC_SETPRETIMEOUT:
			if (copy_from_user(&pretimeout, argp, sizeof(pretimeout)))
				return -EFAULT;
			if (cgos_wdt_set_pretimeout(wdd, pretimeout))
				return -EINVAL;
			break;
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
	cgos_wdt_set_pretimeout(wdd, pretimeout);
	cgos_wdt_set_actions(wdd, timeout_action, pretimeout_action);

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
