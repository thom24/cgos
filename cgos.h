// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Congatec Board Controller driver definitions
 *
 * Copyright (C) 2024 Bootlin
 * Author: Thomas Richard <thomas.richard@bootlin.com>
 */

#define CGOS_CGBC_CMD_GET_FW_REV	0x21

#define CGOS_VERSION_LEN 10

struct cgos_info {
	unsigned char feature;
	unsigned char major;
	unsigned char minor;
	unsigned char compat_id; /* not used */
	char version[CGOS_VERSION_LEN];
};

struct cgos_device_data {
	void __iomem		*io_hcnm;
	void __iomem		*io_hcc;
	u8			session_id;
	struct device		*dev;
	struct cgos_info	info;
	struct mutex		lock;
};

struct cgos_platform_data {
	struct resource		ioresource_hcnm;
	struct resource		ioresource_hcc;
	int	(*command)		(struct cgos_device_data *, u8 *, u8, u8 *, u8, u8 *);
	int	(*register_cells)	(struct cgos_device_data *);
	int	(*map)			(struct platform_device *, struct cgos_device_data *);
	int	(*init)			(struct cgos_device_data *);
	void	(*close)		(struct cgos_device_data *);
};

int cgos_command(struct cgos_device_data *cgos, u8 *cmd, u8 cmd_size, u8 *data,
		 u8 data_size, u8 *status);
