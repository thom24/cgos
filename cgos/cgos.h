struct cgos_device_data {
	void __iomem	*io_hcnm;
	void __iomem	*io_hcc;
	u8		session_id;
	struct device	*dev;
	struct mutex	lock;
};

struct cgos_platform_data {
	struct resource	ioresource_hcnm;
	struct resource	ioresource_hcc;
	void (*get_hardware_mutex)	(struct cgos_device_data *);
	void (*release_hardware_mutex)	(struct cgos_device_data *);
};


