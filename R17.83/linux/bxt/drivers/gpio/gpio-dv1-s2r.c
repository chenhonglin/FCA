#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define DRIVER_NAME			"dv1_gpio_s2r"
#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))
#define GPIO_HIGH			1
#define EMMC_BASE_START		0xf8c0052C
#define VALUE_TO_WRITE		0x0001F100
#define IOREMAP_SIZE		4

#define PMC_CFG_ADDR			0xFE043008
/*
 * Reset Power Cycle Duration (pwr_cyc_dur): BIT=9:8=2'b11: 1-2 seconds
 * Default valie of PMC_CFG is 0 to get minimum reboot delay set 0x00000300
 * Ideally this needs to be set as part of ABL as cittently Intel not supporting this in ABL we are doing it here
 * Ref: 557556_557556_EDS_Volume_2_Revision_0p7.pdf
 */
#define PMC_CFG_VALUE			0x00000300
#define PMC_CFG_IOREMAP_SIZE		4

#define LPC_AD_DW1_BASE       0xf8c0062c /* LPC_AD0 address */
#define LPC_AD_DW1_OFFSET     8
#define LPC_AD1_DW1_BASE      (LPC_AD_DW1_BASE + LPC_AD_DW1_OFFSET)
#define LPC_AD_S2R_VALUE      0x0001F100
#define LPC_AD_DFTL_VALUE     0x0001f000

#define LPSS_I2C_SDA_SCL_DW1_BASE     0xf8c70000
#define LPSS_I2C_SDA_SCL_S2R_VALUE    0x0001F100
#define LPSS_I2C_SDA_SCL_DFLT_VALUE   0x0001f300

#define GP_CAMERASB01_DW1_BASE         0xf8c5069C
#define CAM_PCIE_S2R_VALUE             0x0001F100
#define CAM_PCIE_DW1_OFFSET            8
#define GP_CAM_B01_DFTL_VALUE          0x0001f05c
#define GP_CAM_B03_DFTL_VALUE          0x0001f05e

#define PCIE_WAKE0_B_DW1_BASE		0xf8c00504  //GPIO_205
#define PCIE_DFTL_VALUE                0x0003f000

#if 0
#define PCIE_WAKE1_B_DW1_BASE           0xf8c0050C //south-west community GPIO_206

#define LPSS_UART0_RTS_B_DW1_BASE       0xf8c50644 //north community GPIO_40
#define LPSS_UART0_RTS_B_DFLT_VALUE     0x0001f245
#endif

static int lpss_i2c_dw1[] = { //LPSS_I2C,1,2,5,6,7
        0x514, 0x51C, 0x524, 0x52c,
        0x554, 0x55c, 0x564, 0x56c,
        0x574, 0x57c
};

static int gpio_num[] = {
	435, 437, 438, 
	441, 450, 451, 453, 458,
        /*466 - GPIO is driver USB media (host port) port.
                Suspend fails on putting this pin to HIGH-Z*/
    467, 468, 469,
	470, 471, 478, 479, 482,
	483, 443, 444, 445, 452,
	457, 459, 464,

//GPIO  8,   12,  13,  19,
	442, 446, 447, 453
#if 0
	435, 436, 437, 438, 440,
	441, 450, 451, 453, 458,
	461, 466, 467, 468, 469,
	470, 471, 478, 479, 482,
	483, 443, 444, 445, 452,
	457, 459, 464
#endif
};

#if defined(CONFIG_PM_SLEEP)
static int dv1_gpio_s2r_core_suspend(struct device *dev)
{
	int i = 0, ret = 0;
	void __iomem *emmc_base_addr;
	void __iomem *lpc_base_addr;
	void __iomem *lpss_base_addr;
	void __iomem *cam_base_addr;
	void __iomem *pcie_base_addr;
	unsigned int addr;

	pr_err("%s: arr size: %d\n", __func__, ARRAY_SIZE(gpio_num));

	for (i = 0; i < ARRAY_SIZE(gpio_num); i++) {
		ret = gpio_direction_input(gpio_num[i]);
		if (ret != 0) {
			pr_err("%s: Failed to set GPIO %d dir in\n",
				 __func__, gpio_num[i]);
			break;
		}
	}

	for (i = 0, addr = EMMC_BASE_START; i < 9; i++, addr = addr + 8) {
		emmc_base_addr = ioremap(addr, IOREMAP_SIZE);
		writel(VALUE_TO_WRITE, emmc_base_addr);
		iounmap(emmc_base_addr);
	}

	for (i = 0, addr = LPC_AD1_DW1_BASE; i < 5; i++,
	     addr = addr + LPC_AD_DW1_OFFSET) {
		lpc_base_addr = ioremap(addr, IOREMAP_SIZE);
		writel(LPC_AD_S2R_VALUE, lpc_base_addr);
		iounmap(lpc_base_addr);
	}

	for (i = 0, addr = LPSS_I2C_SDA_SCL_DW1_BASE;
	     i < ARRAY_SIZE(lpss_i2c_dw1); i++) {
		lpss_base_addr = ioremap( (addr + lpss_i2c_dw1[i]), IOREMAP_SIZE);
		writel(LPSS_I2C_SDA_SCL_S2R_VALUE, lpss_base_addr);
		iounmap(lpss_base_addr);
	}

	for (i = 0, addr = GP_CAMERASB01_DW1_BASE; i < 2; i++,
		addr = addr + CAM_PCIE_DW1_OFFSET) {
		cam_base_addr = ioremap(addr, IOREMAP_SIZE);
		writel(CAM_PCIE_S2R_VALUE, cam_base_addr);
		iounmap(cam_base_addr);
	}

	pcie_base_addr = ioremap(PCIE_WAKE0_B_DW1_BASE, IOREMAP_SIZE);
	writel(CAM_PCIE_S2R_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);

#if 0
        pcie_base_addr = ioremap(PCIE_WAKE1_B_DW1_BASE, IOREMAP_SIZE);
        writel(CAM_PCIE_S2R_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);

        pcie_base_addr = ioremap(LPSS_UART0_RTS_B_DW1_BASE, IOREMAP_SIZE);
        writel(CAM_PCIE_S2R_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);
#endif
	return ret;
}

static int dv1_gpio_s2r_core_resume(struct device *dev)
{
	int i = 0, ret = 0;
	void __iomem *lpc_base_addr;
	void __iomem *lpss_base_addr;
	void __iomem *cam_base_addr;
	void __iomem *pcie_base_addr;
	unsigned int addr;

	pr_err("%s: DV1 GPIO S2R core resume call\n", __func__);

#if 0
        pcie_base_addr = ioremap(LPSS_UART0_RTS_B_DW1_BASE, IOREMAP_SIZE);
        writel(LPSS_UART0_RTS_B_DFLT_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);

        pcie_base_addr = ioremap(PCIE_WAKE1_B_DW1_BASE, IOREMAP_SIZE);
        writel(PCIE_DFTL_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);
#endif

	pcie_base_addr = ioremap(PCIE_WAKE0_B_DW1_BASE, IOREMAP_SIZE);
	writel(PCIE_DFTL_VALUE, pcie_base_addr);
	iounmap(pcie_base_addr);

	for (i = 0, addr = GP_CAMERASB01_DW1_BASE; i < 2; i++,
	     addr = addr + CAM_PCIE_DW1_OFFSET) {
		cam_base_addr = ioremap(addr, IOREMAP_SIZE);
                if(i == 0)
		    writel(GP_CAM_B01_DFTL_VALUE, cam_base_addr);
                if(i == 1)
		    writel(GP_CAM_B03_DFTL_VALUE, cam_base_addr);
		iounmap(cam_base_addr);
	}

	for (i = 0, addr = LPC_AD1_DW1_BASE; i < 5; i++,
	     addr = addr + LPC_AD_DW1_OFFSET) {
		lpc_base_addr = ioremap(addr, IOREMAP_SIZE);
		writel(LPC_AD_DFTL_VALUE, lpc_base_addr);
		iounmap(lpc_base_addr);
	}

	for (i = 0, addr = LPSS_I2C_SDA_SCL_DW1_BASE;
	     i < ARRAY_SIZE(lpss_i2c_dw1); i++) {
		lpss_base_addr = ioremap( (addr + lpss_i2c_dw1[i]), IOREMAP_SIZE);
		writel(LPSS_I2C_SDA_SCL_DFLT_VALUE, lpss_base_addr);
		iounmap(lpss_base_addr);
	}

	for (i = 0; i < ARRAY_SIZE(gpio_num); i++) {
		ret = gpio_direction_output(gpio_num[i], GPIO_HIGH);
		if (ret != 0) {
			pr_err("%s: Failed to set GPIO %d dir out\n",
				__func__, gpio_num[i]);
			break;
		}
	}
	return ret;
}
#endif

const struct dev_pm_ops dv1_gpio_s2r_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dv1_gpio_s2r_core_suspend, dv1_gpio_s2r_core_resume)
};

static int dv1_gpio_s2r_probe(struct i2c_client *client,
			const struct i2c_device_id *i2c_id)
{
	void __iomem *pmc_cfg_base_addr;
	pr_err("%s: DV1 GPIO S2R probe\n", __func__);

	/* Configure SoC reboot delay to minimum value i.e., 1-2seconds */
	pmc_cfg_base_addr = ioremap(PMC_CFG_ADDR, PMC_CFG_IOREMAP_SIZE);
	if (pmc_cfg_base_addr) {
		writel(PMC_CFG_VALUE, pmc_cfg_base_addr);
		iounmap(pmc_cfg_base_addr);
	} else {
		pr_err("%s: Could not ioremap PMC_CFG_ADDR\n", __func__);
	}

	return 0;
}

//static int dv1_gpio_s2r_remove(struct platform_device *pdev)
static int dv1_gpio_s2r_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id dv1_gpio_s2r_dummy_id[] = {
	{DRIVER_NAME, 0},
	{}
};

static struct i2c_driver dv1_gpio_s2r_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &dv1_gpio_s2r_pm_ops,
	},
	.probe = dv1_gpio_s2r_probe,
	.remove = dv1_gpio_s2r_remove,
	.id_table = dv1_gpio_s2r_dummy_id,
};

module_i2c_driver(dv1_gpio_s2r_driver);

MODULE_LICENSE("GPL");
