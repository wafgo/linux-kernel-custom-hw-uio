/*
 * uio disparity coprocessor driver
 *
 * Copyright (C) 2016 Wadim Mueller
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/sizes.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/io.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/cacheflush.h>
#include <asm/dma-mapping.h>

// CTRL_BUS
#define CTRL_BPP          		0x10
#define CTRL_XDIM         		0x18
#define CTRL_YDIM         		0x20
#define CTRL_CURRENT_ROW  		0x28
#define CTRL_CURRENT_ROW_CTRL  	0x2c
#define CTRL_MAXDISPARITY 		0x30

// AXILiteS_BUS
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved

#define AXILITES_AP_CTRL        0x00
#define AXILITES_GIE            0x04
#define AXILITES_IER            0x08
#define AXILITES_ISR            0x0c
#define AXILITES_AP_RETURN      0x10

#define AXILITES_LEFT   		0x18
#define AXILITES_RIGHT  		0x20
#define AXILITES_OUTPUT 		0x28

#define GLOBAL_IRQ_EN			BIT(0)
#define AP_DONE_IRQ_EN			BIT(0)

#define DCX_DRV_NAME			"disp-coproc"
#define DCX_DRV_VERSION			"V1-A"

enum
{
	MEM_LEFT_IDX = 1,
	MEM_RIGHT_IDX,
	MEM_OUT_IDX,
};

struct dcx_parameter
{
	u32 max_width;
	u32 max_height;
	u32 bpp;
	u32 fb_size;
	u32 num_of_fb;
	u32 disp_bpp;
	u32 max_disparity;
};

struct dcx_device
{
	struct platform_device* pdev;
	struct uio_info info;
	void __iomem *ctrl_base;
	void __iomem *axiS_base;
	struct dcx_parameter parameter;
	struct clk *clk;
};

#define to_dcx_dev(_info) \
		container_of(_info, struct dcx_device, info)

static irqreturn_t dcx_irq_handler(int irq, struct uio_info *info)
{
	struct dcx_device *dcx = (struct dcx_device *)info->priv;
	u32 isr_reg;

	/* simply disable IRQ */
	isr_reg = readl(dcx->axiS_base + AXILITES_ISR);
	writel(isr_reg, dcx->axiS_base + AXILITES_ISR);

	return IRQ_HANDLED;
}

static void dcx_get_dt_image_param(struct dcx_device *dcx)
{
	int ret;
	struct device_node* dev_node = dcx->pdev->dev.of_node;

	ret = of_property_read_u32(dev_node, "max-image-width", &dcx->parameter.max_width);

	if (ret < 0) {
		dcx->parameter.max_width = 1280;
		dev_warn(&dcx->pdev->dev, "could not get image-width from dt, falling back to %d\n", dcx->parameter.max_width);
	}

	ret = of_property_read_u32(dev_node, "max-image-height", &dcx->parameter.max_height);

	if (ret < 0) {
		dcx->parameter.max_height = 720;
		dev_warn(&dcx->pdev->dev, "could not get image-height from dt, falling back to %d\n", dcx->parameter.max_height);
	}

	ret = of_property_read_u32(dev_node, "image-bpp", &dcx->parameter.bpp);

	if (ret < 0) {
		dcx->parameter.bpp = 24;
		dev_warn(&dcx->pdev->dev, "could not get image-bpp from dt, falling back to %d\n", dcx->parameter.bpp);
	}

	ret = of_property_read_u32(dev_node, "max-disparity", &dcx->parameter.max_disparity);

	if (ret < 0) {
		dcx->parameter.max_disparity = 60;
		dev_warn(&dcx->pdev->dev, "could not get max-disparity from dt, falling back to %d\n", dcx->parameter.max_disparity);
	}

	ret = of_property_read_u32(dev_node, "disparity-bpp", &dcx->parameter.disp_bpp);
	if (ret < 0) {
		dcx->parameter.disp_bpp = 32;
		dev_warn(&dcx->pdev->dev, "could not get disparity-bpp from dt, falling back to %d\n", dcx->parameter.disp_bpp);
	}

	dcx->parameter.fb_size = (dcx->parameter.max_height * dcx->parameter.max_width * (dcx->parameter.bpp >> 3));
}

static int reserve_uio_mem(struct dcx_device* dcx, struct resource* res)
{
	void *virt;
	int i;
	const char* names[] = {"left-fb", "right-fb"};
	struct uio_mem* uio_mem = dcx->info.mem;
	dma_addr_t dma_addr;
	u32 disp_buff_size = dcx->parameter.max_width * dcx->parameter.max_height * (dcx->parameter.disp_bpp >> 3);
	uio_mem->name = "dcx-control-regs";
	uio_mem->addr = res->start;
	uio_mem->size = resource_size(res);
	uio_mem->memtype = UIO_MEM_PHYS;

	uio_mem++;

	for (i = 0; i < ARRAY_SIZE(names); ++i, uio_mem++) {
		uio_mem->name = names[i];

		virt = dma_alloc_coherent(&dcx->pdev->dev, dcx->parameter.fb_size, &dma_addr, GFP_KERNEL);

		if (!virt) {
			dev_err(&dcx->pdev->dev, "%s: unable to request fb-mem for %s\n", __func__, uio_mem->name);
			return -ENOMEM;
		}
		uio_mem->addr = dma_to_phys(&dcx->pdev->dev,dma_addr);
		uio_mem->internal_addr = virt;
		uio_mem->size = dcx->parameter.fb_size;
		uio_mem->memtype = UIO_MEM_PHYS;
	}

	uio_mem->name = "disparty-mem";

	virt = dma_alloc_coherent(&dcx->pdev->dev, disp_buff_size, &dma_addr, GFP_KERNEL);

	if (!virt) {
		dev_err(&dcx->pdev->dev, "%s: unable to request fb-mem for %s\n", __func__, uio_mem->name);
		return -ENOMEM;
	}

	uio_mem->addr = dma_to_phys(&dcx->pdev->dev,dma_addr);;
	uio_mem->size = disp_buff_size;
	uio_mem->internal_addr = virt;
	uio_mem->memtype = UIO_MEM_PHYS;

	return 0;
}

static void init_dcx(struct dcx_device* dcx)
{
	int i;
	u32 offsets[] = {AXILITES_LEFT, AXILITES_RIGHT, AXILITES_OUTPUT};
	struct uio_mem* uio_mem = &dcx->info.mem[1];

	writel(dcx->parameter.bpp, dcx->ctrl_base + CTRL_BPP);
	writel(dcx->parameter.max_width, dcx->ctrl_base + CTRL_XDIM);
	writel(dcx->parameter.max_height, dcx->ctrl_base + CTRL_YDIM);
	writel(dcx->parameter.max_disparity, dcx->ctrl_base + CTRL_MAXDISPARITY);

	for (i = 0; i < ARRAY_SIZE(offsets); ++i, uio_mem++) {
		u32 pic_offset = offsets[i];
		writel(uio_mem->addr, dcx->axiS_base + pic_offset);
	}

	writel(AP_DONE_IRQ_EN, dcx->axiS_base + AXILITES_IER);
	writel(GLOBAL_IRQ_EN, dcx->axiS_base + AXILITES_GIE);

	return;
}


static int dcx_probe(struct platform_device *pdev)
{
	struct dcx_device *dcx;
	struct resource *res;
	int irq;
	int ret;

	dcx = devm_kzalloc(&pdev->dev, (sizeof(*dcx)), GFP_KERNEL);
	if (!dcx)
		return -ENOMEM;

	dcx->pdev = pdev;
	dcx_get_dt_image_param(dcx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/*FIXME: convert it to just one AXILite Interface*/
	dcx->axiS_base = devm_ioremap_resource(&pdev->dev, res);
	dcx->ctrl_base = dcx->axiS_base + (resource_size(res) >> 1);

	if (!dcx->ctrl_base) {
		dev_err(&pdev->dev, "unable to iomap registers\n");
		return -ENOMEM;
	}

	dcx->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dcx->clk)) {
		dev_err(&pdev->dev, "could not get clock %s\n", __clk_get_name(dcx->clk));
		return PTR_ERR(dcx->clk);
	}

	ret = clk_prepare_enable(dcx->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clock.\n");
		return ret;
	}

	ret = reserve_uio_mem(dcx, res);

	if (ret < 0)
		goto err_clk_dis;

	dcx->info.name = DCX_DRV_NAME;
	dcx->info.version = DCX_DRV_VERSION;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = irq;
		goto err_clk_dis;
	}

	dcx->info.irq = irq;
	dcx->info.handler = dcx_irq_handler;
	dcx->info.priv = dcx;

	ret = uio_register_device(&pdev->dev, &dcx->info);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register to UIO\n");
		goto err_clk_dis;
	}

	init_dcx(dcx);
	platform_set_drvdata(pdev, dcx);

	return 0;

err_clk_dis:
	clk_disable_unprepare(dcx->clk);
	return ret;
}

static int dcx_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id dc_of_match[] =
{
	{ .compatible = "h_da,axi-disparity-coprocessor-v1a", },
	{ /* sentinel */}
};

MODULE_DEVICE_TABLE(of, dc_of_match);

static struct platform_driver dcx_driver =
{
	.driver = {
			.name = "zynq-vivado-hls-disparity-coprocessor-driver",
			.of_match_table = dc_of_match,
	},
	.probe = dcx_probe,
	.remove = dcx_remove,
};

module_platform_driver(dcx_driver);

MODULE_AUTHOR("Wadim Mueller");
MODULE_DESCRIPTION("Xilinx Zynq AXI Disparity Coprocessor created with Vivado HLS");
MODULE_LICENSE("GPL");
