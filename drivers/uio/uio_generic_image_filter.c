/*
 * uio_generic_image_filter.c
 *
 * Copyright (C) 2017 Wadim Mueller
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
#include "linux/platform_device.h"
#include <linux/slab.h>
#include "linux/module.h"
#include "linux/device.h"
#include "linux/of_address.h"
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/uio_driver.h>
#include <asm/memory.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

#define DRIVER_VERSION							"v1.0"

#define IMAGE_FILTER_MAJOR						10

#define IMAGE_FILTER_IOCTL_BASE					'S'
#define IMAGE_FILTER_START						_IO(IMAGE_FILTER_IOCTL_BASE, 0)
#define IMAGE_FILTER_STOP						_IO(IMAGE_FILTER_IOCTL_BASE, 1)
#define IMAGE_FILTER_SET_DIM					_IO(IMAGE_FILTER_IOCTL_BASE, 2)

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
// 0x14 : Data signal of rows
//        bit 31~0 - rows[31:0] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of cols
//        bit 31~0 - cols[31:0] (Read/Write)
// 0x20 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define FILTER_AP_CTRL   				0x00
#define FILTER_GIE       				0x04
#define FILTER_IER       				0x08
#define FILTER_ISR       				0x0c
#define FILTER_ROWS 					0x14
#define FILTER_COLS 					0x1c

#define FILTER_CTRL_START				BIT(0)
#define FILTER_CTRL_DONE				BIT(1)
#define FILTER_CTRL_IDLE				BIT(2)
#define FILTER_CTRL_READY				BIT(3)
#define FILTER_CTRL_AUTO_RESTART		BIT(7)

#define FILTER_GIE_EN					BIT(0)

#define FILTER_AP_DONE_IRQ_EN			BIT(0)
#define FILTER_AP_READY_IRQ_EN			BIT(1)

enum
{
	VIDEO_BUF_IN = 0, VIDEO_BUF_OUT, VIDEO_BUF_CNT,
};

struct vdma_transfer_dim {
	int dx;
	int dy;
};

struct generic_image_filter_dev
{
	struct platform_device *pdev;
	struct uio_info info;
	struct cdev cdev;
	void __iomem * mem_base;
	struct dma_interleaved_template* itemp;
	struct dma_chan *dma_rx;
	struct dma_chan *dma_tx;
	dma_addr_t dma_video_addr[VIDEO_BUF_CNT];
	u32 max_dx, max_dy, bytes_per_pixel;
	const char* filter_name;
	u32 minor;
};

static int probe_generic_image_filter_dt(struct generic_image_filter_dev* idev)
{
	int ret;
	struct device_node* dev_node = idev->pdev->dev.of_node;

	ret = of_property_read_u32(dev_node, "max-dx", &idev->max_dx);

	if (ret < 0) {
		idev->max_dx = 1280;
		dev_warn(&idev->pdev->dev, "could not get max-dx from dt, falling back to %d\n", idev->max_dx);
	}

	ret = of_property_read_u32(dev_node, "max-dy", &idev->max_dy);

	if (ret < 0) {
		idev->max_dy = 720;
		dev_warn(&idev->pdev->dev, "could not get max-dy from dt, falling back to %d\n", idev->max_dy);
	}

	ret = of_property_read_u32(dev_node, "bpp", &idev->bytes_per_pixel);

	if (ret < 0) {
		idev->bytes_per_pixel = 1;
		dev_warn(&idev->pdev->dev, "could not get bpp from dt, falling back to %d\n", idev->bytes_per_pixel);
	} else {
		idev->bytes_per_pixel = idev->bytes_per_pixel >> 3;
	}

	ret = of_property_read_u32(dev_node, "dev-node-minor", &idev->minor);

	if (ret < 0 ) {
		dev_err(&idev->pdev->dev, "could not get device node minor number, please specify in dt\n");
		return -ENODEV;
	}

	ret =  of_property_read_string(dev_node, "filter-name", &idev->filter_name);

	if (ret < 0 ) {
		char* fname = kzalloc(15, GFP_KERNEL);
		if (!fname)
			return -ENOMEM;

		dev_info(&idev->pdev->dev, "could not get name for filter with minor %d\n", idev->max_dx);
		snprintf(fname, 15, "img_filt%d", idev->minor);
		idev->filter_name = fname;
	}

	idev->dma_rx = dma_request_chan(&idev->pdev->dev, "rx");
	idev->dma_tx = dma_request_chan(&idev->pdev->dev, "tx");

	if (IS_ERR(idev->dma_rx) || IS_ERR(idev->dma_tx)) {
		dev_err(&idev->pdev->dev, "could not allocate dma rx/tx channel\n");
		return -ENODEV;
	}

	return 0;
}

static int reserve_video_memory(struct generic_image_filter_dev *idev)
{
	const char* video_names[] = { "video-in", "video-out" };
	int i;
	void *virt;
	struct uio_mem* uio_mem = idev->info.mem;
	dma_addr_t* dma_addr = idev->dma_video_addr;
	struct resource *control_regs = platform_get_resource(idev->pdev, IORESOURCE_MEM, 0);

	u32 video_buff_size = idev->max_dx * idev->max_dy * idev->bytes_per_pixel;
	uio_mem->name = "filter-control-regs";
	uio_mem->addr = control_regs->start;

	uio_mem->size = resource_size(control_regs);
	uio_mem->memtype = UIO_MEM_PHYS;

	uio_mem++;

	for (i = 0; i < ARRAY_SIZE(video_names); ++i, uio_mem++, dma_addr++) {
		uio_mem->name = video_names[i];
		virt = dma_alloc_coherent(&idev->pdev->dev, video_buff_size, dma_addr, GFP_KERNEL);
		if (!virt) {
			dev_err(&idev->pdev->dev, "%s: unable to request video memory for %s\n", __func__, uio_mem->name);
			return -ENOMEM;
		}
		uio_mem->addr = dma_to_phys(&idev->pdev->dev, *dma_addr);
		uio_mem->internal_addr = virt;
		uio_mem->size = video_buff_size;
		uio_mem->memtype = UIO_MEM_PHYS;
	}

	return 0;
}

static irqreturn_t image_filter_irq(int irq, struct uio_info *dev_info) {
	struct generic_image_filter_dev* idev = dev_info->priv;
	u32 isr_reg;

	isr_reg = readl(idev->mem_base + FILTER_ISR);
	writel(isr_reg, idev->mem_base + FILTER_ISR);

	return IRQ_HANDLED;
}

static int config_vdma(struct generic_image_filter_dev* idev)
{
	struct xilinx_vdma_config dma_config;
	int err;

	memset(&dma_config, 0, sizeof(dma_config));

	dma_config.reset = 1;
	err = xilinx_vdma_channel_set_config(idev->dma_rx, &dma_config);
	if (err)
		dev_err(&idev->pdev->dev, "could not reset vdma rx channel %d\n", err);

	err = xilinx_vdma_channel_set_config(idev->dma_tx, &dma_config);
	if (err)
		dev_err(&idev->pdev->dev, "could not reset vdma tx channel %d\n", err);

	dma_config.reset = 0;
	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(idev->dma_rx, &dma_config);
	if (err) {
		dev_err(&idev->pdev->dev, "could not set vdma rx channel config: %d\n", err);
	}

	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(idev->dma_tx, &dma_config);
	if (err) {
		dev_err(&idev->pdev->dev, "could not set vdma tx channel config: %d\n", err);
	}

	return err;
}

static void init_image_filter_registers(struct generic_image_filter_dev *idev)
{
	writel(idev->max_dx, idev->mem_base + FILTER_COLS);
	writel(idev->max_dy, idev->mem_base + FILTER_ROWS);
	writel(FILTER_AP_DONE_IRQ_EN, idev->mem_base + FILTER_GIE);
	writel(FILTER_AP_READY_IRQ_EN, idev->mem_base + FILTER_IER);
	return;
}

static int image_filter_open(struct inode *inode, struct file *file)
{
	struct cdev* cdev = inode->i_cdev;
	struct generic_image_filter_dev* idev = container_of(cdev, struct generic_image_filter_dev, cdev);

	printk(KERN_INFO"Opening Morphological Filter IP driver from %s \n", __func__);
	file->private_data = idev;
	return 0;
}

static void dma_rx_callback(void *completion)
{
	complete(completion);
}

static int image_filter_start_dma_transfer(struct generic_image_filter_dev* idev)
{
	struct dma_async_tx_descriptor *dma_tx_desc;
	struct dma_async_tx_descriptor *dma_rx_desc;
	dma_cookie_t dma_rx_cookie, dma_tx_cookie;
	struct completion rx_cmp;
	unsigned long rx_tmo = msecs_to_jiffies(5000);
	int err;

	config_vdma(idev);
	idev->itemp->dir = DMA_DEV_TO_MEM;

	dma_rx_desc = dmaengine_prep_interleaved_dma(idev->dma_rx, idev->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_rx_desc) {
		dev_err(&idev->pdev->dev, "could not prepare dma rx channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_rx_cookie = dmaengine_submit(dma_rx_desc);

	idev->itemp->dir = DMA_MEM_TO_DEV;

	dma_tx_desc = dmaengine_prep_interleaved_dma(idev->dma_tx, idev->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_tx_desc) {
		dev_err(&idev->pdev->dev, "could not prepare dma tx channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_tx_cookie = dmaengine_submit(dma_tx_desc);

	if (dma_submit_error(dma_rx_cookie) || dma_submit_error(dma_tx_cookie)) {
		dev_err(&idev->pdev->dev, "%s: submit error %d/%d with \n", __func__, dma_rx_cookie, dma_tx_cookie);
		return -EINVAL;
	}

	init_completion(&rx_cmp);
	dma_rx_desc->callback = dma_rx_callback;
	dma_rx_desc->callback_param = &rx_cmp;

	dma_async_issue_pending(idev->dma_tx);
	dma_async_issue_pending(idev->dma_rx);

	writel(FILTER_CTRL_START, idev->mem_base + FILTER_AP_CTRL);

	rx_tmo = wait_for_completion_timeout(&rx_cmp, rx_tmo);

	if (rx_tmo == 0) {
		dev_err(&idev->pdev->dev, "vdma rx timout occured\n");
	}

	return 0;
}

static int image_filter_set_dimensions(struct generic_image_filter_dev* idev, struct vdma_transfer_dim* dim)
{
	writel(readl(idev->mem_base + FILTER_AP_CTRL) & ~FILTER_CTRL_START, idev->mem_base + FILTER_AP_CTRL);

	idev->itemp->numf = dim->dy;
	idev->itemp->sgl[0].size = dim->dx;

	writel(dim->dx, idev->mem_base + FILTER_COLS);
	writel(dim->dy, idev->mem_base + FILTER_ROWS);

	writel(readl(idev->mem_base + FILTER_AP_CTRL) | FILTER_CTRL_START, idev->mem_base + FILTER_AP_CTRL);
	return 0;
}

static long image_filter_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct generic_image_filter_dev* idev = (struct generic_image_filter_dev*) file->private_data;
	int ret = 0;
	struct vdma_transfer_dim dim;

	switch (cmd) {
	case IMAGE_FILTER_START:
		image_filter_start_dma_transfer(idev);
		break;
	case IMAGE_FILTER_STOP:
		break;
	case IMAGE_FILTER_SET_DIM:
		if (copy_from_user(&dim, (void *) arg, sizeof(dim))) {
			return -EFAULT;
		}
		/* sanity check of the dimensions */
		if (dim.dx > idev->max_dx || dim.dy > idev->max_dy || dim.dx <= 0 || dim.dy <= 0)
			return -EFAULT;

		image_filter_set_dimensions(idev, &dim);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations generic_image_filter_fops = {
		.owner = THIS_MODULE,
		.open = image_filter_open,
		.unlocked_ioctl = image_filter_ioctl,
};

static int generic_image_filter_create_cdev(struct generic_image_filter_dev* idev)
{
	int err = 0;
	dev_t devt;
	devt = MKDEV(IMAGE_FILTER_MAJOR, idev->minor);

	cdev_init(&idev->cdev, &generic_image_filter_fops);
	idev->cdev.owner = THIS_MODULE;
	err = cdev_add(&idev->cdev, devt, 1);

	if (err) {
		dev_err(&idev->pdev->dev, "cdev_add() failed with %d\n", err);
		return err;
	}

	return 0;
}

static int generic_image_filter_probe(struct platform_device *pdev)
{
	struct generic_image_filter_dev *idev;
	int ret = -ENODEV;
	int irq;

	idev = devm_kzalloc(&pdev->dev, (sizeof(*idev)), GFP_KERNEL);

	if (!idev)
		return -ENOMEM;

	/* one data_chunk is enough */
	idev->itemp = devm_kzalloc(&pdev->dev, sizeof(struct dma_interleaved_template) + sizeof(struct data_chunk), GFP_KERNEL);

	if (!idev->itemp) {
		dev_err(&pdev->dev, "unable to request memory for dma_interleaved template\n");
		ret = -ENOMEM;
		goto err_free_mem_filter;
	}

	idev->pdev = pdev;
	idev->mem_base = of_iomap(pdev->dev.of_node, 0);

	irq = platform_get_irq(pdev, 0);
	if (irq  < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = irq;
		goto err_free_mem_all;
	}

	if (!idev->mem_base) {
		ret = -ENOMEM;
		goto err_free_mem_all;
	}

	ret = probe_generic_image_filter_dt(idev);

	if (ret != 0)
		goto err_free_mem_all;

	reserve_video_memory(idev);

	idev->info.name = idev->filter_name;
	idev->info.version = DRIVER_VERSION;
	idev->info.priv = idev;
	idev->info.handler = image_filter_irq;
	idev->info.irq = irq;

	ret = uio_register_device(&pdev->dev, &idev->info);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register UIO device for image filter: error code = %d\n", ret);
		goto err_free_mem_all;
	}

	ret = generic_image_filter_create_cdev(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register character device for image filter : error code = %d\n", ret);
		goto err_free_mem_all;
	}

	init_image_filter_registers(idev);

	idev->itemp->dst_start = idev->dma_video_addr[VIDEO_BUF_OUT];
	idev->itemp->src_start = idev->dma_video_addr[VIDEO_BUF_IN];
	idev->itemp->numf = idev->max_dy;
	idev->itemp->sgl[0].size = idev->max_dx;
	idev->itemp->sgl[0].icg = 0;
	idev->itemp->frame_size = 1;

	dev_info(&pdev->dev, "successfully probed uio image filter\n");
	return 0;

err_free_mem_all:
	kfree(idev->itemp);
err_free_mem_filter:
	kfree(idev);
	return ret;
}

static const struct of_device_id generic_image_filter_of_match[] = {
		{ .compatible = "h_da,image-filter-v1.0" },
		{ },
};

static struct platform_driver generic_image_filter_driver = {
		.probe = generic_image_filter_probe,
		.driver = {
				.name = "generic-image-filter-driver",
				.of_match_table = generic_image_filter_of_match,
		},
};

static int generic_image_filter_init(void)
{
	return platform_driver_register(&generic_image_filter_driver);
}

late_initcall(generic_image_filter_init);

MODULE_AUTHOR("Wadim Mueller <wadim.mueller@gmx.de>");
MODULE_DESCRIPTION("generic driver for Filter IP Cores with a video-in, video-out interface");
MODULE_LICENSE("GPL v2");
