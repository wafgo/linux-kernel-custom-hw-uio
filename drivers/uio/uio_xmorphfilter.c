/*
 * uio_xmorphfilter.c
 *
 *  Created on: 06.03.2017
 *      Author: wadim mueller
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

#define XMORPH_FILTER_MAJOR					10
#define XMORPH_FILTER_MINOR					235

#define XMORPH_IOCTL_BASE					'S'
#define XMORPH_START						_IO(XMORPH_IOCTL_BASE, 0)
#define XMORPH_STOP							_IO(XMORPH_IOCTL_BASE, 1)
#define XMORPH_SET_DIM						_IO(XMORPH_IOCTL_BASE, 2)

static int xmorph_filter_dev_num;
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

#define XMF_AP_CTRL   				0x00
#define XMF_GIE       				0x04
#define XMF_IER       				0x08
#define XMF_ISR       				0x0c
#define XMF_ROWS 					0x14
#define XMF_COLS 					0x1c

#define XMF_CTRL_START				BIT(0)
#define XMF_CTRL_DONE				BIT(1)
#define XMF_CTRL_IDLE				BIT(2)
#define XMF_CTRL_READY				BIT(3)
#define XMF_CTRL_AUTO_RESTART		BIT(7)

#define XMF_GIE_EN					BIT(0)

#define XMF_AP_DONE_IRQ_EN			BIT(0)
#define XMF_AP_READY_IRQ_EN			BIT(1)

enum
{
	VIDEO_BUF_IN = 0, VIDEO_BUF_OUT, VIDEO_BUF_CNT,
};

struct vdma_transfer_dim {
	int dx;
	int dy;
};

struct morph_filter_dev
{
	struct platform_device *pdev;
	struct uio_info info;
	struct cdev cdev;
	void __iomem * mem_base;
	struct dma_interleaved_template* itemp;
	struct dma_chan *dma_rx;
	struct dma_chan *dma_tx;
	dma_addr_t dma_video_addr[VIDEO_BUF_CNT];
	u32 max_dx, max_dy;
};

static int probe_morph_filter_dt(struct morph_filter_dev* mdev)
{
	int ret;
	struct device_node* dev_node = mdev->pdev->dev.of_node;

	ret = of_property_read_u32(dev_node, "max-dx", &mdev->max_dx);

	if (ret < 0) {
		mdev->max_dx = 1280;
		dev_warn(&mdev->pdev->dev, "could not get max-dx from dt, falling back to %d\n", mdev->max_dx);
	}

	ret = of_property_read_u32(dev_node, "max-dy", &mdev->max_dy);

	if (ret < 0) {
		mdev->max_dy = 720;
		dev_warn(&mdev->pdev->dev, "could not get max-dy from dt, falling back to %d\n", mdev->max_dy);
	}

	mdev->dma_rx = dma_request_chan(&mdev->pdev->dev, "rx");
	mdev->dma_tx = dma_request_chan(&mdev->pdev->dev, "tx");

	if (IS_ERR(mdev->dma_rx) || IS_ERR(mdev->dma_tx)) {
		dev_err(&mdev->pdev->dev, "could not allocate dma rx/tx channel\n");
		return -ENODEV;
	}

	return 0;
}

static int reserve_video_memory(struct morph_filter_dev *mdev)
{
	const char* video_names[] = { "video-in", "video-out" };
	int i;
	void *virt;
	struct uio_mem* uio_mem = mdev->info.mem;
	dma_addr_t* dma_addr = mdev->dma_video_addr;
	struct resource *control_regs = platform_get_resource(mdev->pdev, IORESOURCE_MEM, 0);

	/* just grayscale */
	u32 video_buff_size = mdev->max_dx * mdev->max_dy * 1;
	uio_mem->name = "xmorph-control-regs";
	uio_mem->addr = control_regs->start;

	uio_mem->size = resource_size(control_regs);
	uio_mem->memtype = UIO_MEM_PHYS;

	uio_mem++;

	for (i = 0; i < ARRAY_SIZE(video_names); ++i, uio_mem++, dma_addr++) {
		uio_mem->name = video_names[i];
		virt = dma_alloc_coherent(&mdev->pdev->dev, video_buff_size, dma_addr, GFP_KERNEL);
		if (!virt) {
			dev_err(&mdev->pdev->dev, "%s: unable to request video memory for %s\n", __func__, uio_mem->name);
			return -ENOMEM;
		}
		uio_mem->addr = dma_to_phys(&mdev->pdev->dev, *dma_addr);
		uio_mem->internal_addr = virt;
		uio_mem->size = video_buff_size;
		uio_mem->memtype = UIO_MEM_PHYS;
	}

	return 0;
}

static irqreturn_t xmorph_irq(int irq, struct uio_info *dev_info) {
	struct morph_filter_dev* mdev = dev_info->priv;
	u32 isr_reg;

	isr_reg = readl(mdev->mem_base + XMF_ISR);
	writel(isr_reg, mdev->mem_base + XMF_ISR);

	return IRQ_HANDLED;
}

static int config_vdma(struct morph_filter_dev* mdev)
{
	struct xilinx_vdma_config dma_config;
	int err;

	memset(&dma_config, 0, sizeof(dma_config));

	dma_config.reset = 1;
	err = xilinx_vdma_channel_set_config(mdev->dma_rx, &dma_config);
	if (err)
		dev_err(&mdev->pdev->dev, "could not reset vdma rx channel %d\n", err);

	err = xilinx_vdma_channel_set_config(mdev->dma_tx, &dma_config);
	if (err)
		dev_err(&mdev->pdev->dev, "could not reset vdma tx channel %d\n", err);

	dma_config.reset = 0;
	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(mdev->dma_rx, &dma_config);
	if (err) {
		dev_err(&mdev->pdev->dev, "could not set vdma rx channel config: %d\n", err);
	}

	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(mdev->dma_tx, &dma_config);
	if (err) {
		dev_err(&mdev->pdev->dev, "could not set vdma tx channel config: %d\n", err);
	}

	return err;
}

static void init_xmorph_registers(struct morph_filter_dev *mdev)
{
	writel(mdev->max_dx, mdev->mem_base + XMF_COLS);
	writel(mdev->max_dy, mdev->mem_base + XMF_ROWS);
	writel(XMF_AP_DONE_IRQ_EN, mdev->mem_base + XMF_GIE);
	writel(XMF_AP_READY_IRQ_EN, mdev->mem_base + XMF_IER);
	return;
}

static int xmorph_open(struct inode *inode, struct file *file)
{
	struct cdev* cdev = inode->i_cdev;
	struct morph_filter_dev* mdev = container_of(cdev, struct morph_filter_dev, cdev);

	printk(KERN_ERR"Opening Morphological Filter IP driver from %s \n", __func__);
	file->private_data = mdev;
	return 0;
}

static void dma_rx_callback(void *completion)
{
	complete(completion);
}

static int xmorph_start_transfer(struct morph_filter_dev* mdev)
{
	struct dma_async_tx_descriptor *dma_tx_desc;
	struct dma_async_tx_descriptor *dma_rx_desc;
	dma_cookie_t dma_rx_cookie, dma_tx_cookie;
	struct completion rx_cmp;
	unsigned long rx_tmo = msecs_to_jiffies(5000);
	int err;

	config_vdma(mdev);
	mdev->itemp->dir = DMA_DEV_TO_MEM;

	dma_rx_desc = dmaengine_prep_interleaved_dma(mdev->dma_rx, mdev->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_rx_desc) {
		dev_err(&mdev->pdev->dev, "could not prepare dma rx channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_rx_cookie = dmaengine_submit(dma_rx_desc);

	mdev->itemp->dir = DMA_MEM_TO_DEV;

	dma_tx_desc = dmaengine_prep_interleaved_dma(mdev->dma_tx, mdev->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_tx_desc) {
		dev_err(&mdev->pdev->dev, "could not prepare dma tx channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_tx_cookie = dmaengine_submit(dma_tx_desc);

	if (dma_submit_error(dma_rx_cookie) || dma_submit_error(dma_tx_cookie)) {
		dev_err(&mdev->pdev->dev, "%s: submit error %d/%d with \n", __func__, dma_rx_cookie, dma_tx_cookie);
		return -EINVAL;
	}

	init_completion(&rx_cmp);
	dma_rx_desc->callback = dma_rx_callback;
	dma_rx_desc->callback_param = &rx_cmp;

	dma_async_issue_pending(mdev->dma_tx);
	dma_async_issue_pending(mdev->dma_rx);

	writel(XMF_CTRL_START, mdev->mem_base + XMF_AP_CTRL);

	rx_tmo = wait_for_completion_timeout(&rx_cmp, rx_tmo);

	if (rx_tmo == 0) {
		dev_err(&mdev->pdev->dev, "vdma rx timout occured\n");
	}

	return 0;
}

static int xmorph_set_dimensions(struct morph_filter_dev* mdev, struct vdma_transfer_dim* dim)
{
	writel(readl(mdev->mem_base + XMF_AP_CTRL) & ~XMF_CTRL_START, mdev->mem_base + XMF_AP_CTRL);

	mdev->itemp->numf = dim->dy;
	mdev->itemp->sgl[0].size = dim->dx;

	writel(dim->dx, mdev->mem_base + XMF_COLS);
	writel(dim->dy, mdev->mem_base + XMF_ROWS);

	writel(readl(mdev->mem_base + XMF_AP_CTRL) | XMF_CTRL_START, mdev->mem_base + XMF_AP_CTRL);
	return 0;
}

static long xmorph_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct morph_filter_dev* mdev = (struct morph_filter_dev*) file->private_data;
	int ret = 0;
	struct vdma_transfer_dim dim;

	switch (cmd) {
	case XMORPH_START:
		xmorph_start_transfer(mdev);
		break;
	case XMORPH_STOP:
		break;
	case XMORPH_SET_DIM:
		if (copy_from_user(&dim, (void *) arg, sizeof(dim))) {
			return -EFAULT;
		}
		/* sanity check of the dimensions */
		if (dim.dx > mdev->max_dx || dim.dy > mdev->max_dy || dim.dx <= 0 || dim.dy <= 0)
			return -EFAULT;

		xmorph_set_dimensions(mdev, &dim);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations xmorph_filter_fops = {
		.owner = THIS_MODULE,
		.open = xmorph_open,
		.unlocked_ioctl = xmorph_ioctl,
};

static int morph_filter_create_cdev(struct morph_filter_dev* mdev)
{
	int err = 0;
	dev_t devt;
	devt = MKDEV(XMORPH_FILTER_MAJOR, XMORPH_FILTER_MINOR + xmorph_filter_dev_num);

	cdev_init(&mdev->cdev, &xmorph_filter_fops);
	mdev->cdev.owner = THIS_MODULE;
	err = cdev_add(&mdev->cdev, devt, 1);

	if (err) {
		dev_err(&mdev->pdev->dev, "cdev_add() failed with %d\n", err);
		return err;
	}

	xmorph_filter_dev_num++;
	return 0;
}

static int morph_filter_probe(struct platform_device *pdev)
{
	struct morph_filter_dev *mdev;
	int ret = -ENODEV;
	int irq;

	mdev = devm_kzalloc(&pdev->dev, (sizeof(*mdev)), GFP_KERNEL);

	if (!mdev)
		return -ENOMEM;

	/* one data_chunk is enough */
	mdev->itemp = devm_kzalloc(&pdev->dev, sizeof(struct dma_interleaved_template) + sizeof(struct data_chunk), GFP_KERNEL);

	if (!mdev->itemp) {
		dev_err(&pdev->dev, "unable to request memory for dma_interleaved template\n");
		ret = -ENOMEM;
		goto err_free_mem_filter;
	}

	mdev->pdev = pdev;
	mdev->mem_base = of_iomap(pdev->dev.of_node, 0);

	irq = platform_get_irq(pdev, 0);
	if (irq  < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = irq;
		goto err_free_mem_all;
	}

	if (!mdev->mem_base) {
		ret = -ENOMEM;
		goto err_free_mem_all;
	}

	ret = probe_morph_filter_dt(mdev);

	if (ret != 0)
		goto err_free_mem_all;

	reserve_video_memory(mdev);

	mdev->info.name = "xmorph-dev";
	mdev->info.version = "1.0";
	mdev->info.priv = mdev;
	mdev->info.handler = xmorph_irq;
	mdev->info.irq = irq;

	ret = uio_register_device(&pdev->dev, &mdev->info);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register UIO device for morphological filter: error code = %d\n", ret);
		goto err_free_mem_all;
	}

	ret = morph_filter_create_cdev(mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register charecter device for morphological filter: error code = %d\n", ret);
		goto err_free_mem_all;
	}

	init_xmorph_registers(mdev);

	mdev->itemp->dst_start = mdev->dma_video_addr[VIDEO_BUF_OUT];
	mdev->itemp->src_start = mdev->dma_video_addr[VIDEO_BUF_IN];
	mdev->itemp->numf = mdev->max_dy;
	mdev->itemp->sgl[0].size = mdev->max_dx;
	mdev->itemp->sgl[0].icg = 0;
	mdev->itemp->frame_size = 1;

	dev_info(&pdev->dev, "successfully probed uio morphological filter\n");
	return 0;

err_free_mem_all:
	kfree(mdev->itemp);
err_free_mem_filter:
	kfree(mdev);
	return ret;
}

static const struct of_device_id morph_filter_of_match[] = {
		{ .compatible = "h_da,morph-filter-v1.0" },
		{ },
};

static struct platform_driver morph_filter_driver = {
		.probe = morph_filter_probe,
		.driver = {
				.name = "morphological-filter-driver",
				.of_match_table = morph_filter_of_match,
		},
};

static int morph_filter_init(void)
{
	return platform_driver_register(&morph_filter_driver);
}

late_initcall(morph_filter_init);

MODULE_AUTHOR("Wadim Mueller <wadim.mueller@gmx.de>");
MODULE_DESCRIPTION("Driver for morphological Filter IP Core");
MODULE_LICENSE("GPL v2");
