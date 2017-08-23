/*
 * uio_hw_stereo_matcher.c
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

#define USE_VDMA

#define DRIVER_VERSION					"v1.0"

#define SBM_MAJOR						11
#define SBM_MINOR						15

#define SBM_IOCTL_BASE					'S'
#define SBM_START						_IO(SBM_IOCTL_BASE, 0)
#define SBM_STOP						_IO(SBM_IOCTL_BASE, 1)
#define SBM_SET_DIM						_IO(SBM_IOCTL_BASE, 2)

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

#define SBM_AP_CTRL   				0x00
#define SBM_GIE       				0x04
#define SBM_IER       				0x08
#define SBM_ISR       				0x0c
#define SBM_ROWS 					0x14
#define SBM_COLS 					0x1c
#define SBM_CTRL_START				BIT(0)
#define SBM_CTRL_DONE				BIT(1)
#define SBM_CTRL_IDLE				BIT(2)
#define SBM_CTRL_READY				BIT(3)
#define SBM_CTRL_AUTO_RESTART		BIT(7)
#define SBM_GIE_EN					BIT(0)
#define SBM_AP_DONE_IRQ_EN			BIT(0)
#define SBM_AP_READY_IRQ_EN			BIT(1)


enum
{
	VIDEO_BUF_LEFT = 0, VIDEO_BUF_RIGHT, VIDEO_BUF_OUT, VIDEO_BUF_CNT,
};

struct vdma_transfer_dim {
	int dx;
	int dy;
};

struct sbm_dev
{
	struct platform_device *pdev;
	struct uio_info info;
	struct cdev cdev;
	void __iomem * mem_base;
	struct dma_interleaved_template* itemp;
	struct dma_chan *dma_tx_left;
	struct dma_chan *dma_tx_right;
	struct dma_chan *dma_rx;
	dma_addr_t dma_video_addr[VIDEO_BUF_CNT];
	u32 max_dx, max_dy;
	u32 actual_dx, actual_dy;
	const char* sbm_name;
};

static int probe_sbm_dt(struct sbm_dev* sbm)
{
	int ret;
	struct device_node* dev_node = sbm->pdev->dev.of_node;

	ret = of_property_read_u32(dev_node, "max-dx", &sbm->max_dx);

	if (ret < 0) {
		sbm->max_dx = 1280;
		dev_warn(&sbm->pdev->dev, "could not get max-dx from dt, falling back to %d\n", sbm->max_dx);
	}

	ret = of_property_read_u32(dev_node, "max-dy", &sbm->max_dy);

	if (ret < 0) {
		sbm->max_dy = 720;
		dev_warn(&sbm->pdev->dev, "could not get max-dy from dt, falling back to %d\n", sbm->max_dy);
	}

	ret =  of_property_read_string(dev_node, "ip-name", &sbm->sbm_name);

	if (ret < 0) {
		sbm->sbm_name = "hw-sbm";
		dev_warn(&sbm->pdev->dev, "could not get ip-name from dt, falling back to %s\n", sbm->sbm_name);
	}

	sbm->dma_tx_left = dma_request_chan(&sbm->pdev->dev, "left_tx");
	sbm->dma_tx_right = dma_request_chan(&sbm->pdev->dev, "right_tx");
	sbm->dma_rx = dma_request_chan(&sbm->pdev->dev, "rx");

	if (IS_ERR(sbm->dma_tx_left) || IS_ERR(sbm->dma_tx_right) || IS_ERR(sbm->dma_rx)) {
		dev_err(&sbm->pdev->dev, "could not allocate dma rx/tx channel\n");
		return -ENODEV;
	}

	return 0;
}

static int reserve_video_memory(struct sbm_dev* sbm)
{
	const char* video_names[] = { "video-left", "video-right"};
	int i;
	void *virt;
	struct uio_mem* uio_mem = sbm->info.mem;
	dma_addr_t* dma_addr = sbm->dma_video_addr;
	struct resource *control_regs = platform_get_resource(sbm->pdev, IORESOURCE_MEM, 0);

	u32 video_in_buff_size = sbm->max_dx * sbm->max_dy * 1;
	uio_mem->name = "sbm-control-regs";
	uio_mem->addr = control_regs->start;

	uio_mem->size = resource_size(control_regs);
	uio_mem->memtype = UIO_MEM_PHYS;

	uio_mem++;

	for (i = 0; i < ARRAY_SIZE(video_names); ++i, uio_mem++, dma_addr++) {
		uio_mem->name = video_names[i];
		virt = dma_alloc_coherent(&sbm->pdev->dev, video_in_buff_size, dma_addr, GFP_KERNEL);
		if (!virt) {
			dev_err(&sbm->pdev->dev, "%s: unable to request video memory for %s\n", __func__, uio_mem->name);
			return -ENOMEM;
		}
		uio_mem->addr = dma_to_phys(&sbm->pdev->dev, *dma_addr);
		uio_mem->internal_addr = virt;
		uio_mem->size = video_in_buff_size;
		uio_mem->memtype = UIO_MEM_PHYS;
	}

	uio_mem->name = "video-out";
	virt = dma_alloc_coherent(&sbm->pdev->dev, (video_in_buff_size << 1), dma_addr, GFP_KERNEL);
	if (!virt) {
		dev_err(&sbm->pdev->dev, "%s: unable to request video memory for %s\n", __func__, uio_mem->name);
		return -ENOMEM;
	}

	uio_mem->addr = dma_to_phys(&sbm->pdev->dev, *dma_addr);
	uio_mem->internal_addr = virt;
	uio_mem->size = (video_in_buff_size << 1);
	uio_mem->memtype = UIO_MEM_PHYS;
	return 0;
}

static irqreturn_t sbm_irq(int irq, struct uio_info *dev_info)
{
	struct sbm_dev* sbm = dev_info->priv;
	u32 isr_reg;
	isr_reg = readl(sbm->mem_base + SBM_ISR);
	writel(isr_reg, sbm->mem_base + SBM_ISR);
	return IRQ_HANDLED;
}

#ifdef USE_VDMA
static int config_vdma(struct sbm_dev* sbm)
{
	struct xilinx_vdma_config dma_config;
	int err;

	memset(&dma_config, 0, sizeof(dma_config));

	dma_config.reset = 1;
	err = xilinx_vdma_channel_set_config(sbm->dma_rx, &dma_config);
	if (err)
		dev_err(&sbm->pdev->dev, "could not reset vdma rx channel %d\n", err);

	err = xilinx_vdma_channel_set_config(sbm->dma_tx_left, &dma_config);
	if (err)
		dev_err(&sbm->pdev->dev, "could not reset vdma left tx channel %d\n", err);

	err = xilinx_vdma_channel_set_config(sbm->dma_tx_right, &dma_config);
	if (err)
		dev_err(&sbm->pdev->dev, "could not reset vdma right tx channel %d\n", err);

	dma_config.reset = 0;
	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(sbm->dma_rx, &dma_config);
	if (err) {
		dev_err(&sbm->pdev->dev, "could not set vdma rx channel config: %d\n", err);
	}

	dma_config.park = 0;
	err = xilinx_vdma_channel_set_config(sbm->dma_tx_left, &dma_config);
	if (err) {
		dev_err(&sbm->pdev->dev, "could not set vdma left tx channel config: %d\n", err);
	}

	err = xilinx_vdma_channel_set_config(sbm->dma_tx_right, &dma_config);
	if (err) {
		dev_err(&sbm->pdev->dev, "could not set vdma right tx channel config: %d\n", err);
	}

	return err;
}
#endif

static void init_sbm_registers(struct sbm_dev *sbm)
{
	writel(sbm->max_dx, sbm->mem_base + SBM_COLS);
	writel(sbm->max_dy, sbm->mem_base + SBM_ROWS);
	//writel(SBM_AP_DONE_IRQ_EN, sbm->mem_base + SBM_GIE);
	//writel(SBM_AP_READY_IRQ_EN, sbm->mem_base + SBM_IER);
	return;
}

static int sbm_open(struct inode *inode, struct file *file)
{
	struct cdev* cdev = inode->i_cdev;
	struct sbm_dev* sbm = container_of(cdev, struct sbm_dev, cdev);

	printk(KERN_ERR"opening hardware stereo block matcher driver from %s \n", __func__);

	file->private_data = sbm;
	return 0;
}

static void dma_rx_callback(void *completion)
{
	printk(KERN_ERR"------------------> DMA callback occured %s\n", __func__);
	complete(completion);
}

#ifndef USE_VDMA
static void terminate_all_dma_transfers(struct sbm_dev* sbm)
{
	dmaengine_terminate_all(sbm->dma_rx);
	dmaengine_terminate_all(sbm->dma_tx_left);
	dmaengine_terminate_all(sbm->dma_tx_right);
}
#endif

static int sbm_start_dma_transfer(struct sbm_dev* sbm)
{
	struct dma_async_tx_descriptor *dma_tx_left_desc;
	struct dma_async_tx_descriptor *dma_tx_right_desc;
	struct dma_async_tx_descriptor *dma_rx_desc;
	dma_cookie_t dma_rx_cookie, dma_tx_left_cookie, dma_tx_right_cookie;
	struct completion rx_cmp;
	unsigned long rx_tmo = msecs_to_jiffies(5000);
	int err;
#ifdef USE_VDMA
	config_vdma(sbm);
#else
	terminate_all_dma_transfers(sbm);
#endif
	sbm->itemp->dir = DMA_DEV_TO_MEM;
	sbm->itemp->sgl[0].size = (sbm->actual_dx << 1);
	dma_rx_desc = dmaengine_prep_interleaved_dma(sbm->dma_rx, sbm->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_rx_desc) {
		dev_err(&sbm->pdev->dev, "could not prepare dma rx channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_rx_cookie = dmaengine_submit(dma_rx_desc);

	sbm->itemp->dir = DMA_MEM_TO_DEV;
	sbm->itemp->sgl[0].size = sbm->actual_dx;

	sbm->itemp->src_start = sbm->dma_video_addr[VIDEO_BUF_LEFT];
	dma_tx_left_desc = dmaengine_prep_interleaved_dma(sbm->dma_tx_left, sbm->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_tx_left_desc) {
		dev_err(&sbm->pdev->dev, "could not prepare dma tx left channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_tx_left_cookie = dmaengine_submit(dma_tx_left_desc);

	sbm->itemp->dir = DMA_MEM_TO_DEV;

	sbm->itemp->src_start = sbm->dma_video_addr[VIDEO_BUF_RIGHT];
	dma_tx_right_desc = dmaengine_prep_interleaved_dma(sbm->dma_tx_right, sbm->itemp, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!dma_tx_right_desc) {
		dev_err(&sbm->pdev->dev, "could not prepare dma tx right channel for interleaved transfer %d\n", err);
		return -EINVAL;
	}

	dma_tx_right_cookie = dmaengine_submit(dma_tx_right_desc);

	if (dma_submit_error(dma_rx_cookie) || dma_submit_error(dma_tx_right_cookie) || dma_submit_error(dma_tx_left_cookie)) {
		dev_err(&sbm->pdev->dev, "%s: submit error %d/%d/%d with \n", __func__, dma_rx_cookie, dma_tx_right_cookie,dma_tx_left_cookie);
		return -EINVAL;
	}

	init_completion(&rx_cmp);
	dma_rx_desc->callback = dma_rx_callback;
	dma_rx_desc->callback_param = &rx_cmp;

	dma_async_issue_pending(sbm->dma_tx_left);
	dma_async_issue_pending(sbm->dma_tx_right);
	dma_async_issue_pending(sbm->dma_rx);

	writel(SBM_CTRL_START, sbm->mem_base + SBM_AP_CTRL);

	rx_tmo = wait_for_completion_timeout(&rx_cmp, rx_tmo);

	if (rx_tmo == 0) {
		dev_err(&sbm->pdev->dev, "vdma rx timout occured\n");
	}

	return 0;
}

static int sbm_set_dimensions(struct sbm_dev* sbm, struct vdma_transfer_dim* dim)
{
	writel(readl(sbm->mem_base + SBM_AP_CTRL) & ~SBM_CTRL_START, sbm->mem_base + SBM_AP_CTRL);

	sbm->itemp->numf = sbm->actual_dy = dim->dy;
	sbm->itemp->sgl[0].size = sbm->actual_dx = dim->dx;

	writel(dim->dx, sbm->mem_base + SBM_COLS);
	writel(dim->dy, sbm->mem_base + SBM_ROWS);

	writel(readl(sbm->mem_base + SBM_AP_CTRL) | SBM_CTRL_START, sbm->mem_base + SBM_AP_CTRL);
	return 0;
}

static long sbm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sbm_dev* sbm = (struct sbm_dev*) file->private_data;
	int ret = 0;
	struct vdma_transfer_dim dim;

	switch (cmd) {
	case SBM_START:
		sbm_start_dma_transfer(sbm);
		break;
	case SBM_STOP:
		break;
	case SBM_SET_DIM:
		if (copy_from_user(&dim, (void *) arg, sizeof(dim))) {
			return -EFAULT;
		}
		/* sanity check of the dimensions */
		if (dim.dx > sbm->max_dx || dim.dy > sbm->max_dy || dim.dx <= 0 || dim.dy <= 0)
			return -EFAULT;

		sbm_set_dimensions(sbm, &dim);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations sbm_fops = {
		.owner = THIS_MODULE,
		.open = sbm_open,
		.unlocked_ioctl = sbm_ioctl,
};

static int sbm_create_cdev(struct sbm_dev* sbm)
{
	int err = 0;
	dev_t devt;
	devt = MKDEV(SBM_MAJOR, SBM_MINOR);

	cdev_init(&sbm->cdev, &sbm_fops);
	sbm->cdev.owner = THIS_MODULE;
	err = cdev_add(&sbm->cdev, devt, 1);

	if (err) {
		dev_err(&sbm->pdev->dev, "cdev_add() failed with %d\n", err);
		return err;
	}

	return 0;
}

static int sbm_probe(struct platform_device *pdev)
{
	struct sbm_dev *sbm;
	int ret = -ENODEV;
	int irq;

	sbm = devm_kzalloc(&pdev->dev, (sizeof(*sbm)), GFP_KERNEL);

	if (!sbm)
		return -ENOMEM;

	/* one data_chunk is enough */
	sbm->itemp = devm_kzalloc(&pdev->dev, sizeof(struct dma_interleaved_template) + sizeof(struct data_chunk), GFP_KERNEL);

	if (!sbm->itemp) {
		dev_err(&pdev->dev, "unable to request memory for dma_interleaved template\n");
		ret = -ENOMEM;
		goto err_free_mem_sbm;
	}

	sbm->pdev = pdev;
	sbm->mem_base = of_iomap(pdev->dev.of_node, 0);

	irq = platform_get_irq(pdev, 0);
	if (irq  < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = irq;
		goto err_free_mem_all;
	}

	if (!sbm->mem_base) {
		ret = -ENOMEM;
		goto err_free_mem_all;
	}

	ret = probe_sbm_dt(sbm);

	if (ret != 0)
		goto err_free_mem_all;

	reserve_video_memory(sbm);

	sbm->info.name = sbm->sbm_name;
	sbm->info.version = DRIVER_VERSION;
	sbm->info.priv = sbm;
	sbm->info.handler = sbm_irq;
	sbm->info.irq = irq;

	ret = uio_register_device(&pdev->dev, &sbm->info);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register UIO device for image filter: error code = %d\n", ret);
		goto err_free_mem_all;
	}

	ret = sbm_create_cdev(sbm);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register character device for image filter : error code = %d\n", ret);
		goto err_free_mem_all;
	}

	init_sbm_registers(sbm);

	sbm->itemp->dst_start = sbm->dma_video_addr[VIDEO_BUF_OUT];
	sbm->itemp->sgl[0].icg = 0;
	sbm->itemp->frame_size = 1;

	dev_info(&pdev->dev, "successfully probed hardware stereo matcher ip-core\n");
	return 0;

err_free_mem_all:
	kfree(sbm->itemp);
err_free_mem_sbm:
	kfree(sbm);
	return ret;
}

static const struct of_device_id sbm_of_match[] = {
		{ .compatible = "h_da,hw-stereo-matcher" },
		{ },
};

static struct platform_driver sbm_driver = {
		.probe = sbm_probe,
		.driver = {
				.name = "hardware-stereo-block-matcher-driver",
				.of_match_table = sbm_of_match,
		},
};

static int sbm_init(void)
{
	return platform_driver_register(&sbm_driver);
}

late_initcall(sbm_init);

MODULE_AUTHOR("Wadim Mueller <wadim.mueller@gmx.de>");
MODULE_DESCRIPTION("driver for the hw stereo matcher ip-core");
MODULE_LICENSE("GPL v2");
