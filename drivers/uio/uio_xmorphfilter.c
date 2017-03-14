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

#define XMF_AP_CTRL   		0x00
#define XMF_GIE       		0x04
#define XMF_IER       		0x08
#define XMF_ISR       		0x0c
#define XMF_ROWS 			0x14
#define XMF_COLS 			0x1c


struct morph_filter_dev {
	struct platform_device *pdev;
	struct uio_info info;
	void __iomem * mem_base;
	struct dma_chan *dma_rx;
	struct dma_chan *dma_tx;
	int dx, dy;
};

static int probe_morph_filter_dt(struct morph_filter_dev* mdev)
{
	int ret;
	struct device_node* dev_node = mdev->pdev->dev.of_node;

	ret = of_property_read_u32(dev_node, "video-hor", &mdev->dx);

	if (ret < 0) {
		mdev->dx= 1280;
		dev_warn(&mdev->pdev->dev, "could not get video-hor from dt, falling back to %d\n", mdev->dx);
	}

	ret = of_property_read_u32(dev_node, "video-ver", &mdev->dy);

	if (ret < 0) {
		mdev->dy= 720;
		dev_warn(&mdev->pdev->dev, "could not get video-ver from dt, falling back to %d\n", mdev->dy);
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
	const char* video_names[] = {"video-in", "video-out"};
	int i;
	void *virt;
	struct uio_mem* uio_mem = mdev->info.mem;
	dma_addr_t dma_addr;
	struct resource *control_regs = platform_get_resource(mdev->pdev, IORESOURCE_MEM, 0);

	/* just grayscale */
	u32 video_buff_size = mdev->dx * mdev->dy * 1;
	uio_mem->name = "xmorph-control-regs";
	uio_mem->addr = control_regs->start;

	uio_mem->size = resource_size(control_regs);
	uio_mem->memtype = UIO_MEM_PHYS;

	uio_mem++;

	for (i = 0; i < ARRAY_SIZE(video_names); ++i, uio_mem++) {
		uio_mem->name = video_names[i];
		virt = dma_alloc_coherent(&mdev->pdev->dev, video_buff_size, &dma_addr, GFP_KERNEL);
		if (!virt) {
			dev_err(&mdev->pdev->dev, "%s: unable to request video memory for %s\n", __func__, uio_mem->name);
			return -ENOMEM;
		}

		uio_mem->addr = dma_to_phys(&mdev->pdev->dev, dma_addr);
		uio_mem->internal_addr = virt;
		uio_mem->size = video_buff_size;
		uio_mem->memtype = UIO_MEM_PHYS;
	}

	return 0;
}

static void init_xmorph_registers(struct morph_filter_dev *mdev)
{
	writel(mdev->dx, mdev->mem_base + XMF_COLS);
	writel(mdev->dy, mdev->mem_base + XMF_ROWS);

	return;
}

static int morph_filter_probe(struct platform_device *pdev)
{
	struct morph_filter_dev *m_filter;
	int ret = -ENODEV;

	m_filter = devm_kzalloc(&pdev->dev, (sizeof(*m_filter)), GFP_KERNEL);
	if (!m_filter)
		return -ENOMEM;

	m_filter->pdev = pdev;
	m_filter->mem_base = of_iomap(pdev->dev.of_node, 0);

	if (!m_filter->mem_base) {
		ret = -ENOMEM;
		goto err_free_mem;
	}

	ret = probe_morph_filter_dt(m_filter);

	if (ret != 0)
		goto err_free_mem;

	reserve_video_memory(m_filter);

	m_filter->info.name = "xmorph-dev";
	m_filter->info.version = "1.0";
	ret = uio_register_device(&pdev->dev, &m_filter->info);

	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register UIO device for morphological filter: error code = %d\n", ret);
		goto err_free_mem;
	}

	init_xmorph_registers(m_filter);

	dev_info(&pdev->dev, "successfully probed uio morphological filter\n");
	return 0;

err_free_mem:
	kfree(m_filter);
	return ret;
}

static const struct of_device_id morph_filter_of_match[] = {
	{ .compatible = "h_da,morph-filter-v1.0" },
	{},
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
