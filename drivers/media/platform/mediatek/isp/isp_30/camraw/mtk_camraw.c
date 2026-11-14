// SPDX-License-Identifier: GPL-2.0
/* Minimal CAM_A (MRAW) capture helper for MT6833 bring-up. */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>

#define CAMRAW_W 4080
#define CAMRAW_H 3072
#define CAMRAW_SIZE (CAMRAW_W * CAMRAW_H * 2)

struct camraw_dev {
	struct device *dev;
	void *pvirt;
	phys_addr_t pphys;
	size_t psize;
	void __iomem *regs;
	int irq;
	unsigned int irq_count;
	unsigned int status_seen[8];
	unsigned int status_cnt;
	unsigned int cq_armed;
	unsigned int cq_started;
	struct cmdq_client *cmdq;
	void __iomem *gce_regs;
	struct clk_bulk_data *clks;
	int num_clks;
	void *buf;
	dma_addr_t phys;
	struct dentry *dir;
	struct dentry *f_frame;
};

static int camraw_size_show(struct seq_file *s, void *unused)
{
	struct camraw_dev *cr = s->private;

	seq_printf(s, "%u\n%pad\n", CAMRAW_SIZE, &cr->phys);
	return 0;
}
static int camraw_size_open(struct inode *inode, struct file *file)
{
	return single_open(file, camraw_size_show, inode->i_private);
}
static const struct file_operations camraw_size_fops = {
	.owner = THIS_MODULE,
	.open = camraw_size_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t camraw_read(struct file *file, char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	loff_t pos = *ppos;
	size_t avail;

	if (pos >= CAMRAW_SIZE)
		return 0;
	avail = CAMRAW_SIZE - pos;
	if (count > avail)
		count = avail;
	if (copy_to_user(ubuf, (char *)cr->buf + pos, count))
		return -EFAULT;
	*ppos = pos + count;
	return count;
}
static ssize_t camraw_write(struct file *file, const char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	loff_t pos = *ppos;
	size_t avail;

	if (pos >= CAMRAW_SIZE)
		return -ENOSPC;
	avail = CAMRAW_SIZE - pos;
	if (count > avail)
		count = avail;
	if (copy_from_user((char *)cr->buf + pos, ubuf, count))
		return -EFAULT;
	*ppos = pos + count;
	return count;
}

static int camraw_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static const struct file_operations camraw_frame_fops = {
	.owner = THIS_MODULE,
	.open = camraw_open,
	.read = camraw_read,
	.write = camraw_write,
	.llseek = default_llseek,
};

#define CAM_REG_INT6_STATUS	0x0154
#define CAM_REG_INT6_STATUSX	0x0158
#define CAM_REG_INT5_STATUS	0x0144
#define CAM_REG_INT2_STATUS	0x0114
#define CAM_REG_RAW_INT_STATUS	0x0104
#define CAM_REG_IMGO_ERR	0x4068
#define CAM_REG_DMA_CQ_CNT	0x40ac
#define CAM_REG_TG_INTER_ST	0x1f3c

static irqreturn_t mtk_camraw_irq(int irq, void *data)
{
	struct camraw_dev *cr = data;
	unsigned int st = readl(cr->regs + CAM_REG_RAW_INT_STATUS);
	unsigned int st2 = readl(cr->regs + CAM_REG_INT6_STATUSX);
	unsigned int seen = (st << 16) | (st2 & 0xffff);
	int i, noval = 1;

	cr->irq_count++;

	if (cr->cq_armed == 1 && (st & 0x1)) {
		/* sync the CQ start to the VS interrupt like the vendor HAL */
		writel(0, cr->regs + 0x74);
		writel(1, cr->regs + 0x74);
		cr->cq_armed = 2;
		dev_info(cr->dev, "cqsched: START written on VS (tg=%08x)\n",
			 readl(cr->regs + CAM_REG_TG_INTER_ST));
	}
	if (cr->cq_armed == 2) {
		unsigned int s6 = readl(cr->regs + CAM_REG_INT6_STATUS);
		unsigned int s6x = readl(cr->regs + CAM_REG_INT6_STATUSX);
		unsigned int v150 = readl(cr->regs + 0x150);

		if (s6 || s6x || v150 == 0x5a5a5a5a) {
			dev_info(cr->dev,
				 "cqsched: int6=%08x int6x=%08x 150=%08x tg=%08x\n",
				 s6, s6x, v150,
				 readl(cr->regs + CAM_REG_TG_INTER_ST));
			cr->cq_armed = 0;
		}
	}
	for (i = 0; i < 8; i++) {
		if (cr->status_seen[i] == seen) {
			noval = 0;
			break;
		}
	}
	if (noval && cr->status_cnt < 8) {
		cr->status_seen[cr->status_cnt++] = seen;
		dev_info(cr->dev, "irq#%u raw=%08x int6x=%08x tg=%08x imgo_err=%08x cqcnt=%08x\n",
			 cr->irq_count, st, st2,
			 readl(cr->regs + CAM_REG_TG_INTER_ST),
			 readl(cr->regs + CAM_REG_IMGO_ERR),
			 readl(cr->regs + CAM_REG_DMA_CQ_CNT));
	}
	/* ack: status regs are W1C */
	if (st)
		writel(st, cr->regs + CAM_REG_RAW_INT_STATUS);
	if (st2)
		writel(st2, cr->regs + CAM_REG_INT6_STATUSX);
	return IRQ_HANDLED;
}

static ssize_t camraw_cqsched_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	char buf[16];
	int val;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (val == 1) {
		/* program the CQ thread for the ring in camraw buffer */
		writel(0x112, cr->regs + 0x200);
		writel(0x21, cr->regs + 0x210);
		writel(0xDB8, cr->regs + 0x218);
		writel((u32)cr->phys, cr->regs + 0x214);
		cr->cq_armed = 1;
		cr->cq_started = 0;
		dev_info(cr->dev, "cqsched armed: base=%pad size=%x\n",
			 &cr->phys, 0xDB8);
	} else {
		cr->cq_armed = 0;
		dev_info(cr->dev, "cqsched disarmed\n");
	}
	return count;
}
static const struct file_operations camraw_cqsched_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = camraw_cqsched_write,
	.llseek = default_llseek,
};

static int camraw_gce_run(struct camraw_dev *cr, u8 subsys, int cq_mode)
{
	struct cmdq_pkt pkt;
	int ret;

	if (!cr->cmdq)
		return -ENODEV;

	ret = cmdq_pkt_create(cr->cmdq, &pkt, 512);
	if (ret)
		return ret;

	if (cq_mode) {
		cmdq_pkt_write(&pkt, subsys, 0x200, 0x112);
		cmdq_pkt_write(&pkt, subsys, 0x210, 0x21);
		cmdq_pkt_write(&pkt, subsys, 0x218, 0xDB8);
		cmdq_pkt_write(&pkt, subsys, 0x214, (u32)cr->phys);
		cmdq_pkt_write(&pkt, subsys, 0x74, 0);
		cmdq_pkt_write(&pkt, subsys, 0x74, 1);
	} else {
		/* single write used to probe the subsys id */
		cmdq_pkt_write(&pkt, subsys, 0x482c, 0x1234);
	}

	cmdq_pkt_eoc(&pkt);
	dma_sync_single_for_device(cr->cmdq->chan->mbox->dev, pkt.pa_base,
				   pkt.cmd_buf_size, DMA_TO_DEVICE);
	ret = mbox_send_message(cr->cmdq->chan, &pkt);
	if (ret < 0) {
		dev_err(cr->dev, "mbox_send_message failed %d\n", ret);
		cmdq_pkt_destroy(cr->cmdq, &pkt);
		return ret;
	}
	mbox_client_txdone(cr->cmdq->chan, 0);
	{
		struct device *gce = cr->cmdq->chan->mbox->dev;

		pm_runtime_get_sync(gce);
		msleep(5);
		if (cr->gce_regs) {
			void __iomem *th = cr->gce_regs + 0x100 + 12 * 0x80;

			dev_info(cr->dev,
				 "gce th12: en=%08x st=%08x cur=%08x end=%08x\n",
				 readl(th + 0x04), readl(th + 0x0c),
				 readl(th + 0x20), readl(th + 0x24));
		}
		pm_runtime_put_autosuspend(gce);
	}
	msleep(30);
	cmdq_pkt_destroy(cr->cmdq, &pkt);
	return 0;
}

static ssize_t camraw_gce_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	char buf[64];
	unsigned int subsys = 0, cq = 0;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (sscanf(buf, "%u %u", &subsys, &cq) != 2)
		return -EINVAL;
	dev_info(cr->dev, "gce run subsys=%u cq=%u\n", subsys, cq);
	return camraw_gce_run(cr, subsys, cq) ?: count;
}
static const struct file_operations camraw_gce_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = camraw_gce_write,
	.llseek = default_llseek,
};

static int camraw_irqstats_show(struct seq_file *s, void *unused)
{
	struct camraw_dev *cr = s->private;

	seq_printf(s, "irq_count=%u\n", cr->irq_count);
	seq_printf(s, "raw_int=%08x int6x=%08x int2=%08x int5=%08x\n",
		   readl(cr->regs + CAM_REG_RAW_INT_STATUS),
		   readl(cr->regs + CAM_REG_INT6_STATUSX),
		   readl(cr->regs + CAM_REG_INT2_STATUS),
		   readl(cr->regs + CAM_REG_INT5_STATUS));
	seq_printf(s, "tg=%08x imgo_err=%08x cqcnt=%08x\n",
		   readl(cr->regs + CAM_REG_TG_INTER_ST),
		   readl(cr->regs + CAM_REG_IMGO_ERR),
		   readl(cr->regs + CAM_REG_DMA_CQ_CNT));
	return 0;
}
static int camraw_irqstats_open(struct inode *inode, struct file *file)
{
	return single_open(file, camraw_irqstats_show, inode->i_private);
}
static const struct file_operations camraw_irqstats_fops = {
	.owner = THIS_MODULE,
	.open = camraw_irqstats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t camraw_pbuf_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	loff_t pos = *ppos;

	if (pos >= cr->psize)
		return 0;
	if (count > cr->psize - pos)
		count = cr->psize - pos;
	if (copy_to_user(ubuf, (char *)cr->pvirt + pos, count))
		return -EFAULT;
	*ppos = pos + count;
	return count;
}

static ssize_t camraw_pbuf_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	loff_t pos = *ppos;

	if (pos >= cr->psize)
		return -ENOSPC;
	if (count > cr->psize - pos)
		count = cr->psize - pos;
	if (copy_from_user((char *)cr->pvirt + pos, ubuf, count))
		return -EFAULT;
	*ppos = pos + count;
	return count;
}

static const struct file_operations camraw_pbuf_fops = {
	.owner = THIS_MODULE,
	.open = camraw_open,
	.read = camraw_pbuf_read,
	.write = camraw_pbuf_write,
	.llseek = default_llseek,
};

static ssize_t camraw_pphys_read(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	char buf[64];
	int n = scnprintf(buf, sizeof(buf), "%pa\n", &cr->pphys);

	return simple_read_from_buffer(ubuf, count, ppos, buf, n);
}

static const struct file_operations camraw_pphys_fops = {
	.owner = THIS_MODULE,
	.open = camraw_open,
	.read = camraw_pphys_read,
	.llseek = default_llseek,
};

static ssize_t camraw_gcepa_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct camraw_dev *cr = file->private_data;
	char buf[64];
	unsigned int addr = 0, val = 0, cq = 0;
	struct cmdq_pkt pkt;
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (sscanf(buf, "%x %x %u", &addr, &val, &cq) < 2)
		return -EINVAL;
	if (!cr->cmdq)
		return -ENODEV;

	ret = cmdq_pkt_create(cr->cmdq, &pkt, 512);
	if (ret)
		return ret;

	if (cq) {
		/* full CAM_A CQ start sequence via physical-address GCE writes */
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x200, 0x112);
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x210, 0x11);
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x218, 0xDB8);
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x214, (u32)cr->phys);
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x074, 0);
		cmdq_pkt_write_pa(&pkt, 0, 0x1a030000, 0x074, 1);
	} else {
		cmdq_pkt_write_pa(&pkt, 0, addr & ~0xffffUL, addr & 0xffff, val);
	}
	cmdq_pkt_eoc(&pkt);
	dma_sync_single_for_device(cr->cmdq->chan->mbox->dev, pkt.pa_base,
				   pkt.cmd_buf_size, DMA_TO_DEVICE);
	ret = mbox_send_message(cr->cmdq->chan, &pkt);
	if (ret < 0) {
		dev_err(cr->dev, "gcepa send failed %d\n", ret);
		cmdq_pkt_destroy(cr->cmdq, &pkt);
		return ret;
	}
	mbox_client_txdone(cr->cmdq->chan, 0);
	msleep(30);
	cmdq_pkt_destroy(cr->cmdq, &pkt);
	dev_info(cr->dev, "gcepa: addr=%x val=%x cq=%u sent\n", addr, val, cq);
	return count;
}
static const struct file_operations camraw_gcepa_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = camraw_gcepa_write,
	.llseek = default_llseek,
};

static int mtk_camraw_probe(struct platform_device *pdev)
{
	struct camraw_dev *cr;
	int ret;

	cr = devm_kzalloc(&pdev->dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;
	cr->dev = &pdev->dev;
	cr->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cr->regs))
		return PTR_ERR(cr->regs);
	cr->num_clks = devm_clk_bulk_get_all(&pdev->dev, &cr->clks);
	if (cr->num_clks < 0)
		dev_warn(&pdev->dev, "no clocks: %d\n", cr->num_clks);
	else {
		ret = clk_bulk_prepare_enable(cr->num_clks, cr->clks);
		if (ret)
			dev_warn(&pdev->dev, "clk enable failed %d\n", ret);
		dev_info(&pdev->dev, "enabled %d clocks\n", cr->num_clks);
	}
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	cr->buf = dma_alloc_coherent(&pdev->dev, CAMRAW_SIZE, &cr->phys,
				     GFP_KERNEL);
	if (!cr->buf) {
		dev_err(&pdev->dev, "dma_alloc_coherent failed\n");
		return -ENOMEM;
	}
	memset(cr->buf, 0, CAMRAW_SIZE);

	cr->irq = platform_get_irq(pdev, 0);
	if (cr->irq < 0) {
		dev_warn(&pdev->dev, "no irq: %d\n", cr->irq);
	} else {
		ret = devm_request_irq(&pdev->dev, cr->irq, mtk_camraw_irq,
				       IRQF_TRIGGER_NONE, "mtk-camraw", cr);
		if (ret)
			dev_warn(&pdev->dev, "request_irq %d failed %d\n",
				 cr->irq, ret);
		else
			dev_info(&pdev->dev, "irq %d registered\n", cr->irq);
	}

	cr->psize = 0x10000;
	cr->pvirt = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA32, 4);
	if (cr->pvirt) {
		cr->pphys = virt_to_phys(cr->pvirt);
		dev_info(&pdev->dev, "physbuf virt=%p phys=%pa size=%zu\n",
			 cr->pvirt, &cr->pphys, cr->psize);
	} else {
		dev_warn(&pdev->dev, "physbuf alloc failed\n");
	}

	cr->gce_regs = devm_ioremap(&pdev->dev, 0x10228000, 0x4000);
	if (!cr->gce_regs)
		dev_warn(&pdev->dev, "gce ioremap failed\n");

	cr->cmdq = cmdq_mbox_create(&pdev->dev, 0);
	if (IS_ERR(cr->cmdq)) {
		dev_warn(&pdev->dev, "cmdq client failed %ld\n",
			 PTR_ERR(cr->cmdq));
		cr->cmdq = NULL;
	} else {
		dev_info(&pdev->dev, "cmdq client ready\n");
	}

	cr->dir = debugfs_create_dir("camraw", NULL);
	debugfs_create_file("size", 0444, cr->dir, cr,
					 &camraw_size_fops);
	debugfs_create_file("irqstats", 0444, cr->dir, cr,
					 &camraw_irqstats_fops);
	debugfs_create_file("physbuf", 0644, cr->dir, cr,
					 &camraw_pbuf_fops);
	debugfs_create_file("physaddr", 0444, cr->dir, cr,
					 &camraw_pphys_fops);
	debugfs_create_file("cqsched", 0200, cr->dir, cr,
					 &camraw_cqsched_fops);
	debugfs_create_file("gce", 0200, cr->dir, cr,
					 &camraw_gce_fops);
	debugfs_create_file("gcepa", 0200, cr->dir, cr,
					 &camraw_gcepa_fops);
	cr->f_frame = debugfs_create_file("frame", 0644, cr->dir, cr,
					  &camraw_frame_fops);
	platform_set_drvdata(pdev, cr);
	dev_info(&pdev->dev, "camraw buf phys=%pad size=%u\n", &cr->phys,
		 CAMRAW_SIZE);
	return 0;
}

static void mtk_camraw_remove(struct platform_device *pdev)
{
	struct camraw_dev *cr = platform_get_drvdata(pdev);

	debugfs_remove_recursive(cr->dir);
	dma_free_coherent(&pdev->dev, CAMRAW_SIZE, cr->buf, cr->phys);
}

static const struct of_device_id mtk_camraw_of_match[] = {
	{ .compatible = "mediatek,mt6833-camraw" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_camraw_of_match);

static struct platform_driver mtk_camraw_driver = {
	.probe = mtk_camraw_probe,
	.remove = mtk_camraw_remove,
	.driver = {
		.name = "mtk-camraw",
		.of_match_table = mtk_camraw_of_match,
	},
};
module_platform_driver(mtk_camraw_driver);
MODULE_LICENSE("GPL");
