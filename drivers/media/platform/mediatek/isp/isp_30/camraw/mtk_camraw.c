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

#define CAMRAW_W 4080
#define CAMRAW_H 3072
#define CAMRAW_SIZE (CAMRAW_W * CAMRAW_H * 2)

struct camraw_dev {
	struct device *dev;
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

static int mtk_camraw_probe(struct platform_device *pdev)
{
	struct camraw_dev *cr;
	int ret;

	cr = devm_kzalloc(&pdev->dev, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return -ENOMEM;
	cr->dev = &pdev->dev;
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

	cr->dir = debugfs_create_dir("camraw", NULL);
	debugfs_create_file("size", 0444, cr->dir, cr,
					 &camraw_size_fops);
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
