// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal MediaTek SENINF bring-up driver.
 *
 * The SENINF block is the MIPI CSI-2 receiver crossbar in front of the
 * CAM/CAMSV capture engines.  Full capture support needs the CSI-2 lane/VC/DT
 * programming and the CAM TG/DMA path; this driver only maps the register
 * window, enables the SENINF clock and exposes a debugfs register file so the
 * sequence can be developed on hardware.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

struct mtk_seninf {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct dentry *dbg;
};

static ssize_t mtk_seninf_reg_read(struct file *file, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct mtk_seninf *seninf = file->private_data;
	char buf[256];
	int n;

	if (*ppos)
		return 0;

	n = scnprintf(buf, sizeof(buf),
		      "mux_ctrl0=%08x mux_ctrl1=%08x mux_ctrl2=%08x mux_ctrl3=%08x\n"
		      "irq_en=%08x irq_status=%08x chk_ctl=%08x chk_res=%08x\n",
		      readl(seninf->base + 0x400), readl(seninf->base + 0x404),
		      readl(seninf->base + 0x408), readl(seninf->base + 0x40c),
		      readl(seninf->base + 0x4a0), readl(seninf->base + 0x4a8),
		      readl(seninf->base + 0x500), readl(seninf->base + 0x508));
	return simple_read_from_buffer(ubuf, count, ppos, buf, n);
}

static ssize_t mtk_seninf_reg_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct mtk_seninf *seninf = file->private_data;
	char buf[64];
	u32 off, val;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (sscanf(buf, "%x %x", &off, &val) != 2 || off >= 0x10000)
		return -EINVAL;

	dev_info(seninf->dev, "seninf write %04x = %08x\n", off, val);
	writel(val, seninf->base + off);
	return count;
}

static const struct file_operations mtk_seninf_reg_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = mtk_seninf_reg_read,
	.write = mtk_seninf_reg_write,
	.llseek = default_llseek,
};

static int mtk_seninf_probe(struct platform_device *pdev)
{
	struct mtk_seninf *seninf;

	seninf = devm_kzalloc(&pdev->dev, sizeof(*seninf), GFP_KERNEL);
	if (!seninf)
		return -ENOMEM;
	seninf->dev = &pdev->dev;
	platform_set_drvdata(pdev, seninf);

	seninf->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(seninf->base))
		return PTR_ERR(seninf->base);

	seninf->clk = devm_clk_get_enabled(&pdev->dev, "seninf");
	if (IS_ERR(seninf->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(seninf->clk),
				     "failed to enable the seninf clock\n");

	seninf->dbg = debugfs_create_dir("mtk-seninf", NULL);
	debugfs_create_file("reg", 0600, seninf->dbg, seninf,
			    &mtk_seninf_reg_fops);

	dev_info(&pdev->dev, "seninf ready: mux0=%08x mux1=%08x phy-mode=%08x\n",
		 readl(seninf->base + 0x400), readl(seninf->base + 0x404),
		 readl(seninf->base + 0x200));
	return 0;
}

static const struct of_device_id mtk_seninf_of_match[] = {
	{ .compatible = "mediatek,mt6833-seninf" },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);

static struct platform_driver mtk_seninf_drv = {
	.probe = mtk_seninf_probe,
	.driver = {
		.name = "mtk-seninf",
		.of_match_table = mtk_seninf_of_match,
	},
};
module_platform_driver(mtk_seninf_drv);
MODULE_DESCRIPTION("MediaTek SENINF bring-up driver");
MODULE_LICENSE("GPL");
