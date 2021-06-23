/*
 * A GEM style device manager for PCIe based OpenCL accelerators.
 *
 * Copyright (C) 2021 Xilinx, Inc. All rights reserved.
 *
 * Authors: Chien-Wei Lan <chienwei@xilinx.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "../xocl_drv.h"
#include "profile_ioctl.h"

struct xocl_add {
	void __iomem		*base;
	struct device		*dev;
	uint64_t		start_paddr;
	uint64_t		range;
	struct mutex 		lock;
	struct debug_ip_data	data;
};

static ssize_t name_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xocl_add *add = platform_get_drvdata(to_platform_device(dev));
	return sprintf(buf, "aximm_mon_%llu\n",add->data.m_base_address);
}

static DEVICE_ATTR_RO(name);

static struct attribute *add_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static struct attribute_group add_attr_group = {
	.attrs = add_attrs,
};

static int add_remove(struct platform_device *pdev)
{
	struct xocl_add *add;
	void *hdl;

	add = platform_get_drvdata(pdev);
	if (!add) {
		xocl_err(&pdev->dev, "driver data is NULL");
		return -EINVAL;
	}

	sysfs_remove_group(&pdev->dev.kobj, &add_attr_group);

	xocl_drvinst_release(add, &hdl);

	if (add->base)
		iounmap(add->base);

	platform_set_drvdata(pdev, NULL);

	xocl_drvinst_free(hdl);

	return 0;
}

static int add_probe(struct platform_device *pdev)
{
	struct xocl_add *add;
	struct resource *res;
	void *priv;
	int err = 0;

	add = xocl_drvinst_alloc(&pdev->dev, sizeof(struct xocl_add));
	if (!add)
		return -ENOMEM;

	add->dev = &pdev->dev;

	priv = XOCL_GET_SUBDEV_PRIV(&pdev->dev);
	if (priv)
		memcpy(&add->data, priv, sizeof(struct debug_ip_data));

	platform_set_drvdata(pdev, add);
	mutex_init(&add->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENOMEM;
		goto done;
	}


	xocl_info(&pdev->dev, "IO start: 0x%llx, end: 0x%llx",
		res->start, res->end);

	add->base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!add->base) {
		err = -EIO;
		xocl_err(&pdev->dev, "Map iomem failed");
		goto done;
	}

	add->start_paddr = res->start;
	add->range = res->end - res->start + 1;

	err = sysfs_create_group(&pdev->dev.kobj, &add_attr_group);
	if (err) {
		xocl_err(&pdev->dev, "create add sysfs attrs failed: %d", err);
	}

done:
	if (err) {
		add_remove(pdev);
		return err;
	}
	return 0;
}

static int add_open(struct inode *inode, struct file *file)
{
	struct xocl_add *add = NULL;

	add = xocl_drvinst_open_single(inode->i_cdev);
	if (!add)
		return -ENXIO;
	file->private_data = add;
	return 0;
}

static int add_close(struct inode *inode, struct file *file)
{
	struct xocl_add *add = file->private_data;

	xocl_drvinst_close(add);
	return 0;
}

long add_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct xocl_add *add;
	void __user *data;
	long result = 0;

	add = (struct xocl_add *)filp->private_data;
	data = (void __user *)(arg);

	mutex_lock(&add->lock);

	switch (cmd) {
	case 1:
		break;
	default:
		result = -ENOTTY;
	}
	mutex_unlock(&add->lock);

	return result;
}


static int add_mmap(struct file *filp, struct vm_area_struct *vma)
{

	int rc;
	unsigned long off;
	unsigned long phys;
	unsigned long vsize;
	unsigned long psize;
	struct xocl_add *add = (struct xocl_add *)filp->private_data;
	BUG_ON(!add);

	off = vma->vm_pgoff << PAGE_SHIFT;
	/* BAR physical address */
	phys = add->start_paddr + off;
	vsize = vma->vm_end - vma->vm_start;
	/* complete resource */
	psize = add->range - off;


	if (vsize > psize)
		return -EINVAL;

	/*
	 * pages must not be cached as this would result in cache line sized
	 * accesses to the end point
	 */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	/*
	 * prevent touching the pages (byte access) for swap-in,
	 * and prevent the pages from being swapped out
	 */
#ifndef VM_RESERVED
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#else
	vma->vm_flags |= VM_IO | VM_RESERVED;
#endif

	/* make MMIO accessible to user space */
	rc = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
				vsize, vma->vm_page_prot);
	if (rc)
		return -EAGAIN;
	return rc;
}


static const struct file_operations add_fops = {
	.open = add_open,
	.release = add_close,
	.mmap = add_mmap,
	.unlocked_ioctl = add_ioctl,
};

struct xocl_drv_private add_priv = {
	.fops = &add_fops,
	.dev = -1,
};

struct platform_device_id add_id_table[] = {
	{ XOCL_DEVNAME(XOCL_ADD), (kernel_ulong_t)&add_priv },
	{ },
};

static struct platform_driver	add_driver = {
	.probe		= add_probe,
	.remove		= add_remove,
	.driver		= {
		.name = XOCL_DEVNAME(XOCL_ADD),
	},
	.id_table = add_id_table,
};

int __init xocl_init_add(void)
{
	int err = 0;

	err = alloc_chrdev_region(&add_priv.dev, 0, XOCL_MAX_DEVICES,
			XOCL_ADD);
	if (err < 0)
		goto err_chrdev_reg;

	err = platform_driver_register(&add_driver);
	if (err < 0)
		goto err_driver_reg;

	return 0;
err_driver_reg:
	unregister_chrdev_region(add_priv.dev, 1);
err_chrdev_reg:
	return err;
}

void xocl_fini_add(void)
{
	unregister_chrdev_region(add_priv.dev, XOCL_MAX_DEVICES);
	platform_driver_unregister(&add_driver);
}
