/*
 * A GEM style device manager for PCIe based OpenCL accelerators.
 *
 * Copyright (C) 2020 Xilinx, Inc. All rights reserved.
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
#include "kds_client.h"
#include "xrt_ert.h"

#define	ERT_MAX_SLOTS		128

#define	ERT_STATE_GOOD		0x1
#define	ERT_STATE_BAD		0x2

#define CQ_STATUS_OFFSET	(ERT_CQ_STATUS_REGISTER_ADDR - ERT_CSR_ADDR)

//#define	SCHED_VERBOSE	1
/* ERT gpio config has two channels 
 * CHANNEL 0 is control channel :
 * BIT 0: 0x0 Selects interrupts from embedded scheduler HW block
 * 	  0x1 Selects interrupts from the CU INTCs
 * BIT 2-1: TBD
 *
 * CHANNEL 1 is status channel :
 * BIT 0: check microblazer status
 */

#define GPIO_CFG_CTRL_CHANNEL	0x0
#define GPIO_CFG_STA_CHANNEL	0x8

#define SWITCH_TO_CU_INTR	0x1
#define SWITCH_TO_ERT_INTR	~SWITCH_TO_CU_INTR

#define WAKE_MB_UP		0x2
#define CLEAR_MB_WAKEUP		~WAKE_MB_UP

#ifdef SCHED_VERBOSE
#define	ERTUSER_ERR(ert_user, fmt, arg...)	\
	xocl_err(ert_user->dev, fmt "", ##arg)
#define	ERTUSER_WARN(ert_user, fmt, arg...)	\
	xocl_warn(ert_user->dev, fmt "", ##arg)	
#define	ERTUSER_INFO(ert_user, fmt, arg...)	\
	xocl_info(ert_user->dev, fmt "", ##arg)	
#define	ERTUSER_DBG(ert_user, fmt, arg...)	\
	xocl_info(ert_user->dev, fmt "", ##arg)

#else
#define	ERTUSER_ERR(ert_user, fmt, arg...)	\
	xocl_err(ert_user->dev, fmt "", ##arg)
#define	ERTUSER_WARN(ert_user, fmt, arg...)	\
	xocl_warn(ert_user->dev, fmt "", ##arg)	
#define	ERTUSER_INFO(ert_user, fmt, arg...)	\
	xocl_info(ert_user->dev, fmt "", ##arg)	
#define	ERTUSER_DBG(ert_user, fmt, arg...)
#endif


#define sched_debug_packet(packet, size)				\
({									\
	int i;								\
	u32 *data = (u32 *)packet;					\
	for (i = 0; i < size; ++i)					    \
		DRM_INFO("packet(0x%p) execbuf[%d] = 0x%x\n", data, i, data[i]); \
})

extern int kds_echo;

struct command_queue {
	struct device		*dev;
	struct platform_device	*pdev;
	void __iomem		*cfg_gpio;
	void __iomem		*cq_base;
	uint64_t		cq_range;
	bool			polling_mode;
	struct mutex 		lock;

	bool			is_configured;
	bool			ctrl_busy;
	unsigned int		num_slots;
	unsigned int            slot_size;
	// Bitmap tracks busy(1)/free(0) slots in command_queue
	DECLARE_BITMAP(slot_status, ERT_MAX_SLOTS);
	struct xocl_ert_sched_privdata ert_cfg_priv;

	struct xrt_ert_command	*submit_queue[ERT_MAX_SLOTS];

	uint32_t 		ert_dmsg;
	uint32_t		echo;
	uint32_t		intr;
	struct xrt_ert 		ert;
	struct ert_validate_cmd ert_valid;
};

static void command_queue_submit(struct xrt_ert_command *ecmd, void *data);
static int32_t ert_user_gpio_cfg(struct platform_device *pdev, enum ert_gpio_cfg type);

static void ert_intc_enable(struct command_queue *ert_user, bool enable);

static ssize_t clock_timestamp_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%u\n", ert_user->ert_valid.timestamp);
}

static DEVICE_ATTR_RO(clock_timestamp);

static ssize_t snap_shot_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));
	struct xrt_ert *ert = &ert_user->ert;

	return sprintf(buf, "pq:%d pq_ctrl:%d,  rq:%d, rq_ctrl:%d, sq:%d cq:%d\n", ert->pq.num, ert->pq_ctrl.num, ert->rq.num
		,ert->rq_ctrl.num, ert->sq.num, ert->cq.num);
}

static DEVICE_ATTR_RO(snap_shot);

static ssize_t ert_dmsg_store(struct device *dev,
	struct device_attribute *da, const char *buf, size_t count)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));
	u32 val;

	mutex_lock(&ert_user->lock);
	if (kstrtou32(buf, 10, &val) == -EINVAL || val > 2) {
		xocl_err(&to_platform_device(dev)->dev,
			"usage: echo 0 or 1 > ert_dmsg");
		return -EINVAL;
	}

	ert_user->ert_dmsg = val;

	mutex_unlock(&ert_user->lock);
	return count;
}
static DEVICE_ATTR_WO(ert_dmsg);

static ssize_t ert_echo_store(struct device *dev,
	struct device_attribute *da, const char *buf, size_t count)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));
	u32 val;

	mutex_lock(&ert_user->lock);
	if (kstrtou32(buf, 10, &val) == -EINVAL || val > 2) {
		xocl_err(&to_platform_device(dev)->dev,
			"usage: echo 0 or 1 > ert_echo");
		return -EINVAL;
	}

	ert_user->echo = val;

	mutex_unlock(&ert_user->lock);
	return count;
}
static DEVICE_ATTR_WO(ert_echo);

static ssize_t ert_intr_store(struct device *dev,
	struct device_attribute *da, const char *buf, size_t count)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));
	u32 val;

	mutex_lock(&ert_user->lock);
	if (kstrtou32(buf, 10, &val) == -EINVAL || val > 2) {
		xocl_err(&to_platform_device(dev)->dev,
			"usage: echo 0 or 1 > ert_intr");
		return -EINVAL;
	}

	ert_user->intr = val;

	mutex_unlock(&ert_user->lock);
	return count;
}
static DEVICE_ATTR_WO(ert_intr);

static ssize_t mb_sleep_store(struct device *dev,
	struct device_attribute *da, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	u32 go_sleep;

	if (kstrtou32(buf, 10, &go_sleep) == -EINVAL || go_sleep > 2) {
		xocl_err(&to_platform_device(dev)->dev,
			"usage: echo 0 or 1 > mb_sleep");
		return -EINVAL;
	}

	if (go_sleep)
		ert_user_gpio_cfg(pdev, MB_SLEEP);
	else
		ert_user_gpio_cfg(pdev, MB_WAKEUP);

	return count;
}

static ssize_t mb_sleep_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);

	return sprintf(buf, "%d", ert_user_gpio_cfg(pdev, MB_STATUS));
}

static DEVICE_ATTR_RW(mb_sleep);

static ssize_t cq_read_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cq_read_single);
}

static DEVICE_ATTR_RO(cq_read_cnt);

static ssize_t cq_write_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cq_write_single);
}

static DEVICE_ATTR_RO(cq_write_cnt);

static ssize_t cu_read_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cu_read_single);
}

static DEVICE_ATTR_RO(cu_read_cnt);

static ssize_t cu_write_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct command_queue *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cu_write_single);
}

static DEVICE_ATTR_RO(cu_write_cnt);

static struct attribute *ert_user_attrs[] = {
	&dev_attr_clock_timestamp.attr,
	&dev_attr_ert_dmsg.attr,
	&dev_attr_snap_shot.attr,
	&dev_attr_ert_echo.attr,
	&dev_attr_ert_intr.attr,
	&dev_attr_mb_sleep.attr,
	&dev_attr_cq_read_cnt.attr,
	&dev_attr_cq_write_cnt.attr,
	&dev_attr_cu_read_cnt.attr,
	&dev_attr_cu_write_cnt.attr,
	NULL,
};

static struct attribute_group ert_user_attr_group = {
	.attrs = ert_user_attrs,
};


static int32_t ert_user_gpio_cfg(struct platform_device *pdev, enum ert_gpio_cfg type)
{
	struct command_queue *ert_user = platform_get_drvdata(pdev);
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);
	int32_t ret = 0, val = 0;
	int i;

	if (!ert_user->cfg_gpio) {
		ERTUSER_WARN(ert_user, "%s ERT config gpio not found\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&ert_user->lock);
	val = ioread32(ert_user->cfg_gpio);

	switch (type) {
	case INTR_TO_ERT:
		val &= SWITCH_TO_ERT_INTR;
		iowrite32(val, ert_user->cfg_gpio+GPIO_CFG_CTRL_CHANNEL);
		for (i = 0; i < ert_user->num_slots; i++)
			xocl_intc_ert_config(xdev, i, true);
		/* TODO: This could return error code -EBUSY. */
		xocl_intc_set_mode(xocl_get_xdev(pdev), ERT_INTR);
		break;
	case INTR_TO_CU:
		val |= SWITCH_TO_CU_INTR;
		iowrite32(val, ert_user->cfg_gpio+GPIO_CFG_CTRL_CHANNEL);
		for (i = 0; i < ert_user->num_slots; i++)
			xocl_intc_ert_config(xdev, i, false);
		/* TODO: This could return error code -EBUSY. */
		xocl_intc_set_mode(xocl_get_xdev(pdev), CU_INTR);
		break;
	case MB_WAKEUP:
		val |= WAKE_MB_UP;
		iowrite32(val, ert_user->cfg_gpio+GPIO_CFG_CTRL_CHANNEL);
		break;
	case MB_SLEEP:
		val &= CLEAR_MB_WAKEUP;
		iowrite32(val, ert_user->cfg_gpio+GPIO_CFG_CTRL_CHANNEL);
		/* TODO: submit an EXIT command to ERT thread */
		iowrite32(ERT_EXIT_CMD, ert_user->cq_base);
		ret = ioread32(ert_user->cfg_gpio+GPIO_CFG_STA_CHANNEL);
		while (!ret)
			ret = ioread32(ert_user->cfg_gpio+GPIO_CFG_STA_CHANNEL);
		break;
	case MB_STATUS:
		ret = ioread32(ert_user->cfg_gpio+GPIO_CFG_STA_CHANNEL);
		break;
	default:
		break;
	}
	mutex_unlock(&ert_user->lock);
	return ret;
}

/*
 * type() - Command type
 *
 * @cmd: Command object
 * Return: Type of command
 */
static inline u32
cmd_opcode(struct xrt_ert_command *ecmd)
{
	return ecmd->xcmd->opcode;
}

/**
 * mask_idx32() - Slot mask idx index for a given slot_idx
 *
 * @slot_idx: Global [0..127] index of a CQ slot
 * Return: Index of the slot mask containing the slot_idx
 */
static inline unsigned int
mask_idx32(unsigned int idx)
{
	return idx >> 5;
}
/**
 * idx_in_mask32() - Index of command queue slot within the mask that contains it
 *
 * @slot_idx: Global [0..127] index of a CQ slot
 * Return: Index of slot within the mask that contains it
 */
static inline unsigned int
idx_in_mask32(unsigned int idx, unsigned int mask_idx)
{
	return idx - (mask_idx << 5);
}
#if 0
static int ert_user_bulletin(struct platform_device *pdev, struct ert_cu_bulletin *brd)
{
	struct command_queue *ert_user = platform_get_drvdata(pdev);
	int ret = 0;

	if (!brd)
		return -EINVAL;

	brd->sta.configured = ert_user->is_configured;
	brd->cap.cu_intr = ert_user->cfg_gpio ? 1 : 0;

	return ret;
}
#endif

#if 0
static int ert_user_enable(struct platform_device *pdev, bool enable)
{
	int ret = 0;

	if (enable) {
		ert_user_gpio_cfg(pdev, MB_WAKEUP);
		ert_user_gpio_cfg(pdev, INTR_TO_ERT);
	} else {
		ert_user_gpio_cfg(pdev, MB_SLEEP);
		ert_user_gpio_cfg(pdev, INTR_TO_CU);
	}

	return ret;
}
#endif
static void command_queue_submit(struct xrt_ert_command* ecmd, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);
	u32 slot_addr;
	u32 mask_idx, cq_int_addr, mask;
	struct ert_packet *epkt = NULL;

	printk(KERN_ERR "%s \n", __func__);

	epkt = (struct ert_packet *)ecmd->xcmd->execbuf;

	slot_addr = ecmd->handle * ert_user->slot_size;
	ert_user->submit_queue[ecmd->handle] = ecmd;

	if (kds_echo) {
		ecmd->completed = true;
	} else {

		if (cmd_opcode(ecmd) == OP_START) {
			// write kds selected cu_idx in first cumask (first word after header)
			iowrite32(ecmd->xcmd->cu_idx, ert_user->cq_base + slot_addr + 4);

			// write remaining packet (past header and cuidx)
			xocl_memcpy_toio(ert_user->cq_base + slot_addr + 8,
					 ecmd->xcmd->execbuf+2, (epkt->count-1)*sizeof(u32));
		} else {
			xocl_memcpy_toio(ert_user->cq_base + slot_addr + 4,
				  ecmd->xcmd->execbuf+1, epkt->count*sizeof(u32));
		}

		iowrite32(epkt->header, ert_user->cq_base + slot_addr);
	}
#if 0
	should be moved to common layer
	set_xcmd_timestamp(ecmd->xcmd, KDS_RUNNING);

#endif
	/*
	 * Always try to trigger interrupt to embedded scheduler.
	 * The reason is, the ert configure cmd is also sent to MB/PS through cq,
	 * and at the time the new ert configure cmd is sent, host doesn't know
	 * MB/PS is running in cq polling or interrupt mode. eg, if MB/PS is in
	 * cq interrupt mode, new ert configure is cq polling mode, but the new
	 * ert configure cmd has to be received by MB/PS throught interrupt mode
	 *
	 * Setting the bit in cq status register when MB/PS is in cq polling mode
	 * doesn't do harm since the interrupt is disabled and MB/PS will not read
	 * the register
	 */
	mask_idx = mask_idx32(ecmd->handle);
	cq_int_addr = CQ_STATUS_OFFSET + (mask_idx << 2);
	mask = 1 << idx_in_mask32(ecmd->handle, mask_idx);

	ERTUSER_DBG(ert_user, "++ mb_submit writes slot mask 0x%x to CQ_INT register at addr 0x%x\n",
		    mask, cq_int_addr);
	xocl_intc_ert_write32(xdev, mask, cq_int_addr);

}
#if 0
static struct xocl_command_queue_funcs ert_user_ops = {
	.bulletin = ert_user_bulletin,
	.enable = ert_user_enable,
};
#endif

static const unsigned int no_index = -1;
static void ert_user_reset(struct command_queue *ert_user)
{
	bitmap_zero(ert_user->slot_status, ERT_MAX_SLOTS);
	set_bit(0, ert_user->slot_status);
}

static inline int
ert_return_size(struct xrt_ert_command *ecmd, int max_size)
{
	int ret;

	/* Different opcode has different size of return info */
	switch (cmd_opcode(ecmd)) {
	case OP_GET_STAT:
		ret = max_size;
		break;
	case OP_START_SK:
		ret = 2 * sizeof(u32);
		break;
	case OP_VALIDATE:
	case OP_CLK_CALIB:
		ret = sizeof(struct ert_validate_cmd);		
	default:
		ret = 0;
	};

	return ret;
}

/* ERT would return some information when notify host. Ex. PS kernel start and
 * get CU stat commands. In this case, need read CQ slot to get return info.
 *
 * TODO:
 * Assume there are 64 PS kernel and 2 nornal CUs. The ERT_CU_STAT command
 * requires more than (64+2)*2*4 = 528 bytes (without consider other info).
 * In this case, the slot size needs to be 1K and maximum 64 CQ slots.
 *
 * In old kds, to avoid buffer overflow, it silently truncate return value.
 * Luckily there is always use 16 slots in old kds.
 * But truncate is definitly not ideal, this should be fixed in new KDS.
 */
static inline void
ert_get_return(struct command_queue *ert_user, struct xrt_ert_command *ecmd)
{
	u32 slot_addr;
	int slot_size = ert_user->slot_size;
	int size;
	void *dst = NULL;

	size = ert_return_size(ecmd, slot_size);
	if (!size)
		return;

	slot_addr = ecmd->handle * slot_size;
	
	switch (cmd_opcode(ecmd)) {
	case OP_VALIDATE:
	case OP_CLK_CALIB:
		dst = &ert_user->ert_valid;
		break;
	default:
		dst = ecmd->xcmd->u_execbuf;
		break;
	}

	if (size && dst)
		xocl_memcpy_fromio(dst, ert_user->cq_base + slot_addr, size);
}

static inline bool ert_special_cmd(struct xrt_ert_command *ecmd)
{
	bool ret;

	switch (cmd_opcode(ecmd)) {
	case OP_CONFIG:
	case OP_CONFIG_SK:
	case OP_GET_STAT:
	case OP_CLK_CALIB:
	case OP_VALIDATE:
		ret = true;
		break;
	default:
		ret = false;
	}

	return ret;
}
/*
 * release_slot_idx() - Release specified slot idx
 */
static void
ert_release_slot_idx(struct command_queue *ert_user, unsigned int slot_idx)
{
	clear_bit(slot_idx, ert_user->slot_status);
}

/**
 * command_queue_release() - Release a slot index for a command
 *
 * Special case for control commands that execute in slot 0.  This
 * slot cannot be marked free ever.
 */
static void
command_queue_release(struct xrt_ert_command *ecmd, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;

	if (ecmd->handle == no_index)
		return;

	/* special command always uses slot 0, don't reset bit0*/
	if (!ert_special_cmd(ecmd)) {
		ERTUSER_DBG(ert_user, "ecmd->handle %d\n", ecmd->handle);
		ert_release_slot_idx(ert_user, ecmd->handle);
	}
	ecmd->handle = no_index;
}

static irqreturn_t
ert_versal_isr(void *arg)
{
	struct command_queue *ert_user = (struct command_queue *)arg;
	xdev_handle_t xdev;
	struct xrt_ert_command *ecmd;

	BUG_ON(!ert_user);

	ERTUSER_DBG(ert_user, "-> %s\n", __func__);
	xdev = xocl_get_xdev(ert_user->pdev);

	if (!ert_user->polling_mode) {
		u32 slots[ERT_MAX_SLOTS];
		u32 cnt = 0;
		int slot;
		int i;

		while (!(xocl_mailbox_versal_get(xdev, &slot)))
			slots[cnt++] = slot;

		if (!cnt)
			return IRQ_HANDLED;

		for (i = 0; i < cnt; i++) {
			slot = slots[i];
			ERTUSER_DBG(ert_user, "[%s] slot: %d\n", __func__, slot);
			ecmd = ert_user->submit_queue[slot];
			if (ecmd) {
				ecmd->completed = true;
			} else {
				ERTUSER_ERR(ert_user, "not in submitted queue %d\n", slot);
			}
		}

#if 0
		up(&ert_user->sem);
		/* wake up all scheduler ... currently one only */
		if (xs->stop)
			return;

		if (xs->reset) {
			SCHED_DEBUG("scheduler is resetting after timeout\n");
			scheduler_reset(xs);
		}
#endif
	} else {
		ERTUSER_DBG(ert_user, "unhandled isr \r\n");
		return IRQ_NONE;
	}
	ERTUSER_DBG(ert_user, "<- %s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t
ert_user_isr(int irq, void *arg)
{
	struct command_queue *ert_user = (struct command_queue *)arg;
	xdev_handle_t xdev;
	struct xrt_ert_command *ecmd;

	BUG_ON(!ert_user);

	ERTUSER_DBG(ert_user, "-> xocl_user_event %d\n", irq);
	xdev = xocl_get_xdev(ert_user->pdev);

	BUG_ON(irq>=ERT_MAX_SLOTS);

	if (!ert_user->polling_mode) {

		ecmd = ert_user->submit_queue[irq];
		if (ecmd) {
			ecmd->completed = true;
		} else {
			ERTUSER_ERR(ert_user, "not in submitted queue %d\n", irq);
		}
#if 0
		up(&ert_user->sem);
		/* wake up all scheduler ... currently one only */

		if (xs->stop)
			return;

		if (xs->reset) {
			SCHED_DEBUG("scheduler is resetting after timeout\n");
			scheduler_reset(xs);
		}
#endif
	} else {
		ERTUSER_DBG(ert_user, "unhandled isr irq %d", irq);
		return IRQ_NONE;
	}
	ERTUSER_DBG(ert_user, "<- xocl_user_event %d\n", irq);
	return IRQ_HANDLED;
}

/*
 * acquire_slot_idx() - First available slot index
 */
static unsigned int
command_queue_acquire_slot_idx(struct command_queue *ert_user)
{
	unsigned int idx = find_first_zero_bit(ert_user->slot_status, ERT_MAX_SLOTS);

	if (idx < ert_user->num_slots) {
		set_bit(idx, ert_user->slot_status);
		return idx;
	}
	return no_index;
}

/**
 * command_queue_acquire() - Acquire a slot index for a command
 *
 * This function makes a special case for control commands which
 * must always dispatch to slot 0, otherwise normal acquisition
 */
static int 
command_queue_acquire(struct xrt_ert_command *ecmd, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;
	// slot 0 is reserved for ctrl commands
	if (ert_special_cmd(ecmd)) {
		set_bit(0, ert_user->slot_status);
#if 0
		###### move this part to common layer#####
		if (ert_user->ctrl_busy) {
			ERTUSER_DBG(ert_user, "ctrl slot is busy\n");
			return -1;
		}

		if (cmd_opcode(ecmd) != OP_GET_STAT)
			ert_user->ctrl_busy = true;
#endif
		return (ecmd->handle = 0);
	}

	return (ecmd->handle = command_queue_acquire_slot_idx(ert_user));
}

static void ert_intc_enable(struct command_queue *ert_user, bool enable){
	uint32_t i;
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);

	if (XOCL_DSA_IS_VERSAL(xdev)) {
		if (enable)
			xocl_mailbox_versal_request_intr(xdev, ert_versal_isr, ert_user);
		else
			xocl_mailbox_versal_free_intr(xdev);
		return;
	}

	for (i = 0; i < ert_user->num_slots; i++) {
		if (enable) {
			xocl_intc_ert_request(xdev, i, ert_user_isr, ert_user);
			xocl_intc_ert_config(xdev, i, true);
		} else {
			xocl_intc_ert_config(xdev, i, false);
			xocl_intc_ert_request(xdev, i, NULL, NULL);
		}
	}
}

static void
command_queue_poll(void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);
	struct xrt_ert_command* ecmd = NULL;
	u32 mask = 0;
	u32 slot_idx = 0, section_idx = 0;

	for (section_idx = 0; section_idx < 4; ++section_idx) {
		mask = xocl_intc_ert_read32(xdev, (section_idx<<2));
		if (!mask)
			continue;
		ERTUSER_DBG(ert_user, "mask 0x%x\n", mask);
		for ( slot_idx = 0; slot_idx < 32; mask>>=1, ++slot_idx ) {
			u32 cmd_idx = slot_idx+(section_idx<<5);

			if (!mask)
				break;
			if (mask & 0x1) {
				ecmd = ert_user->submit_queue[cmd_idx];
				if (ecmd) {
					//ert_get_return(ert_user, ecmd);
					ecmd->completed = true;
					ecmd->status = KDS_COMPLETED;
					ERTUSER_DBG(ert_user, "%s -> ecmd %llx xcmd%p\n", __func__, (u64)ecmd, xcmd);
					ert_user->submit_queue[cmd_idx] = NULL;
				} else
					ERTUSER_DBG(ert_user, "ERR: submit queue slot is empty\n");
			}
		}
	}
}


static void 
command_queue_complete(struct xrt_ert_command *ecmd, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;

	ert_get_return(ert_user, ecmd);
}

static void
command_queue_intc_config(bool enable, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;

	ert_user->polling_mode = !enable;

	ert_intc_enable(ert_user, enable);
}

static int
command_queue_config(uint32_t slot_size, void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;

	/*  1. cfg->slot_size need to be 32-bit aligned
	 *  2. the slot num max: 128
	 */

	ERTUSER_DBG(ert_user, "configuring scheduler cq_size(%lld)\n", ert_user->cq_range);
	if (!ert_user->cq_range || !slot_size) {
		ERTUSER_ERR(ert_user, "should not have zero cq_range %lld, slot_size=%d",
			ert_user->cq_range, slot_size);
		return -EINVAL;
	} else if (!IS_ALIGNED(slot_size, 4)) {
		ERTUSER_ERR(ert_user, "slot_size should be 4 bytes aligned, slot_size=%d",
		  slot_size);
		return -EINVAL;
	} else if (slot_size < (ert_user->cq_range/ERT_MAX_SLOTS)) {
		ERTUSER_ERR(ert_user, "slot_size too small=%d", slot_size);
		return -EINVAL;
	}

	ert_user->num_slots = ert_user->cq_range / slot_size;
	ert_user->slot_size = slot_size;

	ert_user_reset(ert_user);

	return 0;
}

static uint64_t
command_queue_size(void *core)
{
	struct command_queue *ert_user = (struct command_queue *)core;
	return ert_user->cq_range;
}

static struct xrt_ert_queue_funcs command_queue_func = {

	.acquire = command_queue_acquire,

	.release = command_queue_release,
	
	.poll    = command_queue_poll,
	
	.submit  = command_queue_submit,
	
	.complete = command_queue_complete,
	
	.intc_config = command_queue_intc_config,
	
	.queue_config = command_queue_config,
	
	.queue_size = command_queue_size,
};
