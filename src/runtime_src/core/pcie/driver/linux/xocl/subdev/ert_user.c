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

#include "kds_client.h"
#include "xrt_ert.h"

//#define	SCHED_VERBOSE	1
#if 0
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
#endif

#ifdef SCHED_VERBOSE
#define	ERTUSER_ERR(ert, fmt, arg...)	\
	xocl_err(ert->dev, fmt "", ##arg)
#define	ERTUSER_WARN(ert, fmt, arg...)	\
	xocl_warn(ert->dev, fmt "", ##arg)	
#define	ERTUSER_INFO(ert, fmt, arg...)	\
	xocl_info(ert->dev, fmt "", ##arg)	
#define	ERTUSER_DBG(ert, fmt, arg...)	\
	xocl_info(ert->dev, fmt "", ##arg)

#else
#define	ERTUSER_ERR(ert, fmt, arg...)	\
	xocl_err(ert->dev, fmt "", ##arg)
#define	ERTUSER_WARN(ert, fmt, arg...)	\
	xocl_warn(ert->dev, fmt "", ##arg)	
#define	ERTUSER_INFO(ert, fmt, arg...)	\
	xocl_info(ert->dev, fmt "", ##arg)	
#define	ERTUSER_DBG(ert, fmt, arg...)

#define sched_debug_packet(packet, size)				\
({									\
	int i;								\
	u32 *data = (u32 *)packet;					\
	for (i = 0; i < size; ++i)					    \
		DRM_INFO("packet(0x%p) execbuf[%d] = 0x%x\n", data, i, data[i]); \
})
#endif
extern int kds_echo;


/* XRT ERT timer macros */
/* A low frequence timer for ERT to check if command timeout */
#define ERT_TICKS_PER_SEC	2
#define ERT_TIMER		(HZ / ERT_TICKS_PER_SEC) /* in jiffies */
#define ERT_EXEC_DEFAULT_TTL	(5UL * ERT_TICKS_PER_SEC)

struct xrt_ert_event {
	struct mutex		  lock;
	void			 *client;
	int			  state;
};

struct xrt_ert_queue {
	struct list_head	head;
	uint32_t 		num;
};

struct xrt_ert {
	struct device		*dev;
	struct platform_device	*pdev;
	bool			polling_mode;
	struct mutex 		lock;
	struct kds_ert		ert;

	/* Configure dynamically */ 
	unsigned int		num_slots;
	unsigned int		slot_size;
	uint64_t		cq_range;
	bool			is_configured;
	bool			ctrl_busy;
	struct xocl_ert_sched_privdata ert_cfg_priv;

	struct xrt_ert_queue	pq;
	struct xrt_ert_queue	pq_ctrl;

	spinlock_t		pq_lock;
	/*
	 * Pending Q is used in thread that is submitting CU cmds.
	 * Other Qs are used in thread that is completing them.
	 * In order to prevent false sharing, they need to be in different
	 * cache lines. Hence we add a "padding" in between (assuming 128-byte
	 * is big enough for most CPU architectures).
	 */
	u64			padding[16];
	/* run queue */
	struct xrt_ert_queue	rq;
	struct xrt_ert_queue	rq_ctrl;


	struct semaphore	sem;
	/* submitted queue */
	struct xrt_ert_queue	sq;

	struct xrt_ert_queue	cq;

	u32			stop;
	bool			bad_state;

	struct mutex		ev_lock;
	struct list_head	events;

	struct timer_list	timer;
	atomic_t		tick;

	struct task_struct	*thread;

	uint32_t 		ert_dmsg;
	uint32_t		echo;
	uint32_t		intr;
	void 			*queue_core;
	struct xrt_ert_queue_funcs    *func;
	struct ert_validate_cmd ert_valid;
};

static ssize_t clock_timestamp_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%u\n", ert_user->ert_valid.timestamp);
}

static DEVICE_ATTR_RO(clock_timestamp);

static ssize_t snap_shot_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "pq:%d pq_ctrl:%d,  rq:%d, rq_ctrl:%d, sq:%d cq:%d\n", ert_user->pq.num, ert_user->pq_ctrl.num, ert_user->rq.num
		,ert_user->rq_ctrl.num, ert_user->sq.num, ert_user->cq.num);
}

static DEVICE_ATTR_RO(snap_shot);

static ssize_t ert_dmsg_store(struct device *dev,
	struct device_attribute *da, const char *buf, size_t count)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));
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
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));
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
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));
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
#if 0
	if (go_sleep)
		ert_user_gpio_cfg(pdev, MB_SLEEP);
	else
		ert_user_gpio_cfg(pdev, MB_WAKEUP);
#endif
	return count;
}

static ssize_t mb_sleep_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);

///	return sprintf(buf, "%d", ert_user_gpio_cfg(pdev, MB_STATUS));

	return 0;
}

static DEVICE_ATTR_RW(mb_sleep);

static ssize_t cq_read_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cq_read_single);
}

static DEVICE_ATTR_RO(cq_read_cnt);

static ssize_t cq_write_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cq_write_single);
}

static DEVICE_ATTR_RO(cq_write_cnt);

static ssize_t cu_read_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

	return sprintf(buf, "%d\n", ert_user->ert_valid.cu_read_single);
}

static DEVICE_ATTR_RO(cu_read_cnt);

static ssize_t cu_write_cnt_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xrt_ert *ert_user = platform_get_drvdata(to_platform_device(dev));

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
static int ert_user_bulletin(struct platform_device *pdev, struct ert_cu_bulletin *brd)
{
	struct xrt_ert *ert_user = platform_get_drvdata(pdev);
	int ret = 0;

	if (!brd)
		return -EINVAL;

	brd->sta.configured = ert_user->is_configured;
	//brd->cap.cu_intr = ert_user->cfg_gpio ? 1 : 0;

	return ret;
}

static int ert_user_enable(struct platform_device *pdev, bool enable)
{
	int ret = 0;
#if 0
	if (enable) {
		ert_user_gpio_cfg(pdev, MB_WAKEUP);
		ert_user_gpio_cfg(pdev, INTR_TO_ERT);
	} else {
		ert_user_gpio_cfg(pdev, MB_SLEEP);
		ert_user_gpio_cfg(pdev, INTR_TO_CU);
	}
#endif
	return ret;
}

static struct xocl_ert_user_funcs ert_user_ops = {
	.bulletin = ert_user_bulletin,
	.enable = ert_user_enable,
};
static void ert_user_submit(struct kds_ert *ert, struct kds_command *xcmd);

static void ert_free_cmd(struct xrt_ert_command* ecmd)
{
	kfree(ecmd);
}

static struct xrt_ert_command* ert_alloc_cmd(struct kds_command *xcmd)
{
	struct xrt_ert_command* ecmd = kzalloc(sizeof(struct xrt_ert_command), GFP_KERNEL);

	if (!ecmd)
		return NULL;

	ecmd->xcmd = xcmd;

	return ecmd;
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void ert_timer(unsigned long data)
{
	struct xrt_ert *ert = (struct xrt_ert *)data;
#else
static void ert_timer(struct timer_list *t)
{
	struct xrt_ert *ert = from_timer(ert, t, timer);
#endif

	atomic_inc(&ert->tick);

	mod_timer(&ert->timer, jiffies + ERT_TIMER);
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

static inline struct kds_client *
first_event_client_or_null(struct xrt_ert *ert)
{
	struct kds_client *curr = NULL;

	if (list_empty(&ert->events))
		return NULL;

	mutex_lock(&ert->ev_lock);
	if (list_empty(&ert->events))
		goto done;

	curr = list_first_entry(&ert->events, struct kds_client, ev_entry);

done:
	mutex_unlock(&ert->ev_lock);
	return curr;
}

static int ert_cfg_cmd(struct xrt_ert *ert_user, struct xrt_ert_command *ecmd)
{
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);
	uint32_t *cdma = xocl_rom_cdma_addr(xdev);
	//unsigned int dsa = ert->ert_cfg_priv.dsa;
	//unsigned int major = ert->ert_cfg_priv.major;
	struct ert_configure_cmd *cfg = (struct ert_configure_cmd *)ecmd->xcmd->execbuf;
	bool ert_enabled = (XOCL_DSA_IS_VERSAL(xdev) || XOCL_DSA_IS_MPSOC(xdev)) ? 1 :
	    xocl_mb_sched_on(xdev);
	bool ert_full = !cfg->dataflow;
	bool ert_poll = cfg->dataflow;
	unsigned int ert_num_slots = 0, slot_size = 0;
	uint64_t cq_range = ert_user->func->queue_size(ert_user->queue_core);

	BUG_ON(!ert_enabled);

	if (cmd_opcode(ecmd) != OP_CONFIG)
		return -EINVAL;
#if 0
	if (major > 3) {
		ERTUSER_ERR(ert_user, "Unknown ERT major version\n");
		return -EINVAL;
	}
#endif
	//ERTUSER_DBG(ert_user, "dsa52 = %d", dsa);

	if (XOCL_DSA_IS_VERSAL(xdev) || XOCL_DSA_IS_MPSOC(xdev)) {
		ERTUSER_INFO(ert_user, "MPSoC polling mode %d", cfg->polling);

		// For MPSoC device, we will use ert_full if we are
		// configured as ert mode even dataflow is configured.
		// And we do not support ert_poll.
		ert_full = cfg->ert;
		ert_poll = false;
	}

	// Mark command as control command to force slot 0 execution
	/*  1. cfg->slot_size need to be 32-bit aligned
	 *  2. the slot num max: 128
	 */
	//ERTUSER_DBG(ert_user, "configuring scheduler cq_size(%lld)\n", cq_range);
	if (cq_range == 0 || cfg->slot_size == 0) {
		ERTUSER_ERR(ert_user, "should not have zeroed value of cq_size=%lld, slot_size=%d",
		    cq_range, cfg->slot_size);
		return -EINVAL;
	} else if (!IS_ALIGNED(cfg->slot_size, 4)) {
		ERTUSER_ERR(ert_user, "slot_size should be 4 bytes aligned, slot_size=%d",
		   cfg->slot_size);
		return -EINVAL;
	}

	ert_user->cq_range = cq_range;

	slot_size = cfg->slot_size;
#if 0

	##### func->max_slot?#############
	if (slot_size < (cq_range/ERT_MAX_SLOTS))
		slot_size = cq_range/ERT_MAX_SLOTS;
#endif
	ert_num_slots = cq_range / slot_size;

	if (ert_full && cfg->cu_dma && ert_num_slots > 32) {
		// Max slot size is 32 because of cudma bug
		ERTUSER_INFO(ert_user, "Limitting CQ size to 32 due to ERT CUDMA bug\n");
		slot_size = cq_range / 32;
	}

	cfg->slot_size = slot_size;

	if (ert_poll) {
		ERTUSER_INFO(ert_user, "configuring dataflow mode with ert polling\n");
		cfg->cu_isr = 0;
		cfg->cu_dma = 0;
	} else if (ert_full) {
		ERTUSER_INFO(ert_user, "configuring embedded scheduler mode\n");
		#if 0
		cfg->dsa52 = dsa;
		#endif
		cfg->cdma = cdma ? 1 : 0;
	}

	if (XDEV(xdev)->priv.flags & XOCL_DSAFLAG_CUDMA_OFF)
		cfg->cu_dma = 0;

	cfg->dmsg = ert_user->ert_dmsg;
	cfg->echo = ert_user->echo;
	cfg->intr = ert_user->intr;

	ert_user->polling_mode = cfg->polling;

	/* if polling, disable intc, vice versa */ 
	ert_user->func->intc_config(!ert_user->polling_mode, ert_user->queue_core);


	// The KDS side of of the scheduler is now configured.  If ERT is
	// enabled, then the configure command will be started asynchronously
	// on ERT.  The shceduler is not marked configured until ERT has
	// completed (exec_finish_cmd); this prevents other processes from
	// submitting commands to same xclbin.  However we must also stop
	// other processes from submitting configure command on this same
	// xclbin while ERT asynchronous configure is running.
	//exec->configure_active = true;

	ERTUSER_INFO(ert_user, "scheduler config ert(%d), dataflow(%d), cudma(%d), cuisr(%d)\n"
 		 , cfg->ert
		 , cfg->dataflow
		 , cfg->cu_dma ? 1 : 0
		 , cfg->cu_isr ? 1 : 0);

	return 0;
}

static void
ert_cfg_host(struct xrt_ert *ert_user, struct xrt_ert_command *ecmd)
{
	xdev_handle_t xdev = xocl_get_xdev(ert_user->pdev);
	struct ert_configure_cmd *cfg = (struct ert_configure_cmd *)ecmd->xcmd->execbuf;
	bool ert = (XOCL_DSA_IS_VERSAL(xdev) || XOCL_DSA_IS_MPSOC(xdev)) ? 1 :
	    xocl_mb_sched_on(xdev);

	BUG_ON(cmd_opcode(ecmd) != OP_CONFIG);
	BUG_ON(!ert);

	if (ecmd->status != KDS_COMPLETED)
		return;

	if (ert_user->func->queue_config(cfg->slot_size, ert_user->queue_core))
		return;

	ert_user->slot_size = cfg->slot_size;

	ert_user->num_slots = ert_user->cq_range / cfg->slot_size;

	ERTUSER_INFO(ert_user, "scheduler config ert completed, polling_mode(%d), slots(%d)\n"
		 , ert_user->polling_mode
		 , ert_user->num_slots);

	ert_user->is_configured = true;

	return;
}

static inline void
ert_post_process(struct xrt_ert *ert_user, struct xrt_ert_command *ecmd)
{
	if (likely(!ert_special_cmd(ecmd)))
		return;

	switch (cmd_opcode(ecmd)) {
	case OP_CONFIG:
		ert_cfg_host(ert_user, ecmd);
		break;
	default:
		break;
	}

	return;
}


static inline bool
ert_pre_process(struct xrt_ert *ert, struct xrt_ert_command *ecmd)
{
	bool bad_cmd = false;

	switch (cmd_opcode(ecmd)) {
	case OP_START:
	case OP_START_SK:
		BUG_ON(ert->ctrl_busy);
#if KERNEL_VERSION(5, 4, 0) > LINUX_VERSION_CODE
		__attribute__ ((fallthrough));
#else
		__attribute__ ((__fallthrough__));
#endif
	case OP_CLK_CALIB:
	case OP_CONFIG_SK:
	case OP_GET_STAT:
	case OP_VALIDATE:
		BUG_ON(!ert->is_configured);
		bad_cmd = false;
		break;
	case OP_CONFIG:
		if (ert_cfg_cmd(ert, ecmd))
			bad_cmd = true;
		break;
	default:
		bad_cmd = true;
	}

	return bad_cmd;
}
/**
 * process_ert_cq() - Process cmd witch is completed
 * @ert_user: Target XRT CU
 */
static inline void process_ert_cq(struct xrt_ert *ert_user)
{
	struct kds_command *xcmd;
	struct xrt_ert_command *ecmd;

	if (!ert_user->cq.num)
		return;

	ERTUSER_DBG(ert_user, "-> %s\n", __func__);

	while (ert_user->cq.num) {
		ecmd = list_first_entry(&ert_user->cq.head, struct xrt_ert_command, list);
		list_del(&ecmd->list);
		xcmd = ecmd->xcmd;
		ert_post_process(ert_user, ecmd);
		ert_user->func->complete(ecmd, ert_user->queue_core);
		ert_user->func->release(ecmd, ert_user->queue_core);
		set_xcmd_timestamp(xcmd, ecmd->status);
		xcmd->cb.notify_host(xcmd, ecmd->status);
		xcmd->cb.free(xcmd);
		ert_free_cmd(ecmd);
		--ert_user->cq.num;
	}

	ERTUSER_DBG(ert_user, "<- %s\n", __func__);
}


/**
 * process_ert_sq() - Process cmd witch is submitted
 * @ert: Target XRT ERT
 */
static inline void process_ert_sq(struct xrt_ert *ert)
{
	struct kds_command *xcmd;
	struct xrt_ert_command *ecmd, *next;
	struct kds_client *ev_client = NULL;
	unsigned int tick;

	if (!ert->sq.num)
		return;

	if (ert->polling_mode)
		ert->func->poll(ert->queue_core);

	ev_client = first_event_client_or_null(ert);

	list_for_each_entry_safe(ecmd, next, &ert->sq.head, list) {
		xcmd = ecmd->xcmd;
		if (ecmd->completed) {
			//ert_get_return(ert, ecmd);
			ecmd->status = KDS_COMPLETED;
		} else if (unlikely(ev_client)) {
			/* Client event happens rarely */
			if (xcmd->client != ev_client)
				continue;

			tick = atomic_read(&ert->tick);
			/* Record command tick to start timeout counting */
			if (!xcmd->tick) {
				xcmd->tick = tick;
				continue;
			}

			/* If xcmd haven't timeout */
			if (tick - xcmd->tick < ERT_EXEC_DEFAULT_TTL)
				continue;

			ecmd->status = KDS_TIMEOUT;
			/* Mark ERT as bad state */
			ert->bad_state = true;
		} else
			continue;

		ERTUSER_DBG(ert, "%s -> ecmd %llx xcmd%p\n", __func__, (u64)ecmd, xcmd);
		list_move_tail(&ecmd->list, &ert->cq.head);
		--ert->sq.num;
		++ert->cq.num;
	}
	ERTUSER_DBG(ert, "<- %s\n", __func__);
}
#if 0
static irqreturn_t
ert_isr(int irq, void *arg)
{
	struct xrt_ert *ert = (struct xocl_ert *)arg;
	//xdev_handle_t xdev;
	struct xrt_ert_command *ecmd;

	BUG_ON(!ert);

	////ERTUSER_DBG(ert, "-> xocl_user_event %d\n", irq);
	//xdev = xocl_get_xdev(ert->pdev);

	BUG_ON(irq>=ERT_MAX_SLOTS);

	if (!ert->polling_mode) {

		//ecmd = ert->submit_queue[irq];
		if (ecmd) {
			ecmd->completed = true;
		} else {
			ERTUSER_ERR(ert, "not in submitted queue %d\n", irq);
		}

		up(&ert->sem);
		/* wake up all scheduler ... currently one only */
#if 0
		if (xs->stop)
			return;

		if (xs->reset) {
			SCHED_DEBUG("scheduler is resetting after timeout\n");
			scheduler_reset(xs);
		}
#endif
	} else {
		//ERTUSER_DBG(ert, "unhandled isr irq %d", irq);
		return IRQ_NONE;
	}
	//ERTUSER_DBG(ert, "<- xocl_user_event %d\n", irq);
	return IRQ_HANDLED;
}
#endif

/**
 * process_ert_rq() - Process run queue
 * @ert: Target XRT ERT
 * @rq: Target running queue
 *
 * Return: return 0 if run queue is empty or no available slot
 *	   Otherwise, return 1
 */
static inline int process_ert_rq(struct xrt_ert *ert, struct xrt_ert_queue *rq)
{
	struct xrt_ert_command *ecmd, *next;
	//u32 slot_addr = 0;
	struct ert_packet *epkt = NULL;
	//xdev_handle_t xdev = xocl_get_xdev(ert->pdev);
	struct kds_client *ev_client = NULL;

	if (!rq->num)
		return 0;

	ev_client = first_event_client_or_null(ert);
	list_for_each_entry_safe(ecmd, next, &rq->head, list) {
		struct kds_command *xcmd = ecmd->xcmd;

		if (unlikely(ert->bad_state || (ev_client == xcmd->client))) {
			ERTUSER_ERR(ert, "%s abort\n", __func__);
			ecmd->status = KDS_ERROR;
			list_move_tail(&ecmd->list, &ert->cq.head);
			--rq->num;
			++ert->cq.num;
			continue;
		}

		if (ert_pre_process(ert, ecmd)) {
			ERTUSER_ERR(ert, "%s bad cmd, opcode: %d\n", __func__, cmd_opcode(ecmd));
			ecmd->status = KDS_ABORT;
			list_move_tail(&ecmd->list, &ert->cq.head);
			--rq->num;
			++ert->cq.num;
			continue;
		}

		if (ert->func->acquire(ecmd, ert->queue_core)) {
			//ERTUSER_DBG(ert, "%s not slot available\n", __func__);
			return 0;
		}
		epkt = (struct ert_packet *)ecmd->xcmd->execbuf;
		//ERTUSER_DBG(ert, "%s op_code %d ecmd->slot_idx %d\n", __func__, cmd_opcode(ecmd), ecmd->slot_idx);

		//sched_debug_packet(epkt, epkt->count+sizeof(epkt->header)/sizeof(u32));

		/* Hardware could be pretty fast, add to sq before touch the CQ_status or cmd queue*/
		list_move_tail(&ecmd->list, &ert->sq.head);
		--rq->num;
		++ert->sq.num;

		//ERTUSER_DBG(ert, "%s slot_addr %x\n", __func__, slot_addr);
		ert->func->submit(ecmd, ert->queue_core);
	}

	return 1;
}

/**
 * process_ert_pq() - Process pending queue
 * @ert: Target XRT ERT
 * @pq: Target pending queue
 * @rq: Target running queue
 *
 * Move all of the pending queue commands to the tail of run queue
 * and re-initialized pending queue
 */
static inline void process_ert_pq(struct xrt_ert *ert, struct xrt_ert_queue *pq, struct xrt_ert_queue *rq)
{
	unsigned long flags;

	/* Get pending queue command number without lock.
	 * The idea is to reduce the possibility of conflict on lock.
	 * Need to check pending command number again after lock.
	 */
	if (!pq->num)
		return;

	spin_lock_irqsave(&ert->pq_lock, flags);
	if (pq->num) {
		list_splice_tail_init(&pq->head, &rq->head);
		rq->num += pq->num;
		pq->num = 0;
	}
	spin_unlock_irqrestore(&ert->pq_lock, flags);
}

static void ert_user_submit(struct kds_ert *kds_ert, struct kds_command *xcmd)
{
	unsigned long flags;
	bool first_command = false;
	struct xrt_ert *ert = container_of(kds_ert, struct xrt_ert, ert);
	struct xrt_ert_command *ecmd = ert_alloc_cmd(xcmd);

	if (!ecmd)
		return;

	//ERTUSER_DBG(ert, "->%s ecmd %llx\n", __func__, (u64)ecmd);
	spin_lock_irqsave(&ert->pq_lock, flags);
	switch (cmd_opcode(ecmd)) {
	case OP_START:
	case OP_START_SK:
		list_add_tail(&ecmd->list, &ert->pq.head);
		++ert->pq.num;
		break;
	case OP_CLK_CALIB:
	case OP_CONFIG_SK:
	case OP_GET_STAT:
	case OP_VALIDATE:
	case OP_CONFIG:
	default:
		list_add_tail(&ecmd->list, &ert->pq_ctrl.head);
		++ert->pq_ctrl.num;
		break;
	}
	first_command = ((ert->pq.num + ert->pq_ctrl.num) == 1);
	spin_unlock_irqrestore(&ert->pq_lock, flags);
	/* Add command to pending queue
	 * wakeup service thread if it is the first command
	 */
	if (first_command)
		up(&ert->sem);

	//ERTUSER_DBG(ert, "<-%s\n", __func__);
	return;
}


static inline bool ert_thread_sleep_condition(struct xrt_ert *ert)
{
	bool ret = false;
	bool polling_sleep = false, intr_sleep = false, no_event = false
	, no_completed_cmd = false, no_submmited_cmd = false
	, cant_submit = false, cant_submit_start = false, cant_submit_ctrl = false
	, no_need_to_fetch_new_cmd = false, no_need_to_fetch_start_cmd = false, no_need_to_fetch_ctrl_cmd = false;


	/* When ert_thread should go to sleep to save CPU usage
	 * 1. There is no event to be processed
	 * 2. We don't have to process command when
	 *    a. We can't submit cmd if we don't have cmd in running queue or submitted queue is full
	 *    b. There is no cmd in pending queue or we still have cmds in running queue
	 *    c. There is no cmd in completed queue
	 * 3. We are not in polling mode and there is no cmd in submitted queue
	 */  

	no_completed_cmd = !ert->cq.num;

	cant_submit_start = (!ert->rq.num) || (ert->sq.num == (ert->num_slots-1));
	cant_submit_ctrl = (!ert->rq_ctrl.num) || (ert->sq.num == 1);
	cant_submit = cant_submit_start && cant_submit_ctrl;

	no_need_to_fetch_start_cmd = ert->rq.num !=0 || !ert->pq.num;
	no_need_to_fetch_ctrl_cmd = ert->rq_ctrl.num !=0 || !ert->pq_ctrl.num;
	no_need_to_fetch_new_cmd = no_need_to_fetch_ctrl_cmd && no_need_to_fetch_start_cmd;

	no_submmited_cmd = !ert->sq.num;

	polling_sleep = no_completed_cmd && no_need_to_fetch_new_cmd && no_submmited_cmd;
	intr_sleep = no_completed_cmd && no_need_to_fetch_new_cmd && cant_submit;

	no_event = first_event_client_or_null(ert) == NULL;


	ret = no_event && ((ert->polling_mode && polling_sleep) || (!ert->polling_mode && intr_sleep));

	return ret;
}

int xrt_ert_thread(void *data)
{
	struct xrt_ert *ert = (struct xrt_ert *)data;
 	int ret = 0;

	mod_timer(&ert->timer, jiffies + ERT_TIMER);

	while (!ert->stop) {
		/* Make sure to submit as many commands as possible.
		 * This is why we call continue here. This is important to make
		 * CU busy, especially CU has hardware queue.
		 */
		if (process_ert_rq(ert, &ert->rq_ctrl))
			continue;

		if (process_ert_rq(ert, &ert->rq))
			continue;
		/* process completed queue before submitted queue, for
		 * two reasons:
		 * - The last submitted command may be still running
		 * - while handling completed queue, running command might done
		 * - process_ert_sq_polling will check CU status, which is thru slow bus
		 */

		process_ert_sq(ert);

		process_ert_cq(ert);

		/* If any event occured, we should drain all the related commands ASAP
		 * It only goes to sleep if there is no event
		 */
		if (ert_thread_sleep_condition(ert)) {
			if (down_interruptible(&ert->sem))
				ret = -ERESTARTSYS;
		}

		process_ert_pq(ert, &ert->pq, &ert->rq);
		process_ert_pq(ert, &ert->pq_ctrl, &ert->rq_ctrl);
	}
	del_timer_sync(&ert->timer);

	if (!ert->bad_state)
		ret = -EBUSY;

	return ret;
}

/**
 * xocl_ert_abort() - Sent an abort event to ERT thread
 * @ert: Target XRT ERT
 * @client: The client tries to abort commands
 *
 * This is used to ask ERT thread to abort all commands from the client.
 */
static void xocl_ert_abort(struct kds_ert *kds_ert, struct kds_client *client, int cu_idx)
{
	struct kds_client *curr;
	struct xrt_ert *ert = container_of(kds_ert, struct xrt_ert, ert);

	mutex_lock(&ert->ev_lock);
	if (list_empty(&ert->events))
		goto add_event;

	/* avoid re-add the same client */
	list_for_each_entry(curr, &ert->events, ev_entry) {
		if (client == curr)
			goto done;
	}

add_event:
	client->ev_type = EV_ABORT;
	list_add_tail(&client->ev_entry, &ert->events);
	/* The process thread may asleep, we should wake it up if 
	 * abort event takes place
	 */
	up(&ert->sem);
done:
	mutex_unlock(&ert->ev_lock);
}


void xrt_ert_queue_func_register(struct xrt_ert* ert, struct xrt_ert_queue_funcs *func)
{
	ert->func = func;
}

/**
 * xocl_ert_abort() - Get done flag of abort
 * @ert: Target XRT ERT
 *
 * Use this to wait for abort event done
 */
static bool xocl_ert_abort_done(struct kds_ert *kds_ert, struct kds_client *client, int cu_idx)
{
	struct kds_client *curr;
	struct kds_client *next;
	struct xrt_ert *ert = container_of(kds_ert, struct xrt_ert, ert);

	mutex_lock(&ert->ev_lock);
	if (list_empty(&ert->events))
		goto done;

	list_for_each_entry_safe(curr, next, &ert->events, ev_entry) {
		if (client != curr)
			continue;

		list_del(&curr->ev_entry);
		break;
	}

done:
	mutex_unlock(&ert->ev_lock);

	return ert->bad_state;
}

static int ert_user_remove(struct platform_device *pdev)
{
	struct xrt_ert *ert_user;
	void *hdl;

	ert_user = platform_get_drvdata(pdev);
	if (!ert_user) {
		xocl_err(&pdev->dev, "driver data is NULL");
		return -EINVAL;
	}

	sysfs_remove_group(&pdev->dev.kobj, &ert_user_attr_group);

	xocl_drvinst_release(ert_user, &hdl);

	//ert_intc_enable(ert_user, false);

	ert_user->stop = 1;
	up(&ert_user->sem);
	(void) kthread_stop(ert_user->thread);

	platform_set_drvdata(pdev, NULL);

	xocl_drvinst_free(hdl);

	return 0;
}

static int ert_user_probe(struct platform_device *pdev)
{
	struct xrt_ert *ert_user;
	struct resource *res;
	int err = 0;
	xdev_handle_t xdev = xocl_get_xdev(pdev);
	struct xocl_ert_sched_privdata *priv = NULL;
	bool ert = xocl_ert_on(xdev);

	/* If XOCL_DSAFLAG_MB_SCHE_OFF is set, we should not probe ert */
	if (!ert) {
		xocl_warn(&pdev->dev, "Disable ERT flag overwrite, don't probe ert_user");
		return -ENODEV;
	}

	ert_user = xocl_drvinst_alloc(&pdev->dev, sizeof(struct xrt_ert));
	if (!ert_user)
		return -ENOMEM;

	ert_user->dev = &pdev->dev;
	ert_user->pdev = pdev;
	/* Initialize pending queue and lock */
	INIT_LIST_HEAD(&ert_user->pq.head);
	INIT_LIST_HEAD(&ert_user->pq_ctrl.head);
	spin_lock_init(&ert_user->pq_lock);
	/* Initialize run queue */
	INIT_LIST_HEAD(&ert_user->rq.head);
	INIT_LIST_HEAD(&ert_user->rq_ctrl.head);

	/* Initialize completed queue */
	INIT_LIST_HEAD(&ert_user->cq.head);
	INIT_LIST_HEAD(&ert_user->sq.head);

	mutex_init(&ert_user->ev_lock);
	INIT_LIST_HEAD(&ert_user->events);

	sema_init(&ert_user->sem, 0);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	setup_timer(&ert_user->timer, ert_timer, (unsigned long)ert_user);
#else
	timer_setup(&ert_user->timer, ert_timer, 0);
#endif
	atomic_set(&ert_user->tick, 0);

	ert_user->thread = kthread_run(ert_user_thread, ert_user, "ert_thread");

	platform_set_drvdata(pdev, ert_user);
	mutex_init(&ert_user->lock);

	if (XOCL_GET_SUBDEV_PRIV(&pdev->dev)) {
		priv = XOCL_GET_SUBDEV_PRIV(&pdev->dev);
		memcpy(&ert_user->ert_cfg_priv, priv, sizeof(*priv));
	} else {
		xocl_err(&pdev->dev, "did not get private data");
	}


	err = sysfs_create_group(&pdev->dev.kobj, &ert_user_attr_group);
	if (err) {
		xocl_err(&pdev->dev, "create ert_user sysfs attrs failed: %d", err);
		goto done;
	}
	ert_user->ert.submit = ert_user_submit;
	ert_user->ert.abort = xocl_ert_abort;
	ert_user->ert.abort_done = xocl_ert_abort_done;
	xocl_kds_init_ert(xdev, &ert_user->ert);

	/* Enable interrupt by default */
	ert_user->num_slots = 128;
	ert_user->polling_mode = false;
	//ert_intc_enable(ert_user, true);
done:
	if (err) {
		ert_user_remove(pdev);
		return err;
	}
	return 0;
}

struct xocl_drv_private ert_user_priv = {
	.ops = &ert_user_ops,
	.dev = -1,
};

struct platform_device_id ert_user_id_table[] = {
	{ XOCL_DEVNAME(XOCL_ERT_USER), (kernel_ulong_t)&ert_user_priv },
	{ },
};

static struct platform_driver	ert_user_driver = {
	.probe		= ert_user_probe,
	.remove		= ert_user_remove,
	.driver		= {
		.name = XOCL_DEVNAME(XOCL_ERT_USER),
	},
	.id_table = ert_user_id_table,
};

int __init xocl_init_ert_user(void)
{
	return platform_driver_register(&ert_user_driver);
}

void xocl_fini_ert_user(void)
{
	platform_driver_unregister(&ert_user_driver);
}