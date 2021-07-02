/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Xilinx Unify CU Model
 *
 * Copyright (C) 2020 Xilinx, Inc. All rights reserved.
 *
 * Authors: min.ma@xilinx.com
 *
 * This file is dual-licensed; you may select either the GNU General Public
 * License version 2 or Apache License, Version 2.0.
 */

#ifndef _XRT_ERT_H
#define _XRT_ERT_H

#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include "xocl_drv.h"
#include "kds_command.h"
#include "kds_core.h"

struct xrt_ert_command {
	struct kds_command		*xcmd;
	struct list_head		list;
	uint32_t			handle;
	bool				completed;
	uint32_t			status;
};

struct xrt_ert_queue_funcs {

	int (*acquire)(struct xrt_ert_command *ecmd, void *core);

	void (*release)(struct xrt_ert_command *ecmd, void *core);

	void (*poll)(void *core);

	void (*submit)(struct xrt_ert_command *ecmd, void *core);

	void (*complete)(struct xrt_ert_command *ecmd, void *core);

	void (*intc_config)(bool enable, void *core);

	int  (*queue_config)(uint32_t slot_size, void *core);

	uint64_t (*queue_size)(void *core);

	uint32_t (*minimum_slot_size)(void *core);

};

enum ert_gpio_cfg {
	INTR_TO_ERT,
	INTR_TO_CU,
	MB_WAKEUP,
	MB_SLEEP,
	MB_STATUS,
};
#endif /* _XRT_ERT_H */
