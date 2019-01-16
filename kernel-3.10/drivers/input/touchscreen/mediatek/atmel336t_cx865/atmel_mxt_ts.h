/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2018 XiaoMi, Inc.
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

#include "ts_key.h"

struct mxt_config_info {
	u8 self_chgtime_max;
	u8 self_chgtime_min;
};

enum {
	T15_T97_KEY = 0,
	T19_KEY,
	T24_KEY,
	T61_KEY,
	T81_KEY,
	T92_KEY,
	T93_KEY,
	T99_KEY,
	T115_KEY,
	T116_KEY,
	NUM_KEY_TYPE
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
/*
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
*/
	const u8 *num_keys;  //len is NUM_KEY_TYPE
	const unsigned int (*keymap)[MAX_KEYS_SUPPORTED_IN_DRIVER];

	unsigned long gpio_reset;
	const char *cfg_name;
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	unsigned int max_y_t;  	//Max value of Touch AA, asix more than this value will process by VirtualKeyHit
						//Note Resolution of TP AA and LCD AA must be same value. For e.g: LCD(1920 * 1080), TP max set as(1919 + virtual area,1079), max_y_t = 1919
	struct mxt_virtual_key_space vkey_space_ratio;
#endif

	const struct mxt_config_info *config_array;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
