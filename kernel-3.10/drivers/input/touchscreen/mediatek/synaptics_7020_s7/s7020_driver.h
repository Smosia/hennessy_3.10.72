/*
 * Copyright (C) 2013 LG Electironics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



/****************************************************************************
* Debugging Macros
****************************************************************************/
#define TPD_TAG                  "[Synaptics S7020] "
#define TPD_FUN(f)               pr_err("[%s %d]\n", __func__, __LINE__)
#define TPD_ERR(fmt, args...)    pr_err("[%s %d] : "fmt, __func__, __LINE__, ##args)
#define TPD_LOG(fmt, args...)    pr_err(fmt, ##args)
