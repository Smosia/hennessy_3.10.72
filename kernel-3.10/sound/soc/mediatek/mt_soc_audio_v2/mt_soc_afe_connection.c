/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *  mt_sco_afe_connection.c
 *
 * Project:
 * --------
 *   MT6583  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_digital_type.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/xlog.h>
#include <mach/irqs.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/mt_reg_base.h>
#include <asm/div64.h>
#include <linux/aee.h>
#include <mach/pmic_mt6325_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
//#include <asm/mach-types.h>

#ifdef DEBUG_AUDDRV
#define PRINTK_AUDDRV(format, args...) printk(format, ##args )
#else
#define PRINTK_AUDDRV(format, args...)
#endif

// mutex lock
static DEFINE_MUTEX(afe_connection_mutex);

/**
* here define conenction table for input and output
*/
static const char mConnectionTable[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
  //  0   1   2     3    4    5    6    7    8     9    10   11   12   13   14   15   16   17   18  19   20   21   22   23   24   25
    {  3,  3,    3,   3,   3,   3, -1,   1,    1,    1,    1,  -1,  -1,    1,    1,    3,    3 ,    1,    1 ,  3 , 25 ,  1  , -1,  -1,  -1,  -1 }, // I00
    {  3,  3,    3,   3,   3, -1,   3, -1, -1,  -1,  -1,  -1,  -1,   1,    1,    1,    3,  -1,   -1 ,  3 ,   3 , -1 ,   1,  -1,  -1,  -1 }, // I01
    {  1,  1,   1,    1,   1,   1, -1,   1,   1, -1,  -1,    1,  -1,    1,    1,    1,    1,    1,     1 , -1,  -1, -1 , -1,  -1,  -1,  -1}, // I02
    {  1,  1,    3,   1,   1,   1, 1,  1,  -1,   1,  -1,   -1,  -1,  -1,  -1,    1,    1,    1,   -1 ,   1 ,   1 ,   1 ,  -1,    1,  -1 ,   1}, // I03
    {  1,  1,    3,   1,   1, -1,   1, -1,   1, -1,    1,  -1,  -1,  -1,  -1,    1,    1,  -1,     1 ,   1 ,   1 ,  -1,   1,  -1,    1 ,   -1}, // I04
    {  3,  3,    3,   3,   3,   1, -1,  0,  -1,   1, -1,   -1,  -1,    1,    1,  -1,  -1,  -1,   -1,    3,    3 ,   1 , -1,  -1,  -1,    -1}, // I05
    {  3,  3,    3,   3,   3, -1,   1, -1,   0, -1,    1,  -1,    0,    1,    1,  -1,  -1,  -1,   -1,    3,    3 , -1,    1 , -1,  -1,   -1}, // I06
    {  3,  3,    3,   3,   3,   1, -1,  0,  -1,   0, -1,   -1, -1,     1,    1,  -1,  -1,  -1,   -1,    3,    3,    1,  -1,  -1,  -1,    -1}, // I07
    {  3,  3,    3,   3,   3, -1,   1, -1,   0, -1,    0,  -1,   0,     1,    1,  -1,  -1,  -1,   -1,    3,    3,  -1,    1,  -1,  -1,    -1}, // I08
    {  1,  1,    1,   1,   1,   1, -1, -1, -1, -1, -1,  -1,   1,   -1,  -1,    1,    1,  -1,   -1,    1,    1 , -1,  -1,    1,    1,    1}, // I09
    {  3, -1,   3,   3, -1,  1, -1,  1,  -1, -1, -1,  -1,  -1,   -1,  -1, -1,  -1,    1,   -1,    1,  -1,    1,    1 ,   1,    -1,    1}, // I10
    { -1, 3,    3, -1,  3, -1,   1, -1,   1, -1, -1,  -1,  -1,  -1,   -1, -1,  -1,  -1,     1,   -1,    1,    1,    1,  -1,     1,    -1}, // I11
    {  3, -1,   3,   3, -1,  0, -1,  1,  -1, -1, -1,  -1,  -1,  -1,  -1,  -1,  -1,    1,   -1,    1,  -1,    1,    1,  -1,   -1,    -1 }, // I12
    { -1, 3,    3, -1,  3, -1,   0, -1,   1, -1, -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,     1,  -1,    1,    1,    1,  -1,   -1,    -1}, // I13
    {  1,  1,    1,   1,   1,   1, -1, -1, -1, -1, -1,  -1,    1,  -1,   -1,   1,     1,  -1,   -1,    1,    1,  -1,  -1, -1,  -1,   -1}, // I14
    {  3,  3,    3,   3,   3,   3, -1, -1, -1,  1,  -1,  -1,  -1,   1,     1,    3,     1,  -1,   -1,  -1,  -1,   1,  -1,  -1,  -1,  -1}, // I15
    {  3,  3,    3,   3,   3, -1,   3, -1, -1, -1,   1,  -1,  -1,    1,    1,    1,    3,   -1,   -1,  -1,  -1, -1,    1,  -1,  -1,  -1}, // I16
    {  1, -1,   3,   1, -1,   1, -1,  1, -1,   1,  -1,    1,  -1,    1,    1,    1,    1,     1,   -1,    1,    1,    1,  -1,    1,  -1,    1}, // I17
    { -1,   1,  3, -1,   1, -1,   1, -1,  1, -1,    1,  -1,    1,    1,    1,    1,    1,   -1,     1,    1,    1,  -1,    1,  -1,    1,  -1}, // I18
    {  3, -1,   3,   3,   3,   1, -1,   1, -1,  1,  -1,    1,  -1,    1,    1,    1,    1,     1,    -1,    1,    1,    1,  -1,    1,  -1,    1}, // I19
    { -1,   3,  3,   3,   3, -1,   1, -1,   1, -1,    1,  -1,    1,    1,    1,    1,    1,   -1,      1,    1,    1,  -1,    1,  -1,    1,  -1}, // I20
    {  1,   1,   1,   1,   1,   1, -1,   1,   1,   1,  -1,  -1,    1,    1,    1,    1,    1,   -1,   -1,    1,    1,  -1,  -1,  -1,    1,  -1}, // I21
};


//remind for AFE_CONN4 bit31
/**
* connection bits of certain bits
*/
static const char mConnectionbits[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
    // 0   1    2    3    4    5      6    7    8   9    10   11   12  13  14   15  16  17  18  19    20  21   22   23  24  25
    {  0,  16,   0, 16,   0, 16,  -1,   2,   5,   8,  12, -1, -1,    2, 15,  16,  22, 14,  18,   8 , 24,  30,  -1, -1, -1,  -1 }, // I00
    {  1,  17,   1, 17,   1, -1,  22,   3,   6,   9,  13, -1, -1,    3, 16,  17,  23, 15,  19,  10 , 26,  -1,    2, -1, -1, -1  }, // I01
    {  2 , 18,   2, 18, -1, 17,  -1, 21, 22, -1, -1,   6, -1,    4, 17,  -1, 24,  23,  24,  -1, -1,  -1 , -1, -1, -1, -1 }, // I02
    {  3,  19,   3, 19,   3, 18,  -1, 26, -1,  0,  -1,  -1, -1,   5, 18,  19,  25,  13, -1 ,  12,  28,  31 , -1,   6, -1,   24},// I03
    {  4,  20,   4, 20,   4, -1,  23, -1, 29, -1,   3,  -1, -1,   6, 19,  20,  26, -1,  16,   13,  29,  -1,   3, -1,   9, -1 }, // I04
    {  5,  21,   5, 21,   5, 19,  -1, 27, -1,  1,  -1,    7, -1,  16, 20,  17,  26,  14, -1,  14,  31,   8, -1, -1, -1, -1 }, // I05
    {  6,  22,   6, 22,   6, -1,  24, -1, 30, -1,   4,  -1,  9,   17, 21,  18,  27, -1,   17, 16,   0,  -1,  11, -1, -1, -1 }, // I06
    {  7,  23,   7, 23,   7, 20,  -1, 28, -1,  2,  -1,    8, -1,  18, 22,  19,  28, 15,   -1, 18,   2,    9, -1, -1, -1, -1 }, // I07
    {  8,  24,   8, 24,   8, -1,  25, -1, 31, -1,   5,  -1, 10,  19, 23,  20,  29, -1,  18,  20,   4, -1 ,  12, -1, -1, -1 }, // I08
    {  9,  25,   9, 25,   9, 21,  27, -1, -1, 10, -1, -1, 11,    7,  20,  21,  27, 16,  20,  22, 26,  -1 ,  -1,  28, 29, 25 }, // I09
    {  0,  -1,   4,  8, -1, 12,  -1, 14, -1, 26,  28, 30,   0,  -1, -1,  -1, -1, 24,  -1,   28, -1,   0,   -1,   2, -1,   6 }, // I10
    { -1,   2,   6, -1, 10, -1 ,13, -1, 15, 27,  29, 31,   1,  -1, -1,  -1, -1, -1,  25, -1 ,  29,   31,    1, -1,   3, -1 }, // I11
    {  0,  -1,   4,  8, -1, 12,  -1, 14, -1,   8,  10, 12, 14,  -1,  -1,  -1,  -1,   6,  -1,    2, -1,   -1,   6, -1, -1, -1 }, // I12
    { -1,   2,   6, -1, 10, -1 ,13, -1, 15,   9,  11, 13, 15,  -1,  -1,  -1,  -1, -1,   7,  -1 ,   3,    4,   7, -1, -1, -1 }, // I13
    { 12,  17, 22, 27,  0,  5,  -1,   4,   7,  11, -1, -1,  12,    8,  21,  28,    1, -1,  -1,  -1 , 27, -1 ,-1, -1, -1, -1 }, // I14
    { 13,  18, 23, 28,  1,  6,  -1, -1, -1,  10, -1, -1, -1,    9,  22,  29,    2, -1,  -1,  23 , -1,  10 ,-1, -1, -1, -1 }, // I15
    { 14,  19, 24, 29,  2, -1,   8, -1, -1, -1,  11, -1, -1,   10,  23,  30,    3, -1,  -1,  -1 , -1,-1, 13, -1, -1, -1 }, // I16
    {   0,  19, 27,   0,  2, 22,   8, 26, -1, 30,  11,   2, -1,   11,  24,  21,  30, 17,  -1,  22 ,   6,   0, -1,   7, -1, -1 }, // I17
    { 14,   3, 28, 29,  1, -1, 24, -1, 28, -1,    0, -1,    4,   12,  25,  22,  31, -1,  21,  23,   7, -1,    4, -1,  10, -1 }, // I18
    {   1,  19, 10, 14, 18, 23,   8, 27, -1, 31, -1,  3,  -1,   13,  26, 23,    0,   6,  -1,  24,  28,    1, -1,   8, -1, -1 }, // I19
    { 14,   4, 12, 16, 20, -1, 25, -1, 29, -1,    1, -1,    5,  14,  27,  24,    1, -1,  7,  25,  29,  -1,   5, -1,  11, -1 }, // I20
    { 12,  13, 14, 15, 16,17,   8, 18,  19, 20, -1, -1,  21,    4,    5,   6,    7, -1,  -1,  22 ,  23,  -1, -1, -1, -1, -1 }, // I21
};

/**
* connection shift bits of certain bits
*/
static const char mShiftConnectionbits[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
    // 0    1    2     3    4     5     6    7    8    9   10  11  12  13  14  15  16  17  18  19  20   21   22   23  24  25
    { 10, 26,  10,  26, 10, 19,  -1, -1, -1, -1, -1, -1, -1, -1, -1, 31,  25, -1, -1 ,  9 ,25 , -1, -1, -1, -1, -1}, // I00
    { 11, 27,  11,  27, 11, -1,  20, -1, -1, -1, -1, -1, -1, -1, -1, 16 ,  4, -1, -1 ,  11, 27 , -1, -1, -1, -1, -1}, // I01
    { -1, -1, -1,  -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I02
    { -1, -1,  25, -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I03
    { -1, -1,  26, -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I04
    { 12, 28,  12 , 28, 12, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 , 15, 30 , -1, -1, -1, -1, -1}, // I05
    { 13, 29,  13,  29, 13, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 , 17 ,  1 , -1, -1, -1, -1, -1}, // I06
    { 14, 30,  14,  30, 14, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 , 19 ,  3 , -1, -1, -1, -1, -1}, // I07
    { 15, 31,  15,  31, 15, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 , 21 ,  5 , -1, -1, -1, -1, -1}, // I08
    { -1, -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I09
    {  1,  -1,   5,   9,  -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I10
    { -1,   3,   7, -1,  11, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I11
    {  1,  -1,   5,   9,  -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I12
    { -1,   3,   7, -1,  11, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I13
    { -1, -1, -1, -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I14
    { 15, 20, 25,  30,   3,   7,  -1,  -1, -1, 10, -1, -1, -1, -1, -1,   0,   2, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I15
    { 16, 21, 26,  31,   4, -1,    9,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   5, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I16
    { 14, 19, 30,  29,   2, -1,    8,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   3, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I17
    { 14, 19, 31,  29,   2, -1,    8,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   3, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I18
    {   2, 19, 11,  16, 19, -1,    8,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   3, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I19
    { 14,  5, 13,  17, 21, -1,    8,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   3, -1, -1 ,-1 , -1 , -1, -1, -1, -1, -1}, // I20
    { 14, 19, 24,  29,   2, -1,    8,  -1, -1, -1, 11, -1, -1, -1, -1, 30,   3, -1, -1 ,-1 , -1 , -1, -1, -1, -1}, // I21

};

/**
* connection of register
*/
static const short mConnectionReg[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
    //   0          1           2       3          4          5          6          7        8          9         10        11      12          13       14        15        16       17         18        19       20       21        22       23      24      25
    { 0x20,   0x20,  0x24,  0x24,  0x28,   0x28 ,     -1,    0x5c,     -1,   0x5c,  0x5c,      -1,     -1,  0x448,  0x448,  0x438, 0x438,   0x5c,   0x5c, 0x464, 0x464,      -1,       -1,     -1,     -1,     -1}, //I00
    { 0x20,   0x20,  0x24,  0x24,  0x28,       -1,   0x28,   0x5c,  0x5c,  0x5c,  0x5c,      -1,     -1,  0x448,  0x448,  0x438, 0x438,   0x5c,   0x5c,0x464, 0x464,      -1,   0xbc,     -1,     -1,     -1}, //I01
    {   -1,     0x20,  0x24,  0x24,     -1,       -1,    0x30,   0x30,     -1,       -1,     -1,   0x2c,     -1,  0x448,  0x448,        -1,      -1,    0x30,   0x30,      -1,      -1,       -1,      -1,     -1,     -1,     -1}, //I02
    { 0x20,   0x20,  0x24,  0x24,  0x28,   0x28,     -1,     0x28,     -1,   0x2C,      -1,      -1,      -1,  0x448,  0x448,  0x438, 0x438,   0x30,      -1, 0x464,0x464,   0x5c,      -1,  0xbc,     -1,  0xbc}, //I03
    { 0x20,   0x20,  0x24,  0x24,  0x28,       -1,   0x28,       -1,  0x28,      -1,  0x2C,      -1,      -1,  0x448,  0x448,  0x438, 0x438,       -1,   0x30,0x464,0x464,       -1,  0xbc,     -1, 0xbc,     -1}, //I04
    { 0x20,   0x20,  0x24,  0x24,  0x28,   0x28,     -1,     0x28,     -1,   0x2C,      -1,      -1,      -1,  0x420,  0x420,        -1,      -1,   0x30,       -1, 0x464,0x464,       -1,     -1,     -1,    -1,     -1}, //I05
    { 0x20,   0x20,  0x24,  0x24,  0x28,       -1,   0x28,       -1,  0x28,     -1,   0x2C,      -1,  0x2C,  0x420,  0x420,        -1,      -1,       -1,   0x30, 0x464,      -1,       -1,     -1,     -1,    -1,     -1}, //I06
    { 0x20,   0x20,  0x24,  0x24,  0x28,   0x28,     -1,    0x28,      -1,   0x2C,      -1,      -1,     -1,  0x420,   0x420,        -1,      -1,   0x30,       -1, 0x464,      -1,       -1,     -1,     -1,    -1,     -1}, //I07
    { 0x20,   0x20,  0x24,  0x24,  0x28,       -1,   0x28,       -1,  0x28,     -1,   0x2C,      -1,   0x2C,  0x420,  0x420,        -1,      -1,       -1,   0x30, 0x464,      -1,       -1,      -1,      -1,    -1,     -1}, //I08
    { 0x20,   0x20,  0x24,  0x24,  0x28,   0x28,     -1,         -1,     -1,  0x5c,      -1,       -1,  0x2C,  0x448,  0x448,  0x438, 0x438,  0x5c,   0x5c,    0x5c,  0x5c,      -1,       -1,  0xbc, 0xbc, 0xbc}, //I09
    {0x420,      -1,    -1,  0x420,     -1, 0x420,    -1,   0x420,     -1,      -1,      -1,        -1,0x448,        -1,        -1,        -1,      -1,  0x420,     -1,  0x448,      -1, 0x448,0x44c,0x44c,     -1,0x44c}, //I10
    {   -1,   0x420,    -1,       -1, 0x420,      -1, 0x420,       -1, 0x420,    -1,      -1,       -1,0x448,        -1,        -1,        -1,      -1,       -1,  0x420,      -1,0x448, 0x448,0x44c,     -1,0x44c,     -1}, //I11
    {0x438, 0x438,    -1,  0x438,     -1,0x438,    -1,   0x438,     -1,      -1,      -1,        -1,     -1,        -1,       -1,        -1,      -1,  0x440,      -1,  0x444,      -1,0x444, 0x444,     -1,     -1,     -1}, //I12
    {   -1,   0x438,    -1,       -1, 0x438,     -1, 0x438,      -1, 0x438,     -1,      -1,        -1,      -1,        -1,        -1,        -1,      -1,       -1, 0x440,       -1,0x444,0x444, 0x444,     -1,     -1,     -1}, //I13
    { 0x2C,   0x2C,  0x2C,  0x2C,  0x30,  0x30,     -1,    0x5c,  0x5c,  0x5c,      -1,        -1,  0x30,  0x448, 0x448,   0x438, 0x440,       -1,       -1,   0x5c,  0x5c,       -1,      -1,     -1,0x44c,     -1}, //I14
    { 0x2C,   0x2C,  0x2C,  0x2C,  0x30,  0x30,     -1,       -1,      -1,   0x30,     -1,        -1,      -1,   0x448, 0x448,   0x438, 0x440,       -1,       -1,       -1,      -1,       -1,     -1,     -1,     -1,     -1}, //I15
    { 0x2C,   0x2C,  0x2C,  0x2C,  0x30,      -1,   0x30,      -1,      -1,      -1,   0x30,       -1,      -1,   0x448, 0x448,   0x438, 0x440,       -1,       -1,      -1,      -1,      -1,      -1,     -1,     -1,     -1}, //I16
    {0x460,  0x2C,  0x2C,  0x5c,  0x30,0x460,   0x30,0x460,      -1,0x460,   0x30, 0x464,      -1,   0x448, 0x448,   0x438, 0x440,   0x5c,       -1, 0x464,      -1,   0xbc,      -1,  0xbc,     -1,  0xbc}, //I17
    { 0x2C, 0x460,  0x2C,  0x2C,  0x5c,      -1, 0x460,      -1,0x460,      -1, 0x464,       -1, 0x464,   0x448, 0x448,   0x438, 0x440,       -1,    0x5c,0x464,      -1,       -1,  0xbc,     -1,  0xbc,     -1}, //I18
    {0x460,   0x2C,0x460,0x460,0x460,0x460,   0x30,0x460,      -1,0x460,   0x30,0x464,      -1,   0x448, 0x448,   0x438, 0x444, 0x464,       -1,  0x5c,   0x5c,  0xbc,      -1, 0xbc,     -1,     -1}, //I19
    { 0x2C, 0x460,0x460,0x460,0x460,      -1, 0x460,      -1,0x460,     -1,  0x464,       -1, 0x464,   0x448, 0x448,   0x438, 0x444,       -1, 0x464,   0x5c,   0x5c,     -1,   0xbc,    -1,  0xbc,     -1}, //I20
    { 0xbc,   0xbc,  0xbc,  0xbc,  0x30,      -1,   0x30,    0xbc,  0xbc,  0xbc,   0x30,       -1,  0xbc,   0x44c, 0x44c,   0x438, 0x440,       -1,       -1,   0xbc,   0xbc,     -1,      -1,     -1,     -1,     -1}, //I21
};


/**
* shift connection of register
*/
static const short mShiftConnectionReg[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
    //    0         1           2         3         4         5         6         7         8         9     10      11     12     13      14      15       16      17     18        19     20      21     22     23      24      25
    {  0x20,   0x20,   0x24,  0x24,  0x28,  0x30,    -1,      -1,      -1,    -1,     -1,    -1,    -1,     -1,     -1, 0x438,     -1,    -1,     -1,0x464, 0x464, -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I00
    {  0x20,   0x20,   0x24,  0x24,  0x28,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,    -1,     -1,     -1,      -1, 0x440,    -1,    -1,0x464, 0x464, -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I01
    {    -1,        -1,       -1,       -1,    -1,     -1,      -1,      -1,      -1,    -1,     -1,    -1 ,  -1,     -1,     -1,      -1,      -1,    -1,    -1,     -1,    -1  , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I02
    {    -1,        -1,    0x30,    -1,      -1,     -1,      -1,      -1,      -1,    -1,     -1,    -1,    -1,     -1,     -1,      -1,      -1,    -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I03
    {    -1,        -1,    0x30,    -1,      -1,     -1,      -1,      -1,      -1,    -1,     -1,    -1,    -1,     -1,     -1,      -1,     -1,     -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I04
    {  0x20,   0x20,   0x24,  0x24,  0x28,    -1,      -1,      -1,      -1,    -1,     -1,  0x2C,   -1,     -1,     -1,      -1,     -1,    -1,    -1,0x464, 0x464, -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I05
    {  0x20,   0x20,   0x24,  0x24,  0x28,    -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,    -1,0x464,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I06
    {  0x20,   0x20,   0x24,  0x24,  0x28,    -1,      -1,      -1,      -1,    -1,     -1,   0x2C,  -1,     -1,     -1,      -1,     -1,    -1,    -1,0x464,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I07
    {  0x20,   0x20,   0x24,  0x24,  0x28,    -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,    -1,0x464,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I08
    {    -1,        -1,       -1,     -1,      -1,     -1,      -1,      -1,     -1,    -1,     -1,    -1,      -1,     -1,     -1,      -1,     -1,    -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I09
    { 0x420,     -1,       -1, 0x420,    -1,     -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I10
    { 0x420, 0x420,     -1,     -1,0x420,     -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,     -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I11
    { 0x438, 0x438,     -1, 0x438,    -1,     -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I12
    {    -1,   0x438,      -1,      -1, 0x438,   -1,      -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,      -1,     -1,    -1,    -1,      -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I13
    {    -1,        -1,       -1,      -1,     -1,     -1,      -1,       -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1,     -1,     -1,    -1,    -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I14
    {  0x2C,   0x2C,      -1,   0x2C,  0x30,  0x30,    -1,      -1,      -1,    -1,     -1,    -1,     -1,     -1,    -1, 0x440,     -1,    -1,    -1,     -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I15
    {  0x2C,   0x2C,      -1,   0x2C,  0x30,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,    -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I16
    {  0x2C,   0x2C,  0xbc,  0x2C,  0x30,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,     -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I17
    {  0x2C,   0x2C,  0xbc,  0x2C,  0x30,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,     -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I18
    {0x460,   0x2C,0x460, 0x460,0x460,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,    -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I19
    {  0x2C, 0x460,0x460, 0x460,0x460,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,    -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I20
    {  0x2C,   0x2C,     -1,   0x2C,  0x30,    -1,  0x30,      -1,      -1,    -1,     -1,    -1,     -1,     -1,     -1,     -1, 0x440,    -1,    -1,    -1,    -1   , -1 ,    -1 ,    -1 ,    -1 ,   -1}, //I21
};

/**
* connection state of register
*/
static char mConnectionState[Soc_Aud_InterConnectionInput_Num_Input][Soc_Aud_InterConnectionOutput_Num_Output] =
{
    // 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I00
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I01
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I02
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I03
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I04
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I05
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I06
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I07
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I08
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I09
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I10
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I11
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I12
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I13
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I14
    //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I15
    //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // I16
};

static bool  CheckBitsandReg(short regaddr , char bits)
{
    if (regaddr <= 0 || bits < 0)
    {
        printk("regaddr = %x bits = %d \n", regaddr, bits);
        return false;
    }
    return true;
}

bool SetConnectionState(uint32 ConnectionState, uint32 Input, uint32 Output)
{
    //printk("SetinputConnection ConnectionState = %d Input = %d Output = %d\n", ConnectionState, Input, Output);
    if ((mConnectionTable[Input][Output]) < 0)
    {
        printk("no connection mpConnectionTable[%d][%d] = %d\n", Input, Output, mConnectionTable[Input][Output]);
    }
    else if ((mConnectionTable[Input][Output]) == 0)
    {
        printk("test only !! mpConnectionTable[%d][%d] = %d\n", Input, Output, mConnectionTable[Input][Output]);
    }
    else
    {
        if (mConnectionTable[Input][Output])
        {
            int connectionBits = 0;
            int connectReg = 0;
            switch (ConnectionState)
            {
                case Soc_Aud_InterCon_DisConnect:
                {
                    //printk("nConnectionState = %d \n", ConnectionState);
                    if ((mConnectionState[Input][Output]&Soc_Aud_InterCon_Connection) == Soc_Aud_InterCon_Connection)
                    {
                        // here to disconnect connect bits
                        connectionBits = mConnectionbits[Input][Output];
                        connectReg = mConnectionReg[Input][Output];
                        if (CheckBitsandReg(connectReg, connectionBits))
                        {
                            Afe_Set_Reg(connectReg, 0 << connectionBits, 1 << connectionBits);
                            mConnectionState[Input][Output] &= ~(Soc_Aud_InterCon_Connection);
                        }
                    }
                    if ((mConnectionState[Input][Output]&Soc_Aud_InterCon_ConnectionShift) == Soc_Aud_InterCon_ConnectionShift)
                    {
                        // here to disconnect connect shift bits
                        connectionBits = mShiftConnectionbits[Input][Output];
                        connectReg = mShiftConnectionReg[Input][Output];
                        if (CheckBitsandReg(connectReg, connectionBits))
                        {
                            Afe_Set_Reg(connectReg, 0 << connectionBits, 1 << connectionBits);
                            mConnectionState[Input][Output] &= ~(Soc_Aud_InterCon_ConnectionShift);
                        }
                    }
                    break;
                }
                case Soc_Aud_InterCon_Connection:
                {
                    //printk("nConnectionState = %d \n", ConnectionState);
                    // here to disconnect connect shift bits
                    connectionBits = mConnectionbits[Input][Output];
                    connectReg = mConnectionReg[Input][Output];
                    if (CheckBitsandReg(connectReg, connectionBits))
                    {
                        Afe_Set_Reg(connectReg, 1 << connectionBits, 1 << connectionBits);
                        mConnectionState[Input][Output] |= Soc_Aud_InterCon_Connection;
                    }
                    break;
                }
                case Soc_Aud_InterCon_ConnectionShift:
                {
                    //printk("nConnectionState = %d \n", ConnectionState);
                    if ((mConnectionTable[Input][Output]&Soc_Aud_InterCon_ConnectionShift) != Soc_Aud_InterCon_ConnectionShift)
                    {
                        printk("donn't support shift opeartion");
                        break;
                    }
                    connectionBits = mShiftConnectionbits[Input][Output];
                    connectReg = mShiftConnectionReg[Input][Output];
                    if (CheckBitsandReg(connectReg, connectionBits))
                    {
                        Afe_Set_Reg(connectReg, 1 << connectionBits, 1 << connectionBits);
                        mConnectionState[Input][Output] |= Soc_Aud_InterCon_ConnectionShift;
                    }
                    break;
                }
                default:
                    printk("no this state ConnectionState = %d \n", ConnectionState);
                    break;
            }
        }
    }
    return true;
}

EXPORT_SYMBOL(SetConnectionState);


