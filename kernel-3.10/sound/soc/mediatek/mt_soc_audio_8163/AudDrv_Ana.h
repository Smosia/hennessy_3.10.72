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
 *   AudDrv_Ana.h
 *
 * Project:
 * --------
 *   MT6583  Audio Driver Ana
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 *   Chipeng Chang (mtk02308)
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/

#ifndef _AUDDRV_ANA_H_
#define _AUDDRV_ANA_H_

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"


/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/

/*****************************************************************************
 *                  R E G I S T E R       D E F I N I T I O N
 *****************************************************************************/
#define PMIC_REG_BASE                    (0x0000)
//6323 pmic reg
//---------------digital pmic  register define -------------------------------------------
#define ABB_AFE_CON0             (PMIC_REG_BASE+0x4000 + 0x0000)
#define ABB_AFE_CON1             (PMIC_REG_BASE+0x4000 + 0x0002)
#define ABB_AFE_CON2             (PMIC_REG_BASE+0x4000 + 0x0004)
#define ABB_AFE_CON3             (PMIC_REG_BASE+0x4000 + 0x0006)
#define ABB_AFE_CON4             (PMIC_REG_BASE+0x4000 + 0x0008)
#define ABB_AFE_CON5             (PMIC_REG_BASE+0x4000 + 0x000A)
#define ABB_AFE_CON6             (PMIC_REG_BASE+0x4000 + 0x000C)
#define ABB_AFE_CON7             (PMIC_REG_BASE+0x4000 + 0x000E)
#define ABB_AFE_CON8             (PMIC_REG_BASE+0x4000 + 0x0010)
#define ABB_AFE_CON9             (PMIC_REG_BASE+0x4000 + 0x0012)
#define ABB_AFE_CON10            (PMIC_REG_BASE+0x4000 + 0x0014)
#define ABB_AFE_CON11            (PMIC_REG_BASE+0x4000 + 0x0016)
#define ABB_AFE_STA0             (PMIC_REG_BASE+0x4000 + 0x0018)
#define ABB_AFE_STA1             (PMIC_REG_BASE+0x4000 + 0x001A)
#define ABB_AFE_STA2             (PMIC_REG_BASE+0x4000 + 0x001C)
#define AFE_UP8X_FIFO_CFG0       (PMIC_REG_BASE+0x4000 + 0x001E)
#define AFE_UP8X_FIFO_LOG_MON0   (PMIC_REG_BASE+0x4000 + 0x0020)
#define AFE_UP8X_FIFO_LOG_MON1   (PMIC_REG_BASE+0x4000 + 0x0022)
#define AFE_PMIC_NEWIF_CFG0      (PMIC_REG_BASE+0x4000 + 0x0024)
#define AFE_PMIC_NEWIF_CFG1      (PMIC_REG_BASE+0x4000 + 0x0026)
#define AFE_PMIC_NEWIF_CFG2      (PMIC_REG_BASE+0x4000 + 0x0028)
#define AFE_PMIC_NEWIF_CFG3      (PMIC_REG_BASE+0x4000 + 0x002A)
#define ABB_AFE_TOP_CON0         (PMIC_REG_BASE+0x4000 + 0x002C)
#define ABB_MON_DEBUG0           (PMIC_REG_BASE+0x4000 + 0x002E)

//---------------digital pmic  register define end ---------------------------------------

//---------------analog pmic  register define start --------------------------------------
#define TOP_CKPDN0                  (PMIC_REG_BASE + 0x102)
#define TOP_CKPDN0_SET              (PMIC_REG_BASE + 0x104)
#define TOP_CKPDN0_CLR              (PMIC_REG_BASE + 0x106)
#define TOP_CKPDN1                  (PMIC_REG_BASE + 0x108)
#define TOP_CKPDN1_SET              (PMIC_REG_BASE + 0x10A)
#define TOP_CKPDN1_CLR              (PMIC_REG_BASE + 0x10C)
#define TOP_CKPDN2                  (PMIC_REG_BASE + 0x10E)
#define TOP_CKPDN2_SET              (PMIC_REG_BASE + 0x110)
#define TOP_CKPDN2_CLR              (PMIC_REG_BASE + 0x112)
#define TOP_CKCON1                  (PMIC_REG_BASE + 0x126)

#ifdef  SPK_CON0
#undef  SPK_CON0
#endif
#define SPK_CON0                    (PMIC_REG_BASE + 0x052)
#ifdef  SPK_CON1
#undef  SPK_CON1
#endif
#define SPK_CON1                    (PMIC_REG_BASE + 0x054)
#ifdef  SPK_CON2
#undef  SPK_CON2
#endif
#define SPK_CON2                    (PMIC_REG_BASE + 0x056)
#ifdef  SPK_CON6
#undef  SPK_CON6
#endif
#define SPK_CON6                    (PMIC_REG_BASE + 0x05E)
#ifdef  SPK_CON7
#undef  SPK_CON7
#endif
#define SPK_CON7                    (PMIC_REG_BASE + 0x060)
#ifdef  SPK_CON8
#undef  SPK_CON8
#endif
#define SPK_CON8                    (PMIC_REG_BASE + 0x062)
#ifdef  SPK_CON9
#undef  SPK_CON9
#endif
#define SPK_CON9                    (PMIC_REG_BASE + 0x064)
#ifdef  SPK_CON10
#undef  SPK_CON10
#endif
#define SPK_CON10                   (PMIC_REG_BASE + 0x066)
#ifdef  SPK_CON11
#undef  SPK_CON11
#endif
#define SPK_CON11                   (PMIC_REG_BASE + 0x068)
#ifdef  SPK_CON12
#undef  SPK_CON12
#endif
#define SPK_CON12                   (PMIC_REG_BASE + 0x06A)
#ifdef  CID
#undef  CID
#endif
#define CID                         (PMIC_REG_BASE + 0x100)

#ifdef  AUDTOP_CON0
#undef  AUDTOP_CON0
#endif
#define AUDTOP_CON0                 (PMIC_REG_BASE + 0x700)
#ifdef  AUDTOP_CON1
#undef  AUDTOP_CON1
#endif
#define AUDTOP_CON1                 (PMIC_REG_BASE + 0x702)
#ifdef  AUDTOP_CON2
#undef  AUDTOP_CON2
#endif
#define AUDTOP_CON2                 (PMIC_REG_BASE + 0x704)
#ifdef  AUDTOP_CON3
#undef  AUDTOP_CON3
#endif
#define AUDTOP_CON3                 (PMIC_REG_BASE + 0x706)
#ifdef  AUDTOP_CON4
#undef  AUDTOP_CON4
#endif
#define AUDTOP_CON4                 (PMIC_REG_BASE + 0x708)
#ifdef  AUDTOP_CON5
#undef  AUDTOP_CON5
#endif
#define AUDTOP_CON5                 (PMIC_REG_BASE + 0x70A)
#ifdef  AUDTOP_CON6
#undef  AUDTOP_CON6
#endif
#define AUDTOP_CON6                 (PMIC_REG_BASE + 0x70C)
#ifdef  AUDTOP_CON7
#undef  AUDTOP_CON7
#endif
#define AUDTOP_CON7                 (PMIC_REG_BASE + 0x70E)
#ifdef  AUDTOP_CON8
#undef  AUDTOP_CON8
#endif
#define AUDTOP_CON8                 (PMIC_REG_BASE + 0x710)
#ifdef  AUDTOP_CON9
#undef  AUDTOP_CON9
#endif
#define AUDTOP_CON9                 (PMIC_REG_BASE + 0x712)


#if 1
//register number

#else
#include <mach/upmu_hw.h>
#endif

void Ana_Set_Reg(uint32 offset, uint32 value, uint32 mask);
uint32  Ana_Get_Reg(uint32 offset);

// for debug usage
void Ana_Log_Print(void);

#endif


