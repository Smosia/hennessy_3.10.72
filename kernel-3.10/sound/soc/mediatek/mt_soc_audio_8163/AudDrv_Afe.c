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
 *   AudioAfe.h
 *
 * Project:
 * --------
 *   MT6583  Audio Driver Afe Register setting
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
#include "AudDrv_Afe.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Def.h"
#include <linux/types.h>


/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         FUNCTION IMPLEMENTATION
 *****************************************************************************/
static bool CheckOffset(uint32 offset)
{
    if (offset > AFE_MAXLENGTH)
    {
        return false;
    }
    return true;
}


/*****************************************************************************
 * FUNCTION
 *  Auddrv_Reg_map
 *
 * DESCRIPTION
 * Auddrv_Reg_map
 *
 *****************************************************************************
 */

/*
 *    global variable control
 */
static const unsigned  int SramCaptureOffSet = (16 * 1024);

// address for ioremap audio hardware register
void *AFE_BASE_ADDRESS = 0;
void *AFE_SRAM_ADDRESS = 0;
void *AFE_TOP_ADDRESS = 0;
void *AFE_CLK_ADDRESS = 0;
void *AFE_INFRA_ADDRESS = 0;
void *APLL_BASE_ADDRESS = 0;

void Auddrv_Reg_map()
{
    AFE_SRAM_ADDRESS = ioremap_nocache(AFE_INTERNAL_SRAM_PHY_BASE, AFE_INTERNAL_SRAM_SIZE);

#ifndef CONFIG_OF
    AFE_BASE_ADDRESS = ioremap_nocache(AUDIO_HW_PHYSICAL_BASE, 0x1000);
#endif

    // temp for hardawre code  set 0x1000629c = 0xd
    AFE_TOP_ADDRESS = ioremap_nocache(AUDIO_POWER_TOP , 0x1000);
    AFE_INFRA_ADDRESS = ioremap_nocache(AUDIO_INFRA_BASE , 0x1000);

    // temp for hardawre code  set clg cfg
    AFE_CLK_ADDRESS = ioremap_nocache(AUDIO_CLKCFG_PHYSICAL_BASE , 0x1000);

    // temp for hardawre code  set pll cfg
    APLL_BASE_ADDRESS = ioremap_nocache(APLL_PHYSICAL_BASE, 0x1000);
}

dma_addr_t  Get_Afe_Sram_Phys_Addr(void)
{
    return (dma_addr_t)AFE_INTERNAL_SRAM_PHY_BASE;
}

dma_addr_t  Get_Afe_Sram_Capture_Phys_Addr(void)
{
    return (dma_addr_t)(AFE_INTERNAL_SRAM_PHY_BASE + SramCaptureOffSet);
}

void *Get_Afe_SramBase_Pointer()
{
    return AFE_SRAM_ADDRESS;
}

void *Get_Afe_SramCaptureBase_Pointer()
{
    char *CaptureSramPointer = (char *)(AFE_SRAM_ADDRESS) + SramCaptureOffSet;
    return (void *)CaptureSramPointer;
}

void *Get_Afe_Powertop_Pointer()
{
    return AFE_TOP_ADDRESS;
}

void *Get_AudClk_Pointer()
{
    return AFE_CLK_ADDRESS;
}

void *Get_Afe_Infra_Pointer()
{
    return AFE_INFRA_ADDRESS;
}

void Afe_Set_Reg(uint32 offset, uint32 value, uint32 mask)
{
    extern void *AFE_BASE_ADDRESS;
    volatile long address;
    volatile uint32 *AFE_Register;
    volatile uint32 val_tmp;

    if (CheckOffset(offset) == false)
    {
        return;
    }

#ifdef AUDIO_MEM_IOREMAP
    //PRINTK_AUDDRV("Afe_Set_Reg AUDIO_MEM_IOREMAP AFE_BASE_ADDRESS = %p\n",AFE_BASE_ADDRESS);
    address = (long)((char *)AFE_BASE_ADDRESS + offset);
#else
    address = (long)(AFE_BASE + offset);
#endif

    AFE_Register = (volatile uint32 *)address;

    //PRINTK_AFE_REG("Afe_Set_Reg offset=%x, value=%x, mask=%x \n",offset,value,mask);
    val_tmp = Afe_Get_Reg(offset);
    val_tmp &= (~mask);
    val_tmp |= (value & mask);
    mt_reg_sync_writel(val_tmp, AFE_Register);
}

uint32 Afe_Get_Reg(uint32 offset)
{
    extern void *AFE_BASE_ADDRESS;
    volatile long address;
    volatile uint32 *value;

    if (CheckOffset(offset) == false)
    {
        return 0xffffffff;
    }

#ifdef AUDIO_MEM_IOREMAP
    //PRINTK_AUDDRV("Afe_Get_Reg AUDIO_MEM_IOREMAP AFE_BASE_ADDRESS = %p\ offset = %xn",AFE_BASE_ADDRESS,offset);
    address = (long)((char *)AFE_BASE_ADDRESS + offset);
#else
    address = (long)(AFE_BASE + offset);
#endif

    value = (volatile uint32 *)(address);
    //PRINTK_AFE_REG("Afe_Get_Reg offset=%x address = %x value = 0x%x\n",offset,address,*value);
    return *value;
}

// function to Set Cfg
uint32 GetClkCfg(uint32 offset)
{
    volatile long address = (long)((char *)AFE_CLK_ADDRESS + offset);
    volatile uint32 *value;
    value = (volatile uint32 *)(address);
    //printk("GetClkCfg offset=%x address = %x value = 0x%x\n", offset, address, *value);
    return *value;
}

void SetClkCfg(uint32 offset, uint32 value, uint32 mask)
{
    volatile long address = (long)((char *)AFE_CLK_ADDRESS + offset);
    volatile uint32 *AFE_Register = (volatile uint32 *)address;
    volatile uint32 val_tmp;
    //printk("SetClkCfg offset=%x, value=%x, mask=%x \n",offset,value,mask);
    val_tmp = GetClkCfg(offset);
    val_tmp &= (~mask);
    val_tmp |= (value & mask);
    mt_reg_sync_writel(val_tmp, AFE_Register);
}

// function to Set Cfg
uint32 GetInfraCfg(uint32 offset)
{
    volatile long address = (long)((char *)AFE_INFRA_ADDRESS + offset);
    volatile uint32 *value;
    value = (volatile uint32 *)(address);
    //printk("GetInfraCfg offset=%x address = %x value = 0x%x\n", offset, address, *value);
    return *value;
}

void SetInfraCfg(uint32 offset, uint32 value, uint32 mask)
{
    volatile long address = (long)((char *)AFE_INFRA_ADDRESS + offset);
    volatile uint32 *AFE_Register = (volatile uint32 *)address;
    volatile uint32 val_tmp;
    //printk("SetInfraCfg offset=%x, value=%x, mask=%x \n",offset,value,mask);
    val_tmp = GetInfraCfg(offset);
    val_tmp &= (~mask);
    val_tmp |= (value & mask);
    mt_reg_sync_writel(val_tmp, AFE_Register);
}


// function to Set pll
uint32 GetpllCfg(uint32 offset)
{
    volatile long address = (long)((char *)APLL_BASE_ADDRESS + offset);
    volatile uint32 *value;
    value = (volatile uint32 *)(address);
    //printk("GetClkCfg offset=%x address = %x value = 0x%x\n", offset, address, *value);
    return *value;
}

void SetpllCfg(uint32 offset, uint32 value, uint32 mask)
{
    volatile long address = (long)((char *)APLL_BASE_ADDRESS + offset);
    volatile uint32 *AFE_Register = (volatile uint32 *)address;
    volatile uint32 val_tmp;
    //printk("SetpllCfg offset=%x, value=%x, mask=%x \n",offset,value,mask);
    val_tmp = GetpllCfg(offset);
    val_tmp &= (~mask);
    val_tmp |= (value & mask);
    mt_reg_sync_writel(val_tmp, AFE_Register);
}

void Afe_Log_Print(void)
{
    AudDrv_Clk_On();
    printk("+AudDrv Afe_Log_Print \n");
    printk("AUDIO_TOP_CON0		   = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON0));
    printk("AUDIO_TOP_CON1		   = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON1));
    printk("AUDIO_TOP_CON2		   = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON2));
    printk("AUDIO_TOP_CON3		   = 0x%x\n", Afe_Get_Reg(AUDIO_TOP_CON3));
    printk("AFE_DAC_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON0));
    printk("AFE_DAC_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_DAC_CON1));
    printk("AFE_I2S_CON 		   = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON));
    printk("AFE_DAIBT_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_DAIBT_CON0));
    printk("AFE_CONN0			   = 0x%x\n", Afe_Get_Reg(AFE_CONN0));
    printk("AFE_CONN1			   = 0x%x\n", Afe_Get_Reg(AFE_CONN1));
    printk("AFE_CONN2			   = 0x%x\n", Afe_Get_Reg(AFE_CONN2));
    printk("AFE_CONN3			   = 0x%x\n", Afe_Get_Reg(AFE_CONN3));
    printk("AFE_CONN4			   = 0x%x\n", Afe_Get_Reg(AFE_CONN4));
    printk("AFE_I2S_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON1));
    printk("AFE_I2S_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON2));
    printk("AFE_MRGIF_CON		   = 0x%x\n", Afe_Get_Reg(AFE_MRGIF_CON));
    printk("AFE_DL1_BASE		   = 0x%x\n", Afe_Get_Reg(AFE_DL1_BASE));
    printk("AFE_DL1_CUR 		   = 0x%x\n", Afe_Get_Reg(AFE_DL1_CUR));
    printk("AFE_DL1_END 		   = 0x%x\n", Afe_Get_Reg(AFE_DL1_END));
    printk("AFE_DL1_D2_BASE 	   = 0x%x\n", Afe_Get_Reg(AFE_DL1_D2_BASE));
    printk("AFE_DL1_D2_CUR		   = 0x%x\n", Afe_Get_Reg(AFE_DL1_D2_CUR));
    printk("AFE_DL1_D2_END		   = 0x%x\n", Afe_Get_Reg(AFE_DL1_D2_END));
    printk("AFE_VUL_D2_BASE 	   = 0x%x\n", Afe_Get_Reg(AFE_VUL_D2_BASE));
    printk("AFE_VUL_D2_END		   = 0x%x\n", Afe_Get_Reg(AFE_VUL_D2_END));
    printk("AFE_VUL_D2_CUR		   = 0x%x\n", Afe_Get_Reg(AFE_VUL_D2_CUR));
    printk("AFE_I2S_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_I2S_CON3));
    printk("AFE_DL2_BASE		   = 0x%x\n", Afe_Get_Reg(AFE_DL2_BASE));
    printk("AFE_DL2_CUR 		   = 0x%x\n", Afe_Get_Reg(AFE_DL2_CUR));
    printk("AFE_DL2_END 		   = 0x%x\n", Afe_Get_Reg(AFE_DL2_END));
    printk("AFE_CONN5			   = 0x%x\n", Afe_Get_Reg(AFE_CONN5));
    printk("AFE_CONN_24BIT		   = 0x%x\n", Afe_Get_Reg(AFE_CONN_24BIT));
    printk("AFE_AWB_BASE		   = 0x%x\n", Afe_Get_Reg(AFE_AWB_BASE));
    printk("AFE_AWB_END 		   = 0x%x\n", Afe_Get_Reg(AFE_AWB_END));
    printk("AFE_AWB_CUR 		   = 0x%x\n", Afe_Get_Reg(AFE_AWB_CUR));
    printk("AFE_VUL_BASE		   = 0x%x\n", Afe_Get_Reg(AFE_VUL_BASE));
    printk("AFE_VUL_END 		   = 0x%x\n", Afe_Get_Reg(AFE_VUL_END));
    printk("AFE_VUL_CUR 		   = 0x%x\n", Afe_Get_Reg(AFE_VUL_CUR));
    printk("AFE_DAI_BASE		   = 0x%x\n", Afe_Get_Reg(AFE_DAI_BASE));
    printk("AFE_DAI_END 		   = 0x%x\n", Afe_Get_Reg(AFE_DAI_END));
    printk("AFE_DAI_CUR 		   = 0x%x\n", Afe_Get_Reg(AFE_DAI_CUR));
    printk("AFE_CONN6			   = 0x%x\n", Afe_Get_Reg(AFE_CONN6));
    printk("AFE_MEMIF_MSB		   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MSB));
    printk("AFE_MEMIF_MON0		   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON0));
    printk("AFE_MEMIF_MON1		   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON1));
    printk("AFE_MEMIF_MON2		   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON2));
    printk("AFE_MEMIF_MON4		   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MON4));
    printk("AFE_ADDA_DL_SRC2_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON0));
    printk("AFE_ADDA_DL_SRC2_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA_DL_SRC2_CON1));
    printk("AFE_ADDA_UL_SRC_CON0   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON0));
    printk("AFE_ADDA_UL_SRC_CON1   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_SRC_CON1));
    printk("AFE_ADDA_TOP_CON0	   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_TOP_CON0));
    printk("AFE_ADDA_UL_DL_CON0    = 0x%x\n", Afe_Get_Reg(AFE_ADDA_UL_DL_CON0));
    printk("AFE_ADDA_SRC_DEBUG	   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG));
    printk("AFE_ADDA_SRC_DEBUG_MON0= 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON0));
    printk("AFE_ADDA_SRC_DEBUG_MON1= 0x%x\n", Afe_Get_Reg(AFE_ADDA_SRC_DEBUG_MON1));
    printk("AFE_ADDA_NEWIF_CFG0    = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG0));
    printk("AFE_ADDA_NEWIF_CFG1    = 0x%x\n", Afe_Get_Reg(AFE_ADDA_NEWIF_CFG1));
    printk("AFE_SIDETONE_DEBUG	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_DEBUG));
    printk("AFE_SIDETONE_MON	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_MON));
    printk("AFE_SIDETONE_CON0	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON0));
    printk("AFE_SIDETONE_COEFF	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_COEFF));
    printk("AFE_SIDETONE_CON1	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_CON1));
    printk("AFE_SIDETONE_GAIN	   = 0x%x\n", Afe_Get_Reg(AFE_SIDETONE_GAIN));
    printk("AFE_SGEN_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_SGEN_CON0));
    printk("AFE_TOP_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_TOP_CON0));
    printk("AFE_ADDA_PREDIS_CON0   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON0));
    printk("AFE_ADDA_PREDIS_CON1   = 0x%x\n", Afe_Get_Reg(AFE_ADDA_PREDIS_CON1));
    printk("AFE_MRGIF_MON0		   = 0x%x\n", Afe_Get_Reg(AFE_MRGIF_MON0));
    printk("AFE_MRGIF_MON1		   = 0x%x\n", Afe_Get_Reg(AFE_MRGIF_MON1));
    printk("AFE_MRGIF_MON2		   = 0x%x\n", Afe_Get_Reg(AFE_MRGIF_MON2));
    printk("AFE_MOD_DAI_BASE	   = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_BASE));
    printk("AFE_MOD_DAI_END 	   = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_END));
    printk("AFE_MOD_DAI_CUR 	   = 0x%x\n", Afe_Get_Reg(AFE_MOD_DAI_CUR));
    printk("AFE_IRQ_MCU_CON 	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CON));
    printk("AFE_IRQ_MCU_STATUS	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_STATUS));
    printk("AFE_IRQ_MCU_CLR 	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CLR));
    printk("AFE_IRQ_MCU_CNT1	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CNT1));
    printk("AFE_IRQ_MCU_CNT2	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CNT2));
    printk("AFE_IRQ_MCU_EN		   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_EN));
    printk("AFE_IRQ_MCU_MON2	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_MON2));
    printk("AFE_IRQ_CNT5		   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_CNT5));
    printk("AFE_IRQ1_MCU_CNT_MON   = 0x%x\n", Afe_Get_Reg(AFE_IRQ1_MCU_CNT_MON));
    printk("AFE_IRQ2_MCU_CNT_MON   = 0x%x\n", Afe_Get_Reg(AFE_IRQ2_MCU_CNT_MON));
    printk("AFE_IRQ1_MCU_EN_CNT_MON= 0x%x\n", Afe_Get_Reg(AFE_IRQ1_MCU_EN_CNT_MON));
    printk("AFE_IRQ_DEBUG		   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_DEBUG));
    printk("AFE_MEMIF_MAXLEN	   = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_MAXLEN));
    printk("AFE_MEMIF_PBUF_SIZE    = 0x%x\n", Afe_Get_Reg(AFE_MEMIF_PBUF_SIZE));
    printk("AFE_IRQ_MCU_CNT7	   = 0x%x\n", Afe_Get_Reg(AFE_IRQ_MCU_CNT7));
    printk("AFE_APLL1_TUNER_CFG    = 0x%x\n", Afe_Get_Reg(AFE_APLL1_TUNER_CFG));
    printk("AFE_APLL2_TUNER_CFG    = 0x%x\n", Afe_Get_Reg(AFE_APLL2_TUNER_CFG));
    printk("AFE_GAIN1_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON0));
    printk("AFE_GAIN1_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON1));
    printk("AFE_GAIN1_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON2));
    printk("AFE_GAIN1_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CON3));
    printk("AFE_GAIN1_CONN		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CONN));
    printk("AFE_GAIN1_CUR		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CUR));
    printk("AFE_GAIN2_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON0));
    printk("AFE_GAIN2_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON1));
    printk("AFE_GAIN2_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON2));
    printk("AFE_GAIN2_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CON3));
    printk("AFE_GAIN2_CONN		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN));
    printk("AFE_GAIN2_CUR		   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CUR));
    printk("AFE_GAIN2_CONN2 	   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN2));
    printk("AFE_GAIN2_CONN3 	   = 0x%x\n", Afe_Get_Reg(AFE_GAIN2_CONN3));
    printk("AFE_GAIN1_CONN2 	   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CONN2));
    printk("AFE_GAIN1_CONN3 	   = 0x%x\n", Afe_Get_Reg(AFE_GAIN1_CONN3));
    printk("AFE_CONN7			   = 0x%x\n", Afe_Get_Reg(AFE_CONN7));
    printk("AFE_CONN8			   = 0x%x\n", Afe_Get_Reg(AFE_CONN8));
    printk("AFE_CONN9			   = 0x%x\n", Afe_Get_Reg(AFE_CONN9));
    printk("AFE_CONN10			   = 0x%x\n", Afe_Get_Reg(AFE_CONN10));
    printk("FPGA_CFG2			   = 0x%x\n", Afe_Get_Reg(FPGA_CFG2));
    printk("FPGA_CFG3			   = 0x%x\n", Afe_Get_Reg(FPGA_CFG3));
    printk("FPGA_CFG0			   = 0x%x\n", Afe_Get_Reg(FPGA_CFG0));
    printk("FPGA_CFG1			   = 0x%x\n", Afe_Get_Reg(FPGA_CFG1));
    printk("FPGA_VER			   = 0x%x\n", Afe_Get_Reg(FPGA_VER));
    printk("FPGA_STC			   = 0x%x\n", Afe_Get_Reg(FPGA_STC));
    printk("AFE_ASRC_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON0));
    printk("AFE_ASRC_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON1));
    printk("AFE_ASRC_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON2));
    printk("AFE_ASRC_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON3));
    printk("AFE_ASRC_CON4		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON4));
    printk("AFE_ASRC_CON5		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON5));
    printk("AFE_ASRC_CON6		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON6));
    printk("AFE_ASRC_CON7		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON7));
    printk("AFE_ASRC_CON8		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON8));
    printk("AFE_ASRC_CON9		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON9));
    printk("AFE_ASRC_CON10		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON10));
    printk("AFE_ASRC_CON11		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON11));
    printk("PCM_INTF_CON		   = 0x%x\n", Afe_Get_Reg(PCM_INTF_CON));
    printk("PCM_INTF_CON2		   = 0x%x\n", Afe_Get_Reg(PCM_INTF_CON2));
    printk("PCM2_INTF_CON		   = 0x%x\n", Afe_Get_Reg(PCM2_INTF_CON));
    printk("AFE_ASRC_CON13		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON13));
    printk("AFE_ASRC_CON14		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON14));
    printk("AFE_ASRC_CON15		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON15));
    printk("AFE_ASRC_CON16		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON16));
    printk("AFE_ASRC_CON17		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON17));
    printk("AFE_ASRC_CON18		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON18));
    printk("AFE_ASRC_CON19		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON19));
    printk("AFE_ASRC_CON20		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON20));
    printk("AFE_ASRC_CON21		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC_CON21));
    printk("AFE_ASRC4_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON0));
    printk("AFE_ASRC4_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON1));
    printk("AFE_ASRC4_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON2));
    printk("AFE_ASRC4_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON3));
    printk("AFE_ASRC4_CON4		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON4));
    printk("AFE_ASRC4_CON5		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON5));
    printk("AFE_ASRC4_CON6		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON6));
    printk("AFE_ASRC4_CON7		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON7));
    printk("AFE_ASRC4_CON8		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON8));
    printk("AFE_ASRC4_CON9		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON9));
    printk("AFE_ASRC4_CON10 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON10));
    printk("AFE_ASRC4_CON11 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON11));
    printk("AFE_ASRC4_CON12 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON12));
    printk("AFE_ASRC4_CON13 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON13));
    printk("AFE_ASRC4_CON14 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC4_CON14));
    printk("AFE_ASRC2_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON0));
    printk("AFE_ASRC2_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON1));
    printk("AFE_ASRC2_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON2));
    printk("AFE_ASRC2_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON3));
    printk("AFE_ASRC2_CON4		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON4));
    printk("AFE_ASRC2_CON5		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON5));
    printk("AFE_ASRC2_CON6		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON6));
    printk("AFE_ASRC2_CON7		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON7));
    printk("AFE_ASRC2_CON8		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON8));
    printk("AFE_ASRC2_CON9		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON9));
    printk("AFE_ASRC2_CON10 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON10));
    printk("AFE_ASRC2_CON11 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON11));
    printk("AFE_ASRC2_CON12 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON12));
    printk("AFE_ASRC2_CON13 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON13));
    printk("AFE_ASRC2_CON14 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC2_CON14));
    printk("AFE_ASRC3_CON0		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON0));
    printk("AFE_ASRC3_CON1		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON1));
    printk("AFE_ASRC3_CON2		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON2));
    printk("AFE_ASRC3_CON3		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON3));
    printk("AFE_ASRC3_CON4		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON4));
    printk("AFE_ASRC3_CON5		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON5));
    printk("AFE_ASRC3_CON6		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON6));
    printk("AFE_ASRC3_CON7		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON7));
    printk("AFE_ASRC3_CON8		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON8));
    printk("AFE_ASRC3_CON9		   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON9));
    printk("AFE_ASRC3_CON10 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON10));
    printk("AFE_ASRC3_CON11 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON11));
    printk("AFE_ASRC3_CON12 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON12));
    printk("AFE_ASRC3_CON13 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON13));
    printk("AFE_ASRC3_CON14 	   = 0x%x\n", Afe_Get_Reg(AFE_ASRC3_CON14));
    printk("AFE_ADDA4_TOP_CON0	   = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_TOP_CON0));
    printk("AFE_ADDA4_UL_SRC_CON0  = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_UL_SRC_CON0));
    printk("AFE_ADDA4_UL_SRC_CON1  = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_UL_SRC_CON1));
    printk("AFE_ADDA4_SRC_DEBUG    = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_SRC_DEBUG));
    printk("AFE_ADDA4_SRC_DEBUG_MON= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_SRC_DEBUG_MON0));
    printk("AFE_ADDA4_SRC_DEBUG_MON= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_SRC_DEBUG_MON1));
    printk("AFE_ADDA4_NEWIF_CFG0   = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_NEWIF_CFG0));
    printk("AFE_ADDA4_NEWIF_CFG1   = 0x%x\n", Afe_Get_Reg(AFE_ADDA4_NEWIF_CFG1));
    printk("AFE_ADDA4_ULCF_CFG_02_0= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_02_01));
    printk("AFE_ADDA4_ULCF_CFG_04_0= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_04_03));
    printk("AFE_ADDA4_ULCF_CFG_06_0= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_06_05));
    printk("AFE_ADDA4_ULCF_CFG_08_0= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_08_07));
    printk("AFE_ADDA4_ULCF_CFG_10_0= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_10_09));
    printk("AFE_ADDA4_ULCF_CFG_12_1= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_12_11));
    printk("AFE_ADDA4_ULCF_CFG_14_1= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_14_13));
    printk("AFE_ADDA4_ULCF_CFG_16_1= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_16_15));
    printk("AFE_ADDA4_ULCF_CFG_18_1= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_18_17));
    printk("AFE_ADDA4_ULCF_CFG_20_1= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_20_19));
    printk("AFE_ADDA4_ULCF_CFG_22_2= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_22_21));
    printk("AFE_ADDA4_ULCF_CFG_24_2= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_24_23));
    printk("AFE_ADDA4_ULCF_CFG_26_2= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_26_25));
    printk("AFE_ADDA4_ULCF_CFG_28_2= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_28_27));
    printk("AFE_ADDA4_ULCF_CFG_30_2= 0x%x\n", Afe_Get_Reg(AFE_ADDA4_ULCF_CFG_30_29));
    AudDrv_Clk_Off();
    printk("-AudDrv Afe_Log_Print \n");
}




// export symbols for other module using
EXPORT_SYMBOL(Afe_Set_Reg);
EXPORT_SYMBOL(Afe_Get_Reg);

EXPORT_SYMBOL(GetClkCfg);
EXPORT_SYMBOL(SetClkCfg);

EXPORT_SYMBOL(GetInfraCfg);
EXPORT_SYMBOL(SetInfraCfg);

EXPORT_SYMBOL(Afe_Log_Print);


