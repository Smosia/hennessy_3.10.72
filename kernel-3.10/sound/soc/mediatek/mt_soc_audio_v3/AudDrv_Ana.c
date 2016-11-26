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
 *   AudDrv_Ana.c
 *
 * Project:
 * --------
 *   MT6583  Audio Driver ana Register setting
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
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"

/* define this to use wrapper to control */
#define AUDIO_USING_WRAP_DRIVER
#ifdef AUDIO_USING_WRAP_DRIVER
#include <mach/mt_pmic_wrap.h>
#endif

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

void Ana_Set_Reg(uint32 offset, uint32 value, uint32 mask)
{
	/* set pmic register or analog CONTROL_IFACE_PATH */
	int ret = 0;
	uint32 Reg_Value;

	PRINTK_ANA_REG("Ana_Set_Reg offset= 0x%x , value = 0x%x mask = 0x%x\n", offset, value,
		       mask);
#ifdef AUDIO_USING_WRAP_DRIVER
	Reg_Value = Ana_Get_Reg(offset);
	Reg_Value &= (~mask);
	Reg_Value |= (value & mask);
	ret = pwrap_write(offset, Reg_Value);
	Reg_Value = Ana_Get_Reg(offset);
	if ((Reg_Value & mask) != (value & mask))
		pr_debug("Ana_Set_Reg  mask = 0x%x ret = %d Reg_Value = 0x%x\n", mask, ret,
			 Reg_Value);
#endif
}
EXPORT_SYMBOL(Ana_Set_Reg);

uint32 Ana_Get_Reg(uint32 offset)
{
	/* get pmic register */
	int ret = 0;
	uint32 Rdata = 0;
#ifdef AUDIO_USING_WRAP_DRIVER
	ret = pwrap_read(offset, &Rdata);
#endif
	PRINTK_ANA_REG("Ana_Get_Reg offset=0x%x,Rdata=0x%x,ret=%d\n", offset, Rdata, ret);
	return Rdata;
}
EXPORT_SYMBOL(Ana_Get_Reg);

void Ana_Log_Print(void)
{
	AudDrv_ANA_Clk_On();
	pr_warn("AFE_UL_DL_CON0	= 0x%x\n", Ana_Get_Reg(AFE_UL_DL_CON0));
	pr_warn("AFE_DL_SRC2_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_H));
	pr_warn("AFE_DL_SRC2_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_L));
	pr_warn("AFE_DL_SDM_CON0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON0));
	pr_warn("AFE_DL_SDM_CON1  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON1));
	pr_warn("AFE_UL_SRC0_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC0_CON0_H));
	pr_warn("AFE_UL_SRC0_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC0_CON0_L));
	pr_warn("AFE_UL_SRC1_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC1_CON0_H));
	pr_warn("AFE_UL_SRC1_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_UL_SRC1_CON0_L));
	pr_warn("PMIC_AFE_TOP_CON0  = 0x%x\n", Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	pr_warn("AFE_AUDIO_TOP_CON0	= 0x%x\n", Ana_Get_Reg(AFE_AUDIO_TOP_CON0));
	pr_warn("PMIC_AFE_TOP_CON0  = 0x%x\n", Ana_Get_Reg(PMIC_AFE_TOP_CON0));
	pr_warn("AFE_DL_SRC_MON0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SRC_MON0));
	pr_warn("AFE_DL_SDM_TEST0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_TEST0));
	pr_warn("AFE_MON_DEBUG0	= 0x%x\n", Ana_Get_Reg(AFE_MON_DEBUG0));
	pr_warn("AFUNC_AUD_CON0	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON0));
	pr_warn("AFUNC_AUD_CON1	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON1));
	pr_warn("AFUNC_AUD_CON2	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON2));
	pr_warn("AFUNC_AUD_CON3	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON3));
	pr_warn("AFUNC_AUD_CON4	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON4));
	pr_warn("AFUNC_AUD_MON0	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_MON0));
	pr_warn("AFUNC_AUD_MON1	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_MON1));
	pr_warn("AUDRC_TUNE_MON0  = 0x%x\n", Ana_Get_Reg(AUDRC_TUNE_MON0));
	pr_warn("AFE_UP8X_FIFO_CFG0	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_CFG0));
	pr_warn("AFE_UP8X_FIFO_LOG_MON0	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON0));
	pr_warn("AFE_UP8X_FIFO_LOG_MON1	= 0x%x\n", Ana_Get_Reg(AFE_UP8X_FIFO_LOG_MON1));
	pr_warn("AFE_DL_DC_COMP_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG0));
	pr_warn("AFE_DL_DC_COMP_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG1));
	pr_warn("AFE_DL_DC_COMP_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_DL_DC_COMP_CFG2));
	pr_warn("AFE_PMIC_NEWIF_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0));
	pr_warn("AFE_PMIC_NEWIF_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1));
	pr_warn("AFE_PMIC_NEWIF_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2));
	pr_warn("AFE_PMIC_NEWIF_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3));
	pr_warn("AFE_SGEN_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG0));
	pr_warn("AFE_SGEN_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG1));
	pr_warn("AFE_VOW_TOP  = 0x%x\n", Ana_Get_Reg(AFE_VOW_TOP));
	pr_warn("AFE_VOW_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG0));
	pr_warn("AFE_VOW_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG1));
	pr_warn("AFE_VOW_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG2));
	pr_warn("AFE_VOW_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG3));
	pr_warn("AFE_VOW_CFG4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG4));
	pr_warn("AFE_VOW_CFG5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_CFG5));
	pr_warn("AFE_VOW_MON0  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON0));
	pr_warn("AFE_VOW_MON1  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON1));
	pr_warn("AFE_VOW_MON2  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON2));
	pr_warn("AFE_VOW_MON3  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON3));
	pr_warn("AFE_VOW_MON4  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON4));
	pr_warn("AFE_VOW_MON5  = 0x%x\n", Ana_Get_Reg(AFE_VOW_MON5));

	pr_warn("AFE_DCCLK_CFG0	= 0x%x\n", Ana_Get_Reg(AFE_DCCLK_CFG0));
	pr_warn("AFE_DCCLK_CFG1	= 0x%x\n", Ana_Get_Reg(AFE_DCCLK_CFG1));

	pr_warn("TOP_CON  = 0x%x\n", Ana_Get_Reg(TOP_CON));
	pr_warn("TOP_STATUS	= 0x%x\n", Ana_Get_Reg(TOP_STATUS));
	pr_warn("TOP_CKPDN_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON0));
	pr_warn("TOP_CKPDN_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON1));
	pr_warn("TOP_CKPDN_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON2));
	pr_warn("TOP_CKPDN_CON3	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON3));
	pr_warn("TOP_CKSEL_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON0));
	pr_warn("TOP_CKSEL_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON1));
	pr_warn("TOP_CKSEL_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKSEL_CON2));
	pr_warn("TOP_CKDIVSEL_CON  = 0x%x\n", Ana_Get_Reg(TOP_CKDIVSEL_CON));
	pr_warn("TOP_CKHWEN_CON	= 0x%x\n", Ana_Get_Reg(TOP_CKHWEN_CON));
	pr_warn("TOP_CKTST_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON0));
	pr_warn("TOP_CKTST_CON1	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON1));
	pr_warn("TOP_CKTST_CON2	= 0x%x\n", Ana_Get_Reg(TOP_CKTST_CON2));
	pr_warn("TOP_CLKSQ  = 0x%x\n", Ana_Get_Reg(TOP_CLKSQ));
	pr_warn("TOP_RST_CON0  = 0x%x\n", Ana_Get_Reg(TOP_RST_CON0));
	pr_warn("TEST_CON0  = 0x%x\n", Ana_Get_Reg(TEST_CON0));
	pr_warn("TEST_OUT  = 0x%x\n", Ana_Get_Reg(TEST_OUT));
	pr_warn("AFE_MON_DEBUG0= 0x%x\n", Ana_Get_Reg(AFE_MON_DEBUG0));
	pr_warn("ZCD_CON0  = 0x%x\n", Ana_Get_Reg(ZCD_CON0));
	pr_warn("ZCD_CON1  = 0x%x\n", Ana_Get_Reg(ZCD_CON1));
	pr_warn("ZCD_CON2  = 0x%x\n", Ana_Get_Reg(ZCD_CON2));
	pr_warn("ZCD_CON3  = 0x%x\n", Ana_Get_Reg(ZCD_CON3));
	pr_warn("ZCD_CON4  = 0x%x\n", Ana_Get_Reg(ZCD_CON4));
	pr_warn("ZCD_CON5  = 0x%x\n", Ana_Get_Reg(ZCD_CON5));
	pr_warn("LDO_CON1  = 0x%x\n", Ana_Get_Reg(LDO_CON1));
	pr_warn("LDO_CON2  = 0x%x\n", Ana_Get_Reg(LDO_CON2));

	pr_warn("LDO_VCON1  = 0x%x\n", Ana_Get_Reg(LDO_VCON1));
	pr_warn("SPK_CON0  = 0x%x\n", Ana_Get_Reg(SPK_CON0));
	pr_warn("SPK_CON1  = 0x%x\n", Ana_Get_Reg(SPK_CON1));
	pr_warn("SPK_CON2  = 0x%x\n", Ana_Get_Reg(SPK_CON2));
	pr_warn("SPK_CON3  = 0x%x\n", Ana_Get_Reg(SPK_CON3));
	pr_warn("SPK_CON4  = 0x%x\n", Ana_Get_Reg(SPK_CON4));
	pr_warn("SPK_CON5  = 0x%x\n", Ana_Get_Reg(SPK_CON5));
	pr_warn("SPK_CON6  = 0x%x\n", Ana_Get_Reg(SPK_CON6));
	pr_warn("SPK_CON7  = 0x%x\n", Ana_Get_Reg(SPK_CON7));
	pr_warn("SPK_CON8  = 0x%x\n", Ana_Get_Reg(SPK_CON8));
	pr_warn("SPK_CON9  = 0x%x\n", Ana_Get_Reg(SPK_CON9));
	pr_warn("SPK_CON10  = 0x%x\n", Ana_Get_Reg(SPK_CON10));
	pr_warn("SPK_CON11  = 0x%x\n", Ana_Get_Reg(SPK_CON11));
	pr_warn("SPK_CON12  = 0x%x\n", Ana_Get_Reg(SPK_CON12));
	pr_warn("SPK_CON13  = 0x%x\n", Ana_Get_Reg(SPK_CON13));
	pr_warn("SPK_CON14  = 0x%x\n", Ana_Get_Reg(SPK_CON14));
	pr_warn("SPK_CON15  = 0x%x\n", Ana_Get_Reg(SPK_CON15));
	pr_warn("SPK_CON16  = 0x%x\n", Ana_Get_Reg(SPK_CON16));
	pr_warn("SPK_ANA_CON0  = 0x%x\n", Ana_Get_Reg(SPK_ANA_CON0));
	pr_warn("SPK_ANA_CON1  = 0x%x\n", Ana_Get_Reg(SPK_ANA_CON1));
	pr_warn("SPK_ANA_CON3  = 0x%x\n", Ana_Get_Reg(SPK_ANA_CON3));
	pr_warn("AUDDEC_ANA_CON0  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON0));
	pr_warn("AUDDEC_ANA_CON1  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON1));
	pr_warn("AUDDEC_ANA_CON2  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON2));
	pr_warn("AUDDEC_ANA_CON3  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON3));
	pr_warn("AUDDEC_ANA_CON4  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON4));
	pr_warn("AUDDEC_ANA_CON5  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON5));
	pr_warn("AUDDEC_ANA_CON6  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON6));
	pr_warn("AUDDEC_ANA_CON7  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON7));
	pr_warn("AUDDEC_ANA_CON8  = 0x%x\n", Ana_Get_Reg(AUDDEC_ANA_CON8));

	pr_warn("AUDENC_ANA_CON0  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON0));
	pr_warn("AUDENC_ANA_CON1  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON1));
	pr_warn("AUDENC_ANA_CON2  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON2));
	pr_warn("AUDENC_ANA_CON3  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON3));
	pr_warn("AUDENC_ANA_CON4  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON4));
	pr_warn("AUDENC_ANA_CON5  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON5));
	pr_warn("AUDENC_ANA_CON6  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON6));
	pr_warn("AUDENC_ANA_CON7  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON7));
	pr_warn("AUDENC_ANA_CON8  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON8));
	pr_warn("AUDENC_ANA_CON9  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON9));
#ifdef _VOW_ENABLE
	pr_warn("AUDENC_ANA_CON11  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON11));
	pr_warn("AUDENC_ANA_CON12  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON12));
	pr_warn("AUDENC_ANA_CON13  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON13));
	pr_warn("AUDENC_ANA_CON14  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON14));
#endif
	pr_warn("AUDENC_ANA_CON10  = 0x%x\n", Ana_Get_Reg(AUDENC_ANA_CON10));

	pr_warn("AUDNCP_CLKDIV_CON0	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON0));
	pr_warn("AUDNCP_CLKDIV_CON1	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON1));
	pr_warn("AUDNCP_CLKDIV_CON2	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON2));
	pr_warn("AUDNCP_CLKDIV_CON3	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON3));
	pr_warn("AUDNCP_CLKDIV_CON4	= 0x%x\n", Ana_Get_Reg(AUDNCP_CLKDIV_CON4));

	pr_warn("TOP_CKPDN_CON0	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CON0));
	pr_warn("GPIO_MODE3	= 0x%x\n", Ana_Get_Reg(GPIO_MODE3));
	AudDrv_ANA_Clk_Off();
	pr_warn("-Ana_Log_Print\n");
}
EXPORT_SYMBOL(Ana_Log_Print);


/* export symbols for other module using */
