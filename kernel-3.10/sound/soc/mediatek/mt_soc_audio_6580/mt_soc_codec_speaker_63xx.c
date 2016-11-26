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
 *   mtk_codec_speaker_63xx
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   Audio codec stub file
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

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_analog_type.h"

#include "mt_soc_pcm_common.h"

/* static DEFINE_MUTEX(Speaker_Ctrl_Mutex); */
/* static DEFINE_SPINLOCK(Speaker_lock); */
/* static int Speaker_Counter = 0; */
/* static bool  Speaker_Trim_init = false; */

void Speaker_ClassD_Open(void)
{
	kal_uint32 i, SPKTrimReg = 0;
	uint32 retyrcount = 10;
	uint32 WaitforReady = 0;

	pr_warn("%s\n", __func__);

	Ana_Set_Reg(SPK_CON2, 0x0214, 0xffff);	/* enable classAB OC function */
	/* Ana_Set_Reg(SPK_CON9, 0x0400, 0xffff); // Set Spk 6dB gain */
	Ana_Set_Reg(SPK_CON9, 0x2418, 0xffff);	/* Set Spk 6dB gain. VCM fast */

	/* enable SPK-Amp with 0dB gain, enable SPK amp offset triming, select class D mode */
	Ana_Set_Reg(SPK_CON0, 0x3008, 0xffff);
	Ana_Set_Reg(SPK_CON0, 0x3009, 0xffff);	/* Enable Class ABD */

	pr_warn("%s, SpkTrim Stage 0 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));
	do {
		/* msleep(5); */
		mdelay(5);
		WaitforReady = Ana_Get_Reg(SPK_CON1);
		WaitforReady = (WaitforReady & 0x8000);
		retyrcount--;
		pr_warn("%s, SpkTrim Stage 0.5 SPK_CON1 %x %d\n", __func__, WaitforReady,
		       retyrcount);
	} while ((WaitforReady == 0) && (retyrcount > 0));
	pr_warn("%s, SpkTrim Stage 1 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));


	Ana_Set_Reg(SPK_CON9, 0x0000, 0x0018);	/* Set Spk 6dB gain. VCM fast. */
	Ana_Set_Reg(SPK_CON0, 0x3001, 0xffff);	/* enable SPK AMP with 0dB gain, select Class D. enable Amp. */

	pr_warn("%s, SpkTrim Stage 2 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));

}

void Speaker_ClassD_close(void)
{
	pr_warn("%s\n", __func__);
	/* Mute Spk amp, select to original class AB mode. disable class-D Amp */
	Ana_Set_Reg(SPK_CON0, 0x0004, 0xffff);
}


void Speaker_ClassAB_Open(void)
{
	kal_uint32 i, SPKTrimReg = 0;
	uint32 retyrcount = 10;
	uint32 WaitforReady = 0;

	pr_warn("%s\n", __func__);

	Ana_Set_Reg(SPK_CON2, 0x0214, 0xffff);	/* enable classAB OC function */
	/* Ana_Set_Reg(SPK_CON9, 0x0400, 0xffff); // Set Spk 6dB gain */
	Ana_Set_Reg(SPK_CON9, 0x2418, 0xffff);	/* Set Spk 6dB gain. VCM fast */

	/* enable SPK-Amp with 0dB gain, enable SPK amp offset triming, select class D mode */
	Ana_Set_Reg(SPK_CON0, 0x3008, 0xffff);
	Ana_Set_Reg(SPK_CON0, 0x3009, 0xffff);	/* Enable Class ABD */

	pr_warn("%s, SpkTrim Stage 0 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));
	do {
		/* msleep(5); */
		mdelay(5);
		WaitforReady = Ana_Get_Reg(SPK_CON1);
		WaitforReady = (WaitforReady & 0x8000);
		retyrcount--;
		pr_warn("%s, SpkTrim Stage 0.5 SPK_CON1 %x %d\n", __func__, WaitforReady,
		       retyrcount);
	} while ((WaitforReady == 0) && (retyrcount > 0));
	pr_warn("%s, SpkTrim Stage 1 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));

	Ana_Set_Reg(SPK_CON0, 0x3008, 0xffff);
	Ana_Set_Reg(SPK_CON0, 0x3004, 0xffff);
	Ana_Set_Reg(SPK_CON9, 0x0000, 0x0018);	/* Set Spk 6dB gain. VCM fast. */
	Ana_Set_Reg(SPK_CON0, 0x3005, 0xffff);	/* enable SPK AMP with 0dB gain, select Class AB. enable Amp. */
	pr_warn("%s, SpkTrim Stage 2 SPK_CON1 %x\n", __func__, Ana_Get_Reg(SPK_CON1));

}

void Speaker_ClassAB_close(void)
{
	pr_warn("%s\n", __func__);
	/* Mute Spk amp, select to original class D mode. disable class-AB Amp */
	Ana_Set_Reg(SPK_CON0, 0x0000, 0xffff);
}

void Speaker_ReveiverMode_Open(void)
{
	pr_warn("%s\n", __func__);

	Ana_Set_Reg(SPK_CON2, 0x0614, 0xffff);	/* enable classAB OC function, enable speaker L receiver mode[6] */
	Ana_Set_Reg(SPK_CON9, 0x0100, 0xffff);
	/* Ana_Set_Reg(SPK_CON9, 0x0400, 0xffff); // Set Spk 6dB gain */

	/* enable SPK AMP with -6dB gain for 2in1 speaker, select Class AB. enable Amp. */
	Ana_Set_Reg(SPK_CON0, 0x1005, 0xffff);
}

void Speaker_ReveiverMode_close(void)
{
	/* Mute Spk amp, select to original class D mode. disable class-AB Amp */
	Ana_Set_Reg(SPK_CON0, 0x0000, 0xffff);
}

bool GetSpeakerOcFlag(void)
{
	unsigned int OCregister = 0;
	unsigned int bitmask = 1;
	bool DmodeFlag = false;
	bool ABmodeFlag = false;
	bool OCFlag = false;
#if 0
	Ana_Set_Reg(TOP_CKPDN_CON2_CLR, 0x3, 0xffff);
#endif
	OCregister = Ana_Get_Reg(SPK_CON6);
	DmodeFlag = OCregister & (bitmask << 14);	/* ; no.14 bit is SPK_D_OC_L_DEG */
	ABmodeFlag = OCregister & (bitmask << 15);	/* ; no.15 bit is SPK_AB_OC_L_DEG */
	pr_warn("OCregister = %d\n", OCregister);
	OCFlag = (DmodeFlag | ABmodeFlag);
	return OCFlag;
}
