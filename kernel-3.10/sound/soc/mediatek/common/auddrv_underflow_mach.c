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
 *   AudDrv_devtree_parser.c
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   AudDrv_devtree_parser
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

#include <linux/aee.h>
#include "auddrv_underflow_mach.h"

#define UnderflowrecordNumber (20)

static bool bEnableDump = false;
// default setting for samplerate and interrupt count
static unsigned int mDlSamplerate =44100;
static const unsigned int DlSampleRateUpperBound = 192000;
static unsigned int InterruptSample =1024;
static const unsigned int InterruptSampleUpperBound = 192000;
//static bool bDumpInit = false;


static unsigned int mDL1InterruptInterval = 0;
static unsigned int mDL1_Interrupt_Interval_Limit = 0;
static unsigned int mDl1Numerator =  8;   //1.6
static unsigned int mDl1denominator = 5;
static unsigned long long Irq_time_t1 = 0, Irq_time_t2 = 0;
static bool bAudioInterruptChange = false;

static unsigned long long UnderflowTime[UnderflowrecordNumber] = {0};
static unsigned int UnderflowCounter = 0;
static unsigned int UnderflowThreshold =3;
static void ClearInterruptTiming(void);
static void DumpUnderFlowTime(void);
static void ClearUnderFlowTime(void);

void Auddrv_Aee_Dump(void);
void Auddrv_Set_Interrupt_Changed(bool bChange);


void SetUnderFlowThreshold(unsigned int Threshold)
{
    UnderflowThreshold = Threshold;
}


// base on devtree name to pares dev tree.
void Auddrv_Aee_Dump(void)
{
	/*pr_warn("+%s\n", __func__);*/
    if(bEnableDump == true)
    {
        aee_kernel_exception_api(__FILE__, __LINE__, DB_OPT_FTRACE, "Audio is blocked", "audio blocked dump ftrace");
    }
    Auddrv_Reset_Dump_State();
	/*pr_warn("-%s\n", __func__);*/
}


/*
/    dump underflow time in kernel
*/
static void DumpUnderFlowTime(void)
{
    int i=0;
	pr_warn("%s\n", __func__);
    for(i=0; i < UnderflowrecordNumber ; i++)
    {
	pr_warn("UnderflowTime[%d] = %llu\n", i, UnderflowTime[i]);
    }
}


/*
/    when pcm playback is close , need to call this function to clear record.
*/

void Auddrv_Set_UnderFlow(void)
{
    unsigned long long underflow_time =sched_clock(); // in ns (10^9)
	pr_warn("%s UnderflowCounter = %d\n", __func__, UnderflowCounter);
    UnderflowTime[UnderflowCounter] =underflow_time;
    UnderflowCounter++;
    UnderflowCounter%= UnderflowrecordNumber;
    if(UnderflowCounter > UnderflowThreshold)
    {
	/*DumpUnderFlowTime();*/
        Auddrv_Aee_Dump();
    }
}

/*
/    dump underflow time in kernel
*/

static void ClearUnderFlowTime(void)
{
    int i=0;
    for(i=0; i < UnderflowrecordNumber ; i++)
    {
        UnderflowTime[i]= 0;
    }
    UnderflowCounter =0;
}


/*
/    when pcm playback is close , need to call this function to clear record.
*/

void Auddrv_Reset_Dump_State(void)
{
    ClearUnderFlowTime();
    Auddrv_Set_Interrupt_Changed(false);
    ClearInterruptTiming();
}

/*
/    when InterruptSample or mDlSamplerate is change , nned to refine mDL1InterruptInterval
*/

void Auddrv_Set_Interrupt_Changed(bool bChange)
{
     bAudioInterruptChange =bChange;
}

/*
/    when in interrupt , call this function to check irq timing
*/
void Auddrv_CheckInterruptTiming(void)
{
    if (Irq_time_t1 == 0)
    {
        Irq_time_t1 = sched_clock(); // in ns (10^9)
    }
    else
    {
        Irq_time_t2 = Irq_time_t1;
        Irq_time_t1 = sched_clock(); // in ns (10^9)
        if(bAudioInterruptChange == true)
        {
            // for Audio Interupt is change , so this interrupt interval may not clear.clearqueue
            ClearInterruptTiming();
            return;
        }
        if ((Irq_time_t1 > Irq_time_t2) && mDL1_Interrupt_Interval_Limit)
        {
		pr_warn("CheckInterruptTiming  t2 = %llu t1 = %llu t1 - t2 = %llu limit = %d\n",
		Irq_time_t2, Irq_time_t1, Irq_time_t1 - Irq_time_t2, mDL1_Interrupt_Interval_Limit);
            Irq_time_t2 = Irq_time_t1 - Irq_time_t2;
            if (Irq_time_t2 > mDL1_Interrupt_Interval_Limit * 1000000)
            {
		pr_warn("CheckInterruptTiming int may be blocked t2 = %llu limit = %d\n",
			Irq_time_t2, mDL1_Interrupt_Interval_Limit);
            }
        }
    }
}

static void ClearInterruptTiming(void)
{
	/*pr_warn("%s\n", __func__);*/
    Irq_time_t1 = 0;
    Irq_time_t2 = 0;
}


/*
/    when InterruptSample or mDlSamplerate is change , nned to refine mDL1InterruptInterval
*/

void RefineInterrruptInterval(void)
{
    mDL1InterruptInterval = ((InterruptSample * 1000) / mDlSamplerate) + 1;
    mDL1_Interrupt_Interval_Limit = mDL1InterruptInterval*mDl1Numerator /mDl1denominator;
	pr_warn("%s interval = %d limit = %d\n", __func__, mDL1InterruptInterval, mDL1_Interrupt_Interval_Limit);
}

/*
/    funtion to set DL sampleRate
*/

bool Auddrv_Set_DlSamplerate(unsigned int Samplerate)
{
	pr_warn("%s Samplerate = %d\n", __func__, Samplerate);
    if(Samplerate < DlSampleRateUpperBound)
    {
        mDlSamplerate = Samplerate;
        RefineInterrruptInterval();
    }
    return true;
}

bool Auddrv_Set_InterruptSample(unsigned int count)
{
	pr_warn("%s count = %d\n", __func__, count);
    if(count < InterruptSampleUpperBound)
    {
        InterruptSample = count;
        RefineInterrruptInterval();
        Auddrv_Set_Interrupt_Changed(true);
    }
    return true;
}

/*
/    funtion to enable / disable dump , only enable will arised aee
*/
bool Auddrv_Enable_dump(bool bEnable)
{
	pr_warn("%s bEnable = %d\n", __func__, bEnable);
    bEnableDump = bEnable;
    return true;
}

