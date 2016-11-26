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
 *   mtk_pcm_capture.c
 *
 * Project:
 * --------
 *   Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio Ul1 data1 uplink
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
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"

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

//information about
AFE_MEM_CONTROL_T  *TDM_VUL_Control_context;
static struct snd_dma_buffer *Capture_dma_buf  = NULL;
static AudioDigtalI2S *mAudioDigitalI2S = NULL;
static DEFINE_SPINLOCK(auddrv_ULInCtl_lock);

/*
 *    function implementation
 */
static void StartAudioCaptureHardware(struct snd_pcm_substream *substream);
static void StopAudioCaptureHardware(struct snd_pcm_substream *substream);
void StartAudioCaptureAnalogHardware(void);
void StopAudioCaptureAnalogHardware(void);
static int mtk_capture_probe(struct platform_device *pdev);
static int mtk_capture_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_capture_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_capture_probe(struct snd_soc_platform *platform);

#define MAX_PCM_DEVICES 4
#define MAX_PCM_SUBSTREAMS  128
#define MAX_MIDI_DEVICES

/* defaults */
#define UL1_MAX_BUFFER_SIZE     (48*1024)
#define MIN_PERIOD_SIZE     64
#define MAX_PERIOD_SIZE     UL1_MAX_BUFFER_SIZE
#define USE_FORMATS         (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE        SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        48000
#define USE_CHANNELS_MIN    1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     1024
#define USE_PERIODS_MAX     16*1024

static struct snd_pcm_hardware mtk_capture_hardware =
{
    .info = (SNDRV_PCM_INFO_INTERLEAVED),
    .formats =      USE_FORMATS,
    .rates =        USE_RATE,
    .rate_min =     USE_RATE_MIN,
    .rate_max =     USE_RATE_MAX,
    .channels_min =     USE_CHANNELS_MIN,
    .channels_max =     USE_CHANNELS_MAX,
    .buffer_bytes_max = UL1_MAX_BUFFER_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min =      1,
    .periods_max =      4096,
    .fifo_size =        0,
};

static void StopAudioCaptureHardware(struct snd_pcm_substream *substream)
{
    printk("StopAudioCaptureHardware \n");

    // here to set interrupt
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

    SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, false);
    if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false)
    {
        SetI2SAdcEnable(false);
    }

    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, false);

    // here to turn off digital part
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O09);
    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O10);

    EnableAfe(false);
}

static void ConfigAdcI2S(struct snd_pcm_substream *substream)
{
    mAudioDigitalI2S->mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
    mAudioDigitalI2S->mBuffer_Update_word = 8;
    mAudioDigitalI2S->mFpga_bit_test = 0;
    mAudioDigitalI2S->mFpga_bit = 0;
    mAudioDigitalI2S->mloopback = 0;
    mAudioDigitalI2S->mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
    mAudioDigitalI2S->mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
    mAudioDigitalI2S->mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
    mAudioDigitalI2S->mI2S_SAMPLERATE = (substream->runtime->rate);
}

static void StartAudioCaptureHardware(struct snd_pcm_substream *substream)
{
    printk("StartAudioCaptureHardware \n");

    ConfigAdcI2S(substream);
    SetI2SAdcIn(mAudioDigitalI2S);

    SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
    SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
    SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O09);
    SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O10);

    if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false)
    {
        SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);
        SetI2SAdcEnable(true);
    }
    else
    {
        SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);
    }

    // here to set interrupt
    SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->period_size);
    SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->rate);
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);

    SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL, substream->runtime->rate);
    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, true);
#if 0
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O09);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O10);

    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O10);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O09);
#endif
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O09);
    SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O10);

    EnableAfe(true);

}

static int mtk_capture_pcm_prepare(struct snd_pcm_substream *substream)
{
    printk("mtk_capture_pcm_prepare substream->rate = %d  substream->channels = %d \n", substream->runtime->rate, substream->runtime->channels);
    return 0;
}

static int mtk_capture_alsa_stop(struct snd_pcm_substream *substream)
{
    AFE_BLOCK_T *Vul_Block = &(TDM_VUL_Control_context->rBlock);
    printk("mtk_capture_alsa_stop \n");
    StopAudioCaptureHardware(substream);
    Vul_Block->u4DMAReadIdx  = 0;
    Vul_Block->u4WriteIdx  = 0;
    Vul_Block->u4DataRemained = 0;
    RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL, substream);
    return 0;
}

static kal_int32 Previous_Hw_cur = 0;
static snd_pcm_uframes_t mtk_capture_pcm_pointer(struct snd_pcm_substream *substream)
{
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    AFE_BLOCK_T *Awb_Block = &(TDM_VUL_Control_context->rBlock);
    PRINTK_AUD_UL1("mtk_capture_pcm_pointer Awb_Block->u4WriteIdx;= 0x%x \n", Awb_Block->u4WriteIdx);
    if (TDM_VUL_Control_context->interruptTrigger == 1)
    {

        Previous_Hw_cur = Awb_Block->u4WriteIdx;
        return Awb_Block->u4WriteIdx >> 2;

        HW_Cur_ReadIdx = Align64ByteSize(Afe_Get_Reg(AFE_AWB_CUR));
        if (HW_Cur_ReadIdx == 0)
        {
            PRINTK_AUD_UL1("[Auddrv] mtk_capture_pcm_pointer  HW_Cur_ReadIdx ==0 \n");
            HW_Cur_ReadIdx = Awb_Block->pucPhysBufAddr;
        }
        HW_memory_index = (HW_Cur_ReadIdx - Awb_Block->pucPhysBufAddr);
        Previous_Hw_cur = HW_memory_index;
        PRINTK_AUD_UL1("[Auddrv] mtk_capture_pcm_pointer =0x%x HW_memory_index = 0x%x\n", HW_Cur_ReadIdx, HW_memory_index);
        TDM_VUL_Control_context->interruptTrigger = 0;
        return (HW_memory_index >> 2);
    }
    return (Previous_Hw_cur >> 2);

}

static void SetVULBuffer(struct snd_pcm_substream *substream,
                         struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    AFE_BLOCK_T *pblock = &TDM_VUL_Control_context->rBlock;
    printk("SetVULBuffer\n");
    pblock->pucPhysBufAddr =  runtime->dma_addr;
    pblock->pucVirtBufAddr =  runtime->dma_area;
    pblock->u4BufferSize = runtime->dma_bytes;
    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;
    printk("u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
           pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    // set dram address top hardware
    Afe_Set_Reg(AFE_VUL_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_VUL_END  , pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

}

static int mtk_capture_pcm_hw_params(struct snd_pcm_substream *substream,
                                     struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
    int ret = 0;
    printk("mtk_capture_pcm_hw_params \n");

    dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
    dma_buf->dev.dev = substream->pcm->card->dev;
    dma_buf->private_data = NULL;
    printk("Capture_dma_buf = %p Capture_dma_buf->area = %p\n", Capture_dma_buf, Capture_dma_buf->area);

    if (Capture_dma_buf->area)
    {
        runtime->dma_bytes = params_buffer_bytes(hw_params);
        runtime->dma_area = Capture_dma_buf->area;
        runtime->dma_addr = Capture_dma_buf->addr;
    }
    else
    {
        printk("mtk_capture_pcm_hw_params snd_pcm_lib_malloc_pages\n");
        ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    }

    printk("mtk_capture_pcm_hw_params dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
           runtime->dma_bytes, runtime->dma_area, (long)runtime->dma_addr);

    printk("runtime->hw.buffer_bytes_max = %zu \n", runtime->hw.buffer_bytes_max);
    SetVULBuffer(substream, hw_params);

    printk("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
           substream->runtime->dma_bytes, substream->runtime->dma_area, (long)substream->runtime->dma_addr);
    return ret;
}

static int mtk_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
    printk("mtk_capture_pcm_hw_free \n");
    if (Capture_dma_buf->area)
    {
        return 0;
    }
    else
    {
        return snd_pcm_lib_free_pages(substream);
    }
}

/* Conventional and unconventional sample rate supported */
static unsigned int Vul1_supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000, 192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(Vul1_supported_sample_rates),
    .list = Vul1_supported_sample_rates,
};

static int mtk_capture_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    int ret = 0;
    AudDrv_Clk_On();
    AudDrv_ADC_Clk_On();

    printk("%s \n", __func__);
    TDM_VUL_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_VUL);

    runtime->hw = mtk_capture_hardware;
    memcpy((void *)(&(runtime->hw)), (void *)&mtk_capture_hardware , sizeof(struct snd_pcm_hardware));
    printk("runtime->hw->rates = 0x%x \n ", runtime->hw.rates);

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (ret < 0)
    {
        printk("snd_pcm_hw_constraint_integer failed\n");
    }

    printk("mtk_capture_pcm_open runtime rate = %d channels = %d \n", runtime->rate, runtime->channels);
    runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        printk("SNDRV_PCM_STREAM_CAPTURE mtkalsa_capture_constraints\n");
    }
    else
    {

    }

    if (ret < 0)
    {
        printk("mtk_capture_pcm_close\n");
        mtk_capture_pcm_close(substream);
        return ret;
    }
    printk("mtk_capture_pcm_open return\n");
    return 0;
}

static int mtk_capture_pcm_close(struct snd_pcm_substream *substream)
{
    AudDrv_ADC_Clk_Off();
    AudDrv_Clk_Off();
    return 0;
}

static int mtk_capture_alsa_start(struct snd_pcm_substream *substream)
{
    printk("mtk_capture_alsa_start \n");
    SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL, substream);
    StartAudioCaptureHardware(substream);
    return 0;
}

static int mtk_capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    printk("mtk_capture_pcm_trigger cmd = %d\n", cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
            return mtk_capture_alsa_start(substream);
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            return mtk_capture_alsa_stop(substream);
    }
    return -EINVAL;
}

static bool CheckNullPointer(void *pointer)
{
    if (pointer == NULL)
    {
        printk("CheckNullPointer pointer = NULL");
        return true;
    }
    return false;
}

static int mtk_capture_pcm_copy(struct snd_pcm_substream *substream,
                                int channel, snd_pcm_uframes_t pos,
                                void __user *dst, snd_pcm_uframes_t count)
{

    AFE_MEM_CONTROL_T *pVUL_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Vul_Block = NULL;
    char *Read_Data_Ptr = (char *)dst;
    ssize_t DMA_Read_Ptr = 0 , read_size = 0, read_count = 0;
    unsigned long flags;

    PRINTK_AUD_UL1("mtk_capture_pcm_copy pos = %lucount = %lu \n ", pos, count);
    count = Align64ByteSize(count << 2);

    // check which memif nned to be write
    pVUL_MEM_ConTrol = TDM_VUL_Control_context;
    Vul_Block = &(pVUL_MEM_ConTrol->rBlock);

    if (pVUL_MEM_ConTrol == NULL)
    {
        printk("cannot find MEM control !!!!!!!\n");
        msleep(50);
        return 0;
    }

    if (Vul_Block->u4BufferSize <= 0)
    {
        msleep(50);
        printk("Vul_Block->u4BufferSize <= 0  =%d\n", Vul_Block->u4BufferSize);
        return 0;
    }

    if (CheckNullPointer((void *)Vul_Block->pucVirtBufAddr))
    {
        printk("CheckNullPointer  pucVirtBufAddr = %p\n", Vul_Block->pucVirtBufAddr);
        return 0;
    }

    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
    if (Vul_Block->u4DataRemained >  Vul_Block->u4BufferSize)
    {
        PRINTK_AUD_UL1("AudDrv_MEMIF_Read u4DataRemained=%x > u4BufferSize=%x" , Vul_Block->u4DataRemained, Vul_Block->u4BufferSize);
        Vul_Block->u4DataRemained = 0;
        Vul_Block->u4DMAReadIdx   = Vul_Block->u4WriteIdx;
    }
    if (count >  Vul_Block->u4DataRemained)
    {
        read_size = Vul_Block->u4DataRemained;
    }
    else
    {
        read_size = count;
    }

    DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

    PRINTK_AUD_UL1("AudDrv_MEMIF_Read finish0, read_count:%x, read_size:%x, u4DataRemained:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x \r\n",
                   (unsigned int)read_count, (unsigned int)read_size, Vul_Block->u4DataRemained, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx);

    if (DMA_Read_Ptr + read_size < Vul_Block->u4BufferSize)
    {
        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {
            printk("AudDrv_MEMIF_Read 1, read_size:%zu, DataRemained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
                   read_size, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }

        if (copy_to_user((void __user *)Read_Data_Ptr, (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size))
        {

            printk("AudDrv_MEMIF_Read Fail 1 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:%zu,read_size:%zu", Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }

        read_count += read_size;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= read_size;
        Vul_Block->u4DMAReadIdx += read_size;
        Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);

        Read_Data_Ptr += read_size;
        count -= read_size;

        PRINTK_AUD_UL1("AudDrv_MEMIF_Read finish1, copy size:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x, u4DataRemained:%x \r\n",
                       (unsigned int)read_size, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
    }

    else
    {
        uint32 size_1 = Vul_Block->u4BufferSize - DMA_Read_Ptr;
        uint32 size_2 = read_size - size_1;

        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {

            printk("AudDrv_MEMIF_Read 2, read_size1:0x%x, DataRemained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
                   size_1, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)Read_Data_Ptr, (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_1))
        {

            printk("AudDrv_MEMIF_Read Fail 2 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:%zu,read_size:%zu",
                   Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }

        read_count += size_1;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= size_1;
        Vul_Block->u4DMAReadIdx += size_1;
        Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);


        PRINTK_AUD_UL1("AudDrv_MEMIF_Read finish2, copy size_1:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x, u4DataRemained:%x \r\n",
                       size_1, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);

        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {

            printk("AudDrv_AWB_Read 3, read_size2:%x, DataRemained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
                   size_2, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)(Read_Data_Ptr + size_1), (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2))
        {

            printk("AudDrv_MEMIF_Read Fail 3 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x , DMA_Read_Ptr:%zu, read_size:%zu", Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return read_count << 2;
        }

        read_count += size_2;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= size_2;
        Vul_Block->u4DMAReadIdx += size_2;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);

        count -= read_size;
        Read_Data_Ptr += read_size;

        PRINTK_AUD_UL1("AudDrv_MEMIF_Read finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
                       size_2, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
    }

    return read_count >> 2;
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
                                   int channel, snd_pcm_uframes_t pos,
                                   snd_pcm_uframes_t count)
{
    printk("dummy_pcm_silence \n");
    return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_capture_pcm_page(struct snd_pcm_substream *substream,
                                         unsigned long offset)
{
    printk("%s \n", __func__);
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_afe_capture_ops =
{
    .open =     mtk_capture_pcm_open,
    .close =    mtk_capture_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_capture_pcm_hw_params,
    .hw_free =  mtk_capture_pcm_hw_free,
    .prepare =  mtk_capture_pcm_prepare,
    .trigger =  mtk_capture_pcm_trigger,
    .pointer =  mtk_capture_pcm_pointer,
    .copy =     mtk_capture_pcm_copy,
    .silence =  mtk_capture_pcm_silence,
    .page =     mtk_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform =
{
    .ops        = &mtk_afe_capture_ops,
    .pcm_new    = mtk_asoc_capture_pcm_new,
    .probe      = mtk_afe_capture_probe,
};

static int mtk_capture_probe(struct platform_device *pdev)
{
    printk("tdm mtk_capture_probe\n");

    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
    if (!pdev->dev.dma_mask)
    {
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_TDMRX_PCM);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_platform);
}

static int mtk_asoc_capture_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    printk("mtk_asoc_capture_pcm_new \n");
    return 0;
}


static int mtk_afe_capture_probe(struct snd_soc_platform *platform)
{
    printk("mtk_afe_capture_probe TODO\n");
    AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_VUL, UL1_MAX_BUFFER_SIZE);
    Capture_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_VUL);
    /*
    Capture_dma_buf = kmalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
    memset((void *)Capture_dma_buf, 0, sizeof(struct snd_dma_buffer));
    printk("mtk_afe_capture_probe dma_alloc_coherent\n");

    Capture_dma_buf->area = dma_alloc_coherent(platform->dev,
                                               UL1_MAX_BUFFER_SIZE,
                                               &Capture_dma_buf->addr, GFP_KERNEL);

    if (Capture_dma_buf->area)
    {
        Capture_dma_buf->bytes = UL1_MAX_BUFFER_SIZE;
    }
    */
    mAudioDigitalI2S =  kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
    return 0;
}


static int mtk_capture_remove(struct platform_device *pdev)
{
    pr_debug("%s\n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_tdm_capture_of_ids[] =
{
    { .compatible = "mediatek,mt_soc_tdm_capture", },
    {}
};
#endif

static struct platform_driver mtk_afe_capture_driver =
{
    .driver = {
        .name = MT_SOC_TDMRX_PCM,
        .owner = THIS_MODULE,
        #ifdef CONFIG_OF
        .of_match_table = mt_soc_tdm_capture_of_ids,
        #endif        
    },
    .probe = mtk_capture_probe,
    .remove = mtk_capture_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_capture_dev;
#endif

static int __init mtk_soc_capture_platform_init(void)
{
    int ret = 0;
    printk("%s\n", __func__);
	#ifndef CONFIG_OF
    soc_mtkafe_capture_dev = platform_device_alloc(MT_SOC_TDMRX_PCM, -1);
    if (!soc_mtkafe_capture_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkafe_capture_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkafe_capture_dev);
        return ret;
    }
    #endif
    ret = platform_driver_register(&mtk_afe_capture_driver);
    return ret;
}
module_init(mtk_soc_capture_platform_init);

static void __exit mtk_soc_platform_exit(void)
{

    printk("%s\n", __func__);
    platform_driver_unregister(&mtk_afe_capture_driver);
}

module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");


