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

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_pcm_common.h"

/* information about */
AFE_MEM_CONTROL_T  *VUL_Control_context;
static struct snd_dma_buffer *Capture_dma_buf  = NULL;
static AudioDigtalI2S *mAudioDigitalI2S = NULL;
static bool mCaptureUseSram = false;
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

static struct snd_pcm_hardware mtk_capture_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats =      SND_SOC_STD_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = UL1_MAX_BUFFER_SIZE,
	.period_bytes_max = UL1_MAX_BUFFER_SIZE,
	.periods_min = UL1_MIN_PERIOD_SIZE,
	.periods_max = UL1_MAX_PERIOD_SIZE,
	.fifo_size = 0,
};

static void StopAudioCaptureHardware(struct snd_pcm_substream *substream)
{
	pr_warn("StopAudioCaptureHardware\n");

	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, false);
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false)
		SetI2SAdcEnable(false);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, false);

	/* here to set interrupt */
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

	/* here to turn off digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O09);
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O10);

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
	pr_warn("StartAudioCaptureHardware\n");

	ConfigAdcI2S(substream);
	SetI2SAdcIn(mAudioDigitalI2S);

	SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
	SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O09);
	SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O10);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false) {
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);
		SetI2SAdcEnable(true);
	} else
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, true);

	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O09);
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O10);

	/* here to set interrupt */
	SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->period_size);
	SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->rate);
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL, substream->runtime->rate);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, true);

	EnableAfe(true);
}

static int mtk_capture_pcm_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int mtk_capture_alsa_stop(struct snd_pcm_substream *substream)
{
	/* AFE_BLOCK_T *Vul_Block = &(VUL_Control_context->rBlock); */
	pr_warn("mtk_capture_alsa_stop\n");
	StopAudioCaptureHardware(substream);
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL, substream);
	return 0;
}

static snd_pcm_uframes_t mtk_capture_pcm_pointer(struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	/* kal_uint32 Frameidx = 0; */
	kal_int32 Hw_Get_bytes = 0;
	bool bIsOverflow = false;
	unsigned long flags;
	AFE_BLOCK_T *UL1_Block = &(VUL_Control_context->rBlock);

	Auddrv_UL1_Spinlock_lock();
	spin_lock_irqsave(&VUL_Control_context->substream_lock, flags);
	PRINTK_AUD_UL1("mtk_capture_pcm_pointer UL1_Block->u4WriteIdx= 0x%x, u4DataRemained=0x%x\n",
		 UL1_Block->u4WriteIdx, UL1_Block->u4DataRemained);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL) == true) {

		HW_Cur_ReadIdx = Align64ByteSize(Afe_Get_Reg(AFE_VUL_CUR));
		if (HW_Cur_ReadIdx == 0) {
			PRINTK_AUD_UL1("[Auddrv] mtk_capture_pcm_pointer  HW_Cur_ReadIdx ==0\n");
			HW_Cur_ReadIdx = UL1_Block->pucPhysBufAddr;
		}
		HW_memory_index = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr);

		/* update for data get to hardware */
		Hw_Get_bytes = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr) - UL1_Block->u4WriteIdx;
		if (Hw_Get_bytes < 0)
			Hw_Get_bytes += UL1_Block->u4BufferSize;

		UL1_Block->u4WriteIdx += Hw_Get_bytes;
		UL1_Block->u4WriteIdx %= UL1_Block->u4BufferSize;
		UL1_Block->u4DataRemained += Hw_Get_bytes;
		if (UL1_Block->u4DataRemained > UL1_Block->u4BufferSize) {
			bIsOverflow = true;
			pr_warn("%s buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, DataRemained:%x, BufferSize:%x\n",
			       __func__, UL1_Block->u4DMAReadIdx, UL1_Block->u4WriteIdx,
			       UL1_Block->u4DataRemained, UL1_Block->u4BufferSize);

			/* reset pointer info */
			UL1_Block->u4DataRemained = 0;
			UL1_Block->u4DMAReadIdx = UL1_Block->u4WriteIdx;
		}

		PRINTK_AUD_UL1("[Auddrv] mtk_capture_pcm_pointer =0x%x HW_memory_index = 0x%x\n",
			 HW_Cur_ReadIdx, HW_memory_index);

		spin_unlock_irqrestore(&VUL_Control_context->substream_lock, flags);
		Auddrv_UL1_Spinlock_unlock();

		if (bIsOverflow == true)
			return -1;

		return audio_bytes_to_frame(substream, HW_memory_index);
	}
	spin_unlock_irqrestore(&VUL_Control_context->substream_lock, flags);
	Auddrv_UL1_Spinlock_unlock();
	return 0;

}

static void SetVULBuffer(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	AFE_BLOCK_T *pblock = &VUL_Control_context->rBlock;
	pr_warn("SetVULBuffer\n");
	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;
	pr_warn("u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
	       pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	/* set dram address top hardware */
	Afe_Set_Reg(AFE_VUL_BASE, pblock->pucPhysBufAddr, 0xffffffff);
	Afe_Set_Reg(AFE_VUL_END, pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

}

static int mtk_capture_pcm_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	int ret = 0;
	pr_warn("mtk_capture_pcm_hw_params\n");

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;

	if (mCaptureUseSram == true) {
		runtime->dma_bytes = params_buffer_bytes(hw_params);
		pr_warn("mtk_capture_pcm_hw_params mCaptureUseSram dma_bytes = %zu\n", runtime->dma_bytes);
		substream->runtime->dma_area = (unsigned char *)Get_Afe_SramBase_Pointer();
		substream->runtime->dma_addr = Get_Afe_Sram_Phys_Addr();
        SetHighAddr(Soc_Aud_Digital_Block_MEM_VUL,false);
	} else if (Capture_dma_buf->area) {
		pr_warn("Capture_dma_buf = %p Capture_dma_buf->area = %p apture_dma_buf->addr = 0x%lx\n",
		       Capture_dma_buf, Capture_dma_buf->area, (long) Capture_dma_buf->addr);
		runtime->dma_bytes = params_buffer_bytes(hw_params);
		runtime->dma_area = Capture_dma_buf->area;
		runtime->dma_addr = Capture_dma_buf->addr;
        SetHighAddr(Soc_Aud_Digital_Block_MEM_VUL,true);
	} else {
		pr_warn("mtk_capture_pcm_hw_params snd_pcm_lib_malloc_pages\n");
		ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
	}

	SetVULBuffer(substream, hw_params);

	pr_warn("mtk_capture_pcm_hw_params dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       substream->runtime->dma_bytes, substream->runtime->dma_area, (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_capture_pcm_hw_free\n");
	if (Capture_dma_buf->area)
		return 0;
	else
		return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_normal_supported_sample_rates),
	.list = soc_normal_supported_sample_rates,
};

static int mtk_capture_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;
	AudDrv_Clk_On();
	AudDrv_ADC_Clk_On();
	VUL_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_VUL);

	/* can allocate sram_dbg */
	AfeControlSramLock();
	if (GetSramState() ==  SRAM_STATE_FREE) {
		pr_warn("mtk_capture_pcm_open use sram\n");
		mtk_capture_hardware.buffer_bytes_max = GetCaptureSramSize();
		SetSramState(SRAM_STATE_CAPTURE);
		mCaptureUseSram = true;
	} else {
		pr_warn("mtk_capture_pcm_open use dram\n");
		mtk_capture_hardware.buffer_bytes_max = UL1_MAX_BUFFER_SIZE;
	}
	AfeControlSramUnLock();

	runtime->hw = mtk_capture_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_capture_hardware , sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	if (ret < 0) {
		pr_err("mtk_capture_pcm_close\n");
		mtk_capture_pcm_close(substream);
		return ret;
	}
	if (mCaptureUseSram == false)
		AudDrv_Emi_Clk_On();
	pr_warn("mtk_capture_pcm_open return\n");
	return 0;
}

static int mtk_capture_pcm_close(struct snd_pcm_substream *substream)
{
	if (mCaptureUseSram == false)
		AudDrv_Emi_Clk_Off();
	if (mCaptureUseSram == true) {
		ClearSramState(SRAM_STATE_CAPTURE);
		mCaptureUseSram = false;
	}
	AudDrv_ADC_Clk_Off();
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_capture_alsa_start(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_capture_alsa_start\n");
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL, substream);
	StartAudioCaptureHardware(substream);
	return 0;
}

static int mtk_capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_warn("mtk_capture_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
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
	if (pointer == NULL) {
		pr_warn("CheckNullPointer pointer = NULL");
		return true;
	}
	return false;
}

static int mtk_capture_pcm_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos,
				void __user *dst, snd_pcm_uframes_t count)
{

	AFE_MEM_CONTROL_T *pVUL_MEM_ConTrol = NULL;
	AFE_BLOCK_T *Vul_Block = NULL;
	char *Read_Data_Ptr = (char *)dst;
	ssize_t DMA_Read_Ptr = 0, read_size = 0, read_count = 0;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	unsigned long flags;

	PRINTK_AUD_UL1("mtk_capture_pcm_copy pos = %lucount = %lu\n ", pos, count);
	/* get total bytes to copy */
	count = Align64ByteSize(audio_frame_to_bytes(substream, count));

	/* check which memif nned to be write */
	pVUL_MEM_ConTrol = VUL_Control_context;
	Vul_Block = &(pVUL_MEM_ConTrol->rBlock);

	if (pVUL_MEM_ConTrol == NULL) {
		pr_err("cannot find MEM control !!!!!!!\n");
		msleep(50);
		return 0;
	}

	if (Vul_Block->u4BufferSize <= 0) {
		msleep(50);
		pr_err("Vul_Block->u4BufferSize <= 0  =%d\n", Vul_Block->u4BufferSize);
		return 0;
	}

	if (CheckNullPointer((void *)Vul_Block->pucVirtBufAddr)) {
		pr_err("CheckNullPointer  pucVirtBufAddr = %p\n", Vul_Block->pucVirtBufAddr);
		return 0;
	}

	spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
	if (Vul_Block->u4DataRemained > Vul_Block->u4BufferSize) {
		PRINTK_AUD_UL1("AudDrv_MEMIF_Read u4DataRemained=%x > u4BufferSize=%x",
			 Vul_Block->u4DataRemained, Vul_Block->u4BufferSize);
		Vul_Block->u4DataRemained = 0;
		Vul_Block->u4DMAReadIdx = Vul_Block->u4WriteIdx;
	}
	if (count > Vul_Block->u4DataRemained)
		read_size = Vul_Block->u4DataRemained;
	else
		read_size = count;

	DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
	spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

	PRINTK_AUD_UL1("%s finish0, read_count:%x, read_size:%x, Remained:%x, ReadIdx:0x%x, WriteIdx:%x \r\n",
		       __func__, (unsigned int)read_count,
		       (unsigned int)read_size, Vul_Block->u4DataRemained,
		       Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx);

	if (DMA_Read_Ptr + read_size < Vul_Block->u4BufferSize) {
		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {
			pr_warn("%s 1, read_size:%zu, Remained:%x, Ptr:0x%zu, DMAReadIdx:%x \r\n",
			       __func__, read_size, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)Read_Data_Ptr,
				 (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size)) {

			pr_err("%s Fail 1 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x, Read_Ptr:%zu,size:%zu",
				__func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
				Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
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

		PRINTK_AUD_UL1("%s finish1, copy size:%x, ReadIdx:0x%x, WriteIdx:%x, Remained:%x \r\n",
			       __func__, (unsigned int)read_size,
			       Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx,
			       Vul_Block->u4DataRemained);
	}

	else {
		uint32 size_1 = Vul_Block->u4BufferSize - DMA_Read_Ptr;
		uint32 size_2 = read_size - size_1;

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {

			PRINTK_AUD_UL1("%s 2, read_size1:%x, Remained:%x, Read_Ptr:%zu, ReadIdx:%x \r\n",
			       __func__, size_1, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), (unsigned int)size_1)) {

			pr_err("%s Fail 2 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x, Read_Ptr:%zu,read_size:%zu",
			       __func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
			       Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
			return 0;
		}

		read_count += size_1;
		spin_lock(&auddrv_ULInCtl_lock);
		Vul_Block->u4DataRemained -= size_1;
		Vul_Block->u4DMAReadIdx += size_1;
		Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
		DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_ULInCtl_lock);


		PRINTK_AUD_UL1("%s finish2, copy size_1:%x, ReadIdx:0x%x, WriteIdx:0x%x, Remained:%x \r\n",
			       __func__, size_1, Vul_Block->u4DMAReadIdx,
			       Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);

		if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx) {

			PRINTK_AUD_UL1("%s 3, read_size2:%x, Remained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
			       __func__, size_2, Vul_Block->u4DataRemained,
			       DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
		}
		if (copy_to_user((void __user *)(Read_Data_Ptr + size_1),
				 (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2)) {

			pr_err("%s Fail 3 copy to user Ptr:%p, Addr:%p, ReadIdx:0x%x , Read_Ptr:%zu, read_size:%zu",
				__func__, Read_Data_Ptr, Vul_Block->pucVirtBufAddr,
				Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
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

		PRINTK_AUD_UL1("%s finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
			       __func__, size_2, Vul_Block->u4DMAReadIdx,
			       Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
	}

	return read_count >> 2;
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
				   int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	pr_warn("dummy_pcm_silence\n");
	return 0;		/* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_capture_pcm_page(struct snd_pcm_substream *substream, unsigned long offset)
{
	pr_warn("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);	/* the same page */
}


static struct snd_pcm_ops mtk_afe_capture_ops = {
	.open = mtk_capture_pcm_open,
	.close = mtk_capture_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_capture_pcm_hw_params,
	.hw_free = mtk_capture_pcm_hw_free,
	.prepare = mtk_capture_pcm_prepare,
	.trigger = mtk_capture_pcm_trigger,
	.pointer = mtk_capture_pcm_pointer,
	.copy = mtk_capture_pcm_copy,
	.silence = mtk_capture_pcm_silence,
	.page = mtk_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops = &mtk_afe_capture_ops,
	.pcm_new = mtk_asoc_capture_pcm_new,
	.probe = mtk_afe_capture_probe,
};

static int mtk_capture_probe(struct platform_device *pdev)
{
	pr_warn("mtk_capture_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (pdev->dev.dma_mask == NULL)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_UL1_PCM);

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}

static int mtk_asoc_capture_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	pr_warn("mtk_asoc_capture_pcm_new\n");
	return 0;
}


static int mtk_afe_capture_probe(struct snd_soc_platform *platform)
{
	pr_warn("mtk_afe_capture_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_VUL, UL1_MAX_BUFFER_SIZE);
	Capture_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_VUL);
	mAudioDigitalI2S =  kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
	return 0;
}


static int mtk_capture_remove(struct platform_device *pdev)
{
	PRINTK_AUD_UL1("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_capture_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_pcm_capture", },
	{}
};
#endif

static struct platform_driver mtk_afe_capture_driver = {
	.driver = {
		.name = MT_SOC_UL1_PCM,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_soc_pcm_capture_of_ids,
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
	pr_warn("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtkafe_capture_dev = platform_device_alloc(MT_SOC_UL1_PCM, -1);
	if (!soc_mtkafe_capture_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkafe_capture_dev);
	if (ret != 0) {
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

	pr_warn("%s\n", __func__);
	platform_driver_unregister(&mtk_afe_capture_driver);
}

module_exit(mtk_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
