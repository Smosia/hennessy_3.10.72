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
 *   mt_soc_pcm_dl2.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio dl2 data1 playback
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
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
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"



static AFE_MEM_CONTROL_T *pMemControl;
static int mPlaybackSramState;
static struct snd_dma_buffer *Dl2_Playback_dma_buf;

static DEFINE_SPINLOCK(auddrv_DL2Ctl_lock);


/*
 *    function implementation
 */

/* void StartAudioPcmHardware(void); */
/* void StopAudioPcmHardware(void); */
static int mtk_soc_dl2_probe(struct platform_device *pdev);
static int mtk_soc_pcm_dl2_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_dl2_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_asoc_dl2_probe(struct snd_soc_platform *platform);

static bool mPrepareDone;
static bool mPlaybackUseSram;

#define USE_RATE        (SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        192000
#define USE_CHANNELS_MIN     1
#define USE_CHANNELS_MAX    2
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     8192

static struct snd_pcm_hardware mtk_pcm_dl2_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_ADV_MT_FMTS,
	.rates = SOC_HIGH_USE_RATE,
	.rate_min = SOC_HIGH_USE_RATE_MIN,
	.rate_max = SOC_HIGH_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = Dl2_MAX_BUFFER_SIZE,
	.period_bytes_max = Dl2_MAX_PERIOD_SIZE,
	.periods_min = SOC_NORMAL_USE_PERIODS_MIN,
	.periods_max = SOC_NORMAL_USE_PERIODS_MAX,
	.fifo_size = 0,
};

static int mtk_pcm_dl2_stop(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL2, false);

	SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, 0);
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, false);

	/* here start digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I07,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I08,
		      Soc_Aud_InterConnectionOutput_O04);

	ClearMemBlock(Soc_Aud_Digital_Block_MEM_DL2);
	return 0;
}

static snd_pcm_uframes_t mtk_pcm_dl2_pointer(struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_uint32 Frameidx = 0;
	kal_int32 Afe_consumed_bytes = 0;
	AFE_BLOCK_T *Afe_Block = &pMemControl->rBlock;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	PRINTK_AUD_DL2(" %s Afe_Block->u4DMAReadIdx = 0x%x\n", __func__, Afe_Block->u4DMAReadIdx);

	Auddrv_Dl2_Spinlock_lock();

	/* get total bytes to copy */
	/* Frameidx = audio_bytes_to_frame(substream , Afe_Block->u4DMAReadIdx); */
	/* return Frameidx; */

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL2) == true) {
		HW_Cur_ReadIdx = Afe_Get_Reg(AFE_DL2_CUR);
		if (HW_Cur_ReadIdx == 0) {
			PRINTK_AUDDRV("[Auddrv] HW_Cur_ReadIdx ==0\n");
			HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
		}

		HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
		if (HW_memory_index >= Afe_Block->u4DMAReadIdx) {
			Afe_consumed_bytes = HW_memory_index - Afe_Block->u4DMAReadIdx;
		} else {
			Afe_consumed_bytes = Afe_Block->u4BufferSize + HW_memory_index -
			    Afe_Block->u4DMAReadIdx;
		}

#ifdef AUDIO_64BYTE_ALIGN	/* no need to do 64byte align */
		Afe_consumed_bytes = Align64ByteSize(Afe_consumed_bytes);
#endif

		Afe_Block->u4DataRemained -= Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx += Afe_consumed_bytes;
		Afe_Block->u4DMAReadIdx %= Afe_Block->u4BufferSize;
		PRINTK_AUD_DL2
		    ("[Auddrv] HW_Cur_ReadIdx =0x%x HW_memory_index = 0x%x Afe_consumed_bytes  = 0x%x\n",
		     HW_Cur_ReadIdx, HW_memory_index, Afe_consumed_bytes);
		Auddrv_Dl2_Spinlock_unlock();

		return audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
	}

	Frameidx = audio_bytes_to_frame(substream, Afe_Block->u4DMAReadIdx);
	Auddrv_Dl2_Spinlock_unlock();
	return Frameidx;
}

static void SetDL2Buffer(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	AFE_BLOCK_T *pblock = &pMemControl->rBlock;

	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;
	pr_warn("SetDL2Buffer u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
	       pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
	/* set dram address top hardware */
	Afe_Set_Reg(AFE_DL2_BASE, pblock->pucPhysBufAddr, 0xffffffff);
	Afe_Set_Reg(AFE_DL2_END, pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);
	memset_io((void *)pblock->pucVirtBufAddr, 0, pblock->u4BufferSize);

}

static int mtk_pcm_dl2_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	/* struct snd_dma_buffer *dma_buf = &substream->dma_buffer; */
	int ret = 0;

	PRINTK_AUDDRV("mtk_pcm_dl2_params\n");

	/* runtime->dma_bytes has to be set manually to allow mmap */
	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);

		substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
		substream->runtime->dma_area = Dl2_Playback_dma_buf->area;
		substream->runtime->dma_addr = Dl2_Playback_dma_buf->addr;
		SetDL2Buffer(substream, hw_params);

	PRINTK_AUDDRV("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
		      substream->runtime->dma_bytes, substream->runtime->dma_area,
		      (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_pcm_dl2_hw_free(struct snd_pcm_substream *substream)
{
	PRINTK_AUDDRV("mtk_pcm_dl2_hw_free\n");
	return 0;
}


static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static int mtk_pcm_dl2_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	PRINTK_AUDDRV("mtk_pcm_dl2_open\n");

		mtk_pcm_dl2_hardware.buffer_bytes_max = GetPLaybackDramSize();
		mPlaybackSramState = SRAM_STATE_PLAYBACKDRAM;
		mPlaybackUseSram = false;
	if (mPlaybackSramState == SRAM_STATE_PLAYBACKDRAM)
		AudDrv_Emi_Clk_On();

	pr_warn("mtk_pcm_dl2_hardware.buffer_bytes_max = %zu mPlaybackSramState = %d\n",
	       mtk_pcm_dl2_hardware.buffer_bytes_max, mPlaybackSramState);
	runtime->hw = mtk_pcm_dl2_hardware;

	AudDrv_Clk_On();
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_pcm_dl2_hardware,
	       sizeof(struct snd_pcm_hardware));
	pMemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_DL2);

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);

	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_integer failed\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pr_warn("SNDRV_PCM_STREAM_PLAYBACK mtkalsa_dl2playback_constraints\n");
	else
		pr_warn("SNDRV_PCM_STREAM_CAPTURE mtkalsa_dl2playback_constraints\n");

	if (ret < 0) {
		pr_err("ret < 0 mtk_soc_pcm_dl2_close\n");
		mtk_soc_pcm_dl2_close(substream);
		return ret;
	}
	/* PRINTK_AUDDRV("mtk_pcm_dl2_open return\n"); */
	return 0;
}

static int mtk_soc_pcm_dl2_close(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	if (mPrepareDone == true) {
		/* stop DAC output */
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);
		if (GetI2SDacEnable() == false)
			SetI2SDacEnable(false);

		RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_DL2, substream);

		EnableAfe(false);
		mPrepareDone = false;
	}

	if (mPlaybackSramState == SRAM_STATE_PLAYBACKDRAM)
		AudDrv_Emi_Clk_Off();

	AfeControlSramLock();
	ClearSramState(mPlaybackSramState);
	mPlaybackSramState = GetSramState();
	AfeControlSramUnLock();
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_pcm_dl2_prepare(struct snd_pcm_substream *substream)
{
	bool mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_16BITS;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (mPrepareDone == false) {
		pr_warn
		    ("%s format = %d SNDRV_PCM_FORMAT_S32_LE = %d SNDRV_PCM_FORMAT_U32_LE = %d\n",
		     __func__, runtime->format, SNDRV_PCM_FORMAT_S32_LE, SNDRV_PCM_FORMAT_U32_LE);
		SetMemifSubStream(Soc_Aud_Digital_Block_MEM_DL2, substream);

		if (runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
		    runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
			/* not support 24bit +++ */
			SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2,
						     AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
						  Soc_Aud_InterConnectionOutput_O03);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
						  Soc_Aud_InterConnectionOutput_O04);
			/* not support 24bit --- */
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_32BITS;
		} else {
			/* not support 24bit +++ */
			SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_DL2,
						     AFE_WLEN_16_BIT);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
						  Soc_Aud_InterConnectionOutput_O03);
			SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
						  Soc_Aud_InterConnectionOutput_O04);
			/* not support 24bit --- */
			mI2SWLen = Soc_Aud_I2S_WLEN_WLEN_16BITS;
		}

		SetSampleRate(Soc_Aud_Digital_Block_MEM_I2S, runtime->rate);

		/* start I2S DAC out */
		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC) == false) {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
			SetI2SDacOut(substream->runtime->rate, false, mI2SWLen);
			SetI2SDacEnable(true);
		} else {
			SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
		}
		/* here to set interrupt_distributor */
		SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->period_size);
		SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->rate);

		EnableAfe(true);
		mPrepareDone = true;
	}
	return 0;
}


static int mtk_pcm_dl2_start(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_warn("%s\n", __func__);
	/* here start digital part */

	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I07,
		      Soc_Aud_InterConnectionOutput_O03);
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I08,
		      Soc_Aud_InterConnectionOutput_O04);

#ifdef CONFIG_MTK_FPGA
	/* set loopback test interconnection */
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I07,
		      Soc_Aud_InterConnectionOutput_O09);
	SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I08,
		      Soc_Aud_InterConnectionOutput_O10);
#endif

	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, true);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_DL2, runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_DL2, runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_DL2, true);

	SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->period_size);
	SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ1_MCU_MODE, runtime->rate);
	EnableAfe(true);
	return 0;
}

static int mtk_pcm_dl2_trigger(struct snd_pcm_substream *substream, int cmd)
{
	PRINTK_AUDDRV("mtk_pcm_trigger cmd = %d\n", cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_dl2_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_dl2_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_dl2_copy(struct snd_pcm_substream *substream,
			int channel, snd_pcm_uframes_t pos,
			void __user *dst, snd_pcm_uframes_t count)
{
	AFE_BLOCK_T *Afe_Block = NULL;
	int copy_size = 0, Afe_WriteIdx_tmp;
	unsigned long flags;
	/* struct snd_pcm_runtime *runtime = substream->runtime; */
	char *data_w_ptr = (char *)dst;

	PRINTK_AUD_DL2("mtk_pcm_copy pos = %lu count = %lu\n ", pos, count);
	/* get total bytes to copy */
	count = audio_frame_to_bytes(substream, count);

	/* check which memif nned to be write */
	Afe_Block = &pMemControl->rBlock;

	PRINTK_AUD_DL2("AudDrv_write WriteIdx=0x%x, ReadIdx=0x%x, DataRemained=0x%x\n",
		       Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

	if (Afe_Block->u4BufferSize == 0) {
		pr_err("AudDrv_write: u4BufferSize=0 Error");
		return 0;
	}

	AudDrv_checkDLISRStatus();

	spin_lock_irqsave(&auddrv_DL2Ctl_lock, flags);
	copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;	/* free space of the buffer */
	spin_unlock_irqrestore(&auddrv_DL2Ctl_lock, flags);
	if (count <= copy_size) {
		if (copy_size < 0)
			copy_size = 0;
		else
			copy_size = count;
	}

#ifdef AUDIO_64BYTE_ALIGN	/* no need to do 64byte align */
	copy_size = Align64ByteSize(copy_size);
#endif
	PRINTK_AUD_DL2("copy_size=0x%x, count=0x%x\n", copy_size, (unsigned int)count);

	if (copy_size != 0) {
		spin_lock_irqsave(&auddrv_DL2Ctl_lock, flags);
		Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
		spin_unlock_irqrestore(&auddrv_DL2Ctl_lock, flags);

		if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) {	/* copy once */
			if (!access_ok(VERIFY_READ, data_w_ptr, copy_size)) {
				PRINTK_AUDDRV("AudDrv_write 0ptr invalid data_w_ptr=%p, size=%d",
					      data_w_ptr, copy_size);
				PRINTK_AUDDRV("AudDrv_write u4BufferSize=%d, u4DataRemained=%d",
					      Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {
				PRINTK_AUD_DL2
				    ("memcpy VirtBufAddr+Afe_WriteIdx= %p,data_w_ptr = %p copy_size = 0x%x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr,
				     copy_size);
				if (copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				     copy_size)) {
					PRINTK_AUDDRV("AudDrv_write Fail copy from user\n");
					return -1;
				}
			}

			spin_lock_irqsave(&auddrv_DL2Ctl_lock, flags);
			Afe_Block->u4DataRemained += copy_size;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DL2Ctl_lock, flags);
			data_w_ptr += copy_size;
			count -= copy_size;

			PRINTK_AUD_DL2
			    ("AudDrv_write finish1, copy:%x, WriteIdx:%x,ReadIdx=%x,Remained:%x, count=%d \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained, (int)count);

		} else {	/* copy twice */
			kal_uint32 size_1 = 0, size_2 = 0;
#ifdef AUDIO_64BYTE_ALIGN	/* no need to do 64byte align */
			size_1 = Align64ByteSize((Afe_Block->u4BufferSize - Afe_WriteIdx_tmp));
			size_2 = Align64ByteSize((copy_size - size_1));
#else
			size_1 = Afe_Block->u4BufferSize - Afe_WriteIdx_tmp;
			size_2 = copy_size - size_1;
#endif
			PRINTK_AUD_DL2("size_1=0x%x, size_2=0x%x\n", size_1, size_2);
			if (!access_ok(VERIFY_READ, data_w_ptr, size_1)) {
				pr_err("AudDrv_write 1ptr invalid data_w_ptr=%p, size_1=%d",
				       data_w_ptr, size_1);
				pr_err("AudDrv_write u4BufferSize=%d, u4DataRemained=%d",
				       Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL2
				    ("mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %p data_w_ptr = %p size_1 = %x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr,
				     size_1);
				if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr,
				      (unsigned int)size_1))) {
					PRINTK_AUDDRV("AudDrv_write Fail 1 copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_DL2Ctl_lock, flags);
			Afe_Block->u4DataRemained += size_1;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
			spin_unlock_irqrestore(&auddrv_DL2Ctl_lock, flags);

			if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2)) {
				PRINTK_AUDDRV
				    ("AudDrv_write 2ptr invalid data_w_ptr=%p, size_1=%d, size_2=%d",
				     data_w_ptr, size_1, size_2);
				PRINTK_AUDDRV("AudDrv_write u4BufferSize=%d, u4DataRemained=%d",
					      Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
			} else {

				PRINTK_AUD_DL2
				    ("mcmcpy VirtBufAddr+Afe_WriteIdx= %p,data_w_ptr+size_1 = %p size_2 = %x\n",
				     Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp,
				     data_w_ptr + size_1, (unsigned int)size_2);
				if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp),
				      (data_w_ptr + size_1), size_2))) {
					PRINTK_AUDDRV("AudDrv_write Fail 2  copy from user");
					return -1;
				}
			}
			spin_lock_irqsave(&auddrv_DL2Ctl_lock, flags);

			Afe_Block->u4DataRemained += size_2;
			Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
			Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
			spin_unlock_irqrestore(&auddrv_DL2Ctl_lock, flags);
			count -= copy_size;
			data_w_ptr += copy_size;

			PRINTK_AUD_DL2
			    ("AudDrv_write finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
			     copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx,
			     Afe_Block->u4DataRemained);
		}
	}
	return 0;
}

static int mtk_pcm_dl2_silence(struct snd_pcm_substream *substream,
			   int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	PRINTK_AUDDRV("%s\n", __func__);
	return 0;		/* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_pcm_dl2_page(struct snd_pcm_substream *substream, unsigned long offset)
{
	PRINTK_AUDDRV("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]);	/* the same page */
}

static struct snd_pcm_ops mtk_dl2_ops = {
	.open = mtk_pcm_dl2_open,
	.close = mtk_soc_pcm_dl2_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_pcm_dl2_params,
	.hw_free = mtk_pcm_dl2_hw_free,
	.prepare = mtk_pcm_dl2_prepare,
	.trigger = mtk_pcm_dl2_trigger,
	.pointer = mtk_pcm_dl2_pointer,
	.copy = mtk_pcm_dl2_copy,
	.silence = mtk_pcm_dl2_silence,
	.page = mtk_pcm_dl2_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops = &mtk_dl2_ops,
	.pcm_new = mtk_asoc_pcm_dl2_new,
	.probe = mtk_asoc_dl2_probe,
};

#ifdef CONFIG_OF

static const struct of_device_id mt_soc_pcm_dl2_of_ids[] = {
	{.compatible = "mediatek,mt_soc_pcm_dl2",},
	{}
};

#endif

static int mtk_soc_dl2_probe(struct platform_device *pdev)
{
	PRINTK_AUDDRV("%s\n", __func__);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node) {
		dev_set_name(&pdev->dev, "%s", MT_SOC_DL2_PCM);
	} else {
		pr_err("%s invalid of_node\n", __func__);
		return -ENODEV;
	}

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));


	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}

static int mtk_asoc_pcm_dl2_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	PRINTK_AUDDRV("%s\n", __func__);
	return ret;
}


static int mtk_asoc_dl2_probe(struct snd_soc_platform *platform)
{
	PRINTK_AUDDRV("mtk_asoc_dl2_probe\n");
	/* allocate dram */
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_DL2,
				   Dl2_MAX_BUFFER_SIZE);
	Dl2_Playback_dma_buf = Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_DL2);
	return 0;
}

static int mtk_soc_dl2_remove(struct platform_device *pdev)
{
	PRINTK_AUDDRV("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver mtk_dl2_driver = {
	.driver = {
		   .name = MT_SOC_DL2_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_dl2_of_ids,
#endif
		   },
	.probe = mtk_soc_dl2_probe,
	.remove = mtk_soc_dl2_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkdl2_dev;
#endif

static int __init mtk_dl2_soc_platform_init(void)
{
	int ret;

	PRINTK_AUDDRV("%s\n", __func__);

#ifndef CONFIG_OF
	soc_mtkdl2_dev = platform_device_alloc(MT_SOC_DL2_PCM, -1);
	if (!soc_mtkdl2_dev)
		return -ENOMEM;


	ret = platform_device_add(soc_mtkdl2_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkdl2_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_dl2_driver);
	return ret;

}
module_init(mtk_dl2_soc_platform_init);

static void __exit mtk_dl2_soc_platform_exit(void)
{
	PRINTK_AUDDRV("%s\n", __func__);

	platform_driver_unregister(&mtk_dl2_driver);
}
module_exit(mtk_dl2_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
