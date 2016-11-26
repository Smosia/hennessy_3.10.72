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
 *   mt_soc_pcm_hdmi.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio hdmi playback
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
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"

#if 0
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
//#include <asm/mach-types.h>
#endif

//information about

static AFE_MEM_CONTROL_T *pMemControl = NULL;
static bool fake_buffer = 1;

struct snd_dma_buffer *HDMI_dma_buf  = NULL;


static DEFINE_SPINLOCK(auddrv_hdmi_lock);

/*
 *    function implementation
 */

static int mtk_hdmi_probe(struct platform_device *pdev);
static int mtk_pcm_hdmi_close(struct snd_pcm_substream *substream);
static int mtk_asoc_pcm_hdmi_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_hdmi_probe(struct snd_soc_platform *platform);


#define MAX_PCM_DEVICES     4
#define MAX_PCM_SUBSTREAMS  128
#define MAX_MIDI_DEVICES
//#define _DEBUG_TDM_KERNEL_
#define _NO_SRAM_USAGE_
#define _TDM_8CH_SGEN_TEST 1


/* defaults */
//#define HDMI_MAX_BUFFER_SIZE     (192*1024)
//#define MIN_PERIOD_SIZE       64
//#define MAX_PERIOD_SIZE     HDMI_MAX_BUFFER_SIZE
#define USE_FORMATS         (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE        SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_192000
#define USE_RATE_MIN        8000
#define USE_RATE_MAX        192000
#define USE_CHANNELS_MIN     1
#define USE_CHANNELS_MAX    8
#define USE_PERIODS_MIN     512
#define USE_PERIODS_MAX     16384

struct mt_pcm_hdmi_priv {
	bool prepared;
	unsigned int hdmi_loop_type;
	unsigned int hdmi_sinegen_switch;
	unsigned int cached_sample_rate;
	struct snd_dma_buffer *hdmi_dma_buf;
};

static kal_int32 Previous_Hw_cur = 0;

#ifdef _TDM_8CH_SGEN_TEST

static uint32 table_sgen_golden_values[64] =
{
    0x0FE50FE5, 0x285E1C44, 0x3F4A285E, 0x53C73414,
    0x650C3F4A, 0x726F49E3, 0x7B6C53C7, 0x7FAB5CDC,
    0x7F02650C, 0x79776C43, 0x6F42726F, 0x60C67781,
    0x4E917B6C, 0x39587E27, 0x21EB7FAB, 0x09307FF4,
    0xF01A7F02, 0xD7A17CD6, 0xC0B67977, 0xAC3874ED,
    0x9AF36F42, 0x8D906884, 0x849360C6, 0x80545818,
    0x80FD4E91, 0x86884449, 0x90BD3958, 0x9F3A2DDA,
    0xB16E21EB, 0xC6A715A8, 0xDE140930, 0xF6CFFCA1,
    0x0FE5F01A, 0x285EE3BB, 0x3F4AD7A1, 0x53C7CBEB,
    0x650CC0B6, 0x726FB61C, 0x7B6CAC38, 0x7FABA323,
    0x7F029AF3, 0x797793BC, 0x6F428D90, 0x60C6887E,
    0x4E918493, 0x395881D8, 0x21EB8054, 0x0930800B,
    0xF01A80FD, 0xD7A18329, 0xC0B68688, 0xAC388B12,
    0x9AF390BD, 0x8D90977B, 0x84939F3A, 0x8054A7E7,
    0x80FDB16E, 0x8688BBB6, 0x90BDC6A7, 0x9F3AD225,
    0xB16EDE14, 0xC6A7EA57, 0xDE14F6CF, 0xF6CF035E
};



static  uint32 table_sgen_4ch_golden_values[128] =
{
    //step   2              1              4              3
    //    ch2       ch1            ch4          ch3
    0x0FE50FE5,  0x0FE50FE5,
    0x285E1C44,  0x3F4A3414,
    0x3F4A285E,  0x650C53C7,
    0x53C73414,  0x7B6C6C43,
    0x650C3F4A,  0x7F027B6C,
    0x726F49E3,  0x6F427FF4,
    0x7B6C53C7,  0x4E917977,
    0x7FAB5CDC,  0x21EB6884,
    0x7F02650C,  0xF01A4E91,
    0x79776C43,  0xC0B62DDA,
    0x6F42726F,  0x9AF30930,
    0x60C67781,  0x8493E3BB,
    0x4E917B6C,  0x80FDC0B6,
    0x39587E27,  0x90BDA323,
    0x21EB7FAB,  0xB16E8D90,
    0x09307FF4,  0xDE1481D8,
    0xF01A7F02,  0x0FE580FD,
    0xD7A17CD6,  0x0FE58B12,
    0xC0B67977,  0x3F4A9F3A,
    0xAC3874ED,  0x650CBBB6,
    0x9AF36F42,  0x7B6CDE14,
    0x8D906884,  0x7F02035E,
    0x849360C6,  0x6F420FE5,
    0x80545818,  0x4E913414,
    0x80FD4E91,  0x21EB53C7,
    0x86884449,  0xF01A6C43,
    0x90BD3958,  0xC0B67B6C,
    0x9F3A2DDA,  0x9AF37FF4,
    0xB16E21EB,  0x84937977,
    0xC6A715A8,  0x80FD6884,
    0xDE140930,  0x90BD4E91,
    0xF6CFFCA1,  0xB16E2DDA,
    0x0FE5F01A,  0xDE140930,
    0x285EE3BB,  0x0FE5E3BB,
    0x3F4AD7A1,  0x0FE5C0B6,
    0x53C7CBEB,  0x3F4AA323,
    0x650CC0B6,  0x650C8D90,
    0x726FB61C,  0x7B6C81D8,
    0x7B6CAC38,  0x7F0280FD,
    0x7FABA323,  0x6F428B12,
    0x7F029AF3,  0x4E919F3A,
    0x797793BC,  0x21EBBBB6,
    0x6F428D90,  0xF01ADE14,
    0x60C6887E,  0xC0B6035E,
    0x4E918493,  0x9AF30FE5,
    0x395881D8,  0x84933414,
    0x21EB8054,  0x80FD53C7,
    0x0930800B,  0x90BD6C43,
    0xF01A80FD,  0xB16E7B6C,
    0xD7A18329,  0xDE147FF4,
    0xC0B68688,  0x0FE57977,
    0xAC388B12,  0x0FE56884,
    0x9AF390BD,  0x3F4A4E91,
    0x8D90977B,  0x650C2DDA,
    0x84939F3A,  0x7B6C0930,
    0x8054A7E7,  0x7F02E3BB,
    0x80FDB16E,  0x6F42C0B6,
    0x8688BBB6,  0x4E91A323,
    0x90BDC6A7,  0x21EB8D90,
    0x9F3AD225,  0xF01A81D8,
    0xB16EDE14,  0xC0B680FD,
    0xC6A7EA57,  0x9AF38B12,
    0xDE14F6CF,  0x84939F3A,
    0xF6CF035E,  0x80FDBBB6
};


static  uint32 table_sgen_8ch_golden_values[] =
{
    0x0FE50000,  0x0FE50FE5,    0x0FE50FE5,  0x0FE50FE5,
    0x285E0000,  0x3F4A3414,    0x285E1C44,  0x3F4A3414,
    0x3F4A0000,  0x650C53C7,    0x3F4A285E,  0x650C53C7,
    0x53C70000,  0x7B6C6C43,    0x53C73414,  0x7B6C6C43,
    0x650C0000,  0x7F027B6C,    0x650C3F4A,  0x7F027B6C,
    0x726F0000,  0x6F427FF4,    0x726F49E3,  0x6F427FF4,
    0x7B6C0000,  0x4E917977,    0x7B6C53C7,  0x4E917977,
    0x7FAB0000,  0x21EB6884,    0x7FAB5CDC,  0x21EB6884,
    0x7F020000,  0xF01A4E91,    0x7F02650C,  0xF01A4E91,
    0x79770000,  0xC0B62DDA,    0x79776C43,  0xC0B62DDA,
    0x6F420000,  0x9AF30930,    0x6F42726F,  0x9AF30930,
    0x60C60000,  0x8493E3BB,    0x60C67781,  0x8493E3BB,
    0x4E910000,  0x80FDC0B6,    0x4E917B6C,  0x80FDC0B6,
    0x39580000,  0x90BDA323,    0x39587E27,  0x90BDA323,
    0x21EB0000,  0xB16E8D90,    0x21EB7FAB,  0xB16E8D90,
    0x09300000,  0xDE1481D8,    0x09307FF4,  0xDE1481D8,
    0xF01A0000,  0x0FE580FD,    0xF01A7F02,  0x0FE580FD,
    0xD7A10000,  0x0FE58B12,    0xD7A17CD6,  0x0FE58B12,
    0xC0B60000,  0x3F4A9F3A,    0xC0B67977,  0x3F4A9F3A,
    0xAC380000,  0x650CBBB6,    0xAC3874ED,  0x650CBBB6,
    0x9AF30000,  0x7B6CDE14,    0x9AF36F42,  0x7B6CDE14,
    0x8D900000,  0x7F02035E,    0x8D906884,  0x7F02035E,
    0x84930000,  0x6F420FE5,    0x849360C6,  0x6F420FE5,
    0x80540000,  0x4E913414,    0x80545818,  0x4E913414,
    0x80FD0000,  0x21EB53C7,    0x80FD4E91,  0x21EB53C7,
    0x86880000,  0xF01A6C43,    0x86884449,  0xF01A6C43,
    0x90BD0000,  0xC0B67B6C,    0x90BD3958,  0xC0B67B6C,
    0x9F3A0000,  0x9AF37FF4,    0x9F3A2DDA,  0x9AF37FF4,
    0xB16E0000,  0x84937977,    0xB16E21EB,  0x84937977,
    0xC6A70000,  0x80FD6884,    0xC6A715A8,  0x80FD6884,
    0xDE140000,  0x90BD4E91,    0xDE140930,  0x90BD4E91,
    0xF6CF0000,  0xB16E2DDA,    0xF6CFFCA1,  0xB16E2DDA,
    0x0FE50000,  0xDE140930,    0x0FE5F01A,  0xDE140930,
    0x285E0000,  0x0FE5E3BB,    0x285EE3BB,  0x0FE5E3BB,
    0x3F4A0000,  0x0FE5C0B6,    0x3F4AD7A1,  0x0FE5C0B6,
    0x53C70000,  0x3F4AA323,    0x53C7CBEB,  0x3F4AA323,
    0x650C0000,  0x650C8D90,    0x650CC0B6,  0x650C8D90,
    0x726F0000,  0x7B6C81D8,    0x726FB61C,  0x7B6C81D8,
    0x7B6C0000,  0x7F0280FD,    0x7B6CAC38,  0x7F0280FD,
    0x7FAB0000,  0x6F428B12,    0x7FABA323,  0x6F428B12,
    0x7F020000,  0x4E919F3A,    0x7F029AF3,  0x4E919F3A,
    0x79770000,  0x21EBBBB6,    0x797793BC,  0x21EBBBB6,
    0x6F420000,  0xF01ADE14,    0x6F428D90,  0xF01ADE14,
    0x60C60000,  0xC0B6035E,    0x60C6887E,  0xC0B6035E,
    0x4E910000,  0x9AF30FE5,    0x4E918493,  0x9AF30FE5,
    0x39580000,  0x84933414,    0x395881D8,  0x84933414,
    0x21EB0000,  0x80FD53C7,    0x21EB8054,  0x80FD53C7,
    0x09300000,  0x90BD6C43,    0x0930800B,  0x90BD6C43,
    0xF01A0000,  0xB16E7B6C,    0xF01A80FD,  0xB16E7B6C,
    0xD7A10000,  0xDE147FF4,    0xD7A18329,  0xDE147FF4,
    0xC0B60000,  0x0FE57977,    0xC0B68688,  0x0FE57977,
    0xAC380000,  0x0FE56884,    0xAC388B12,  0x0FE56884,
    0x9AF30000,  0x3F4A4E91,    0x9AF390BD,  0x3F4A4E91,
    0x8D900000,  0x650C2DDA,    0x8D90977B,  0x650C2DDA,
    0x84930000,  0x7B6C0930,    0x84939F3A,  0x7B6C0930,
    0x80540000,  0x7F02E3BB,    0x8054A7E7,  0x7F02E3BB,
    0x80FD0000,  0x6F42C0B6,    0x80FDB16E,  0x6F42C0B6,
    0x86880000,  0x4E91A323,    0x8688BBB6,  0x4E91A323,
    0x90BD0000,  0x21EB8D90,    0x90BDC6A7,  0x21EB8D90,
    0x9F3A0000,  0xF01A81D8,    0x9F3AD225,  0xF01A81D8,
    0xB16E0000,  0xC0B680FD,    0xB16EDE14,  0xC0B680FD,
    0xC6A70000,  0x9AF38B12,    0xC6A7EA57,  0x9AF38B12,
    0xDE140000,  0x84939F3A,    0xDE14F6CF,  0x84939F3A,
    0xF6CF0000,  0x80FDBBB6,    0xF6CF035E,  0x80FDBBB6,
};



static void copysinewavetohdmi(unsigned int channels)
{
    unsigned char  *Bufferaddr = HDMI_dma_buf->area ;
    int Hhdmi_Buffer_length  = HDMI_dma_buf->bytes;
    uint32 arraybytes = 0;
    uint32 *SinewaveArr = NULL;
    int i = 0;
    if (channels == 2)
    {
        arraybytes = ARRAY_SIZE(table_sgen_golden_values) * sizeof(table_sgen_golden_values[0]);
        SinewaveArr = table_sgen_golden_values;
    }
    if (channels == 4)
    {
        arraybytes = ARRAY_SIZE(table_sgen_4ch_golden_values) * sizeof(table_sgen_4ch_golden_values[0]);
        SinewaveArr = table_sgen_4ch_golden_values;
    }
    if (channels == 8)
    {
        arraybytes = ARRAY_SIZE(table_sgen_8ch_golden_values) * sizeof(table_sgen_8ch_golden_values[0]);
        SinewaveArr = table_sgen_8ch_golden_values;
    }
    if (channels == 0 )
    {
        memset((void *)(Bufferaddr ), 0x7f7f7f7f, Hhdmi_Buffer_length); // using for observe data
        printk("use fix pattern Bufferaddr = %p Hhdmi_Buffer_length = %d\n",Bufferaddr,Hhdmi_Buffer_length);
        return;
    }

    printk("%s buffer area = %p arraybytes = %d bufferlength = %zu\n", __func__,    HDMI_dma_buf->area , arraybytes, HDMI_dma_buf->bytes);
    for (i = 0; i < HDMI_dma_buf->bytes ; i += arraybytes)
    {
        printk("Bufferaddr + i = %p arraybytes = %d \n",Bufferaddr + i,arraybytes);
        memcpy((void *)(Bufferaddr + i), (void *)SinewaveArr, arraybytes);
    }

    for (i = 0; i < 512 ; i++ )
    {
        printk("Bufferaddr[%d] = %x \n",i, *(Bufferaddr + i));
    }


}

static void SetHDMIAddress(void)
{
    printk("%s buffer length = %d\n", __func__,    (int)HDMI_dma_buf->bytes);
    Afe_Set_Reg(AFE_HDMI_BASE , HDMI_dma_buf->addr , 0xffffffff);
    Afe_Set_Reg(AFE_HDMI_END  , HDMI_dma_buf->addr + (HDMI_dma_buf->bytes - 1), 0xffffffff);
}
#endif

static bool bHdmiPrepared = false;

static int mHdmi_sidegen_control = 0;
static const char *HDMI_SIDEGEN[] = {"Off", "On"};

static const struct soc_enum Audio_Hdmi_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(HDMI_SIDEGEN), HDMI_SIDEGEN),
};

static int Audio_hdmi_SideGen_Get(struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_hdmi_SideGen_Get = %d\n", mHdmi_sidegen_control);
    ucontrol->value.integer.value[0] = mHdmi_sidegen_control;
    return 0;
}

static int Audio_hdmi_SideGen_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(HDMI_SIDEGEN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }

#ifdef _TDM_8CH_SGEN_TEST
    mHdmi_sidegen_control = ucontrol->value.integer.value[0];

    if (mHdmi_sidegen_control)
    {
        uint32 samplerate = 44100;
        uint32 Channels = 2;
        uint32 HDMIchaanel = 8;
        uint32 Tdm_Lrck = 0;
        uint32 MclkDiv = 0;

        //Enable Audio Driver Clock
        AudDrv_Clk_On();
        AudDrv_Emi_Clk_On();

        //Enable APLL
        EnableApll(samplerate, true);
        EnableApllTuner(samplerate, true);
        //HDMI I2S clock setting
        MclkDiv = SetCLkMclk(Soc_Aud_HDMI_MCK, samplerate);
        SetCLkHdmiBclk(MclkDiv, samplerate, 2 , 32);

        //enable mclk divider
        EnableSpdifDivPower(AUDIO_APLL_SPDIF_DIV,  true);
        //enable bck divider
        EnableHDMIDivPower(AUDIO_APLL_HDMI_BCK_DIV,  true);
        //turn on hdmi clk
        SetHdmiClkOn();

        //Set Mem buffer
        SetHDMIAddress();
        copysinewavetohdmi(8);

        //config hdmi irq
        SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, 44100);

        //config hdmi interface
        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_HDMI, AFE_WLEN_16_BIT); //now fix 16bit
        SetHdmiTdm1Config(44100, AFE_DATA_WLEN_32BIT);
        SetHdmiTdm2Config(44100);
        SetHDMIChannels(HDMIchaanel);

        //config hdmi connection
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I30, Soc_Aud_InterConnectionOutput_O30);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I31, Soc_Aud_InterConnectionOutput_O31);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I30, Soc_Aud_InterConnectionOutput_O32);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I31, Soc_Aud_InterConnectionOutput_O33);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I30, Soc_Aud_InterConnectionOutput_O34);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I31, Soc_Aud_InterConnectionOutput_O35);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I30, Soc_Aud_InterConnectionOutput_O36);
        SetHDMIConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I31, Soc_Aud_InterConnectionOutput_O37);

        //Enable hdmi Memory Path
        SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_HDMI, true);
        //enable irq
        SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, true);
        //enable hdmi out
        SetHDMIEnable(true);
        //enable afe
        EnableAfe(true);
        //enable TDM
        SetTDMEnable(HDMIchaanel);

    }
    else
    {
        uint32 samplerate = 44100;
        uint32 Channels = 2;
        uint32 HDMIchaanel = 8;
        uint32 Tdm_Lrck = 0;
        uint32 MclkDiv = 0;

        SetTDMEnable(false);

        SetHDMIEnable(false);

        SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, false);

        SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_HDMI, false);

        EnableAfe(false);

        SetHdmiPcmInterConnection(Soc_Aud_InterCon_DisConnect, HDMIchaanel);

        SetHdmiClkOff();
        EnableSpdifDivPower(AUDIO_APLL_SPDIF_DIV,  false);
        EnableHDMIDivPower(AUDIO_APLL_HDMI_BCK_DIV,  false);

        AudDrv_Emi_Clk_Off();
        AudDrv_Clk_Off();

    }
#endif
    return 0;
}

static const struct snd_kcontrol_new Audio_snd_hdmi_controls[] =
{
    SOC_ENUM_EXT("Audio_Hdmi_SideGen_Switch", Audio_Hdmi_Enum[0], Audio_hdmi_SideGen_Get, Audio_hdmi_SideGen_Set)
};


static struct snd_pcm_hardware mtk_hdmi_hardware =
{
    .info = (SNDRV_PCM_INFO_MMAP |
    SNDRV_PCM_INFO_INTERLEAVED |
    SNDRV_PCM_INFO_RESUME |
    SNDRV_PCM_INFO_MMAP_VALID),
    .formats =      SND_SOC_ADV_MT_FMTS,
    .rates =        SOC_HIGH_USE_RATE,
    .rate_min =     SOC_HIGH_USE_RATE_MIN,
    .rate_max =     SOC_HIGH_USE_RATE_MAX,
    .channels_min =     USE_CHANNELS_MIN,
    .channels_max =     USE_CHANNELS_MAX,
    .buffer_bytes_max = HDMI_MAX_BUFFER_SIZE,
    .period_bytes_max = HDMI_MAX_PERIODBYTE_SIZE,
    .periods_min =      HDMI_MIN_PERIOD_SIZE,
    .periods_max =      HDMI_MAX_2CH_16BIT_PERIOD_SIZE,
    .fifo_size =        0,
};


static struct snd_soc_pcm_runtime *pruntimehdmi = NULL;


static int mtk_pcm_hdmi_stop(struct snd_pcm_substream *substream)
{
    AFE_BLOCK_T *Afe_Block = &(pMemControl->rBlock);
    struct snd_pcm_runtime *runtime = substream->runtime;
    printk("mtk_pcm_hdmi_stop \n");

    SetTDMEnable(false);

    SetHDMIEnable(false);

    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, false);

    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_HDMI, false);

    EnableAfe(false);

    SetHdmiPcmInterConnection(Soc_Aud_InterCon_DisConnect, runtime->channels);

    RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_HDMI, substream);

    SetHdmiClkOff();
    EnableSpdifDivPower(AUDIO_APLL_SPDIF_DIV,  false);
    EnableHDMIDivPower(AUDIO_APLL_HDMI_BCK_DIV,  false);

    Afe_Block->u4DMAReadIdx  = 0;
    Afe_Block->u4WriteIdx  = 0;
    Afe_Block->u4DataRemained = 0;

    return 0;
}


static snd_pcm_uframes_t mtk_pcm_hdmi_pointer(struct snd_pcm_substream *substream)
{
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    AFE_BLOCK_T *Afe_Block = &(pMemControl->rBlock);

    PRINTK_AUD_HDMI("mtk_pcm_hdmi_pointer u4DMAReadIdx=%x\n",Afe_Block->u4DMAReadIdx);

    if (pMemControl->interruptTrigger == 1)
    {
    	#if 0 // K2 removed
        HW_Cur_ReadIdx = Afe_Get_Reg(AFE_HDMI_CUR);
		#endif
        if (HW_Cur_ReadIdx == 0)
        {
            PRINTK_AUD_HDMI("[mtk_pcm_hdmi_pointer] HW_Cur_ReadIdx ==0 \n");
            HW_Cur_ReadIdx = Afe_Block->pucPhysBufAddr;
        }
        HW_memory_index = (HW_Cur_ReadIdx - Afe_Block->pucPhysBufAddr);
        Previous_Hw_cur = HW_memory_index;
        PRINTK_AUD_HDMI("[mtk_pcm_hdmi_pointer] HW_Cur_ReadIdx =0x%x HW_memory_index = 0x%x pointer return = 0x%x \n", HW_Cur_ReadIdx, HW_memory_index,(HW_memory_index >> 2));
        pMemControl->interruptTrigger = 0;
        return (HW_memory_index >> 2);
    }
    return (Previous_Hw_cur >> 2);
}


static void SetHDMIBuffer(struct snd_pcm_substream *substream,
                          struct snd_pcm_hw_params *hw_params)
{

    kal_uint32 volatile u4tmpMrg1;

    struct snd_pcm_runtime *runtime = substream->runtime;
    AFE_BLOCK_T *pblock = &(pMemControl->rBlock);

    pblock->pucPhysBufAddr =  runtime->dma_addr;
    pblock->pucVirtBufAddr =  runtime->dma_area;
    pblock->u4BufferSize = runtime->dma_bytes;
    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;
    PRINTK_AUD_HDMI("%s dma_bytes = %d dma_area = %p dma_addr = 0x%x\n", __func__, pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);


    Afe_Set_Reg(AFE_HDMI_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_HDMI_END  , pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

    u4tmpMrg1 = Afe_Get_Reg(AFE_HDMI_BASE);

    u4tmpMrg1 &= 0x00ffffff;

    PRINTK_AUD_HDMI("SetHDMIBuffer AFE_HDMI_BASE =0x%x \n", u4tmpMrg1);

}


static int mtk_pcm_hdmi_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
    int ret = 0;

    PRINTK_AUD_HDMI("%s \n", __func__);

    dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
    dma_buf->dev.dev = substream->pcm->card->dev;
    dma_buf->private_data = NULL;

    if (fake_buffer)
    {
        PRINTK_AUD_HDMI("[mtk_pcm_hdmi_hw_params] HDMI_dma_buf->area\n");

#ifdef _NO_SRAM_USAGE_
        runtime->dma_bytes = HDMI_dma_buf->bytes;
        runtime->dma_area = HDMI_dma_buf->area;
        runtime->dma_addr = HDMI_dma_buf->addr;
        runtime->buffer_size = HDMI_dma_buf->bytes;
#else
        runtime->dma_area = (unsigned char *)Get_Afe_SramBase_Pointer();
        runtime->dma_addr = AFE_INTERNAL_SRAM_PHY_BASE;
        runtime->dma_bytes = params_buffer_bytes(hw_params);
        runtime->buffer_size = runtime->dma_bytes;
#endif

    }
    else
    {
        PRINTK_AUD_HDMI("[mtk_pcm_hdmi_hw_params] snd_pcm_lib_malloc_pages\n");
        ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    }
    PRINTK_AUD_HDMI("2 dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
                  substream->runtime->dma_bytes, substream->runtime->dma_area, (long)substream->runtime->dma_addr);

    SetHDMIBuffer(substream, hw_params);
    return ret;
}


static int mtk_pcm_hdmi_hw_free(struct snd_pcm_substream *substream)
{
    PRINTK_AUD_HDMI("mtk_pcm_hdmi_hw_free \n");
    if (fake_buffer)
    {
        return 0;
    }
    return snd_pcm_lib_free_pages(substream);
}


/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] =
{
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000, 192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(supported_sample_rates),
    .list = supported_sample_rates,
    .mask = 0,
};


static int mtk_pcm_hdmi_open(struct snd_pcm_substream *substream)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

    runtime->hw = mtk_hdmi_hardware;
    pMemControl = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_HDMI);

    memcpy((void *)(&(runtime->hw)), (void *)&mtk_hdmi_hardware , sizeof(struct snd_pcm_hardware));

    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &constraints_sample_rates);
  //  ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

    if (ret < 0)
    {
        PRINTK_AUD_HDMI("snd_pcm_hw_constraint_integer failed\n");
        return ret;
    }

    //print for hw pcm information
    printk("mtk_pcm_hdmi_open runtime rate = %d channels = %d substream->pcm->device = %d\n",
                  runtime->rate, runtime->channels, substream->pcm->device);
    runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

	pr_info("%s substream->pcm->device = %d\n", __func__, substream->pcm->device);

    AudDrv_Clk_On();
    AudDrv_Emi_Clk_On();
    PRINTK_AUD_HDMI("mtk_pcm_hdmi_open return\n");
    return 0;
}


static int mtk_pcm_hdmi_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (bHdmiPrepared) {
		EnableApllTuner(runtime->rate, false);
		EnableApll(runtime->rate, false);
		bHdmiPrepared = false;
	}

    AudDrv_Emi_Clk_Off();
    AudDrv_Clk_Off();

    return 0;
}

static int mtk_pcm_hdmi_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    uint32 Tdm_Lrck = 0;
    uint32 MclkDiv = 0;
    AFE_BLOCK_T *Afe_Block = &(pMemControl->rBlock);

    if (!bHdmiPrepared)
    {
		EnableApll(runtime->rate, true);
        EnableApllTuner(runtime->rate, true);
    }

    return 0;
}



static int mtk_pcm_hdmi_start(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    uint32 mclkDiv;
    kal_uint32 volatile u4RegValue;
    kal_uint32 volatile u4tmpValue;
    kal_uint32 volatile u4tmpValue1;
    kal_uint32 volatile u4tmpValue2;
    //uint32 u32AudioI2S = 0;

    SetMemifSubStream(Soc_Aud_Digital_Block_MEM_HDMI, substream);

    //HDMI I2S clock setting
    mclkDiv = SetCLkMclk(Soc_Aud_HDMI_MCK, runtime->rate);
    SetCLkHdmiBclk(mclkDiv, runtime->rate, 2 , 32);

    //enable mclk divider
    EnableSpdifDivPower(AUDIO_APLL_SPDIF_DIV,  true);
    //enable bck divider
    EnableHDMIDivPower(AUDIO_APLL_HDMI_BCK_DIV,  true);
    //turn on hdmi clk
    SetHdmiClkOn();

    //config hdmi irq
    SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, runtime->period_size);

    //config hdmi interface
    SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_HDMI, AFE_WLEN_16_BIT); //now fix 16bit
    SetHdmiTdm1Config(runtime->channels, AFE_DATA_WLEN_32BIT);
    SetHdmiTdm2Config(runtime->channels);
    SetHDMIChannels(runtime->channels);

    //config hdmi connection
    SetHdmiPcmInterConnection(Soc_Aud_InterCon_Connection, runtime->channels);

    //Enable hdmi Memory Path
    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_HDMI, true);
    //enable irq
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ5_MCU_MODE, true);
    //enable hdmi out
    SetHDMIEnable(true);
    //enable afe
    EnableAfe(true);
    //enable TDM
    SetTDMEnable(runtime->channels);

    printk("[mtk_pcm_hdmi_start] AFE_IRQ_MCU_STATUS =0x%x AFE_IRQ_MCU_EN= 0x%x, AFE_IRQ_CNT5=0x%x, AFE_IRQ_DEBUG =0x%x\n", u4RegValue, u4tmpValue, u4tmpValue1, u4tmpValue2);

    return 0;
}

static int mtk_pcm_hdmi_trigger(struct snd_pcm_substream *substream, int cmd)
{
    PRINTK_AUD_HDMI("mtk_pcm_hdmi_trigger cmd = %d\n", cmd);

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
            return mtk_pcm_hdmi_start(substream);
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            return mtk_pcm_hdmi_stop(substream);
    }
    return -EINVAL;
}

static int mtk_pcm_hdmi_copy(struct snd_pcm_substream *substream,
                             int channel, snd_pcm_uframes_t pos,
                             void __user *dst, snd_pcm_uframes_t count)
{
    AFE_BLOCK_T  *Afe_Block = NULL;
    int copy_size = 0, Afe_WriteIdx_tmp;
    unsigned long flags;
    char *data_w_ptr = (char *)dst;
    count = count << 2;

    // check which memif nned to be write
    Afe_Block = &(pMemControl->rBlock);


    // handle for buffer management

    PRINTK_AUD_HDMI("[mtk_pcm_hdmi_copy] count = %d WriteIdx=%x, ReadIdx=%x, DataRemained=%x \n",
            (kal_uint32)count, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

    if (Afe_Block->u4BufferSize == 0)
    {
        PRINTK_AUD_HDMI("AudDrv_write: u4BufferSize=0 Error");
        return 0;
    }

    spin_lock_irqsave(&auddrv_hdmi_lock, flags);
    copy_size = Afe_Block->u4BufferSize - Afe_Block->u4DataRemained;  //  free space of the buffer
    spin_unlock_irqrestore(&auddrv_hdmi_lock, flags);
    if (count <=  copy_size)
    {
        if (copy_size < 0)
        {
            copy_size = 0;
        }
        else
        {
            copy_size = count;
        }
    }

    if (copy_size != 0)
    {
        spin_lock_irqsave(&auddrv_hdmi_lock, flags);
        Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
        spin_unlock_irqrestore(&auddrv_hdmi_lock, flags);

        if (Afe_WriteIdx_tmp + copy_size < Afe_Block->u4BufferSize) // copy once
        {
            if (!access_ok(VERIFY_READ, data_w_ptr, copy_size))
            {
                PRINTK_AUD_HDMI("[mtk_pcm_hdmi_copy] 0ptr invalid data_w_ptr=%p, size=%d", data_w_ptr, copy_size);
                PRINTK_AUD_HDMI("[mtk_pcm_hdmi_copy] u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {

                PRINTK_AUD_HDMI2("[mtk_pcm_hdmi_copy]memcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= 0x%x data_w_ptr = %p copy_size = 0x%x\n",
                       Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp, data_w_ptr, copy_size);
                if (copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr, copy_size))
                {
                    PRINTK_AUD_HDMI("AudDrv_write Fail copy from user \n");
                    return -1;
                }
            }

            spin_lock_irqsave(&auddrv_hdmi_lock, flags);
            Afe_Block->u4DataRemained += copy_size;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + copy_size;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            spin_unlock_irqrestore(&auddrv_hdmi_lock, flags);
            data_w_ptr += copy_size;
            count -= copy_size;

            PRINTK_AUD_HDMI2("[mtk_pcm_hdmi_copy] finish1, copy_size:%x, WriteIdx:%x, ReadIdx=%x, DataRemained:%x \r\n",
                               copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);

        }
        else  // copy twice
        {
            kal_uint32 size_1 = 0, size_2 = 0;
            size_1 = Afe_Block->u4BufferSize - Afe_WriteIdx_tmp;
            size_2 = copy_size - size_1;
            if (!access_ok(VERIFY_READ, data_w_ptr, size_1))
            {
                PRINTK_AUD_HDMI("HDMI_write 1ptr invalid data_w_ptr=%p, size_1=%d", data_w_ptr, size_1);
                PRINTK_AUD_HDMI("HDMI_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {

                PRINTK_AUD_HDMI2("[mtk_pcm_hdmi_copy]mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr = %p size_1 = %x\n",
                    Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr,size_1);

                if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), data_w_ptr , size_1)))
                {
                    PRINTK_AUD_HDMI("HDMI_write Fail 1 copy from user");
                    return -1;
                }
            }
            spin_lock_irqsave(&auddrv_hdmi_lock, flags);
            Afe_Block->u4DataRemained += size_1;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_1;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            Afe_WriteIdx_tmp = Afe_Block->u4WriteIdx;
            spin_unlock_irqrestore(&auddrv_hdmi_lock, flags);

            if (!access_ok(VERIFY_READ, data_w_ptr + size_1, size_2))
            {
                PRINTK_AUD_HDMI("HDMI_write 2ptr invalid data_w_ptr=%p, size_1=%d, size_2=%d", data_w_ptr, size_1, size_2);
                PRINTK_AUD_HDMI("HDMI_write u4BufferSize=%d, u4DataRemained=%d", Afe_Block->u4BufferSize, Afe_Block->u4DataRemained);
            }
            else
            {

                PRINTK_AUD_HDMI2("[mtk_pcm_hdmi_copy]mcmcpy Afe_Block->pucVirtBufAddr+Afe_WriteIdx= %x data_w_ptr+size_1 = %p size_2 = %x\n",
                    Afe_Block->pucVirtBufAddr+Afe_WriteIdx_tmp,data_w_ptr+size_1,size_2);
                if ((copy_from_user((Afe_Block->pucVirtBufAddr + Afe_WriteIdx_tmp), (data_w_ptr + size_1), size_2)))
                {
                    PRINTK_AUD_HDMI("[mtk_pcm_hdmi_copy] Fail 2  copy from user");
                    return -1;
                }
            }
            spin_lock_irqsave(&auddrv_hdmi_lock, flags);

            Afe_Block->u4DataRemained += size_2;
            Afe_Block->u4WriteIdx = Afe_WriteIdx_tmp + size_2;
            Afe_Block->u4WriteIdx %= Afe_Block->u4BufferSize;
            spin_unlock_irqrestore(&auddrv_hdmi_lock, flags);
            count -= copy_size;
            data_w_ptr += copy_size;

            PRINTK_AUD_HDMI2("[mtk_pcm_hdmi_copy] finish2, copy size:%x, WriteIdx:%x,ReadIdx=%x DataRemained:%x \r\n",
                   copy_size, Afe_Block->u4WriteIdx, Afe_Block->u4DMAReadIdx, Afe_Block->u4DataRemained);
        }
    }

    return 0;
    //PRINTK_AUD_HDMI("dummy_pcm_copy return \n");
}

static int mtk_pcm_hdmi_silence(struct snd_pcm_substream *substream,
                                int channel, snd_pcm_uframes_t pos,
                                snd_pcm_uframes_t count)
{
    PRINTK_AUD_HDMI("%s \n", __func__);
    return 0; /* do nothing */
}

static void *dummy_page[2];

static struct page *mtk_pcm_page(struct snd_pcm_substream *substream,
                                 unsigned long offset)
{
    PRINTK_AUD_HDMI("%s \n", __func__);
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static struct snd_pcm_ops mtk_hdmi_ops =
{
    .open =     mtk_pcm_hdmi_open,
    .close =    mtk_pcm_hdmi_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_pcm_hdmi_hw_params,
    .hw_free =  mtk_pcm_hdmi_hw_free,
    .prepare =  mtk_pcm_hdmi_prepare,
    .trigger =  mtk_pcm_hdmi_trigger,
    .pointer =  mtk_pcm_hdmi_pointer,
    .copy =     mtk_pcm_hdmi_copy,
    .silence =  mtk_pcm_hdmi_silence,
    .page =     mtk_pcm_page,
};

static struct snd_soc_platform_driver mtk_hdmi_soc_platform =
{
    .ops        = &mtk_hdmi_ops,
    .pcm_new    = mtk_asoc_pcm_hdmi_new,
    .probe      = mtk_afe_hdmi_probe,
};

static int mtk_hdmi_probe(struct platform_device *pdev)
{
    PRINTK_AUD_HDMI("%s \n", __func__);

    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
    if (!pdev->dev.dma_mask)
    {
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_HDMI_PCM);
    }

    PRINTK_AUD_HDMI("%s: dev name %s\n", __func__, dev_name(&pdev->dev));


    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_hdmi_soc_platform);
}

static int mtk_asoc_pcm_hdmi_new(struct snd_soc_pcm_runtime *rtd)
{
    int ret = 0;
    pruntimehdmi  = rtd;
    PRINTK_AUD_HDMI("%s\n", __func__);
    return ret;
}

static int mtk_afe_hdmi_probe(struct snd_soc_platform *platform)
{

    HDMI_dma_buf = kmalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
    memset((void *)HDMI_dma_buf, 0, sizeof(struct snd_dma_buffer));
    PRINTK_AUD_HDMI("mtk_afe_hdmi_probe dma_alloc_coherent HDMI_dma_buf->addr=0x%lx \n", (long)HDMI_dma_buf->addr);

    HDMI_dma_buf->area = dma_alloc_coherent(platform->dev,
                                            HDMI_MAX_BUFFER_SIZE,
                                            &HDMI_dma_buf->addr, GFP_KERNEL);  ///* virtual pointer */
    if (HDMI_dma_buf->area)
    {
        HDMI_dma_buf->bytes = HDMI_MAX_BUFFER_SIZE;
    }
    snd_soc_add_platform_controls(platform, Audio_snd_hdmi_controls,
                                  ARRAY_SIZE(Audio_snd_hdmi_controls));
    return 0;
}

static int mtk_afe_remove(struct platform_device *pdev)
{
    PRINTK_AUD_HDMI("%s \n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_hdmi_of_ids[] =
{
    { .compatible = "mediatek,mt_soc_pcm_hdmi", },
    {}
};
#endif

static struct platform_driver mtk_hdmi_driver =
{
    .driver = {
        .name = MT_SOC_HDMI_PCM,
        .owner = THIS_MODULE,
        #ifdef CONFIG_OF
        .of_match_table = mt_soc_pcm_hdmi_of_ids,
        #endif
    },
    .probe = mtk_hdmi_probe,
    .remove = mtk_afe_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkhdmi_dev;
#endif

static int __init mtk_hdmi_soc_platform_init(void)
{
    int ret;
    PRINTK_AUD_HDMI("%s \n", __func__);
	#ifndef CONFIG_OF
    soc_mtkhdmi_dev = platform_device_alloc(MT_SOC_HDMI_PCM, -1);
    if (!soc_mtkhdmi_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkhdmi_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkhdmi_dev);
        return ret;
    }
    #endif
    ret = platform_driver_register(&mtk_hdmi_driver);
    return ret;

}
module_init(mtk_hdmi_soc_platform_init);

static void __exit mtk_hdmi_soc_platform_exit(void)
{
    PRINTK_AUD_HDMI("%s \n", __func__);

    platform_driver_unregister(&mtk_hdmi_driver);
}
module_exit(mtk_hdmi_soc_platform_exit);


//EXPORT_SYMBOL(Auddrv_Hdmi_Interrupt_Handler);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");


