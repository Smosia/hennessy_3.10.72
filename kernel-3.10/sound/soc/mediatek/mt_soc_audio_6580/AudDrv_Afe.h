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
 *   MT6580  Audio Driver Afe Register setting
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 *   Ir Lian (mtk00976)
 *   Harvey Huang (mtk03996)
 *   Chipeng Chang (mtk02308)
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/

#ifndef _AUDDRV_AFE_H_
#define _AUDDRV_AFE_H_

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include <linux/types.h>

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

typedef enum {
	AFE_MEM_NONE = 0,
	AFE_MEM_DL1,
	AFE_MEM_DL1_DATA2,
	AFE_MEM_DL2,
	AFE_MEM_VUL,
	AFE_MEM_DAI,
	AFE_MEM_I2S,
	AFE_MEM_AWB,
	AFE_MEM_MOD_PCM,
} MEM_INTERFACE_T;

typedef enum {
	AFE_8000HZ = 0,
	AFE_11025HZ = 1,
	AFE_12000HZ = 2,
	AFE_16000HZ = 3,
	AFE_22050HZ = 4,
	AFE_24000HZ = 5,
	AFE_32000HZ = 6,
	AFE_44100HZ = 7,
	AFE_48000HZ = 8
} SAMPLINGRATE_T;

typedef enum {
	AFE_DAIMOD_8000HZ = 0x0,
	AFE_DAIMOD_16000HZ = 0x1,
} DAIMOD_SAMPLINGRATE_T;

typedef enum {
	AFE_STEREO = 0x0,
	AFE_MONO = 0x1
} MEMIF_CH_CFG_T;

typedef enum {
	AFE_MONO_USE_L = 0x0,
	AFE_MONO_USE_R = 0x1
} MEMIF_MONO_SEL_T;

typedef enum {
	AFE_DUP_WR_DISABLE = 0x0,
	AFE_DUP_WR_ENABLE = 0x1
} MEMIF_DUP_WRITE_T;

typedef struct {
	uint32 u4AFE_MEMIF_BUF_BASE;
	uint32 u4AFE_MEMIF_BUF_END;
	uint32 u4AFE_MEMIF_BUF_WP;
	uint32 u4AFE_MEMIF_BUF_RP;
} MEMIF_BUF_T;

typedef struct {
	MEM_INTERFACE_T eMemInterface;
	SAMPLINGRATE_T eSamplingRate;
	DAIMOD_SAMPLINGRATE_T eDaiModSamplingRate;
	MEMIF_CH_CFG_T eChannelConfig;
	 MEMIF_MONO_SEL_T eMonoSelect;	/* Used when AWB and VUL and data is mono */
	MEMIF_DUP_WRITE_T eDupWrite;	/* Used when MODPCM and DAI */
	MEMIF_BUF_T rBufferSetting;
} MEMIF_CONFIG_T;

/* I2S related */
typedef enum {
	I2S_EIAJ = 0x0,
	I2S_I2S = 0x1
} I2SFMT_T;

typedef enum {
	I2S_16BIT = 0x0,
	I2S_32BIT = 0x1
} I2SWLEN_T;

typedef enum {
	I2S_NOSWAP = 0x0,
	I2S_LRSWAP = 0x1
} I2SSWAP_T;

typedef enum {
	I2S_DISABLE = 0x0,
	I2S_ENABLE = 0x1
} I2SEN_T;

typedef enum {
	I2S_MASTER = 0x0,
	I2S_SLAVE = 0x1
} I2SSRC_T;

typedef enum {
	I2S_OUT = 0x0,
	I2S_IN = 0x1
} I2SDIR_T;


/* PCM related */
typedef enum {
	PCM_1 = 0x0,		/* (O7, O8, I9) */
	PCM_2 = 0x1		/* (O17, O18, I14) */
} PCM_MODULE;

typedef enum {
	PCM_DISABLE = 0x0,
	PCM_ENABLE = 0x1
} PCMEN_T;

typedef enum {
	PCM_I2S = 0x0,
	PCM_EIAJ = 0x1,
	PCM_MODEA = 0x2,
	PCM_MODEB = 0x3
} PCMFMT_T;

typedef enum {
	PCM_8K = 0x0,
	PCM_16K = 0x1
} PCMMODE_T;

typedef enum {
	PCM_16BIT = 0x0,
	PCM_32BIT = 0x1
} PCMWLEN_T;


typedef enum {
	PCM_MASTER = 0x0,
	PCM_SLAVE = 0x1
} PCMCLKSRC_T;


typedef enum {
	PCM_GO_ASRC = 0x0,	/* (ASRC)       Set to 0 when source & destination uses different crystal */
	PCM_GO_ASYNC_FIFO = 0x1	/* (Async FIFO) Set to 1 when source & destination uses same crystal */
} PCMBYPASRC_T;


typedef enum {
	PCM_DMTX = 0x0,		/* dual mic on TX */
	PCM_SMTX = 0x1		/* single mic on TX (In BT mode, only L channel data is sent on PCM TX.) */
} PCMBTMODE_T;


typedef enum {
	PCM_SYNC_LEN_1_BCK = 0x0,
	PCM_SYNC_LEN_N_BCK = 0x1
} PCMSYNCTYPE_T;

typedef enum {
	PCM_INT_MD = 0x0,
	PCM_EXT_MD = 0x1
} PCMEXTMODEM_T;


typedef enum {
	PCM_VBT_16K_MODE_DISABLE = 0x0,
	PCM_VBT_16K_MODE_ENABLE = 0x1
} PCMVBT16KMODE_T;


typedef enum {
	PCM_NOINV = 0x0,
	PCM_INV = 0x1
} PCMCLKINV_T;

typedef enum {
	PCM_LB_DISABLE = 0x0,
	PCM_LB_ENABLE = 0x1
} PCMLOOPENA_T;

typedef enum {
	PCM_TXFIX_OFF = 0x0,
	PCM_TXFIX_ON = 0x1
} PCMTXFIXEN_T;

typedef struct {
	PCMFMT_T ePcmFmt;
	PCMMODE_T ePcm8k16kmode;
	PCMWLEN_T ePcmWlen;
	PCMCLKSRC_T ePcmClkSrc;
	PCMBYPASRC_T ePcmBypassASRC;
	PCMEXTMODEM_T ePcmModemSel;
	PCMVBT16KMODE_T ePcmVbt16kSel;
} PCM_INFO_T;


/* BT PCM */
typedef enum {
	BTPCM_DISABLE = 0x0,
	BTPCM_ENABLE = 0x1
} BTPCMEN_T;

typedef enum {
	BTPCM_8K = 0x0,
	BTPCM_16K = 0x1
} BTPCMMODE_T;



/* Interconnection related */
typedef enum {
	I00 = 0,
	I01 = 1,
	I02 = 2,
	I03 = 3,
	I04 = 4,
	I05 = 5,
	I06 = 6,
	I07 = 7,
	I08 = 8,
	I09 = 9,
	IN_MAX
} ITRCON_IN_T;

typedef enum {
	O00 = 0,
	O01 = 1,
	O02 = 2,
	O03 = 3,
	O04 = 4,
	O05 = 5,
	O06 = 6,
	O07 = 7,
	O08 = 8,
	O09 = 9,
	O010 = 10,
	O011 = 11,
	O012 = 12,
	OUT_MAX
} ITRCON_OUT_T;

/* IRQ related */
typedef enum {
	IRQ1 = 1,
	IRQ2 = 2,
	IRQ5 = 3,		/* HDMI */
	IRQ6 = 4,		/* SPDIF */
	IRQMAX
} IRQTYPE_T;


/* Side tone filter related */
typedef enum {
	I3I4 = 0,
	HW_SINE = 1,
	I5I6 = 2,
} STF_SRC_T;

/* Sine wave generator related */
typedef enum {
	SINE_TONE_CH1 = 0,
	SINE_TONE_CH2 = 1,
	SINE_TONE_STEREO = 2
} SINE_TONE_CH_T;

typedef enum {
	SINE_TONE_128 = 0,
	SINE_TONE_64 = 1,
	SINE_TONE_32 = 2,
	SINE_TONE_16 = 3,
	SINE_TONE_8 = 4,
	SINE_TONE_4 = 5,
	SINE_TONE_2 = 6,
	SINE_TONE_1 = 7
} SINE_TONE_AMP_T;

typedef enum {
	SINE_TONE_8K = 0,
	SINE_TONE_11K = 1,
	SINE_TONE_12K = 2,
	SINE_TONE_16K = 3,
	SINE_TONE_22K = 4,
	SINE_TONE_24K = 5,
	SINE_TONE_32K = 6,
	SINE_TONE_44K = 7,
	SINE_TONE_48K = 8,
	SINE_TONE_LOOPBACK = 9
} SINE_TONE_SINEMODE_T;

typedef enum {
	SINE_TONE_LOOPBACK_I0_I1 = 0,
	SINE_TONE_LOOPBACK_I2 = 1,
	SINE_TONE_LOOPBACK_I3_I4 = 2,
	SINE_TONE_LOOPBACK_I5_I6 = 3,
	SINE_TONE_LOOPBACK_I7_I8 = 4,
	SINE_TONE_LOOPBACK_I9_I10 = 5,
	SINE_TONE_LOOPBACK_I11_I12 = 6,
	SINE_TONE_LOOPBACK_O0_O1 = 7,
	SINE_TONE_LOOPBACK_O2 = 8,
	SINE_TONE_LOOPBACK_O3_O4 = 9,
	SINE_TONE_LOOPBACK_O5_O6 = 10,
	SINE_TONE_LOOPBACK_O7_O8 = 11,
	SINE_TONE_LOOPBACK_O9_O10 = 12,
	SINE_TONE_LOOPBACK_O11 = 13,
	SINE_TONE_LOOPBACK_O12 = 14
} SINE_TONE_LOOPBACK_T;

typedef struct {
	uint32 u4ch1_freq_div;	/* 64/n sample/period */
	SINE_TONE_AMP_T rch1_amp_div;
	SINE_TONE_SINEMODE_T rch1_sine_mode;
	uint32 u4ch2_freq_div;	/* 64/n sample/period */
	SINE_TONE_AMP_T rch2_amp_div;
	SINE_TONE_SINEMODE_T rch2_sine_mode;
	SINE_TONE_LOOPBACK_T rloopback_mode;
} AFE_SINEGEN_INFO_T;


/*****************************************************************************
 *                          C O N S T A N T S
 *****************************************************************************/
#define AUDIO_HW_PHYSICAL_BASE  (0x11140000L)
#define AUDIO_HW_VIRTUAL_BASE   (0xF2070000L)

#ifdef AUDIO_MEM_IOREMAP
#define AFE_BASE                (0L)
#else
#define AFE_BASE                (AUDIO_HW_VIRTUAL_BASE)
#endif

/* Internal sram: 0x12004000~0x12007FFF (16K) */
#define AFE_INTERNAL_SRAM_PHY_BASE  (0x11141000L)
#define AFE_INTERNAL_SRAM_VIR_BASE  (AUDIO_HW_VIRTUAL_BASE - 0x70000+0x8000)
#define AFE_INTERNAL_SRAM_SIZE  (0x4000)

/* Dram */
#define AFE_EXTERNAL_DRAM_SIZE  (0x4000)
/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/

/*****************************************************************************
 *                  R E G I S T E R       D E F I N I T I O N
 *****************************************************************************/

#ifdef AUDIO_TOP_CON0
#undef AUDIO_TOP_CON0
#define AUDIO_TOP_CON0  (AFE_BASE + 0x0000)
#endif
#define AUDIO_TOP_CON0  (AFE_BASE + 0x0000)

#define AUDIO_TOP_CON1  (AFE_BASE + 0x0004)
#define AUDIO_TOP_CON2  (AFE_BASE + 0x0008)
#define AUDIO_TOP_CON3  (AFE_BASE + 0x000C)
#define AFE_DAC_CON0    (AFE_BASE + 0x0010)
#define AFE_DAC_CON1    (AFE_BASE + 0x0014)
#define AFE_I2S_CON     (AFE_BASE + 0x0018)

#define AFE_CONN0       (AFE_BASE + 0x0020)
#define AFE_CONN1       (AFE_BASE + 0x0024)
#define AFE_CONN2       (AFE_BASE + 0x0028)
#define AFE_CONN3       (AFE_BASE + 0x002C)
#define AFE_CONN4       (AFE_BASE + 0x0030)

#define AFE_I2S_CON1    (AFE_BASE + 0x0034)
#define AFE_I2S_CON2    (AFE_BASE + 0x0038)

/* Memory interface */
#define AFE_DL1_BASE    (AFE_BASE + 0x0040)
#define AFE_DL1_CUR     (AFE_BASE + 0x0044)
#define AFE_DL1_END     (AFE_BASE + 0x0048)

#define AFE_I2S_CON3    (AFE_BASE + 0x004C)
#define AFE_DL2_BASE    (AFE_BASE + 0x0050)
#define AFE_DL2_CUR     (AFE_BASE + 0x0054)
#define AFE_DL2_END     (AFE_BASE + 0x0058)
#define AFE_AWB_BASE    (AFE_BASE + 0x0070)
#define AFE_AWB_END     (AFE_BASE + 0x0078)
#define AFE_AWB_CUR     (AFE_BASE + 0x007C)
#define AFE_VUL_BASE    (AFE_BASE + 0x0080)
#define AFE_VUL_END     (AFE_BASE + 0x0088)
#define AFE_VUL_CUR     (AFE_BASE + 0x008C)

/* Memory interface monitor */
#define AFE_MEMIF_MON0 (AFE_BASE + 0x00D0)
#define AFE_MEMIF_MON1 (AFE_BASE + 0x00D4)
#define AFE_MEMIF_MON2 (AFE_BASE + 0x00D8)
#define AFE_MEMIF_MON4 (AFE_BASE + 0x00E0)

#define AFE_ADDA_DL_SRC2_CON0   (AFE_BASE+0x00108)
#define AFE_ADDA_DL_SRC2_CON1   (AFE_BASE+0x0010C)
#define AFE_ADDA_UL_SRC_CON0    (AFE_BASE+0x00114)
#define AFE_ADDA_UL_SRC_CON1    (AFE_BASE+0x00118)
#define AFE_ADDA_TOP_CON0       (AFE_BASE+0x00120)
#define AFE_ADDA_UL_DL_CON0     (AFE_BASE+0x00124)
#define AFE_ADDA_SRC_DEBUG      (AFE_BASE+0x0012C)
#define AFE_ADDA_SRC_DEBUG_MON0 (AFE_BASE+0x00130)
#define AFE_ADDA_SRC_DEBUG_MON1 (AFE_BASE+0x00134)
#define AFE_ADDA_NEWIF_CFG0     (AFE_BASE+0x00138)
#define AFE_ADDA_NEWIF_CFG1     (AFE_BASE+0x0013C)

#define AFE_SIDETONE_DEBUG  (AFE_BASE + 0x01D0)
#define AFE_SIDETONE_MON    (AFE_BASE + 0x01D4)
#define AFE_SIDETONE_CON0   (AFE_BASE + 0x01E0)
#define AFE_SIDETONE_COEFF  (AFE_BASE + 0x01E4)
#define AFE_SIDETONE_CON1   (AFE_BASE + 0x01E8)
#define AFE_SIDETONE_GAIN   (AFE_BASE + 0x01EC)

#define AFE_SGEN_CON0   (AFE_BASE + 0x01F0)
#define AFE_TOP_CON0    (AFE_BASE + 0x0200)

#define AFE_ADDA_PREDIS_CON0    (AFE_BASE+0x00260)
#define AFE_ADDA_PREDIS_CON1    (AFE_BASE+0x00264)

#define AFE_MOD_DAI_BASE (AFE_BASE + 0x0330)
#define AFE_MOD_DAI_END  (AFE_BASE + 0x0338)
#define AFE_MOD_DAI_CUR  (AFE_BASE + 0x033C)

#define AFE_IRQ_MCU_CON             (AFE_BASE + 0x03A0)
#define AFE_IRQ_MCU_STATUS          (AFE_BASE + 0x03A4)
#define AFE_IRQ_MCU_CLR             (AFE_BASE + 0x03A8)
#define AFE_IRQ_MCU_CNT1            (AFE_BASE + 0x03AC)
#define AFE_IRQ_MCU_CNT2            (AFE_BASE + 0x03B0)
#define AFE_IRQ_MCU_MON2            (AFE_BASE + 0x03B8)

#define AFE_IRQ1_MCU_CNT_MON        (AFE_BASE + 0x03C0)

#define AFE_IRQ2_MCU_CNT_MON        (AFE_BASE + 0x03C4)
#define AFE_IRQ1_MCU_EN_CNT_MON     (AFE_BASE + 0x03C8)

#define AFE_MEMIF_MINLEN	(AFE_BASE + 0x03D0)
#define AFE_MEMIF_MAXLEN	(AFE_BASE + 0x03D4)
#define AFE_MEMIF_PBUF_SIZE         (AFE_BASE + 0x03D8)

/* AFE GAIN CONTROL REGISTER */
#define AFE_GAIN1_CON0         (AFE_BASE + 0x0410)
#define AFE_GAIN1_CON1         (AFE_BASE + 0x0414)
#define AFE_GAIN1_CON2         (AFE_BASE + 0x0418)
#define AFE_GAIN1_CON3         (AFE_BASE + 0x041C)
#define AFE_GAIN1_CONN         (AFE_BASE + 0x0420)
#define AFE_GAIN1_CUR          (AFE_BASE + 0x0424)
#define AFE_GAIN2_CON0         (AFE_BASE + 0x0428)
#define AFE_GAIN2_CON1         (AFE_BASE + 0x042C)
#define AFE_GAIN2_CON2         (AFE_BASE + 0x0430)
#define AFE_GAIN2_CON3         (AFE_BASE + 0x0434)
#define AFE_GAIN2_CONN         (AFE_BASE + 0x0438)
#define AFE_GAIN2_CUR          (AFE_BASE + 0x043C)
#define AFE_GAIN2_CONN2        (AFE_BASE + 0x0440)

/* here is only fpga needed */
#define FPGA_CFG2           (AFE_BASE + 0x4B8)
#define FPGA_CFG3           (AFE_BASE + 0x4BC)
#define FPGA_CFG0           (AFE_BASE + 0x4C0)
#define FPGA_CFG1           (AFE_BASE + 0x4C4)
/* #define FPGA_VER               (AFE_BASE + 0x4C8) */
#define FPGA_STC                (AFE_BASE + 0x4CC)

#define AFE_ASRC_CON0   (AFE_BASE + 0x500)
#define AFE_ASRC_CON1   (AFE_BASE + 0x504)
#define AFE_ASRC_CON2   (AFE_BASE + 0x508)
#define AFE_ASRC_CON3   (AFE_BASE + 0x50C)
#define AFE_ASRC_CON4   (AFE_BASE + 0x510)
#define AFE_ASRC_CON5   (AFE_BASE + 0x514)
#define AFE_ASRC_CON6   (AFE_BASE + 0x518)
#define AFE_ASRC_CON7   (AFE_BASE + 0x51C)
#define AFE_ASRC_CON8   (AFE_BASE + 0x520)
#define AFE_ASRC_CON9   (AFE_BASE + 0x524)
#define AFE_ASRC_CON10  (AFE_BASE + 0x528)
#define AFE_ASRC_CON11  (AFE_BASE + 0x52C)

#define PCM_INTF_CON    (AFE_BASE + 0x530)
#define PCM_INTF_CON2   (AFE_BASE + 0x538)
#define PCM2_INTF_CON   (AFE_BASE + 0x53C)

/* Add */
#define AFE_ASRC_CON13  (AFE_BASE+0x550)
#define AFE_ASRC_CON14  (AFE_BASE+0x554)
#define AFE_ASRC_CON15  (AFE_BASE+0x558)
#define AFE_ASRC_CON16  (AFE_BASE+0x55C)
#define AFE_ASRC_CON17  (AFE_BASE+0x560)
#define AFE_ASRC_CON18  (AFE_BASE+0x564)
#define AFE_ASRC_CON19  (AFE_BASE+0x568)
#define AFE_ASRC_CON20  (AFE_BASE+0x56C)
#define AFE_ASRC_CON21  (AFE_BASE+0x570)

#define AFE_REGISTER_OFFSET (0x574)

/* #define AFE_MASK_ALL  (0xffffffff) */

#define AFE_MAXLENGTH           (AFE_BASE + AFE_REGISTER_OFFSET)


/* do afe register ioremap */
void Auddrv_Reg_map(void);

void Afe_Set_Reg(uint32 offset, uint32 value, uint32 mask);
uint32 Afe_Get_Reg(uint32 offset);

/* for debug usage */
void Afe_Log_Print(void);

/* function to get pointer */
dma_addr_t Get_Afe_Sram_Phys_Addr(void);
dma_addr_t Get_Afe_Sram_Capture_Phys_Addr(void);
void *Get_Afe_SramBase_Pointer(void);
void *Get_Afe_SramCaptureBase_Pointer(void);
#endif
