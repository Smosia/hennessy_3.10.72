#ifndef AUDDRV_BTCVSD_H
#define AUDDRV_BTCVSD_H

#include <mach/mt_typedefs.h>
#include "AudioBTCVSDDef.h"

#undef DEBUG_AUDDRV
#ifdef DEBUG_AUDDRV
#define PRINTK_AUDDRV(format, args...) pr_debug(format, ##args)
#else
#define PRINTK_AUDDRV(format, args...)
#endif

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
	BT_SCO_TXSTATE_IDLE = 0x0,
	BT_SCO_TXSTATE_INIT,
	BT_SCO_TXSTATE_READY,
	BT_SCO_TXSTATE_RUNNING,
	BT_SCO_TXSTATE_ENDING,
	BT_SCO_RXSTATE_IDLE = 0x10,
	BT_SCO_RXSTATE_INIT,
	BT_SCO_RXSTATE_READY,
	BT_SCO_RXSTATE_RUNNING,
	BT_SCO_RXSTATE_ENDING,
	BT_SCO_TXSTATE_DIRECT_LOOPBACK
} CVSD_STATE;

typedef enum {
	BT_SCO_DIRECT_BT2ARM,
	BT_SCO_DIRECT_ARM2BT
} BT_SCO_DIRECT;


typedef enum {
	BT_SCO_CVSD_30 = 0,
	BT_SCO_CVSD_60 = 1,
	BT_SCO_CVSD_90 = 2,
	BT_SCO_CVSD_120 = 3,
	BT_SCO_CVSD_10 = 4,
	BT_SCO_CVSD_20 = 5,
	BT_SCO_CVSD_MAX = 6
} BT_SCO_PACKET_LEN;


typedef struct {
	dma_addr_t pucTXPhysBufAddr;
	dma_addr_t pucRXPhysBufAddr;
	kal_uint8 *pucTXVirtBufAddr;
	kal_uint8 *pucRXVirtBufAddr;
	kal_int32 u4TXBufferSize;
	kal_int32 u4RXBufferSize;
} CVSD_MEMBLOCK_T;


typedef struct {
	kal_uint8 PacketBuf[SCO_RX_PACKER_BUF_NUM][SCO_RX_PLC_SIZE + BTSCO_CVSD_PACKET_VALID_SIZE];
	kal_bool PacketValid[SCO_RX_PACKER_BUF_NUM];
	int iPacket_w;
	int iPacket_r;
	kal_uint8 TempPacketBuf[BT_SCO_PACKET_180];
	kal_bool fOverflow;
	kal_uint32 u4BufferSize;	/*RX packetbuf size*/
} BT_SCO_RX_T;


typedef struct {
	kal_uint8 PacketBuf[SCO_TX_PACKER_BUF_NUM][SCO_TX_ENCODE_SIZE];
	kal_int32 iPacket_w;
	kal_int32 iPacket_r;
	kal_uint8 TempPacketBuf[BT_SCO_PACKET_180];
	kal_bool fUnderflow;
	kal_uint32 u4BufferSize;	/*TX packetbuf size*/
} BT_SCO_TX_T;

CVSD_MEMBLOCK_T BT_CVSD_Mem;

/* here is temp address for ioremap BT hardware register*/
void *BTSYS_PKV_BASE_ADDRESS = 0;
void *BTSYS_SRAM_BANK2_BASE_ADDRESS = 0;
void *AUDIO_INFRA_BASE_VIRTUAL = 0;

#ifdef CONFIG_OF
unsigned long btsys_pkv_physical_base = 0;
unsigned long btsys_sram_bank2_physical_base = 0;
unsigned long infra_base = 0;
/* INFRA_MISC address=AUDIO_INFRA_BASE_PHYSICAL + INFRA_MISC_OFFSET */
unsigned long infra_misc_offset = 0;
/* bit 11 of INFRA_MISC */
unsigned long conn_bt_cvsd_mask = 0;
/* BTSYS_PKV_BASE_ADDRESS+cvsd_mcu_read_offset */
unsigned long cvsd_mcu_read_offset = 0;
/* BTSYS_PKV_BASE_ADDRESS+cvsd_mcu_write_offset */
unsigned long cvsd_mcu_write_offset = 0;
/* BTSYS_PKV_BASE_ADDRESS+cvsd_packet_indicator */
unsigned long cvsd_packet_indicator = 0;
#else
#define AUDIO_BTSYS_PKV_PHYSICAL_BASE  (0x18000000)
#define AUDIO_BTSYS_SRAM_BANK2_PHYSICAL_BASE  (0x18080000)
#define AUDIO_INFRA_BASE_PHYSICAL (0x10000000)
/* INFRA_MISC address=AUDIO_INFRA_BASE_PHYSICAL + INFRA_MISC_OFFSET */
#define INFRA_MISC_OFFSET (0x0700)
/* bit 11 of INFRA_MISC */
#define conn_bt_cvsd_mask (0x00000800)
#define CVSD_MCU_READ_OFFSET (0xFD0)
#define CVSD_MCU_WRITE_OFFSET (0xFD4)
#define CVSD_PACKET_INDICATOR (0xFD8)
#endif
#define AP_BT_CVSD_IRQ_LINE (260)
u32 btcvsd_irq_number = AP_BT_CVSD_IRQ_LINE;

#endif
