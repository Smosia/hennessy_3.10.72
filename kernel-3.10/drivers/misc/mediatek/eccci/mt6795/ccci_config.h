#ifndef ECCCI_INTERNAL_OPTION
#define ECCCI_INTERNAL_OPTION
/*================================================ */
/*Bool option part*/
/*================================================*/
/*#define CCCI_STATISTIC*/
#define CCCI_USE_NAPI
#define FEATURE_GET_MD_GPIO_NUM
#define FEATURE_GET_MD_GPIO_VAL

#ifdef CONFIG_ARM64
#define FEATURE_GET_MD_PMIC_ADC_NUM
#define FEATURE_GET_MD_PMIC_ADC_VAL
#else
#define FEATURE_GET_MD_ADC_NUM
#define FEATURE_GET_MD_ADC_VAL
#endif

#define FEATURE_GET_MD_EINT_ATTR
/*#define FEATURE_GET_MD_BAT_VOL*/
#define FEATURE_PM_IPO_H
/*#define FEATURE_DFO_EN*/
#define FEATURE_SEQ_CHECK_EN
#define FEATURE_POLL_MD_EN
#if 0 /*DEPRECATED */
#define FEATURE_GET_TD_EINT_NUM
#define FEATURE_GET_DRAM_TYPE_CLK
#endif

#define ENABLE_EMI_PROTECTION
#define ENABLE_DRAM_API
#define ENABLE_MEM_REMAP_HW
/*#define ENABLE_CHIP_VER_CHECK*/
/*#define ENABLE_2G_3G_CHECK*/
#define ENABLE_MEM_SIZE_CHECK
/*#define ENABLE_MD_WDT_DBG*/

#ifdef CONFIG_MTK_CONN_MD
#define FEATURE_CONN_MD_EXP_EN
#endif

#define FEATURE_USING_4G_MEMORY_API

#define FEATURE_RF_CLK_BUF
/*#define FEATURE_LOW_BATTERY_SUPPORT */

/*================================================ */
/* misc size description */
#define CCCI_SMEM_DUMP_SIZE      4096 /* smem size we dump when EE */
#define CCCC_SMEM_CCIF_SRAM_SIZE 16 /* SRAM size we dump into smem */
#define CCCI_SMEM_SLEEP_MODE_DBG_DUMP 512 /* only dump first 512bytes in sleep mode info */
/*================================================*/
/* share memory region description */
#define CCCI_SMEM_OFFSET_EXCEPTION 0 /* offset in whole share memory region */
#define CCCI_SMEM_SIZE_EXCEPTION (64*1024) /* exception smem total size */
/*== subset of exception region begains ==*/
#define CCCI_SMEM_OFFSET_CCCI_DEBUG CCCI_SMEM_OFFSET_EXCEPTION /* MD CCCI debug info */
#define CCCI_SMEM_SIZE_CCCI_DEBUG 2048
#define CCCI_SMEM_OFFSET_MDSS_DEBUG (CCCI_SMEM_OFFSET_EXCEPTION+CCCI_SMEM_SIZE_CCCI_DEBUG) /* MD SS debug info */
#define CCCI_SMEM_OFFSET_CCIF_SRAM (CCCI_SMEM_OFFSET_MDSS_DEBUG+1024-CCCC_SMEM_CCIF_SRAM_SIZE)
#define CCCI_SMEM_OFFSET_EPON (CCCI_SMEM_OFFSET_EXCEPTION+0xC64)
#define CCCI_SMEM_OFFSET_SEQERR (CCCI_SMEM_OFFSET_EXCEPTION+0x34)
#define CCCI_SMEM_SIZE_MDSS_DEBUG 2048 /* MD SS debug info size except MD1 */
#define CCCI_SMEM_SIZE_SLEEP_MODE_DBG 1024 /* MD sleep mode debug info section in exception region tail */
#define CCCI_SMEM_OFFSET_SLEEP_MODE_DBG (CCCI_SMEM_OFFSET_EXCEPTION+CCCI_SMEM_SIZE_EXCEPTION \
										-CCCI_SMEM_SIZE_SLEEP_MODE_DBG)
/*== subset of exception region ends ==*/
#define CCCI_SMEM_OFFSET_RUNTIME (CCCI_SMEM_OFFSET_EXCEPTION+CCCI_SMEM_SIZE_EXCEPTION)
#define CCCI_SMEM_SIZE_RUNTIME_AP 0 /* AP runtime data size */
#define CCCI_SMEM_SIZE_RUNTIME_MD 0 /* MD runtime data size */
#define CCCI_SMEM_SIZE_RUNTIME	(CCCI_SMEM_SIZE_RUNTIME_AP+CCCI_SMEM_SIZE_RUNTIME_MD)
/*================================================ */

/*================================================ */
/*Configure value option part*/
/*================================================*/
#ifdef CONFIG_ARM64
#define AP_PLATFORM_INFO    "MT6795E1"
#else
#define AP_PLATFORM_INFO    "MT6595E2"
#endif
#define CCCI_MTU            (3584-128)
#define CCCI_NET_MTU        (1500)
#define SKB_POOL_SIZE_4K    (256)	/*2*MD */
#define SKB_POOL_SIZE_1_5K  (256)	/*2*MD */
#define SKB_POOL_SIZE_16    (64)	/*2*MD */
#define BM_POOL_SIZE        (SKB_POOL_SIZE_4K+SKB_POOL_SIZE_1_5K+SKB_POOL_SIZE_16)
#define RELOAD_TH            3	/*reload pool if pool size dropped below 1/RELOAD_TH */
#define MD_HEADER_VER_NO    (3)
#define MEM_LAY_OUT_VER     (1)
#define AP_MD_HS_V2          2	/*handshake version*/

#define CCCI_MEM_ALIGN      (SZ_32M)
#define CCCI_SMEM_ALIGN_MD1 (0x200000)	/*2M */
#define CCCI_SMEM_ALIGN_MD2 (0x200000)	/*2M */

#define CURR_SEC_CCCI_SYNC_VER (1)	/*Note: must sync with sec lib, if ccci and sec has dependency change */
#define CCCI_DRIVER_VER     0x20110118

#define CCCI_LOG_LEVEL 5
#endif
