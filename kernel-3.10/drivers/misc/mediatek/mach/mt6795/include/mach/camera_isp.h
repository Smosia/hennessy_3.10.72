#ifndef _MT_ISP_H
#define _MT_ISP_H

#include <linux/ioctl.h>

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

/**
    function declaration
*/
extern void mt_irq_set_sens(unsigned int irq, unsigned int sens);
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);
/**
    enforce kernel log enable
*/
/* #define KERNEL_LOG  //enable debug log flag if defined */
#define ISR_LOG_ON		/* turn on log print at isr if defined */

#define T_STAMP_2_0		/* time stamp workaround method. (increase timestamp baseon fix fps, not read at each isr) */

#define SIG_ERESTARTSYS 512	/* ERESTARTSYS */
/*******************************************************************************
*
********************************************************************************/
#define ISP_DEV_MAJOR_NUMBER    251
#define ISP_MAGIC               'k'
/*******************************************************************************
*
********************************************************************************/
/* CAM_CTL_INT_P1_STATUS */
#define ISP_IRQ_P1_STATUS_VS1_INT_ST                ((unsigned int)1<<0)
#define ISP_IRQ_P1_STATUS_TG1_INT1_ST               ((unsigned int)1<<1)
#define ISP_IRQ_P1_STATUS_TG1_INT2_ST               ((unsigned int)1<<2)
#define ISP_IRQ_P1_STATUS_EXPDON1_ST                ((unsigned int)1<<3)
#define ISP_IRQ_P1_STATUS_TG1_ERR_ST                ((unsigned int)1<<4)
#define ISP_IRQ_P1_STATUS_TG1_GBERR                 ((unsigned int)1<<5)
#define ISP_IRQ_P1_STATUS_CQ0_ERR                   ((unsigned int)1<<6)
#define ISP_IRQ_P1_STATUS_CQ0_VS_ERR_ST             ((unsigned int)1<<7)
#define ISP_IRQ_P1_STATUS_IMGO_DROP_FRAME_ST        ((unsigned int)1<<8)
#define ISP_IRQ_P1_STATUS_RRZO_DROP_FRAME_ST        ((unsigned int)1<<9)
#define ISP_IRQ_P1_STATUS_PASS1_DON_ST              ((unsigned int)1<<10)
#define ISP_IRQ_P1_STATUS_rsv_11                    ((unsigned int)1<<11)
#define ISP_IRQ_P1_STATUS_SOF1_INT_ST               ((unsigned int)1<<12)
#define ISP_IRQ_P1_STATUS_rsv_13                    ((unsigned int)1<<13)
#define ISP_IRQ_P1_STATUS_AF_DON_ST                 ((unsigned int)1<<16)
#define ISP_IRQ_P1_STATUS_FLK_DON_ST                ((unsigned int)1<<17)
#define ISP_IRQ_P1_STATUS_rsv_18                    ((unsigned int)1<<18)
#define ISP_IRQ_P1_STATUS_IMGO_ERR_ST               ((unsigned int)1<<20)
#define ISP_IRQ_P1_STATUS_AAO_ERR_ST                ((unsigned int)1<<21)
#define ISP_IRQ_P1_STATUS_LCSO_ERR_ST               ((unsigned int)1<<22)
#define ISP_IRQ_P1_STATUS_RRZO_ERR_ST               ((unsigned int)1<<23)
#define ISP_IRQ_P1_STATUS_ESFKO_ERR_ST              ((unsigned int)1<<24)
#define ISP_IRQ_P1_STATUS_FLK_ERR_ST                ((unsigned int)1<<25)
#define ISP_IRQ_P1_STATUS_LSC_ERR_ST                ((unsigned int)1<<26)
#define ISP_IRQ_P1_STATUS_FBC_RRZO_DON_ST           ((unsigned int)1<<27)
#define ISP_IRQ_P1_STATUS_FBC_IMGO_DON_ST           ((unsigned int)1<<28)
#define ISP_IRQ_P1_STATUS_rsv_29                    ((unsigned int)1<<29)
#define ISP_IRQ_P1_STATUS_DMA_ERR_ST                ((unsigned int)1<<30)
#define ISP_IRQ_P1_STATUS_rsv_31                    ((unsigned int)1<<31)
/* CAM_CTL_INT_P1_STATUS2 */
#define ISP_IRQ_P1_STATUS2_IMGO_DONE_ST              ((unsigned int)1<<0)
#define ISP_IRQ_P1_STATUS2_UFEO_DONE_ST              ((unsigned int)1<<1)
#define ISP_IRQ_P1_STATUS2_RRZO_DONE_ST              ((unsigned int)1<<2)
#define ISP_IRQ_P1_STATUS2_ESFKO_DONE_ST             ((unsigned int)1<<3)
#define ISP_IRQ_P1_STATUS2_LCSO_DONE_ST              ((unsigned int)1<<4)
#define ISP_IRQ_P1_STATUS2_AAO_DONE_ST               ((unsigned int)1<<5)
#define ISP_IRQ_P1_STATUS2_BPCI_DONE_ST              ((unsigned int)1<<6)
#define ISP_IRQ_P1_STATUS2_LSCI_DONE_ST              ((unsigned int)1<<7)
#define ISP_IRQ_P1_STATUS2_AF_TAR_DONE_ST            ((unsigned int)1<<8)
#define ISP_IRQ_P1_STATUS2_AF_FLO1_DONE_ST           ((unsigned int)1<<9)
#define ISP_IRQ_P1_STATUS2_AF_FLO2_DONE_ST           ((unsigned int)1<<10)
#define ISP_IRQ_P1_STATUS2_AF_FLO3_DONE_ST           ((unsigned int)1<<11)
#define ISP_IRQ_P1_STATUS2_rsv_12                    ((unsigned int)1<<12)
/* CAM_CTL_INT_P1_STATUS_D */
#define ISP_IRQ_P1_STATUS_D_VS1_INT_ST                ((unsigned int)1<<0)
#define ISP_IRQ_P1_STATUS_D_TG1_INT1_ST               ((unsigned int)1<<1)
#define ISP_IRQ_P1_STATUS_D_TG1_INT2_ST               ((unsigned int)1<<2)
#define ISP_IRQ_P1_STATUS_D_EXPDON1_ST                ((unsigned int)1<<3)
#define ISP_IRQ_P1_STATUS_D_TG1_ERR_ST                ((unsigned int)1<<4)
#define ISP_IRQ_P1_STATUS_D_TG1_GBERR                 ((unsigned int)1<<5)
#define ISP_IRQ_P1_STATUS_D_CQ0_ERR                   ((unsigned int)1<<6)
#define ISP_IRQ_P1_STATUS_D_CQ0_VS_ERR_ST             ((unsigned int)1<<7)
#define ISP_IRQ_P1_STATUS_D_IMGO_DROP_FRAME_ST        ((unsigned int)1<<8)
#define ISP_IRQ_P1_STATUS_D_RRZO_DROP_FRAME_ST        ((unsigned int)1<<9)
#define ISP_IRQ_P1_STATUS_D_PASS1_DON_ST              ((unsigned int)1<<10)
#define ISP_IRQ_P1_STATUS_D_rsv_11                    ((unsigned int)1<<11)
#define ISP_IRQ_P1_STATUS_D_SOF1_INT_ST               ((unsigned int)1<<12)
#define ISP_IRQ_P1_STATUS_D_rsv_13                    ((unsigned int)1<<13)
#define ISP_IRQ_P1_STATUS_D_AF_DON_ST                 ((unsigned int)1<<16)
#define ISP_IRQ_P1_STATUS_D_rsv_17                    ((unsigned int)1<<17)
#define ISP_IRQ_P1_STATUS_D_IMGO_ERR_ST               ((unsigned int)1<<20)
#define ISP_IRQ_P1_STATUS_D_AAO_ERR_ST                ((unsigned int)1<<21)
#define ISP_IRQ_P1_STATUS_D_LCSO_ERR_ST               ((unsigned int)1<<22)
#define ISP_IRQ_P1_STATUS_D_RRZO_ERR_ST               ((unsigned int)1<<23)
#define ISP_IRQ_P1_STATUS_D_AFO_ERR_ST                ((unsigned int)1<<24)
#define ISP_IRQ_P1_STATUS_D_rsv_25                    ((unsigned int)1<<25)
#define ISP_IRQ_P1_STATUS_D_LSC_ERR_ST                ((unsigned int)1<<26)
#define ISP_IRQ_P1_STATUS_D_FBC_RRZO_DON_ST           ((unsigned int)1<<27)
#define ISP_IRQ_P1_STATUS_D_FBC_IMGO_DON_ST           ((unsigned int)1<<28)
#define ISP_IRQ_P1_STATUS_D_rsv_29                    ((unsigned int)1<<29)
#define ISP_IRQ_P1_STATUS_D_DMA_ERR_ST                ((unsigned int)1<<30)
#define ISP_IRQ_P1_STATUS_D_rsv_31                    ((unsigned int)1<<31)
/* CAM_CTL_INT_P1_STATUS2_D */
#define ISP_IRQ_P1_STATUS2_D_IMGO_D_DONE_ST            ((unsigned int)1<<0)
#define ISP_IRQ_P1_STATUS2_D_rsv_1                     ((unsigned int)1<<1)
#define ISP_IRQ_P1_STATUS2_D_RRZO_D_DONE_ST            ((unsigned int)1<<2)
#define ISP_IRQ_P1_STATUS2_D_AFO_D_DONE_ST             ((unsigned int)1<<3)
#define ISP_IRQ_P1_STATUS2_D_LCSO_D_DONE_ST            ((unsigned int)1<<4)
#define ISP_IRQ_P1_STATUS2_D_AAO_D_DONE_ST             ((unsigned int)1<<5)
#define ISP_IRQ_P1_STATUS2_D_BPCI_D_DONE_ST            ((unsigned int)1<<6)
#define ISP_IRQ_P1_STATUS2_D_LSCI_D_DONE_ST            ((unsigned int)1<<7)
#define ISP_IRQ_P1_STATUS2_D_AF_TAR_DONE_ST            ((unsigned int)1<<8)
#define ISP_IRQ_P1_STATUS2_D_AF_FLO1_DONE_ST           ((unsigned int)1<<9)
#define ISP_IRQ_P1_STATUS2_D_AF_FLO2_DONE_ST           ((unsigned int)1<<10)
#define ISP_IRQ_P1_STATUS2_D_AF_FLO3_DONE_ST           ((unsigned int)1<<11)
#define ISP_IRQ_P1_STATUS2_D_rsv_12                    ((unsigned int)1<<12)
/* CAM_CTL_SEL_P1 */
#define ISP_CAM_CTL_SEL_P1_IMGO_SEL                    ((unsigned int)1<<6)
/* CAM_CTL_SEL_P1_D */
#define ISP_CAM_CTL_SEL_P1_D_IMGO_SEL                  ((unsigned int)1<<6)


/* CAM_CTL_INT_P2_STATUS */
#define ISP_IRQ_P2_STATUS_CQ_ERR_ST                 ((unsigned int)1<<0)
#define ISP_IRQ_P2_STATUS_PASS2_DON_ST              ((unsigned int)1<<1)
#define ISP_IRQ_P2_STATUS_TILE_DON_ST               ((unsigned int)1<<2)
#define ISP_IRQ_P2_STATUS_CQ_DON_ST                 ((unsigned int)1<<3)
#define ISP_IRQ_P2_STATUS_TDR_ERR_ST                ((unsigned int)1<<4)
#define ISP_IRQ_P2_STATUS_PASS2A_DON_ST             ((unsigned int)1<<5)
#define ISP_IRQ_P2_STATUS_PASS2B_DON_ST             ((unsigned int)1<<6)
#define ISP_IRQ_P2_STATUS_PASS2C_DON_ST             ((unsigned int)1<<7)
#define ISP_IRQ_P2_STATUS_CQ1_DONE_ST               ((unsigned int)1<<8)
#define ISP_IRQ_P2_STATUS_CQ2_DONE_ST               ((unsigned int)1<<9)
#define ISP_IRQ_P2_STATUS_CQ3_DONE_ST               ((unsigned int)1<<10)
#define ISP_IRQ_P2_STATUS_PASS2A_ERR_TRIG_ST        ((unsigned int)1<<11)
#define ISP_IRQ_P2_STATUS_PASS2B_ERR_TRIG_ST        ((unsigned int)1<<12)
#define ISP_IRQ_P2_STATUS_PASS2C_ERR_TRIG_ST        ((unsigned int)1<<13)
#define ISP_IRQ_P2_STATUS_IMGI_DONE_ST              ((unsigned int)1<<15)
#define ISP_IRQ_P2_STATUS_UFDI_DONE_ST              ((unsigned int)1<<16)
#define ISP_IRQ_P2_STATUS_VIPI_DONE_ST              ((unsigned int)1<<17)
#define ISP_IRQ_P2_STATUS_VIP2I_DONE_ST             ((unsigned int)1<<18)
#define ISP_IRQ_P2_STATUS_VIP3I_DONE_ST             ((unsigned int)1<<19)
#define ISP_IRQ_P2_STATUS_LCEI_DONE_ST              ((unsigned int)1<<20)
#define ISP_IRQ_P2_STATUS_MFBO_DONE_ST              ((unsigned int)1<<21)
#define ISP_IRQ_P2_STATUS_IMG2O_DONE_ST             ((unsigned int)1<<22)
#define ISP_IRQ_P2_STATUS_IMG3O_DONE_ST             ((unsigned int)1<<23)
#define ISP_IRQ_P2_STATUS_IMG3BO_DONE_ST            ((unsigned int)1<<24)
#define ISP_IRQ_P2_STATUS_IMG3CO_DONE_ST            ((unsigned int)1<<25)
#define ISP_IRQ_P2_STATUS_FEO_DONE_ST               ((unsigned int)1<<26)
#define ISP_IRQ_P2_STATUS_rsv_27                    ((unsigned int)1<<27)
/* CAM_CTL_INT_STATUSX //P1 AND P2 */
#define ISP_IRQ_STATUSX_VS1_INT_ST                ((unsigned int)1<<0)
#define ISP_IRQ_STATUSX_TG1_INT1_ST               ((unsigned int)1<<1)
#define ISP_IRQ_STATUSX_TG1_INT2_ST               ((unsigned int)1<<2)
#define ISP_IRQ_STATUSX_EXPDON1_ST                ((unsigned int)1<<3)
#define ISP_IRQ_STATUSX_TG1_ERR_ST                ((unsigned int)1<<4)
#define ISP_IRQ_STATUSX_TG1_GBERR                 ((unsigned int)1<<5)
#define ISP_IRQ_STATUSX_CQ0_ERR                   ((unsigned int)1<<6)
#define ISP_IRQ_STATUSX_CQ0_VS_ERR_ST             ((unsigned int)1<<7)
#define ISP_IRQ_STATUSX_IMGO_DROP_FRAME_ST        ((unsigned int)1<<8)
#define ISP_IRQ_STATUSX_RRZO_DROP_FRAME_ST        ((unsigned int)1<<9)
#define ISP_IRQ_STATUSX_PASS1_DON_ST              ((unsigned int)1<<10)
#define ISP_IRQ_STATUSX_rsv_11                    ((unsigned int)1<<11)
#define ISP_IRQ_STATUSX_SOF1_INT_ST               ((unsigned int)1<<12)
#define ISP_IRQ_STATUSX_CQ_ERR_ST                 ((unsigned int)1<<13)
#define ISP_IRQ_STATUSX_PASS2_DON_ST              ((unsigned int)1<<14)
#define ISP_IRQ_STATUSX_TILE_DON_ST               ((unsigned int)1<<15)
#define ISP_IRQ_STATUSX_AF_DON_ST                 ((unsigned int)1<<16)
#define ISP_IRQ_STATUSX_FLK_DON_ST                ((unsigned int)1<<17)
#define ISP_IRQ_STATUSX_rsv_18					  ((unsigned int)1<<18)
#define ISP_IRQ_STATUSX_CQ_DON_ST                 ((unsigned int)1<<19)
#define ISP_IRQ_STATUSX_IMGO_ERR_ST               ((unsigned int)1<<20)
#define ISP_IRQ_STATUSX_AAO_ERR_ST                ((unsigned int)1<<21)
#define ISP_IRQ_STATUSX_LCSO_ERR_ST               ((unsigned int)1<<22)
#define ISP_IRQ_STATUSX_RRZO_ERR_ST               ((unsigned int)1<<23)
#define ISP_IRQ_STATUSX_ESFKO_ERR_ST              ((unsigned int)1<<24)
#define ISP_IRQ_STATUSX_FLK_ERR_ST                ((unsigned int)1<<25)
#define ISP_IRQ_STATUSX_LSC_ERR_ST                ((unsigned int)1<<26)
#define ISP_IRQ_STATUSX_FBC_RRZO_DON_ST           ((unsigned int)1<<27)
#define ISP_IRQ_STATUSX_FBC_IMGO_DON_ST           ((unsigned int)1<<28)
#define ISP_IRQ_STATUSX_rsv_29                    ((unsigned int)1<<29)
#define ISP_IRQ_STATUSX_DMA_ERR_ST                ((unsigned int)1<<30)
#define ISP_IRQ_STATUSX_rsv_31                    ((unsigned int)1<<31)
/* CAM_CTL_INT_STATUS2X //P1_D */
#define ISP_IRQ_STATUS2X_VS1_INT_ST                ((unsigned int)1<<0)
#define ISP_IRQ_STATUS2X_TG1_INT1_ST               ((unsigned int)1<<1)
#define ISP_IRQ_STATUS2X_TG1_INT2_ST               ((unsigned int)1<<2)
#define ISP_IRQ_STATUS2X_EXPDON1_ST                ((unsigned int)1<<3)
#define ISP_IRQ_STATUS2X_TG1_ERR_ST                ((unsigned int)1<<4)
#define ISP_IRQ_STATUS2X_TG1_GBERR                 ((unsigned int)1<<5)
#define ISP_IRQ_STATUS2X_CQ0_ERR                   ((unsigned int)1<<6)
#define ISP_IRQ_STATUS2X_CQ0_VS_ERR_ST             ((unsigned int)1<<7)
#define ISP_IRQ_STATUS2X_IMGO_DROP_FRAME_ST        ((unsigned int)1<<8)
#define ISP_IRQ_STATUS2X_RRZO_DROP_FRAME_ST        ((unsigned int)1<<9)
#define ISP_IRQ_STATUS2X_PASS1_DON_ST              ((unsigned int)1<<10)
#define ISP_IRQ_STATUS2X_rsv_11                    ((unsigned int)1<<11)
#define ISP_IRQ_STATUS2X_SOF1_INT_ST               ((unsigned int)1<<12)
#define ISP_IRQ_STATUS2X_rsv_13                    ((unsigned int)1<<13)
#define ISP_IRQ_STATUS2X_rsv_14                    ((unsigned int)1<<14)
#define ISP_IRQ_STATUS2X_rsv_15                    ((unsigned int)1<<15)
#define ISP_IRQ_STATUS2X_AF_DON_ST                 ((unsigned int)1<<16)
#define ISP_IRQ_STATUS2X_rsv_17                    ((unsigned int)1<<17)
#define ISP_IRQ_STATUS2X_rsv_18					   ((unsigned int)1<<18)
#define ISP_IRQ_STATUS2X_rsv_19                    ((unsigned int)1<<19)
#define ISP_IRQ_STATUS2X_IMGO_ERR_ST               ((unsigned int)1<<20)
#define ISP_IRQ_STATUS2X_AAO_ERR_ST                ((unsigned int)1<<21)
#define ISP_IRQ_STATUS2X_LCSO_ERR_ST               ((unsigned int)1<<22)
#define ISP_IRQ_STATUS2X_RRZO_ERR_ST               ((unsigned int)1<<23)
#define ISP_IRQ_STATUS2X_AFO_ERR_ST                ((unsigned int)1<<24)
#define ISP_IRQ_STATUS2X_rsv_25                    ((unsigned int)1<<25)
#define ISP_IRQ_STATUS2X_LSC_ERR_ST                ((unsigned int)1<<26)
#define ISP_IRQ_STATUS2X_FBC_RRZO_DON_ST           ((unsigned int)1<<27)
#define ISP_IRQ_STATUS2X_FBC_IMGO_DON_ST           ((unsigned int)1<<28)
#define ISP_IRQ_STATUS2X_rsv_29                    ((unsigned int)1<<29)
#define ISP_IRQ_STATUS2X_DMA_ERR_ST                ((unsigned int)1<<30)
#define ISP_IRQ_STATUS2X_rsv_31                    ((unsigned int)1<<31)
/* ISP_IRQ_STATUS3X //ALL DMA */
#define ISP_IRQ_STATUS3X_IMGO_DONE_ST              ((unsigned int)1<<0)
#define ISP_IRQ_STATUS3X_UFEO_DONE_ST              ((unsigned int)1<<1)
#define ISP_IRQ_STATUS3X_RRZO_DONE_ST              ((unsigned int)1<<2)
#define ISP_IRQ_STATUS3X_ESFKO_DONE_ST             ((unsigned int)1<<3)
#define ISP_IRQ_STATUS3X_LCSO_DONE_ST              ((unsigned int)1<<4)
#define ISP_IRQ_STATUS3X_AAO_DONE_ST               ((unsigned int)1<<5)
#define ISP_IRQ_STATUS3X_BPCI_DONE_ST              ((unsigned int)1<<6)
#define ISP_IRQ_STATUS3X_LSCI_DONE_ST              ((unsigned int)1<<7)
#define ISP_IRQ_STATUS3X_IMGO_D_DONE_ST            ((unsigned int)1<<8)
#define ISP_IRQ_STATUS3X_RRZO_D_DONE_ST            ((unsigned int)1<<9)
#define ISP_IRQ_STATUS3X_AFO_D_DONE_ST             ((unsigned int)1<<10)
#define ISP_IRQ_STATUS3X_LCSO_D_DONE_ST            ((unsigned int)1<<11)
#define ISP_IRQ_STATUS3X_AAO_D_DONE_ST             ((unsigned int)1<<12)
#define ISP_IRQ_STATUS3X_BPCI_D_DONE_ST            ((unsigned int)1<<13)
#define ISP_IRQ_STATUS3X_LCSI_D_DONE_ST            ((unsigned int)1<<14)
#define ISP_IRQ_STATUS3X_IMGI_DONE_ST              ((unsigned int)1<<15)
#define ISP_IRQ_STATUS3X_UFDI_DONE_ST              ((unsigned int)1<<16)
#define ISP_IRQ_STATUS3X_VIPI_DONE_ST              ((unsigned int)1<<17)
#define ISP_IRQ_STATUS3X_VIP2I_DONE_ST			   ((unsigned int)1<<18)
#define ISP_IRQ_STATUS3X_VIP3I_DONE_ST             ((unsigned int)1<<19)
#define ISP_IRQ_STATUS3X_LCEI_DONE_ST              ((unsigned int)1<<20)
#define ISP_IRQ_STATUS3X_MFBO_DONE_ST              ((unsigned int)1<<21)
#define ISP_IRQ_STATUS3X_IMG2O_DONE_ST             ((unsigned int)1<<22)
#define ISP_IRQ_STATUS3X_IMG3O_DONE_ST             ((unsigned int)1<<23)
#define ISP_IRQ_STATUS3X_IMG3BO_DONE_ST            ((unsigned int)1<<24)
#define ISP_IRQ_STATUS3X_IMG3CO_DONE_ST            ((unsigned int)1<<25)
#define ISP_IRQ_STATUS3X_FEO_DONE_ST               ((unsigned int)1<<26)
#define ISP_IRQ_STATUS3X_rsv_27                    ((unsigned int)1<<27)
#define ISP_IRQ_STATUS3X_rsv_28                    ((unsigned int)1<<28)
#define ISP_IRQ_STATUS3X_rsv_29                    ((unsigned int)1<<29)
#define ISP_IRQ_STATUS3X_rsv_30                    ((unsigned int)1<<30)
#define ISP_IRQ_STATUS3X_rsv_31                    ((unsigned int)1<<31)
/* SENINF1_IRQ_INTSTA */
#define SENINF1_IRQ_OVERRUN_IRQ_STA					   ((unsigned int)1<<0)
#define SENINF1_IRQ_CRCERR_IRQ_STA					   ((unsigned int)1<<1)
#define SENINF1_IRQ_FSMERR_IRQ_STA					   ((unsigned int)1<<2)
#define SENINF1_IRQ_VSIZEERR_IRQ_STA				   ((unsigned int)1<<3)
#define SENINF1_IRQ_HSIZEERR_IRQ_STA				   ((unsigned int)1<<4)
#define SENINF1_IRQ_SENSOR_VSIZEERR_IRQ_STA			   ((unsigned int)1<<5)
#define SENINF1_IRQ_SENSOR_HSIZEERR_IRQ_STA			   ((unsigned int)1<<6)
/* SENINF2_IRQ_INTSTA */
#define SENINF2_IRQ_OVERRUN_IRQ_STA					   ((unsigned int)1<<0)
#define SENINF2_IRQ_CRCERR_IRQ_STA					   ((unsigned int)1<<1)
#define SENINF2_IRQ_FSMERR_IRQ_STA					   ((unsigned int)1<<2)
#define SENINF2_IRQ_VSIZEERR_IRQ_STA				   ((unsigned int)1<<3)
#define SENINF2_IRQ_HSIZEERR_IRQ_STA				   ((unsigned int)1<<4)
#define SENINF2_IRQ_SENSOR_VSIZEERR_IRQ_STA			   ((unsigned int)1<<5)
#define SENINF2_IRQ_SENSOR_HSIZEERR_IRQ_STA			   ((unsigned int)1<<6)
/* SENINF3_IRQ_INTSTA */
#define SENINF3_IRQ_OVERRUN_IRQ_STA					   ((unsigned int)1<<0)
#define SENINF3_IRQ_CRCERR_IRQ_STA					   ((unsigned int)1<<1)
#define SENINF3_IRQ_FSMERR_IRQ_STA					   ((unsigned int)1<<2)
#define SENINF3_IRQ_VSIZEERR_IRQ_STA				   ((unsigned int)1<<3)
#define SENINF3_IRQ_HSIZEERR_IRQ_STA				   ((unsigned int)1<<4)
#define SENINF3_IRQ_SENSOR_VSIZEERR_IRQ_STA			   ((unsigned int)1<<5)
#define SENINF3_IRQ_SENSOR_HSIZEERR_IRQ_STA			   ((unsigned int)1<<6)
/* SENINF4_IRQ_INTSTA */
#define SENINF4_IRQ_OVERRUN_IRQ_STA					   ((unsigned int)1<<0)
#define SENINF4_IRQ_CRCERR_IRQ_STA					   ((unsigned int)1<<1)
#define SENINF4_IRQ_FSMERR_IRQ_STA					   ((unsigned int)1<<2)
#define SENINF4_IRQ_VSIZEERR_IRQ_STA				   ((unsigned int)1<<3)
#define SENINF4_IRQ_HSIZEERR_IRQ_STA				   ((unsigned int)1<<4)
#define SENINF4_IRQ_SENSOR_VSIZEERR_IRQ_STA			   ((unsigned int)1<<5)
#define SENINF4_IRQ_SENSOR_HSIZEERR_IRQ_STA			   ((unsigned int)1<<6)
/* REG_CAMSV_INT_STATUS */
#define ISP_IRQ_CAMSV_STATUS_VS1_ST						((unsigned int)1<<0)
#define ISP_IRQ_CAMSV_STATUS_TG_ST1						((unsigned int)1<<1)
#define ISP_IRQ_CAMSV_STATUS_TG_ST2						((unsigned int)1<<2)
#define ISP_IRQ_CAMSV_STATUS_EXPDON1_ST					((unsigned int)1<<3)
#define ISP_IRQ_CAMSV_STATUS_TG_ERR_ST					((unsigned int)1<<4)
#define ISP_IRQ_CAMSV_STATUS_TG_GBERR_ST				((unsigned int)1<<5)
#define ISP_IRQ_CAMSV_STATUS_TG_DROP_ST					((unsigned int)1<<6)
#define ISP_IRQ_CAMSV_STATUS_TG_SOF1_ST					((unsigned int)1<<7)
#define ISP_IRQ_CAMSV_STATUS_rsv_8						((unsigned int)1<<8)
#define ISP_IRQ_CAMSV_STATUS_rsv_9						((unsigned int)1<<9)
#define ISP_IRQ_CAMSV_STATUS_PASS1_DON_ST				((unsigned int)1<<10)
#define ISP_IRQ_CAMSV_STATUS_rsv_11						((unsigned int)1<<11)
#define ISP_IRQ_CAMSV_STATUS_rsv_12						((unsigned int)1<<12)
#define ISP_IRQ_CAMSV_STATUS_rsv_13						((unsigned int)1<<13)
#define ISP_IRQ_CAMSV_STATUS_rsv_14						((unsigned int)1<<14)
#define ISP_IRQ_CAMSV_STATUS_rsv_15						((unsigned int)1<<15)
#define ISP_IRQ_CAMSV_STATUS_IMGO_ERR_ST				((unsigned int)1<<16)
#define ISP_IRQ_CAMSV_STATUS_IMGO_OVERR_ST				((unsigned int)1<<17)
#define ISP_IRQ_CAMSV_STATUS_rsv_18						((unsigned int)1<<18)
#define ISP_IRQ_CAMSV_STATUS_IMGO_DROP_ST				((unsigned int)1<<19)
/* REG_ISP_IRQ_CAMSV2_STATUS */
#define ISP_IRQ_CAMSV2_STATUS_VS1_ST					((unsigned int)1<<0)
#define ISP_IRQ_CAMSV2_STATUS_TG_ST1					((unsigned int)1<<1)
#define ISP_IRQ_CAMSV2_STATUS_TG_ST2					((unsigned int)1<<2)
#define ISP_IRQ_CAMSV2_STATUS_EXPDON1_ST				((unsigned int)1<<3)
#define ISP_IRQ_CAMSV2_STATUS_TG_ERR_ST					((unsigned int)1<<4)
#define ISP_IRQ_CAMSV2_STATUS_TG_GBERR_ST				((unsigned int)1<<5)
#define ISP_IRQ_CAMSV2_STATUS_TG_DROP_ST				((unsigned int)1<<6)
#define ISP_IRQ_CAMSV2_STATUS_TG_SOF1_ST				((unsigned int)1<<7)
#define ISP_IRQ_CAMSV2_STATUS_rsv_8						((unsigned int)1<<8)
#define ISP_IRQ_CAMSV2_STATUS_rsv_9						((unsigned int)1<<9)
#define ISP_IRQ_CAMSV2_STATUS_PASS1_DON_ST				((unsigned int)1<<10)
#define ISP_IRQ_CAMSV2_STATUS_rsv_11					((unsigned int)1<<11)
#define ISP_IRQ_CAMSV2_STATUS_rsv_12					((unsigned int)1<<12)
#define ISP_IRQ_CAMSV2_STATUS_rsv_13					((unsigned int)1<<13)
#define ISP_IRQ_CAMSV2_STATUS_rsv_14					((unsigned int)1<<14)
#define ISP_IRQ_CAMSV2_STATUS_rsv_15					((unsigned int)1<<15)
#define ISP_IRQ_CAMSV2_STATUS_IMGO_ERR_ST				((unsigned int)1<<16)
#define ISP_IRQ_CAMSV2_STATUS_IMGO_OVERR_ST				((unsigned int)1<<17)
#define ISP_IRQ_CAMSV2_STATUS_rsv_18					((unsigned int)1<<18)
#define ISP_IRQ_CAMSV2_STATUS_IMGO_DROP_ST				((unsigned int)1<<19)

/*******************************************************************************
*
********************************************************************************/

/* defined if want to support multiple dequne and enque or camera 3.0 */
/**
    support multiple deque and enque if defined.
    note: still en/de que 1 buffer each time only
    e.g:
	deque();
	deque();
	enque();
	enque();
*/
#define _rtbc_buf_que_2_0_


typedef enum {
	ISP_IRQ_CLEAR_NONE,
	ISP_IRQ_CLEAR_WAIT,
	ISP_IRQ_CLEAR_STATUS,
	ISP_IRQ_CLEAR_ALL
} ISP_IRQ_CLEAR_ENUM;

typedef enum {
	ISP_IRQ_TYPE_INT_P1_ST,	/* P1 */
	ISP_IRQ_TYPE_INT_P1_ST2,	/* P1_DMA */
	ISP_IRQ_TYPE_INT_P1_ST_D,	/* P1_D */
	ISP_IRQ_TYPE_INT_P1_ST2_D,	/* P1_DMA_D */
	ISP_IRQ_TYPE_INT_P2_ST,	/* P2 */
	ISP_IRQ_TYPE_INT_STATUSX,	/* STATUSX, P1 AND P2 */
	ISP_IRQ_TYPE_INT_STATUS2X,	/* STATUS2X, P1_D */
	ISP_IRQ_TYPE_INT_STATUS3X,	/* STATUS3X, ALL DMA */
	ISP_IRQ_TYPE_ISP_AMOUNT,
	ISP_IRQ_TYPE_INT_SENINF1 = ISP_IRQ_TYPE_ISP_AMOUNT,
	ISP_IRQ_TYPE_INT_SENINF2,
	ISP_IRQ_TYPE_INT_SENINF3,
	ISP_IRQ_TYPE_INT_SENINF4,
	ISP_IRQ_TYPE_SENINF_AMOUNT,
	ISP_IRQ_TYPE_INT_CAMSV = ISP_IRQ_TYPE_SENINF_AMOUNT,
	ISP_IRQ_TYPE_INT_CAMSV2,
	ISP_IRQ_TYPE_AMOUNT
} ISP_IRQ_TYPE_ENUM;


typedef enum {
	ISP_IRQ_USER_ISPDRV = 0,
	ISP_IRQ_USER_MW = 1,
	ISP_IRQ_USER_3A = 2,
	ISP_IRQ_USER_HWSYNC = 3,
	ISP_IRQ_USER_ACDK = 4,
	ISP_IRQ_USER_EIS = 5,
	ISP_IRQ_USER_VHDR = 6,
	ISP_IRQ_USER_MAX
} ISP_IRQ_USER_ENUM;
typedef enum			/* special user for specific operation */
{
	ISP_IRQ_WAITIRQ_SPEUSER_NONE = 0,
	ISP_IRQ_WAITIRQ_SPEUSER_EIS = 1,
	ISP_IRQ_WAITIRQ_SPEUSER_NUM
} ISP_IRQ_WAITIRQ_SPEUSER_ENUM;

typedef struct {
	unsigned int tLastSig_sec;	/* time stamp of the latest occuring signal */
	unsigned int tLastSig_usec;	/* time stamp of the latest occuring signal */
} ISP_IRQ_TIME_STRUCT;

typedef struct {
	unsigned int tLastSOF2P1done_sec;	/* time stamp of the last closest occuring sof signal for pass1 done */
	unsigned int tLastSOF2P1done_usec;	/* time stamp of the last closest occuring sof signal for pass1 done */
} ISP_EIS_META_STRUCT;


typedef struct {
	ISP_IRQ_CLEAR_ENUM Clear;	/* remove this */
	ISP_IRQ_TYPE_ENUM Type;
	unsigned int Status;
	int UserNumber;
	unsigned int Timeout;
	char *UserName;
	unsigned int irq_TStamp;
	unsigned int bDumpReg;
	ISP_IRQ_TIME_STRUCT TimeInfo;	/* v3 */
	ISP_EIS_META_STRUCT EisMeta;
	ISP_IRQ_WAITIRQ_SPEUSER_ENUM SpecUser;
} ISP_WAIT_IRQ_STRUCT;

typedef struct {
	ISP_IRQ_TYPE_ENUM Type;
	ISP_IRQ_USER_ENUM UserNumber;
	unsigned int Status;
} ISP_READ_IRQ_STRUCT;

typedef struct {
	ISP_IRQ_TYPE_ENUM Type;
	ISP_IRQ_USER_ENUM UserNumber;
	unsigned int Status;
} ISP_CLEAR_IRQ_STRUCT;

typedef enum {
	ISP_HOLD_TIME_VD,
	ISP_HOLD_TIME_EXPDONE
} ISP_HOLD_TIME_ENUM;

typedef struct {
	unsigned int Addr;	/* register's addr */
	unsigned int Val;	/* register's value */
} ISP_REG_STRUCT;

typedef struct {
	/* unsigned int Data;   // pointer to ISP_REG_STRUCT */
	ISP_REG_STRUCT *pData;
	unsigned int Count;	/* count */
} ISP_REG_IO_STRUCT;

typedef void (*pIspCallback) (void);

typedef enum {
	/* Work queue. It is interruptible, so there can be "Sleep" in work queue function. */
	ISP_CALLBACK_WORKQUEUE_VD,
	ISP_CALLBACK_WORKQUEUE_EXPDONE,
	/* Tasklet. It is uninterrupted, so there can NOT be "Sleep" in tasklet function. */
	ISP_CALLBACK_TASKLET_VD,
	ISP_CALLBACK_TASKLET_EXPDONE,
	ISP_CALLBACK_AMOUNT
} ISP_CALLBACK_ENUM;

typedef struct {
	ISP_CALLBACK_ENUM Type;
	pIspCallback Func;
} ISP_CALLBACK_STRUCT;

/*  */
/* length of the two memory areas */
#define P1_DEQUE_CNT    1
#define RT_BUF_TBL_NPAGES 16
#define ISP_RT_BUF_SIZE 16
#define ISP_RT_CQ0C_BUF_SIZE (ISP_RT_BUF_SIZE)	/* (ISP_RT_BUF_SIZE>>1) */
/* pass1 setting sync index */
#define ISP_REG_P1_CFG_IDX 0x4090
/*  */
typedef enum {
	_cam_tg_ = 0,
	_cam_tg2_,		/* 1 */
	_camsv_tg_,		/* 2 */
	_camsv2_tg_,		/* 3 */
	_cam_tg_max_
} _isp_tg_enum_;

/*  */
typedef enum {
	_imgi_ = 0,
	_vipi_,			/* 1 */
	_vip2i_,		/* 2 */
	_vip3i_,		/* 3 */
	_imgo_,			/* 4 */
	_ufdi_,			/* 5 */
	_lcei_,			/* 6 */
	_ufeo_,			/* 7 */
	_rrzo_,			/* 8 */
	_imgo_d_,		/* 9 */
	_rrzo_d_,		/* 10 */
	_img2o_,		/* 11 */
	_img3o_,		/* 12 */
	_img3bo_,		/* 13 */
	_img3co_,		/* 14 */
	_camsv_imgo_,		/* 15 */
	_camsv2_imgo_,		/* 16 */
	_mfbo_,			/* 17 */
	_feo_,			/* 18 */
	_wrot_,			/* 19 */
	_wdma_,			/* 20 */
	_jpeg_,			/* 21 */
	_venc_stream_,		/* 21 */
	_rt_dma_max_
} _isp_dma_enum_;
/*  */
typedef struct {
	unsigned int w;		/* tg size */
	unsigned int h;
	unsigned int xsize;	/* dmao xsize */
	unsigned int stride;
	unsigned int fmt;
	unsigned int pxl_id;
	unsigned int wbn;
	unsigned int ob;
	unsigned int lsc;
	unsigned int rpg;
	unsigned int m_num_0;
	unsigned int frm_cnt;
	unsigned int bus_size;
} ISP_RT_IMAGE_INFO_STRUCT;

typedef struct {
	unsigned int srcX;	/* crop window start point */
	unsigned int srcY;
	unsigned int srcW;	/* crop window size */
	unsigned int srcH;
	unsigned int dstW;	/* rrz out size */
	unsigned int dstH;
} ISP_RT_RRZ_INFO_STRUCT;

typedef struct {
	unsigned int x;		/* in pix */
	unsigned int y;		/* in pix */
	unsigned int w;		/* in byte */
	unsigned int h;		/* in byte */
} ISP_RT_DMAO_CROPPING_STRUCT;

typedef struct {
	unsigned int memID;
	unsigned int size;
	long long base_vAddr;
	unsigned int base_pAddr;
	unsigned int timeStampS;
	unsigned int timeStampUs;
	unsigned int bFilled;
	unsigned int bProcessRaw;
	ISP_RT_IMAGE_INFO_STRUCT image;
	ISP_RT_RRZ_INFO_STRUCT rrzInfo;
	ISP_RT_DMAO_CROPPING_STRUCT dmaoCrop;	/* imgo */
	unsigned int bDequeued;
	signed int bufIdx;	/* used for replace buffer */
} ISP_RT_BUF_INFO_STRUCT;
/*  */
typedef struct {
	unsigned int count;
	unsigned int sof_cnt;	/* cnt for current sof */
	unsigned int img_cnt;	/* cnt for mapping to which sof */
	/* rome support only deque 1 image at a time */
	/* ISP_RT_BUF_INFO_STRUCT  data[ISP_RT_BUF_SIZE]; */
	ISP_RT_BUF_INFO_STRUCT data[P1_DEQUE_CNT];
} ISP_DEQUE_BUF_INFO_STRUCT;
/*  */
typedef struct {
	unsigned int start;	/* current DMA accessing buffer */
	unsigned int total_count;	/* total buffer number.Include Filled and empty */
	unsigned int empty_count;	/* total empty buffer number include current DMA accessing buffer */
	unsigned int pre_empty_count;	/* previous total empty buffer number include current DMA accessing buffer */
	unsigned int active;
	unsigned int read_idx;
	unsigned int img_cnt;	/* cnt for mapping to which sof */
	ISP_RT_BUF_INFO_STRUCT data[ISP_RT_BUF_SIZE];
} ISP_RT_RING_BUF_INFO_STRUCT;
/*  */
typedef enum {
	ISP_RT_BUF_CTRL_ENQUE,	/* 0 */
#ifdef _rtbc_buf_que_2_0_
	ISP_RT_BUF_CTRL_ENQUE_IMD,	/* for directly enque */
#else
	ISP_RT_BUF_CTRL_ENQUE_IMD = ISP_RT_BUF_CTRL_ENQUE,
#endif
	ISP_RT_BUF_CTRL_EXCHANGE_ENQUE,	/* 1 */
	ISP_RT_BUF_CTRL_DEQUE,	/* 2 */
	ISP_RT_BUF_CTRL_IS_RDY,	/* 3 */
#ifdef _rtbc_buf_que_2_0_
	ISP_RT_BUF_CTRL_DMA_EN,	/* 4 */
#endif
	ISP_RT_BUF_CTRL_GET_SIZE,	/* 5 */
	ISP_RT_BUF_CTRL_CLEAR,	/* 6 */
	ISP_RT_BUF_CTRL_CUR_STATUS,	/* 7 */
	ISP_RT_BUF_CTRL_MAX
} ISP_RT_BUF_CTRL_ENUM;
/*  */
typedef enum {
	ISP_RTBC_STATE_INIT,	/* 0 */
	ISP_RTBC_STATE_SOF,
	ISP_RTBC_STATE_DONE,
	ISP_RTBC_STATE_MAX
} ISP_RTBC_STATE_ENUM;
/*  */
typedef enum {
	ISP_RTBC_BUF_EMPTY,	/* 0 */
	ISP_RTBC_BUF_FILLED,	/* 1 */
	ISP_RTBC_BUF_LOCKED,	/* 2 */
} ISP_RTBC_BUF_STATE_ENUM;
/*  */
typedef enum {
	ISP_RROCESSED_RAW,	/* 0 */
	ISP_PURE_RAW,		/* 1 */
} ISP_RAW_TYPE_ENUM;
/*  */
typedef struct {
	ISP_RTBC_STATE_ENUM state;
	unsigned long dropCnt;
	ISP_RT_RING_BUF_INFO_STRUCT ring_buf[_rt_dma_max_];
} ISP_RT_BUF_STRUCT;
/*  */
typedef struct {
	ISP_RT_BUF_CTRL_ENUM ctrl;
	_isp_dma_enum_ buf_id;
	/* unsigned int            data_ptr; */
	/* unsigned int            ex_data_ptr; //exchanged buffer */
	ISP_RT_BUF_INFO_STRUCT *data_ptr;
	ISP_RT_BUF_INFO_STRUCT *ex_data_ptr;	/* exchanged buffer */
	unsigned char *pExtend;
} ISP_BUFFER_CTRL_STRUCT;
/*  */
/* reference count */
#define _use_kernel_ref_cnt_
/*  */
typedef enum {
	ISP_REF_CNT_GET,	/* 0 */
	ISP_REF_CNT_INC,	/* 1 */
	ISP_REF_CNT_DEC,	/* 2 */
	ISP_REF_CNT_DEC_AND_RESET_P1_P2_IF_LAST_ONE,	/* 3 */
	ISP_REF_CNT_DEC_AND_RESET_P1_IF_LAST_ONE,	/* 4 */
	ISP_REF_CNT_DEC_AND_RESET_P2_IF_LAST_ONE,	/* 5 */
	ISP_REF_CNT_MAX
} ISP_REF_CNT_CTRL_ENUM;
/*  */
typedef enum {
	ISP_REF_CNT_ID_IMEM,	/* 0 */
	ISP_REF_CNT_ID_ISP_FUNC,	/* 1 */
	ISP_REF_CNT_ID_GLOBAL_PIPE,	/* 2 */
	ISP_REF_CNT_ID_P1_PIPE,	/* 3 */
	ISP_REF_CNT_ID_P2_PIPE,	/* 4 */
	ISP_REF_CNT_ID_MAX,
} ISP_REF_CNT_ID_ENUM;
/*  */
typedef struct {
	ISP_REF_CNT_CTRL_ENUM ctrl;
	ISP_REF_CNT_ID_ENUM id;
	signed int *data_ptr;
} ISP_REF_CNT_CTRL_STRUCT;

/* struct for enqueue/dequeue control in ihalpipe wrapper */
typedef enum {
	ISP_ED_BUFQUE_CTRL_ENQUE_FRAME = 0,	/* 0,signal that a specific buffer is enqueued */
	ISP_ED_BUFQUE_CTRL_WAIT_DEQUE,	/* 1,a dequeue thread is waiting to do dequeue */
	ISP_ED_BUFQUE_CTRL_DEQUE_SUCCESS,	/* 2,signal that a buffer is dequeued (success) */
	ISP_ED_BUFQUE_CTRL_DEQUE_FAIL,	/* 3,signal that a buffer is dequeued (fail) */
	ISP_ED_BUFQUE_CTRL_WAIT_FRAME,	/* 4,wait for a specific buffer */
	ISP_ED_BUFQUE_CTRL_WAKE_WAITFRAME,	/* 5,wake all sleeped users to check buffer is dequeued or not */
	ISP_ED_BUFQUE_CTRL_CLAER_ALL,	/* 6,free all recored dequeued buffer */
	ISP_ED_BUFQUE_CTRL_MAX
} ISP_ED_BUFQUE_CTRL_ENUM;

typedef struct {
	ISP_ED_BUFQUE_CTRL_ENUM ctrl;
	unsigned int processID;
	unsigned int callerID;
	int p2burstQIdx;
	int p2dupCQIdx;
	unsigned int timeoutUs;
} ISP_ED_BUFQUE_STRUCT;

typedef enum {
	ISP_ED_BUF_STATE_NONE = -1,
	ISP_ED_BUF_STATE_ENQUE = 0,
	ISP_ED_BUF_STATE_RUNNING,
	ISP_ED_BUF_STATE_WAIT_DEQUE_FAIL,
	ISP_ED_BUF_STATE_DEQUE_SUCCESS,
	ISP_ED_BUF_STATE_DEQUE_FAIL
} ISP_ED_BUF_STATE_ENUM;
/********************************************************************************************
 pass1 real time buffer control use cq0c
********************************************************************************************/
/*  */
#define _rtbc_use_cq0c_

#define _MAGIC_NUM_ERR_HANDLING_


#if defined(_rtbc_use_cq0c_)
/*  */
typedef struct _cq_cmd_st_ {
	unsigned int inst;
	unsigned int data_ptr_pa;
} CQ_CMD_ST;
/*
typedef struct _cq_cmd_rtbc_st_
{
    CQ_CMD_ST imgo;
    CQ_CMD_ST img2o;
    CQ_CMD_ST cq0ci;
    CQ_CMD_ST end;
}CQ_CMD_RTBC_ST;
*/
typedef struct _cq_info_rtbc_st_ {
	CQ_CMD_ST imgo;
	CQ_CMD_ST rrzo;
	CQ_CMD_ST next_cq0ci;
	CQ_CMD_ST end;
	unsigned int imgo_base_pAddr;
	unsigned int rrzo_base_pAddr;
	signed int imgo_buf_idx;	/* used for replace buffer */
	signed int rrzo_buf_idx;	/* used for replace buffer */
} CQ_INFO_RTBC_ST;
typedef struct _cq_ring_cmd_st_ {
	CQ_INFO_RTBC_ST cq_rtbc;
	unsigned long next_pa;
	struct _cq_ring_cmd_st_ *pNext;
} CQ_RING_CMD_ST;
typedef struct _cq_rtbc_ring_st_ {
	CQ_RING_CMD_ST rtbc_ring[ISP_RT_CQ0C_BUF_SIZE];
	unsigned int imgo_ring_size;
	unsigned int rrzo_ring_size;
} CQ_RTBC_RING_ST;
#endif

/* CQ0B for AE smoothing, set obc_gain0~3 */
typedef struct _cq0b_info_rtbc_st_ {
	CQ_CMD_ST ob;
	CQ_CMD_ST end;
} CQ0B_INFO_RTBC_ST;

typedef struct _cq0b_ring_cmd_st_ {
	CQ0B_INFO_RTBC_ST cq0b_rtbc;
	unsigned long next_pa;
	struct _cq0b_ring_cmd_st_ *pNext;
} CQ0B_RING_CMD_ST;

typedef struct _cq0b_rtbc_ring_st_ {
	CQ0B_RING_CMD_ST rtbc_ring;
} CQ0B_RTBC_RING_ST;
/*  */

typedef volatile union _CQ_RTBC_FBC_ {
	volatile struct {
		unsigned int FBC_CNT:4;
		unsigned int rsv_4:7;
		unsigned int RCNT_INC:1;
		unsigned int rsv_12:2;
		unsigned int FBC_EN:1;
		unsigned int LOCK_EN:1;
		unsigned int FB_NUM:4;
		unsigned int RCNT:4;
		unsigned int WCNT:4;
		unsigned int DROP_CNT:4;
	} Bits;
	unsigned int Reg_val;
} CQ_RTBC_FBC;


#ifdef CONFIG_COMPAT

typedef struct {
	ISP_IRQ_CLEAR_ENUM Clear;	/* remove this */
	ISP_IRQ_TYPE_ENUM Type;
	unsigned int Status;
	int UserNumber;
	unsigned int Timeout;
	compat_uptr_t UserName;
	unsigned int irq_TStamp;
	unsigned int bDumpReg;
	ISP_IRQ_TIME_STRUCT TimeInfo;	/* v3 */
	ISP_EIS_META_STRUCT EisMeta;
	ISP_IRQ_WAITIRQ_SPEUSER_ENUM SpecUser;
} compat_ISP_WAIT_IRQ_STRUCT;


typedef struct {
	ISP_IRQ_TYPE_ENUM Type;
	ISP_IRQ_USER_ENUM UserNumber;
	unsigned int Status;
} compat_ISP_READ_IRQ_STRUCT;

typedef struct {
	ISP_IRQ_TYPE_ENUM Type;
	ISP_IRQ_USER_ENUM UserNumber;
	unsigned int Status;
} compat_ISP_CLEAR_IRQ_STRUCT;


typedef struct {
	compat_uptr_t pData;
	unsigned int Count;	/* count */
} compat_ISP_REG_IO_STRUCT;

/*  */
typedef struct {
	ISP_RT_BUF_CTRL_ENUM ctrl;
	_isp_dma_enum_ buf_id;
	compat_uptr_t data_ptr;
	compat_uptr_t ex_data_ptr;	/* exchanged buffer */
	compat_uptr_t pExtend;
} compat_ISP_BUFFER_CTRL_STRUCT;


typedef struct {
	ISP_REF_CNT_CTRL_ENUM ctrl;
	ISP_REF_CNT_ID_ENUM id;
	compat_uptr_t data_ptr;
} compat_ISP_REF_CNT_CTRL_STRUCT;


#endif
/*  */
/********************************************************************************************

********************************************************************************************/


/*******************************************************************************
*
********************************************************************************/
typedef enum {
	ISP_CMD_RESET_CAM_P1,	/* Reset */
	ISP_CMD_RESET_CAM_P2,
	ISP_CMD_RESET_CAMSV,
	ISP_CMD_RESET_CAMSV2,
	ISP_CMD_RESET_BUF,
	ISP_CMD_READ_REG,	/* Read register from driver */
	ISP_CMD_WRITE_REG,	/* Write register to driver */
	ISP_CMD_HOLD_TIME,
	ISP_CMD_HOLD_REG,	/* Hold reg write to hw, on/off */
	ISP_CMD_WAIT_IRQ,	/* Wait IRQ */
	ISP_CMD_READ_IRQ,	/* Read IRQ */
	ISP_CMD_CLEAR_IRQ,	/* Clear IRQ */
	ISP_CMD_DUMP_REG,	/* Dump ISP registers , for debug usage */
	ISP_CMD_SET_USER_PID,	/* for signal */
	ISP_CMD_RT_BUF_CTRL,	/* for pass buffer control */
	ISP_CMD_REF_CNT,	/* get imem reference count */
	ISP_CMD_DEBUG_FLAG,	/* Dump message level */
	ISP_CMD_REGISTER_IRQ,	/* register for a specific irq */
	ISP_CMD_UNREGISTER_IRQ,	/* unregister for a specific irq */
	ISP_CMD_ED_QUEBUF_CTRL,
	ISP_CMD_UPDATE_REGSCEN,
	ISP_CMD_QUERY_REGSCEN,
	ISP_CMD_UPDATE_BURSTQNUM,
	ISP_CMD_QUERY_BURSTQNUM,
	ISP_CMD_DUMP_ISR_LOG,	/* dump isr log */
	ISP_CMD_GET_CUR_SOF,
	ISP_CMD_GET_DMA_ERR,
	ISP_CMD_GET_INT_ERR,
	ISP_CMD_GET_DROP_FRAME,	/* dump current frame informaiton, 1 for drop frmae, 2 for last working frame */
#ifdef T_STAMP_2_0
	ISP_CMD_SET_FPS,
#endif
	ISP_CMD_WAKELOCK_CTRL,	/* add for android L flash light suspend/resume issue */
} ISP_CMD_ENUM;
/*  */
#define ISP_RESET_CAM_P1    _IO(ISP_MAGIC, ISP_CMD_RESET_CAM_P1)
#define ISP_RESET_CAM_P2    _IO(ISP_MAGIC, ISP_CMD_RESET_CAM_P2)
#define ISP_RESET_CAMSV     _IO(ISP_MAGIC, ISP_CMD_RESET_CAMSV)
#define ISP_RESET_CAMSV2    _IO(ISP_MAGIC, ISP_CMD_RESET_CAMSV2)
#define ISP_RESET_BUF       _IO(ISP_MAGIC, ISP_CMD_RESET_BUF)
#define ISP_READ_REGISTER   _IOWR(ISP_MAGIC, ISP_CMD_READ_REG,      ISP_REG_IO_STRUCT)
#define ISP_WRITE_REGISTER  _IOWR(ISP_MAGIC, ISP_CMD_WRITE_REG,     ISP_REG_IO_STRUCT)
#define ISP_HOLD_REG_TIME   _IOW(ISP_MAGIC, ISP_CMD_HOLD_TIME,     ISP_HOLD_TIME_ENUM)
#define ISP_HOLD_REG        _IOW(ISP_MAGIC, ISP_CMD_HOLD_REG,      bool)
#define ISP_WAIT_IRQ        _IOW(ISP_MAGIC, ISP_CMD_WAIT_IRQ,      ISP_WAIT_IRQ_STRUCT)
#define ISP_READ_IRQ        _IOR(ISP_MAGIC, ISP_CMD_READ_IRQ,      ISP_READ_IRQ_STRUCT)
#define ISP_CLEAR_IRQ       _IOW(ISP_MAGIC, ISP_CMD_CLEAR_IRQ,     ISP_CLEAR_IRQ_STRUCT)
#define ISP_DUMP_REG        _IO(ISP_MAGIC, ISP_CMD_DUMP_REG)
#define ISP_SET_USER_PID    _IOW(ISP_MAGIC, ISP_CMD_SET_USER_PID,    unsigned int)
#define ISP_BUFFER_CTRL     _IOWR(ISP_MAGIC, ISP_CMD_RT_BUF_CTRL,    ISP_BUFFER_CTRL_STRUCT)
#define ISP_REF_CNT_CTRL    _IOWR(ISP_MAGIC, ISP_CMD_REF_CNT,       ISP_REF_CNT_CTRL_STRUCT)
#define ISP_DEBUG_FLAG      _IOW(ISP_MAGIC, ISP_CMD_DEBUG_FLAG,    unsigned char*)
#define ISP_REGISTER_IRQ    _IOW(ISP_MAGIC, ISP_CMD_REGISTER_IRQ,  ISP_WAIT_IRQ_STRUCT)
#define ISP_UNREGISTER_IRQ    _IOW(ISP_MAGIC, ISP_CMD_UNREGISTER_IRQ,  ISP_WAIT_IRQ_STRUCT)
#define ISP_ED_QUEBUF_CTRL     _IOWR(ISP_MAGIC, ISP_CMD_ED_QUEBUF_CTRL, ISP_ED_BUFQUE_STRUCT)
#define ISP_UPDATE_REGSCEN     _IOWR(ISP_MAGIC, ISP_CMD_UPDATE_REGSCEN, unsigned int)
#define ISP_QUERY_REGSCEN     _IOR(ISP_MAGIC, ISP_CMD_QUERY_REGSCEN, unsigned int)
#define ISP_UPDATE_BURSTQNUM _IOW(ISP_MAGIC, ISP_CMD_UPDATE_BURSTQNUM, int)
#define ISP_QUERY_BURSTQNUM _IOR(ISP_MAGIC, ISP_CMD_QUERY_BURSTQNUM, int)
#define ISP_DUMP_ISR_LOG    _IO(ISP_MAGIC, ISP_CMD_DUMP_ISR_LOG)
#define ISP_GET_CUR_SOF     _IOR(ISP_MAGIC, ISP_CMD_GET_CUR_SOF,      unsigned int)
#define ISP_GET_DMA_ERR     _IOWR(ISP_MAGIC, ISP_CMD_GET_DMA_ERR,      unsigned char*)
#define ISP_GET_INT_ERR     _IOR(ISP_MAGIC, ISP_CMD_GET_INT_ERR,      unsigned char*)
#define ISP_GET_DROP_FRAME  _IOWR(ISP_MAGIC, ISP_CMD_GET_DROP_FRAME,      unsigned int)
#ifdef T_STAMP_2_0
#define ISP_SET_FPS     _IOW(ISP_MAGIC, ISP_CMD_SET_FPS,    unsigned int)
#endif
#define ISP_WAKELOCK_CTRL     _IOWR(ISP_MAGIC, ISP_CMD_WAKELOCK_CTRL,      unsigned int)

#ifdef CONFIG_COMPAT

#define COMPAT_ISP_READ_REGISTER   _IOWR(ISP_MAGIC, ISP_CMD_READ_REG,      compat_ISP_REG_IO_STRUCT)
#define COMPAT_ISP_WRITE_REGISTER  _IOWR(ISP_MAGIC, ISP_CMD_WRITE_REG,     compat_ISP_REG_IO_STRUCT)
#define COMPAT_ISP_DEBUG_FLAG     _IOW(ISP_MAGIC, ISP_CMD_DEBUG_FLAG,     compat_uptr_t)
#define COMPAT_ISP_GET_DMA_ERR     _IOWR(ISP_MAGIC, ISP_CMD_GET_DMA_ERR,  compat_uptr_t)
#define COMPAT_ISP_GET_INT_ERR     _IOR(ISP_MAGIC, ISP_CMD_GET_INT_ERR, compat_uptr_t)

#define COMPAT_ISP_WAIT_IRQ        _IOW(ISP_MAGIC, ISP_CMD_WAIT_IRQ,      compat_ISP_WAIT_IRQ_STRUCT)
#define COMPAT_ISP_READ_IRQ        _IOR(ISP_MAGIC, ISP_CMD_READ_IRQ,      compat_ISP_READ_IRQ_STRUCT)
#define COMPAT_ISP_BUFFER_CTRL     _IOWR(ISP_MAGIC, ISP_CMD_RT_BUF_CTRL,    compat_ISP_BUFFER_CTRL_STRUCT)
#define COMPAT_ISP_REF_CNT_CTRL    _IOWR(ISP_MAGIC, ISP_CMD_REF_CNT,       compat_ISP_REF_CNT_CTRL_STRUCT)

#endif
/*  */
bool ISP_RegCallback(ISP_CALLBACK_STRUCT *pCallback);
bool ISP_UnregCallback(ISP_CALLBACK_ENUM Type);
int32_t ISP_MDPClockOnCallback(uint64_t engineFlag);
int32_t ISP_MDPDumpCallback(uint64_t engineFlag, int level);
int32_t ISP_MDPResetCallback(uint64_t engineFlag);

int32_t ISP_MDPClockOffCallback(uint64_t engineFlag);

int32_t ISP_BeginGCECallback(uint32_t taskID, uint32_t *regCount, uint32_t **regAddress);
int32_t ISP_EndGCECallback(uint32_t taskID, uint32_t regCount, uint32_t *regValues);

/*  */
#endif
