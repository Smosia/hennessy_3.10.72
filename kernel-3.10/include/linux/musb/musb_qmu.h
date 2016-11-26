#ifndef _MUSB_QMU_H_
#define _MUSB_QMU_H_

#ifdef MUSB_QMU_SUPPORT
#include "musb_core.h"		/* for struct musb */

extern int musb_qmu_init(struct musb *musb);
extern void musb_qmu_exit(struct musb *musb); 
extern void musb_kick_D_CmdQ(struct musb *musb, struct musb_request *request);
extern void musb_disable_q_all(struct musb *musb);
extern irqreturn_t musb_q_irq(struct musb *musb);
extern void musb_flush_qmu(u32 ep_num,  u8 isRx);
extern void musb_restart_qmu(struct musb *musb, u32 ep_num, u8 isRx);
extern bool musb_is_qmu_stop(u32 ep_num, u8 isRx);
extern void musb_tx_zlp_qmu(struct musb *musb, u32 ep_num);

/*FIXME, not good layer present */
extern void mtk_qmu_enable(struct musb *musb, u8 EP_Num, u8 isRx);

#endif
#endif
