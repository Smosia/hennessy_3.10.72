#ifndef __EXTD_DRV_LOG_H__
#define __EXTD_DRV_LOG_H__

#include <linux/printk.h>

/* HDMI log functions*/
#define HDMI_LOG(fmt, arg...)	pr_debug("[EXTD-HDMI]:"fmt, ##arg)
#define HDMI_FUNC()		pr_debug("[EXTD-HDMI]:%s\n", __func__)
#define HDMI_ERR(fmt, arg...)	pr_err("[EXTD-HDMI]:"fmt, ##arg)

/* Eink log functions */
#define EPD_LOG(fmt, arg...)	pr_debug("[EXTD-EPD]:"fmt, ##arg)
#define EPD_FUNC()		pr_debug("[EXTD-EPD]:%s\n", __func__)
#define EPD_ERR(fmt, arg...)	pr_err("[EXTD-EPD]:"fmt, ##arg)

/*external display - multi-control log functions */
#define MULTI_COTRL_LOG(fmt, arg...)	pr_debug("[EXTD-MULTI]:"fmt, ##arg)
#define MULTI_COTRL_ERR(fmt, arg...)	pr_err("[EXTD-MULTI]:"fmt, ##arg)
#define MULTI_COTRL_FUNC()		pr_debug("[EXTD-MULTI]:%s\n", __func__)

/*external display log functions*/
#define EXT_DISP_LOG(fmt, arg...)	pr_debug("[EXTD]:"fmt, ##arg)
#define EXT_DISP_ERR(fmt, arg...)	pr_err("[EXTD]:"fmt, ##arg)
#define EXT_DISP_FUNC()			pr_debug("[EXTD]:%s\n", __func__)

/*external display mgr log functions*/
#define EXT_MGR_LOG(fmt, arg...)	pr_debug("[EXTD-MGR]:"fmt, ##arg)
#define EXT_MGR_ERR(fmt, arg...)	pr_err("[EXTD-MGR]:"fmt, ##arg)
#define EXT_MGR_FUNC()			pr_debug("[EXTD-MGR]:%s\n", __func__)

/*external display - factory log functions*/
#define EXTD_FACTORY_LOG(fmt, arg...)		pr_debug("[EXTD]:"fmt, ##arg)
#define EXTD_FACTORY_ERR(fmt, arg...)		pr_err("[EXTD]:"fmt, ##arg)
#define EXTD_FACTORY_FUNC()			pr_debug("[EXTD]:%s\n", __func__)

#define RETIF(cond, rslt)       { if ((cond)) {HDMI_LOG("return in %d\n", __LINE__); return (rslt); } }
#define RET_VOID_IF(cond)       { if ((cond)) {HDMI_LOG("return in %d\n", __LINE__); return; } }
#define RETIF_NOLOG(cond, rslt) { if ((cond)) return (rslt); }
#define RET_VOID_IF_NOLOG(cond) { if ((cond)) return; }
#define RETIFNOT(cond, rslt)    { if (!(cond)) {HDMI_LOG("return in %d\n", __LINE__); return (rslt); } }
#endif
