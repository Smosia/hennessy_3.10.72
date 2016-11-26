#ifndef _VAL_LOG_H_
#define _VAL_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/kernel.h>
/* #include <linux/xlog.h> */


#define MFV_LOG_ERROR   /* error */
#ifdef MFV_LOG_ERROR
#define MFV_LOGE(...) pr_err(__VA_ARGS__);
#else
#define MFV_LOGE(...)
#endif

#define MFV_LOG_WARNING /* warning */
#ifdef MFV_LOG_WARNING
#define MFV_LOGW(...) pr_warn(__VA_ARGS__);
#else
#define MFV_LOGW(...)
#endif


#define MFV_LOG_DEBUG   /* debug information */
#ifdef MFV_LOG_DEBUG
#define MFV_LOGD(...) pr_debug(__VA_ARGS__);
#else
#define MFV_LOGD(...)
#endif

#define MFV_LOG_INFO   /* info information */
#ifdef MFV_LOG_INFO
#define MFV_LOGI(...) pr_info(__VA_ARGS__);
#else
#define MFV_LOGI(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _VAL_LOG_H_ */
