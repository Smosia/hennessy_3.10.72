#ifndef _DISPLAY_RECOREDR_H_
#define _DISPLAY_RECOREDR_H_

#include <linux/types.h>
#include <linux/mmprofile.h>
#include "disp_event.h"
#include "ddp_info.h"
typedef enum {
	DPREC_EVENT_CMDQ_SET_DIRTY = 0xff00,
	DPREC_EVENT_CMDQ_WAIT_STREAM_EOF,
	DPREC_EVENT_CMDQ_SET_EVENT_ALLOW,
	DPREC_EVENT_CMDQ_FLUSH,
	DPREC_EVENT_CMDQ_RESET,
	DPREC_EVENT_FRAME_DONE,
	DPREC_EVENT_FRAME_START
} DPREC_EVENT;

typedef enum {
	DPREC_ERROR_FRAMEDONE_TIMEOUT = 0xffff00,
	DPREC_ERROR_FENCE_PREPRAE_ERROR,
	DPREC_ERROR_FENCE_SETINPUT_ERROR,
	DPREC_ERROR_FENCE_RELEASE_ERROR,
	DPREC_ERROR_CMDQ_TIMEOUT
} DPREC_ERROR_ENUM;

typedef struct {
	int overall_switch;
	int cmm_dump;
	int cmm_dump_use_va;
	int systrace;
} dprec_debug_control;

typedef struct {
	char *module_name;
	unsigned int module_reg_base;
} reg_base_map;

typedef struct {
	char *event_string;
	DPREC_EVENT event;
} event_string_map;

typedef struct {
	char *log_name;
	unsigned int begin_frm_seq;
	unsigned int end_frm_seq;
} met_log_map;

typedef enum {
	DPREC_STM_OP_IDLE = 0,
	DPREC_STM_OP_RUNNING,
} DPREC_STM_STATE;

typedef enum {
	DPREC_STM_OP_SET_INPUT,
	DPREC_STM_OP_TRIGGER,
	DPREC_STM_OP_VSYNC,
	DPREC_STM_OP_FRAME_DONE,
} DPREC_STM_EVENT;

typedef enum {
	DPREC_LOGGER_PRIMARY_TRIGGER = 0,
	DPREC_LOGGER_PRIMARY_CONFIG,
	DPREC_LOGGER_PRIMARY_CMDQ_SET_DIRTY,
	DPREC_LOGGER_PRIMARY_CMDQ_FLUSH,
	DPREC_LOGGER_PRIMARY_BUFFER_KEEP,
	DPREC_LOGGER_PRIMARY_MUTEX,
	DPREC_LOGGER_DISPMGR_PREPARE,
	DPREC_LOGGER_DISPMGR_SET_INPUT,
	DPREC_LOGGER_DISPMGR_TRIGGER,
	DPREC_LOGGER_DISPMGR_RELEASE,
	DPREC_LOGGER_DISPMGR_CACHE_SYNC,
	DPREC_LOGGER_DISPMGR_WAIT_VSYNC,
	DPREC_LOGGER_RDMA0_TRANSFER,
	DPREC_LOGGER_DSI_EXT_TE,
	DPREC_LOGGER_ESD_RECOVERY,
	DPREC_LOGGER_ESD_CHECK,
	DPREC_LOGGER_ESD_CMDQ,
	DPREC_LOGGER_WDMA_DUMP,
	DPREC_LOGGER_EXTD_START,
	DPREC_LOGGER_EXTD_STATUS = DPREC_LOGGER_EXTD_START,
	DPREC_LOGGER_EXTD_ERR_INFO,
	DPREC_LOGGER_EXTD_PREPARE,
	DPREC_LOGGER_EXTD_SET_INPUT,
	DPREC_LOGGER_EXTD_TRIGGER,
	DPREC_LOGGER_EXTD_RELEASE,
	DPREC_LOGGER_EXTD_IRQ,
	DPREC_LOGGER_EXTD_END = DPREC_LOGGER_EXTD_IRQ,
	DPREC_LOGGER_NUM
} DPREC_LOGGER_ENUM;

#define DPREC_LOGGER_LEVEL_ALL					0xFFFFFFFF
#define DPREC_LOGGER_LEVEL_DEFAULT				(DPREC_LOGGER_LEVEL_MMP | DPREC_LOGGER_LEVEL_LOGGER)
#define DPREC_LOGGER_LEVEL_UART_LOG			(0x1<<0)
#define DPREC_LOGGER_LEVEL_MOBILE_LOG			(0x1<<1)
#define DPREC_LOGGER_LEVEL_MMP				(0x1<<2)
#define DPREC_LOGGER_LEVEL_SYSTRACE			(0x1<<3)
#define DPREC_LOGGER_LEVEL_AEE_DUMP			(0x1<<4)
#define DPREC_LOGGER_LEVEL_LOGGER			(0x1<<5)


typedef struct {
	unsigned long long period_frame;
	unsigned long long period_total;
	unsigned long long period_max_frame;
	unsigned long long period_min_frame;
	unsigned long long ts_start;
	unsigned long long ts_trigger;
	unsigned long long count;
} dprec_logger;

typedef struct {
	int8_t name[24];
	MMP_Event mmp;
	uint32_t level;
	dprec_logger logger;
	/* spinlock_t spinlock; */
} dprec_logger_event;

typedef enum {
	DPREC_LOGGER_ERROR,
	DPREC_LOGGER_FENCE,
	DPREC_LOGGER_DEBUG,
	DPREC_LOGGER_DUMP,
	DPREC_LOGGER_HWOP,
	DPREC_LOGGER_PR_NUM
} DPREC_LOGGER_PR_TYPE;


#define DPREC_ERROR_LOG_BUFFER_LENGTH (1024*16)

void dprec_event_op(DPREC_EVENT event);
void dprec_reg_op(void *cmdq, unsigned int reg, unsigned int val, unsigned int mask);
int dprec_handle_option(unsigned int option);
int dprec_option_enabled(void);
int dprec_init(void);
void dprec_logger_trigger(DPREC_LOGGER_ENUM source, unsigned int val1, unsigned int val2);
void dprec_logger_start(DPREC_LOGGER_ENUM source, unsigned int val1, unsigned int val2);
void dprec_logger_done(DPREC_LOGGER_ENUM source, unsigned int val1, unsigned int val2);
void dprec_logger_reset(DPREC_LOGGER_ENUM source);
void dprec_logger_reset_all(void);
int dprec_logger_get_result_string(DPREC_LOGGER_ENUM source, char *stringbuf, int strlen);
int dprec_logger_get_result_string_all(char *stringbuf, int strlen);
void dprec_stub_irq(unsigned int irq_bit);
void dprec_stub_event(DISP_PATH_EVENT event);
unsigned int dprec_get_vsync_count(void);
void dprec_logger_submit(DPREC_LOGGER_ENUM source, unsigned long long period, unsigned int fence_idx);

void dprec_logger_dump(char *string);
void dprec_logger_dump_reset(void);
char *dprec_logger_get_dump_addr();
unsigned int dprec_logger_get_dump_len(void);
unsigned long long dprec_logger_get_current_hold_period(unsigned int type_logsrc);
int dprec_logger_get_buf(DPREC_LOGGER_PR_TYPE type, char *stringbuf, int strlen);
int dprec_logger_pr(unsigned int type, char *fmt, ...);
void dprec_logger_event_init(dprec_logger_event *p, char *name, uint32_t level, MMP_Event *mmp_root);
void dprec_start(dprec_logger_event *event, unsigned int val1, unsigned int val2);
void dprec_done(dprec_logger_event *event, unsigned int val1, unsigned int val2);
void dprec_trigger(dprec_logger_event *event, unsigned int val1, unsigned int val2);
void dprec_submit(dprec_logger_event *event, unsigned int val1, unsigned int val2);

int dprec_mmp_dump_wdma_layer(void *wdma_layer, unsigned int wdma_num);
void dprec_logger_frame_seq_begin(unsigned int session_id, unsigned frm_sequence);
void dprec_logger_frame_seq_end(unsigned int session_id, unsigned frm_sequence);

/*1:primary, 2:external, 3:memory*/
int dprec_mmp_dump_ovl_layer(OVL_CONFIG_STRUCT *ovl_layer,unsigned int l,unsigned int session);

extern unsigned int gCapturePriLayerEnable;
extern unsigned int gCaptureWdmaLayerEnable;
extern unsigned int gCapturePriLayerDownX;
extern unsigned int gCapturePriLayerDownY;
extern unsigned int gCapturePriLayerNum;
#endif
