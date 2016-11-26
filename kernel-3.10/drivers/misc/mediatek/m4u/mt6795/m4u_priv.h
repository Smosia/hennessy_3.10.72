#ifndef __M4U_PRIV_H__
#define __M4U_PRIV_H__
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/aee.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/mmprofile.h>
#include <mach/m4u.h>
#include "m4u_reg.h"
#include "m4u_pgtable.h"

#define M4UMSG(string, args...)	pr_err("[M4U] "string,##args)
#define M4UINFO(string, args...) pr_debug("[M4U] "string,##args)


#include "m4u_hw.h"

#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
#define M4U_TEE_SERVICE_ENABLE
#endif

//#define M4U_FPGAPORTING
#define M4U_PROFILE
#define M4U_4GBDRAM
#define M4U_DVT 0

#ifndef M4U_PROFILE
#define MMProfileLogEx(...)
#define MMProfileEnable(...)
#define MMProfileStart(...)
#endif


#ifndef dmac_map_area
#define dmac_map_area __dma_map_area
#endif

#ifndef dmac_unmap_area 
#define dmac_unmap_area __dma_unmap_area
#endif

#ifndef dmac_flush_range
#define dmac_flush_range __dma_flush_range
#endif

#ifndef outer_clean_all
#define outer_clean_all(...)
#endif

#ifndef outer_flush_all
#define outer_flush_all(...)
#endif

#ifdef M4U_FPGAPORTING
#define enable_clock(...)
#define disable_clock(...)
#define smp_inner_dcache_flush_all(...)
#define register_larb_monitor(...)
#endif

struct m4u_device 
{
    struct miscdevice dev;
    struct proc_dir_entry *m4u_dev_proc_entry;
    struct device *pDev[TOTAL_M4U_NUM];
    struct dentry *debug_root;
    unsigned long m4u_base[TOTAL_M4U_NUM];
    unsigned int irq_num[TOTAL_M4U_NUM];
};


typedef struct
{
    imu_pgd_t *pgd;
    dma_addr_t pgd_pa;
    struct mutex  pgtable_mutex;
    unsigned int pgsize_bitmap;

}m4u_domain_t;


typedef struct
{
    struct list_head link;
    unsigned long va;
    unsigned int mva;
    unsigned int size;
    M4U_PORT_ID port;
    unsigned int prot;
    unsigned int flags;
    struct sg_table *sg_table;

    unsigned int mva_align;
    unsigned int size_align;
    int seq_id;
    unsigned long mapped_kernel_va_for_debug;
} m4u_buf_info_t;


typedef struct _M4U_MAU
{
    M4U_PORT_ID port;             
    bool write;                 
    unsigned int mva;
    unsigned int size; 	
    bool enable;
    bool force; 		
}M4U_MAU_STRUCT;

typedef struct _M4U_TF
{
    M4U_PORT_ID port;             
    bool fgEnable;                 		
}M4U_TF_STRUCT;


//================================
//=== define in m4u_mva.c=========

typedef int (mva_buf_fn_t)(void* priv, unsigned int mva_start, unsigned int mva_end, void *data);

void m4u_mvaGraph_init(void* priv_reserve);
void m4u_mvaGraph_dump_raw(void);
void m4u_mvaGraph_dump(void);
void* mva_get_priv_ext(unsigned int mva);
int mva_for_each_priv(mva_buf_fn_t *fn, void* data);
unsigned int get_first_valid_mva();
int m4u_confirm_all_invalidated(int m4u_index);
int m4u_confirm_range_invalidated(int m4u_index, unsigned int MVAStart, unsigned int MVAEnd);
void m4u_enable_prefetch(int m4u_index, int fgenable);
int m4u_enable_error_hang(int m4u_index, int fgenable);
int m4u_enable_MTLB_allshare(int m4u_index, int fgenable);
int m4u_manual_insert_entry(int m4u_id, unsigned int EntryMVA, int layer, int pt_type, int secure, int Lock, unsigned int pa);
int m4u_dump_valid_main_tlb(int m4u_id, int m4u_slave_id);
int m4u_get_pt_type(m4u_domain_t *domain, unsigned int mva);

int mau_start_monitor(int m4u_id, int m4u_slave_id, int mau_set,
                      int wr, int vir, int io, int bit32,
                      unsigned int start, unsigned int end, unsigned int port_mask, unsigned int larb_mask);



void* mva_get_priv(unsigned int mva);
unsigned int m4u_do_mva_alloc(unsigned long va, unsigned int size, void *priv);
unsigned int m4u_do_mva_alloc_fix(unsigned int mva, unsigned int size, void *priv);
int m4u_do_mva_free(unsigned int mva, unsigned int size);

//=================================
//==== define in m4u_pgtable.c=====
void m4u_dump_pgtable(m4u_domain_t *domain, struct seq_file *seq);
void m4u_dump_pte_nolock(m4u_domain_t *domain, unsigned int mva);
void m4u_dump_pte(m4u_domain_t *domain, unsigned int mva);
int m4u_pgtable_init(struct m4u_device *m4u_dev, m4u_domain_t *m4u_domain);
int m4u_map_4K(m4u_domain_t* m4u_domain, unsigned int mva, unsigned long pa, unsigned int prot);
int m4u_clean_pte(m4u_domain_t *domain, unsigned int mva, unsigned int size);

unsigned long m4u_get_pte(m4u_domain_t *domain, unsigned int mva);


//=================================
//==== define in m4u_hw.c     =====
void m4u_invalid_tlb_by_range(m4u_domain_t *m4u_domain,unsigned int mva_start,unsigned int mva_end);
void m4u_invalid_tlb_all(int m4u_id);
m4u_domain_t * m4u_get_domain_by_port(M4U_PORT_ID port);
m4u_domain_t * m4u_get_domain_by_id(int id);
int m4u_get_domain_nr(void);
int m4u_reclaim_notify(int port, unsigned int mva, unsigned int size);
int m4u_hw_init(struct m4u_device *m4u_dev, int m4u_id);
int m4u_hw_deinit(struct m4u_device *m4u_dev, int m4u_id);
int m4u_reg_backup(void);
int m4u_reg_restore(void);
int m4u_insert_seq_range(M4U_PORT_ID port,unsigned int MVAStart,unsigned int MVAEnd);
int m4u_invalid_seq_range_by_id(int port, int seq_id);
void m4u_print_port_status(struct seq_file *seq, int only_print_active);

int m4u_dump_main_tlb(int m4u_id, int m4u_slave_id);
int m4u_dump_pfh_tlb(int m4u_id);
int m4u_domain_init(struct m4u_device *m4u_dev, void* priv_reserve);

int config_mau(M4U_MAU_STRUCT mau);
int m4u_enable_tf(int port, bool fgenable);


extern int gM4U_4G_DRAM_Mode;
extern unsigned int enable_4G(void);

//=================================
//==== define in m4u.c     =====
int m4u_dump_buf_info(struct seq_file * seq);
int m4u_map_sgtable(m4u_domain_t *m4u_domain, unsigned int mva,
                    struct sg_table *sg_table, unsigned int size, unsigned int prot);
int m4u_unmap(m4u_domain_t *domain, unsigned int mva, unsigned int size);


void m4u_get_pgd(m4u_client_t* client, M4U_PORT_ID port, void** pgd_va, void** pgd_pa, unsigned int* size);
unsigned long m4u_mva_to_pa(m4u_client_t* client, M4U_PORT_ID port, unsigned int mva);
int m4u_query_mva_info(unsigned int mva, unsigned int size, unsigned int *real_mva, unsigned int *real_size);


//=================================
//==== define in m4u_debug.c =====
int m4u_debug_init(struct m4u_device *m4u_dev);

static inline dma_addr_t get_sg_phys(struct scatterlist *sg)
{
    dma_addr_t pa;
    pa = sg_dma_address(sg);
    if(pa == 0)
        pa = sg_phys(sg);
    return pa;
}

#define M4U_PGD_SIZE (16*1024)

#define M4U_LOG_LEVEL_HIGH    3
#define M4U_LOG_LEVEL_MID     2
#define M4U_LOG_LEVEL_LOW     1

extern int gM4U_log_level;
extern int gM4U_log_to_uart;
#define _M4ULOG(level, string, args...) \
do{\
    if(level > gM4U_log_level)\
    {\
        if(level > gM4U_log_to_uart)\
            pr_warn("[M4U] "string, ##args);\
        else\
            pr_debug("[M4U] "string, ##args);\
    }\
}while(0)

#define M4ULOG_LOW(string, args...) _M4ULOG(M4U_LOG_LEVEL_LOW, string, ##args)
#define M4ULOG_MID(string, args...) _M4ULOG(M4U_LOG_LEVEL_MID, string, ##args)
#define M4ULOG_HIGH(string, args...) _M4ULOG(M4U_LOG_LEVEL_HIGH, string, ##args)


#define M4UERR(string, args...) do {\
	pr_err("[M4U] error:"string,##args);  \
	aee_kernel_exception("M4U", "[M4U] error:"string,##args);  \
}while(0)

#define m4u_aee_print(string, args...) do{\
    char m4u_name[100];\
    snprintf(m4u_name,100, "[M4U]"string, ##args); \
    aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_MMPROFILE_BUFFER, m4u_name, "[M4U] error"string, ##args);  \
	pr_err("[M4U] error:"string,##args);  \
}while(0)
    /*aee_kernel_warning(m4u_name, "[M4U] error:"string,##args); */

#define M4U_PRINT_LOG_OR_SEQ(seq_file, fmt, args...) \
    do{\
        if(seq_file)\
            seq_printf(seq_file, fmt, ##args);\
        else\
            pr_debug(fmt, ##args);\
    }while(0)


//=======================================
//==== other macros ============
#define M4U_GET_PAGE_NUM(va,size) ((((va)&(PAGE_SIZE-1))+(size)+(PAGE_SIZE-1))>>12)
#define M4U_PAGE_MASK 0xfffL

typedef enum {
    M4U_MMP_ALLOC_MVA=0,
    M4U_MMP_DEALLOC_MVA,
    M4U_MMP_CONFIG_PORT,
    M4U_MMP_M4U_ERROR,
    M4U_MMP_CACHE_SYNC,
    M4U_MMP_TOGGLE_CG,    
    M4U_MMP_MAX,
}M4U_MMP_TYPE;
extern MMP_Event M4U_MMP_Events[M4U_MMP_MAX];


typedef struct 
{
	M4U_PORT_ID port;	
    unsigned long BufAddr;				
	unsigned int BufSize;				
	unsigned int prot;
	unsigned int MVAStart;						 
	unsigned int MVAEnd;
    unsigned int flags;

}M4U_MOUDLE_STRUCT;


typedef struct 
{
    M4U_PORT_ID port;             
    M4U_CACHE_SYNC_ENUM eCacheSync;
    unsigned long va;                 
    unsigned int size; 
    unsigned int mva;
}M4U_CACHE_STRUCT;

typedef struct _M4U_DMA {
	M4U_PORT_ID port;
	M4U_DMA_TYPE eDMAType;
	M4U_DMA_DIR eDMADir;
	unsigned long va;
	unsigned int size;
	unsigned int mva;
} M4U_DMA_STRUCT;

//IOCTL commnad
#define MTK_M4U_MAGICNO 'g'
#define MTK_M4U_T_POWER_ON            _IOW(MTK_M4U_MAGICNO, 0, int)
#define MTK_M4U_T_POWER_OFF           _IOW(MTK_M4U_MAGICNO, 1, int)
#define MTK_M4U_T_DUMP_REG            _IOW(MTK_M4U_MAGICNO, 2, int)
#define MTK_M4U_T_DUMP_INFO           _IOW(MTK_M4U_MAGICNO, 3, int)
#define MTK_M4U_T_ALLOC_MVA           _IOWR(MTK_M4U_MAGICNO,4, int)
#define MTK_M4U_T_DEALLOC_MVA         _IOW(MTK_M4U_MAGICNO, 5, int)
#define MTK_M4U_T_INSERT_TLB_RANGE    _IOW(MTK_M4U_MAGICNO, 6, int)
#define MTK_M4U_T_INVALID_TLB_RANGE   _IOW(MTK_M4U_MAGICNO, 7, int)
#define MTK_M4U_T_INVALID_TLB_ALL     _IOW(MTK_M4U_MAGICNO, 8, int)
#define MTK_M4U_T_MANUAL_INSERT_ENTRY _IOW(MTK_M4U_MAGICNO, 9, int)
#define MTK_M4U_T_CACHE_SYNC          _IOW(MTK_M4U_MAGICNO, 10, int)
#define MTK_M4U_T_CONFIG_PORT         _IOW(MTK_M4U_MAGICNO, 11, int)
#define MTK_M4U_T_CONFIG_ASSERT       _IOW(MTK_M4U_MAGICNO, 12, int)
#define MTK_M4U_T_INSERT_WRAP_RANGE   _IOW(MTK_M4U_MAGICNO, 13, int)
#define MTK_M4U_T_MONITOR_START       _IOW(MTK_M4U_MAGICNO, 14, int)
#define MTK_M4U_T_MONITOR_STOP        _IOW(MTK_M4U_MAGICNO, 15, int)
#define MTK_M4U_T_RESET_MVA_RELEASE_TLB  _IOW(MTK_M4U_MAGICNO, 16, int)
#define MTK_M4U_T_CONFIG_PORT_ROTATOR _IOW(MTK_M4U_MAGICNO, 17, int)
#define MTK_M4U_T_QUERY_MVA           _IOW(MTK_M4U_MAGICNO, 18, int)
#define MTK_M4U_T_M4UDrv_CONSTRUCT    _IOW(MTK_M4U_MAGICNO, 19, int)
#define MTK_M4U_T_M4UDrv_DECONSTRUCT  _IOW(MTK_M4U_MAGICNO, 20, int)
#define MTK_M4U_T_DUMP_PAGETABLE      _IOW(MTK_M4U_MAGICNO, 21, int)
#define MTK_M4U_T_REGISTER_BUFFER     _IOW(MTK_M4U_MAGICNO, 22, int)
#define MTK_M4U_T_CACHE_FLUSH_ALL     _IOW(MTK_M4U_MAGICNO, 23, int)
#define MTK_M4U_T_CONFIG_PORT_ARRAY   _IOW(MTK_M4U_MAGICNO, 26, int)
#define MTK_M4U_T_CONFIG_MAU          _IOW(MTK_M4U_MAGICNO, 27, int)
#define MTK_M4U_T_CONFIG_TF           _IOW(MTK_M4U_MAGICNO, 28, int)
#define MTK_M4U_T_DMA_OP              _IOW(MTK_M4U_MAGICNO, 29, int)
#define MTK_M4U_T_SEC_INIT            _IOW(MTK_M4U_MAGICNO, 50, int)


#ifdef M4U_TEE_SERVICE_ENABLE
int m4u_config_port_tee(M4U_PORT_STRUCT* pM4uPort);
int m4u_larb_backup_sec(unsigned int larb_idx);
int m4u_larb_restore_sec(unsigned int larb_idx);
int m4u_config_port_array_tee(unsigned char* port_array);
int m4u_sec_init(void);

#endif

#endif
