#ifndef __ION_DRV_H__
#define __ION_DRV_H__
#include <linux/version.h>

#include <ion.h>

#include <linux/ion_debugger.h>
/* Structure definitions */

typedef enum {
	ION_CMD_SYSTEM,
	ION_CMD_MULTIMEDIA
} ION_CMDS;

typedef enum {
	ION_MM_CONFIG_BUFFER,
	ION_MM_SET_DEBUG_INFO,
	ION_MM_GET_DEBUG_INFO,
	ION_MM_SET_SF_BUF_INFO,
	ION_MM_GET_SF_BUF_INFO
} ION_MM_CMDS;

typedef enum {
	ION_SYS_CACHE_SYNC,
	ION_SYS_GET_PHYS,
	ION_SYS_GET_CLIENT,
	ION_SYS_SET_HANDLE_BACKTRACE,
	ION_SYS_SET_CLIENT_NAME,
	ION_SYS_DMA_OP,
} ION_SYS_CMDS;

typedef enum {
	ION_CACHE_CLEAN_BY_RANGE,
	ION_CACHE_INVALID_BY_RANGE,
	ION_CACHE_FLUSH_BY_RANGE,
	ION_CACHE_CLEAN_BY_RANGE_USE_VA,
	ION_CACHE_INVALID_BY_RANGE_USE_VA,
	ION_CACHE_FLUSH_BY_RANGE_USE_VA,
	ION_CACHE_CLEAN_ALL,
	ION_CACHE_INVALID_ALL,
	ION_CACHE_FLUSH_ALL
} ION_CACHE_SYNC_TYPE;

typedef enum {
	ION_ERROR_CONFIG_LOCKED = 0x10000
} ION_ERROR_E;

typedef struct ion_sys_cache_sync_param {
	union {
		ion_user_handle_t handle;
		struct ion_handle *kernel_handle;
	};
	void *va;
	unsigned int size;
	ION_CACHE_SYNC_TYPE sync_type;
} ion_sys_cache_sync_param_t;

typedef enum {
	ION_DMA_MAP_AREA,
	ION_DMA_UNMAP_AREA,
	ION_DMA_MAP_AREA_VA,
	ION_DMA_UNMAP_AREA_VA,
	ION_DMA_CACHE_FLUSH_ALL
} ION_DMA_TYPE;

typedef enum {
	ION_DMA_FROM_DEVICE,
	ION_DMA_TO_DEVICE,
	ION_DMA_BIDIRECTIONAL,
} ION_DMA_DIR;

typedef struct ion_dma_param {
	union {
		ion_user_handle_t handle;
		void *kernel_handle;
	};
	void *va;
	unsigned int size;
	ION_DMA_TYPE dma_type;
	ION_DMA_DIR dma_dir;
} ion_sys_dma_param_t;

typedef struct ion_sys_get_phys_param {
	union {
		ion_user_handle_t handle;
		struct ion_handle *kernel_handle;
	};
	unsigned int phy_addr;
	unsigned long len;
} ion_sys_get_phys_param_t;

#define ION_MM_DBG_NAME_LEN 16
#define ION_MM_SF_BUF_INFO_LEN 16

typedef struct __ion_sys_client_name {
	char name[ION_MM_DBG_NAME_LEN];
} ion_sys_client_name_t;

typedef struct ion_sys_get_client_param {
	unsigned int client;
} ion_sys_get_client_param_t;

typedef struct ion_sys_data {
	ION_SYS_CMDS sys_cmd;
	union {
		ion_sys_cache_sync_param_t cache_sync_param;
		ion_sys_get_phys_param_t get_phys_param;
		ion_sys_get_client_param_t get_client_param;
		ion_sys_client_name_t client_name_param;
		ion_sys_record_t record_param;
		ion_sys_dma_param_t dma_param;
	};
} ion_sys_data_t;

typedef struct ion_mm_config_buffer_param {
	union {
		ion_user_handle_t handle;
		struct ion_handle *kernel_handle;
	};
	int eModuleID;
	unsigned int security;
	unsigned int coherent;
} ion_mm_config_buffer_param_t;

typedef struct __ion_mm_buf_debug_info {
	union {
		ion_user_handle_t handle;
		struct ion_handle *kernel_handle;
	};
	char dbg_name[ION_MM_DBG_NAME_LEN];
	unsigned int value1;
	unsigned int value2;
	unsigned int value3;
	unsigned int value4;
} ion_mm_buf_debug_info_t;

typedef struct __ion_mm_sf_buf_info {
	union {
		ion_user_handle_t handle;
		struct ion_handle *kernel_handle;
	};
	unsigned int info[ION_MM_SF_BUF_INFO_LEN];
} ion_mm_sf_buf_info_t;

typedef struct ion_mm_data {
	ION_MM_CMDS mm_cmd;
	union {
		ion_mm_config_buffer_param_t config_buffer_param;
		ion_mm_buf_debug_info_t buf_debug_info_param;
		ion_mm_sf_buf_info_t sf_buf_info_param;
	};
} ion_mm_data_t;

#ifdef __KERNEL__

#define ION_LOG_TAG "ion_dbg"
#include <linux/xlog.h>
#define IONMSG(string, args...)	printk("[ION]"string, ##args)
#define IONTMP(string, args...)  printk("[ION]"string, ##args)
#define ion_aee_print(string, args...) do {\
	char ion_name[100];\
	snprintf(ion_name, 100, "["ION_LOG_TAG"]"string, ##args); \
	aee_kernel_warning(ion_name, "["ION_LOG_TAG"]error:"string, ##args);  \
} while (0)

/* Exported global variables */
extern struct ion_device *g_ion_device;

/* Exported functions */
long ion_kernel_ioctl(struct ion_client *client, unsigned int cmd, unsigned long arg);
struct ion_handle *ion_drv_get_handle(struct ion_client *client, int user_handle,
				      struct ion_handle *kernel_handle, int from_kernel);
int ion_drv_put_kernel_handle(void *kernel_handle);

/**
 * ion_mm_heap_total_memory() - get mm heap total buffer size.
 */
size_t ion_mm_heap_total_memory(void);
/**
 * ion_mm_heap_total_memory() - get mm heap buffer detail info.
 */
void ion_mm_heap_memory_detail(void);
int ion_drv_create_FB_heap(ion_phys_addr_t fb_base, size_t fb_size);

typedef int (ion_mm_buf_destroy_callback_t) (struct ion_buffer *buffer, unsigned int phyAddr);
int ion_mm_heap_register_buf_destroy_callback(struct ion_buffer *buffer,
					      ion_mm_buf_destroy_callback_t *fn);

int ion_cache_sync_flush_all(int fd);
int ion_dma_map_area(int fd, ion_user_handle_t handle, int dir);
int ion_dma_unmap_area(int fd, ion_user_handle_t handle, int dir);
void ion_dma_map_area_va(void *start, size_t size, ION_DMA_DIR dir);
void ion_dma_unmap_area_va(void *start, size_t size, ION_DMA_DIR dir);

extern int record_ion_info(int from_kernel, ion_sys_record_t *param);
extern char *get_userString_from_hashTable(char *string_name, unsigned int len);
extern struct ion_heap *ion_mm_heap_create(struct ion_platform_heap *unused);

#endif

#endif
