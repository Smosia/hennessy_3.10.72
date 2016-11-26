#ifndef __ION_DRV_PRIV_H__
#define __ION_DRV_PRIV_H__

#include "ion_priv.h"

#ifdef ION_HISTORY_RECORD 
extern int ion_history_init(void);
#else
static inline int ion_history_init(void)
{
	return 0;
}
#endif

int ion_mm_heap_for_each_pool(int (*fn)(int high, int order, int cache, size_t size));
struct ion_heap * ion_drv_get_heap(struct ion_device *dev, int heap_id, int need_lock);
int ion_drv_create_heap(struct ion_platform_heap *heap_data);

#endif

