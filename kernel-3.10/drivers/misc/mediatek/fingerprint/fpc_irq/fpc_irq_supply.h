/* Fingerprint Cards, Hybrid Touch sensor driver
 *
 * Copyright (c) 2014,2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2018 XiaoMi, Inc.
 *
 *
 * Software license : "Dual BSD/GPL"
 * see <linux/module.h> and ./Documentation
 * for  details.
 *
*/

#ifndef __FPC_IRQ_SUPPLY_H
#define __FPC_IRQ_SUPPLY_H

#include "fpc_irq_common.h"

extern int fpc_irq_supply_init(fpc_irq_data_t *fpc_irq_data);

extern int fpc_irq_supply_destroy(fpc_irq_data_t *fpc_irq_data);

extern int fpc_irq_supply_set(fpc_irq_data_t *fpc_irq_data, bool req_state);

#endif /* __FPC_IRQ_SUPPLY_H */


