#ifndef PHYTIUM_CAN_PORT_H
#define PHYTIUM_CAN_PORT_H

#include "jc4010_can.h"

/* Platform adapter.
 *
 * The JC4010 protocol code is platform-independent. This file is the only
 * place that should call the Phytium Standalone SDK CAN driver.
 *
 * Return 0 on success, negative value on failure.
 */
int phytium_can_init(void);
int phytium_can_send(const Jc4010CanFrame *frame);

#endif

