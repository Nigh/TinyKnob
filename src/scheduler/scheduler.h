
#ifndef APP_SCHEDULER_H__
#define APP_SCHEDULER_H__

#include <stdint.h>
#include "uevent.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCHED_QUEUE_LENGTH (32)

void app_sched_init(void);
uint32_t app_sched_event_put(uevt_t const* p_event_data, fpevt_h handler);
void app_sched_execute(void);

#ifdef __cplusplus
}
#endif

#endif // APP_SCHEDULER_H__
