
#include "scheduler.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

__attribute__((weak)) __inline void CRITICAL_REGION_ENTER(void) {
}
__attribute__((weak)) __inline void CRITICAL_REGION_EXIT(void) {
}

typedef struct {
	fpevt_h handler;
	uevt_t event_data;
} event_t;

static volatile uint8_t m_queue_start_index; /**< Index of queue entry at the start of the queue. */
static volatile uint8_t m_queue_end_index;   /**< Index of queue entry at the end of the queue. */
__attribute__((aligned(4))) event_t m_sched_queue[SCHED_QUEUE_LENGTH];

static __inline uint8_t next_index(uint8_t index) {
	return (index < SCHED_QUEUE_LENGTH - 1) ? (index + 1) : 0;
}

static __inline uint8_t app_sched_queue_full() {
	uint8_t tmp = m_queue_start_index;
	return next_index(m_queue_end_index) == tmp;
}
static __inline uint8_t app_sched_queue_empty() {
	uint8_t tmp = m_queue_start_index;
	return m_queue_end_index == tmp;
}
void app_sched_init(void) {
	m_queue_end_index = 0;
	m_queue_start_index = 0;
}

uint32_t app_sched_event_put(uevt_t const* p_event_data, fpevt_h handler) {
	uint32_t err_code;
	uint16_t event_index = 0xFFFF;

	CRITICAL_REGION_ENTER();
	if(!app_sched_queue_full()) {
		event_index = m_queue_end_index;
		m_queue_end_index = next_index(m_queue_end_index);
	}
	CRITICAL_REGION_EXIT();

	if(event_index != 0xFFFF) {
		m_sched_queue[event_index].handler = handler;
		if((p_event_data != NULL)) {
			memcpy(&m_sched_queue[event_index].event_data, p_event_data, sizeof(uevt_t));
		}
		err_code = 0;
	} else {
		err_code = 0xFFFFFFFF;
	}

	return err_code;
}

void app_sched_execute(void) {
	while(!app_sched_queue_empty()) {
		// Since this function is only called from the main loop, there is no
		// need for a critical region here, however a special care must be taken
		// regarding update of the queue start index (see the end of the loop).
		uint16_t event_index = m_queue_start_index;

		uevt_t* p_event_data;
		fpevt_h event_handler;

		p_event_data = &(m_sched_queue[event_index].event_data);
		event_handler = m_sched_queue[event_index].handler;
		event_handler(p_event_data);
		// Event processed, now it is safe to move the queue start index,
		// so the queue entry occupied by this event can be used to store
		// a next one.
		m_queue_start_index = next_index(m_queue_start_index);
	}
}
