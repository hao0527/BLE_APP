#ifndef BEARER_EVENT_H__
#define BEARER_EVENT_H__

#include <stdint.h>
//#include "pan_mesh.h"
//#include "pan_mesh_config_core.h"
#include "queue.h"

/**
 * @defgroup BEARER_EVENT Event handler for bearer layer
 * @ingroup MESH_CORE
 * Schedules bearer events for asynchronous processing in configured IRQ priority level.
 * @{
 */

/** Invalid bearer event flag, for indicating a flag that hasn't been added. */
#define BEARER_EVENT_FLAG_INVALID   0xFFFFFFFF

/** Callback function type for generic event processing. */
typedef void(*bearer_event_callback_t)(void* p_context);
/** Callback function type for flag events. */
typedef bool(*bearer_event_flag_callback_t)(void);

/** Bearer event flag type. */
typedef uint32_t bearer_event_flag_t;

/** Bearer event sequential type. */
typedef struct
{
    queue_elem_t queue_elem; /**< Used for linked list operation, should not be altered by the user. */
    bearer_event_callback_t callback; /**< Callback function to be called. */
    void * p_context; /**< Pointer to a context variable to be passed to the callback. */
    volatile bool event_pending; /**< Flag for indicating if an event is currently pending. */
} bearer_event_sequential_t;

/**
 * Initialize the bearer event module.
 *
 * @param[in] irq_priority Bearer event IRQ priority (PAN_MESH_IRQ_PRIORITY_THREAD if thread mode).
 */
void bearer_event_init(uint8_t irq_priority);

/**
 * Post generic event for asynchronous processing.
 * @param[in] callback Callback function to call.
 * @param[in] p_context Pointer to a context variable to be passed to the callback.
 *
 * @retval PAN_SUCCESS The event was successfully posted for processing.
 * @retval PAN_ERROR_NO_MEM The event fifo was full, and the event will not be processed.
 */
uint32_t bearer_event_generic_post(bearer_event_callback_t callback, void* p_context);

/**
 * Prevent the event handler from firing. Start of critical section.
 */
void bearer_event_critical_section_begin(void);

/**
 * Prevent the event handler from firing. End of critical section.
 */
void bearer_event_critical_section_end(void);

/**
 * Add a bearer_event flag callback.
 *
 * @note Will assert if there are no more flags available for allocation. If
 * this happens, increase @ref BEARER_EVENT_FLAG_COUNT appropriately.
 *
 * @param[in] callback Callback function pointer that will be called every time
 * the returned flag is set.
 *
 * @returns A flag that can be referenced in @ref bearer_event_flag_set to trigger the given callback.
 */
bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback);

/**
 * Set the given event flag, triggering the corresponding flag callback as soon as possible.
 *
 * @note Will assert if the given flag doesn't have a callback.
 *
 * @param[in] flag Flag to trigger.
 */
void bearer_event_flag_set(bearer_event_flag_t flag);

/**
 * Add a sequential bearer event object.
 *
 * This event type guarantees that a new event can not be posted before the previous event on this
 * event object has been processed (i.e. the callback function has been executed).
 *
 * @param[out] p_seq Sequential event handle.
 * @param[in] callback Callback function pointer that will be called every time the event is posted.
 * @param[in] p_context Pointer to a context variable to be passed to the callback.
 */
void bearer_event_sequential_add(bearer_event_sequential_t * p_seq, bearer_event_callback_t callback, void * p_context);

/**
 * Post a sequential bearer event.
 *
 * @note Any event data must be written so that it is available through the context pointer that was
 *       passed to bearer_event_sequential_add() before calling bearer_event_sequential_post().
 *
 * @param[in] p_seq Sequential event handle.
 *
 * @retval PAN_SUCCESS The event was successfully posted for processing.
 * @retval PAN_ERROR_BUSY An event is already pending.
 */
uint32_t bearer_event_sequential_post(bearer_event_sequential_t * p_seq);

/**
 * Check if a sequential bearer event is pending.
 *
 * @param[in] p_seq Sequential event handle.
 *
 * @retval true Event is pending.
 * @retval false Event is not pending.
 */
bool bearer_event_sequential_pending(bearer_event_sequential_t * p_seq);

/**
 * Handle pending bearer events.
 *
 * @retval true Handling is done, i.e. no more events are pending.
 * @retval false Handling is not done, i.e. events are still pending.
 */
bool bearer_event_handler(void);

/**
 * Check whether the processor is currently executing in the bearer event IRQ priority.
 *
 * @returns Whether the processor is currently executing in the bearer event IRQ priority.
 */
bool bearer_event_in_correct_irq_priority(void);

/** @} */

void trigger_event_handler(void);
#endif /* BEARER_EVENT_H__ */
