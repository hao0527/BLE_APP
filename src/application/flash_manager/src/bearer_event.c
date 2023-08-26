#include "bearer_event.h"
//#include "pan_mesh_config_bearer.h"
#include <stddef.h>
//#include "fifo.h"
#include "utils.h"
#include "mesh_error.h"
#include "bitfield.h"
#include "stack_svc_api.h"
//#include "scanner.h"
#define EVENT_IRQn          EINT0_IRQn
#define EVENT_IRQHandler    EINT0_IRQHandler


/*****************************************************************************
* Local type definitions
*****************************************************************************/
/** The different types of bearer events. */
typedef enum
{
    BEARER_EVENT_TYPE_GENERIC,  /**< Generic event type */
} bearer_event_type_t;

/** Bearer event type which is queued for asynchronous processing. */
typedef struct
{
    bearer_event_type_t type;                   /**< Event type of this event. */
    union
    {                               
        struct
        {
            bearer_event_callback_t callback;   /**< Callback function to call during processing of generic event. */
            void* p_context;                    /**< Context pointer to give to callback. */
        } generic;                              /**< Generic event parameters */
    } params;                                   /**< Parameters for async event */
} bearer_event_t;


/*****************************************************************************
* Static globals
*****************************************************************************/
///** FIFO structure for bearer event handler */
// fifo_t m_bearer_event_fifo;
///** FIFO buffer for bearer event handler */
// bearer_event_t m_bearer_event_fifo_buffer[BEARER_EVENT_FIFO_SIZE];
///** IRQ critical section mask */
// uint32_t m_critical;
/** Event flag field. */
uint32_t m_flags[BITFIELD_BLOCK_COUNT(BEARER_EVENT_FLAG_COUNT)];
/** Lookup table of flag event handlers. */
bearer_event_flag_callback_t m_flag_event_callbacks[BEARER_EVENT_FLAG_COUNT];
/** Number of flags allocated. */
uint32_t m_flag_count;
/** Queue of scheduled sequential events. */
queue_t m_sequential_event_queue;
///** Bearer event IRQ priority. */
// uint8_t m_irq_priority;
/*****************************************************************************
* System callback functions
*****************************************************************************/


#if !defined(HOST)
/* IRQ handler for asynchronous processing */
void EVENT_Handler(void)
{
    (void)bearer_event_handler();
}
#endif

/*****************************************************************************
* Static functions
*****************************************************************************/

void trigger_event_handler(void)
{

    (void) NVIC_SetPendingIRQ(EVENT_IRQn);

}


/*****************************************************************************
* Interface functions
*****************************************************************************/
void bearer_event_init(uint8_t irq_priority)
{
    queue_init(&m_sequential_event_queue);
    (void) NVIC_SetPriority(EVENT_IRQn, irq_priority);
    (void) NVIC_EnableIRQ(EVENT_IRQn);
	
	m_flag_count = 0;
	memset(m_flags,0,sizeof(uint32_t) * BITFIELD_BLOCK_COUNT(BEARER_EVENT_FLAG_COUNT));
	memset(m_flag_event_callbacks,0,sizeof(bearer_event_flag_callback_t) * BEARER_EVENT_FLAG_COUNT);
	((interrupt_register_handler)SVC_interrupt_register)(EINT0_IRQ,EVENT_Handler);
}


void bearer_event_critical_section_begin(void)
{
    //__disable_irq(); 
    //scanner_disable();
    //NVIC_DisableIRQ(TMR0_IRQn);
}

void bearer_event_critical_section_end(void)
{
    //scanner_disable();
    //scanner_enable();
    //NVIC_EnableIRQ(TMR0_IRQn);
   //__enable_irq(); 
}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback)
{
    MESH_ASSERT(callback != NULL);

    /* Check if we can still fit flags in the pool. */
    MESH_ASSERT(m_flag_count < BEARER_EVENT_FLAG_COUNT);

    
    __disable_irq();

    uint32_t flag = m_flag_count++;
    m_flag_event_callbacks[flag] = callback;

    __enable_irq();

    return flag;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    MESH_ASSERT(flag < m_flag_count);  
    __disable_irq();
    bitfield_set((uint32_t *) m_flags, flag);
    trigger_event_handler();
    __enable_irq();
}

void bearer_event_sequential_add(bearer_event_sequential_t * p_seq, bearer_event_callback_t callback, void * p_context)
{
    MESH_ASSERT(p_seq != NULL);
    MESH_ASSERT(callback != NULL);

    __disable_irq();

    p_seq->callback = callback;
    p_seq->p_context = p_context;
    p_seq->event_pending = false;
    p_seq->queue_elem.p_data = p_seq;

    /* Note: The queue element is not pushed to the queue until the event is actually posted */

    __enable_irq();
}

uint32_t bearer_event_sequential_post(bearer_event_sequential_t * p_seq)
{
    uint32_t status;

    MESH_ASSERT(p_seq != NULL);
    MESH_ASSERT(p_seq->callback != NULL);

   
    __disable_irq();

    if (p_seq->event_pending)
    {
        status = PAN_ERROR_BUSY;
    }
    else
    {
        p_seq->event_pending = true;
        queue_push(&m_sequential_event_queue, &p_seq->queue_elem);
        trigger_event_handler();
        status = PAN_SUCCESS;
    }

    __enable_irq();

    return status;
}

bool bearer_event_sequential_pending(bearer_event_sequential_t * p_seq)
{
    MESH_ASSERT(p_seq != NULL);
    return p_seq->event_pending;
}

bool bearer_event_handler(void)
{
    bool done = true;
    
    /* Handle flag events */
    for (uint32_t i = 0; i < m_flag_count; i++)
    {
        if (bitfield_get((uint32_t *) m_flags, i))
        {
            
            __disable_irq();
            bitfield_clear((uint32_t *) m_flags, i);
            __enable_irq();

            /* Retriggering flag and returning if callback is not done with its task to avoid
             * starvation of other low priority events. This way incoming packets can be processed
             * one by one, while other events can be processed in between. */
            bool callback_done = m_flag_event_callbacks[i]();
            if (!callback_done)
            {
                done = false;
                bearer_event_flag_set(i);
            }
        }
    }

    /* Handle sequential events */
    queue_elem_t * p_queue_elem = queue_pop(&m_sequential_event_queue);
    while (p_queue_elem != NULL)
    {
        bearer_event_sequential_t * p_seq = (bearer_event_sequential_t *)p_queue_elem->p_data;

        MESH_ASSERT(p_seq->event_pending);
        p_seq->callback(p_seq->p_context);
        p_seq->event_pending = false;

        p_queue_elem = queue_pop(&m_sequential_event_queue);
    }

    return done;
}

bool bearer_event_in_correct_irq_priority(void)
{
    return true;
}
