#include <string.h>

#include "mesh_flash.h"
#include "hal_flash_driver.h"
#include "bearer_event.h"
//#include "fifo.h"
#include "hal_flash_driver.h"
#include "mesh_error.h"
#include "msqueue.h"
//#include "bearer_handler.h"
#include "utils.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/** Number of flash operations that can be queued at once. */
#define FLASH_OP_QUEUE_LEN					(6)

/** Maximum overhead of processing the flash queue. */
#define FLASH_PROCESS_TIME_OVERHEAD		    (500)

/* A single page erase operation must fit inside a bearer action */
//PAN_MESH_STATIC_ASSERT(FLASH_TIME_TO_ERASE_PAGE_US + FLASH_PROCESS_TIME_OVERHEAD <= BEARER_ACTION_DURATION_MAX_US);

/* The queue length will overflow if it's longer than 256 */
PAN_MESH_STATIC_ASSERT(FLASH_OP_QUEUE_LEN < 256);
/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    FLASH_OP_STAGE_FREE,
    FLASH_OP_STAGE_QUEUED,
    FLASH_OP_STAGE_PROCESSED,
    FLASH_OP_STAGES
} flash_op_stage_t;

typedef struct
{
    uint16_t event_token;
    uint16_t push_token;
    uint32_t processed_bytes; /**< How many bytes have been processed in the current event. */
    mesh_flash_op_cb_t cb;
    struct
    {
        msq_t queue;
        uint8_t stages[FLASH_OP_STAGES];
        flash_operation_t elems[FLASH_OP_QUEUE_LEN];
    } flash_op_queue;
    bearer_action_t action;
    bool active;
} flash_user_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
bool                m_suspended; /**< Suspend flag, preventing flash operations while set. */

flash_user_t        m_users[MESH_FLASH_USERS];
bearer_event_flag_t m_event_flag;
uint32_t suspend_count;
/** Constant "All operations" operation, used to signalize that all operations
 * have been completed for a user. */
 const flash_operation_t m_all_operations =
{
    .type = FLASH_OP_TYPE_ALL
}; /*lint !e785 Too few initializers for flash_operation_t. */

/*****************************************************************************
* Static functions
*****************************************************************************/
 void flash_op_start(timestamp_t start_time, void * p_args);

 void write_as_much_as_possible(const flash_operation_t* p_write_op, timestamp_t available_time, uint32_t* p_bytes_written)
{
    uint32_t offset = *p_bytes_written;
    MESH_ASSERT(p_write_op->type == FLASH_OP_TYPE_WRITE);
    uint32_t bytes_to_write;// = WORD_SIZE * (available_time / FLASH_TIME_TO_WRITE_ONE_WORD_US);
//	printf("p_write_op->params.write.length:%d,offset:%d\n",p_write_op->params.write.length,offset);
//	printf("bytes_to_write1:%d\n",bytes_to_write);

//    if (bytes_to_write > p_write_op->params.write.length - offset)
//    {
        bytes_to_write = p_write_op->params.write.length - offset;
//    }
    MESH_ASSERT(bytes_to_write > 0);
//	printf("bytes_to_write2:%d\n",bytes_to_write);
    MESH_ASSERT(pan_flash_write(&p_write_op->params.write.p_start_addr[offset / WORD_SIZE],
                    &p_write_op->params.write.p_data[offset / WORD_SIZE],
                    bytes_to_write) == PAN_SUCCESS);

    *p_bytes_written += bytes_to_write;
}

 void erase_as_much_as_possible(const flash_operation_t * p_erase_op, timestamp_t available_time, uint32_t* p_bytes_erased)
{
    uint32_t offset = *p_bytes_erased;
    MESH_ASSERT(p_erase_op->type == FLASH_OP_TYPE_ERASE);
    uint32_t bytes_to_erase;// = PAGE_SIZE * (available_time / FLASH_TIME_TO_ERASE_PAGE_US);
    //if (bytes_to_erase > p_erase_op->params.erase.length - offset)
    {
        bytes_to_erase = p_erase_op->params.erase.length - offset;
    }
    MESH_ASSERT(bytes_to_erase > 0);

    MESH_ASSERT(pan_flash_erase(&p_erase_op->params.erase.p_start_addr[offset / WORD_SIZE],
                    bytes_to_erase) == PAN_SUCCESS);

    *p_bytes_erased += bytes_to_erase;
}

/** Call the callbacks of all users for all processed events. */
 bool send_end_events(void)
{
    for (mesh_flash_user_t i = (mesh_flash_user_t) 0; i < MESH_FLASH_USERS; i++)
    {
        uint32_t notified_events = 0;
        flash_operation_t * p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        while (p_op != NULL)
        {
            if (m_users[i].cb != NULL)
            {
                m_users[i].cb(i, p_op, m_users[i].event_token);
            }
            m_users[i].event_token++;
            msq_move(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
            notified_events++;
            p_op = msq_get(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_PROCESSED);
        }
        /* When every item in the queue has been processed and notified, we
         * tell the user. */
        if (notified_events != 0 &&
            (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) == FLASH_OP_QUEUE_LEN) &&
            m_users[i].cb != NULL)
        {
            m_users[i].cb(i, &m_all_operations, 0);
        }
    }
    return true;
}

 inline void end_event_schedule(void)
{
    bearer_event_flag_set(m_event_flag);
}

 bool execute_next_operation_chunk(flash_user_t * p_user, flash_operation_t * p_op, uint32_t available_time)
{
    uint32_t operation_length = 0;

    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            write_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
            operation_length = p_op->params.write.length;
			//printf("operation_length:%d\n",operation_length);
            break;

        case FLASH_OP_TYPE_ERASE:
			//printf("operation_length:%d\n",operation_length);
            erase_as_much_as_possible(p_op, available_time, &p_user->processed_bytes);
            operation_length = p_op->params.erase.length;
            break;

        default:
            MESH_ASSERT(false);
    }

    return (operation_length == p_user->processed_bytes);
}



 void flash_op_schedule(flash_user_t * p_user)
{
    flash_operation_t * p_op = msq_get(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
    if (p_op != NULL)
    {
        p_user->action.start_cb = flash_op_start;
        p_user->action.p_args = p_user;
        p_user->active = true;

        p_user->action.start_cb(0x123, p_user->action.p_args);
        //MESH_ASSERT(PAN_SUCCESS == bearer_handler_action_enqueue(&p_user->action));
    }
}

 void flash_op_start(timestamp_t start_time, void * p_args)
{
    flash_user_t * p_user = (flash_user_t *)p_args;
    

    /* Terminate bearer action immediately if suspended.
       Will be rescheduled when suspension is removed. */
    if (m_suspended)
    {
        //bearer_handler_action_end();  //zhangzhao
        p_user->active = false;
        return;
    }

    flash_operation_t * p_op = msq_get(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
    MESH_ASSERT(p_op != NULL);

    /* Normally a flash operation takes just a fraction of the theoretical max time.
       For flash operations that are (theoretically) bigger than the maximum duration of a single
       bearer action we therefore try to execute several chunks of this operation. */
    for (;;)
    {
        bool operation_done = execute_next_operation_chunk(p_user, p_op, 0xf);
        if (operation_done)
        {
            //bearer_handler_action_end();  //zhangzhao
            msq_move(&p_user->flash_op_queue.queue, FLASH_OP_STAGE_QUEUED);
            p_user->processed_bytes = 0;
            p_user->active = false;
            end_event_schedule();
            flash_op_schedule(p_user);
            break;
        }
    }
}

 void init_flash_op_queue(flash_user_t * p_user)
{
    p_user->flash_op_queue.queue.elem_count = FLASH_OP_QUEUE_LEN;
    p_user->flash_op_queue.queue.elem_size = sizeof(flash_operation_t);
    p_user->flash_op_queue.queue.p_elem_array = p_user->flash_op_queue.elems;
    p_user->flash_op_queue.queue.stage_count = FLASH_OP_STAGES;
    p_user->flash_op_queue.queue.p_stages = p_user->flash_op_queue.stages;
    msq_init(&p_user->flash_op_queue.queue);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void mesh_flash_init(void)
{
	memset((uint8_t *)m_users,0,sizeof(flash_user_t) *MESH_FLASH_USERS);
    m_event_flag = bearer_event_flag_add(send_end_events);
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
	suspend_count = 0;
	m_suspended = false;
}

void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb)
{
    MESH_ASSERT(user < MESH_FLASH_USERS);
    m_users[user].cb = cb;
}

uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token)
{
    MESH_ASSERT(user < MESH_FLASH_USERS);
    MESH_ASSERT(p_op != NULL);

    if (p_op->type == FLASH_OP_TYPE_WRITE)
    {
        MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_start_addr));
        MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.p_data));
        MESH_ASSERT(IS_WORD_ALIGNED(p_op->params.write.length));
        MESH_ASSERT(p_op->params.write.length != 0);
    }
    else if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.p_start_addr));
        MESH_ASSERT(IS_PAGE_ALIGNED(p_op->params.erase.length));
        MESH_ASSERT(p_op->params.erase.length != 0);
    }
    else
    {
        /* operation type must be WRITE or ERASE */
        MESH_ASSERT(false);
    }
    
    uint32_t status;
    __disable_irq();
    flash_operation_t * p_free_op = msq_get(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
    if (p_free_op == NULL)
    {
        status = PAN_ERROR_NO_MEM;
    }
    else
    {
        msq_move(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        memcpy(p_free_op, p_op, sizeof(flash_operation_t));
        status = PAN_SUCCESS;
        if (p_token != NULL)
        {
            *p_token = m_users[user].push_token;
        }
        m_users[user].push_token++;

        if (!m_users[user].active && !m_suspended)
        {
            flash_op_schedule(&m_users[user]);
        }
    }
    __enable_irq();

    return status;
}

uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user)
{
    if (user < MESH_FLASH_USERS)
    {
        
       __disable_irq();
        uint32_t available_slots = msq_available(&m_users[user].flash_op_queue.queue, FLASH_OP_STAGE_FREE);
        __enable_irq();

        return available_slots;
    }
    else
    {
        return 0;
    }
}

bool mesh_flash_in_progress(void)
{
   
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        __disable_irq();
        bool operations_in_progress = (msq_available(&m_users[i].flash_op_queue.queue, FLASH_OP_STAGE_FREE) != FLASH_OP_QUEUE_LEN);
        __enable_irq();

        if (operations_in_progress)
        {
            return true;
        }
    }
    return false;
}

void mesh_flash_set_suspended(bool suspend)
{
    __disable_irq();
    MESH_ASSERT(suspend || suspend_count > 0);
    if (suspend)
    {
        suspend_count++;
    }
    else
    {
        suspend_count--;
        if (suspend_count == 0)
        {
            for (uint32_t user = 0; user < MESH_FLASH_USERS; user++)
            {
                if (!m_users[user].active)
                {
                    flash_op_schedule(&m_users[user]);
                }
            }
        }
    }
    m_suspended = (suspend_count > 0);
    __enable_irq();
}

#ifdef UNIT_TEST
/**
 * @internal
 * Test-utility function to reset the state of the module between tests. Not
 * exposed in the header, as it should never be called when running on target.
 */
void mesh_flash_reset(void)
{
    m_event_flag = 0;
    m_suspended = false;
    memset(m_users, 0, sizeof(m_users));
    for (uint32_t i = 0; i < MESH_FLASH_USERS; i++)
    {
        init_flash_op_queue(&m_users[i]);
    }
}
#endif
