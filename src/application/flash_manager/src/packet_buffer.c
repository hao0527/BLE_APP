#include <stdbool.h>
#include <string.h>
#include "packet_buffer.h"
#include "utils.h"
//#include "mesh_assert.h"
#include "mesh_error.h"


/*******************************                  *******************************
******************************** Local functions ********************************
********************************                  *******************************/

 inline uint16_t m_get_packet_buffer_index(const packet_buffer_t * p_buffer, const packet_buffer_packet_t * p_packet)
{
    return (uint16_t)((const uint8_t *)p_packet - p_buffer->buffer);
}


 inline packet_buffer_packet_t * m_get_packet(const packet_buffer_t * p_buffer, uint16_t index)
{
    return (packet_buffer_packet_t *) &p_buffer->buffer[index];
}

 void m_index_increment(const packet_buffer_t * p_buffer, uint16_t * p_index)
{
    uint16_t packet_size = sizeof(packet_buffer_packet_t) + m_get_packet(p_buffer, *p_index)->size;
    uint16_t next = *p_index + ALIGN_VAL(packet_size, WORD_SIZE);

    if (next > p_buffer->size - sizeof(packet_buffer_packet_t))
    {
        /* can't fit a header, roll over. */
        next = 0;
    }

    *p_index = next;
}

 packet_buffer_packet_t * m_get_next_packet(const packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    uint16_t index = m_get_packet_buffer_index(p_buffer, p_packet);
    m_index_increment(p_buffer, &index);
    return m_get_packet(p_buffer, index);
}

 void m_free_popped_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    MESH_ASSERT((uint8_t *) p_packet == &p_buffer->buffer[p_buffer->tail]);

    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;

    m_index_increment(p_buffer, &p_buffer->tail);
    if (m_get_packet(p_buffer, p_buffer->tail)->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_buffer->tail = 0;
    }
}

 void m_free_reserved_packet(packet_buffer_t * p_buffer, packet_buffer_packet_t * p_packet)
{
    /* Only the head can be reserved */
    MESH_ASSERT((uint8_t *) p_packet == &p_buffer->buffer[p_buffer->head]);

    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
}

 uint16_t m_max_packet_len_get(const packet_buffer_t *  const p_buffer)
{
    return (p_buffer->size - sizeof(packet_buffer_packet_t));
}

 packet_buffer_packet_t * m_reserve_packet(packet_buffer_t * p_buffer, uint16_t length)
{
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->head);
    p_packet->size = length;
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_RESERVED;

#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif
    return p_packet;
}

 void m_reset_buffer(packet_buffer_t * p_buffer)
{
    m_get_packet(p_buffer, 0)->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    p_buffer->head                          = 0;
    p_buffer->tail                          = 0;
}

/* Checks if there is sufficient space in the packet buffer for the given packet length,
 * moves the packet_buffer head and tail indexes as necessary. */
 uint32_t m_prepare_for_reserve(packet_buffer_t * p_buffer, uint16_t length)
{
    uint16_t packet_len_with_header = ALIGN_VAL(length + sizeof(packet_buffer_packet_t), WORD_SIZE);
    uint32_t status;

    if (p_buffer->head < p_buffer->tail)
    {
        if (packet_len_with_header <= (p_buffer->tail - p_buffer->head))
        {
            status = PAN_SUCCESS;
        }
        else
        {
            /* There's just no space */
            status = PAN_ERROR_NO_MEM;
        }
    }
    else if (p_buffer->head > p_buffer->tail)
    {
        uint32_t space_before_end = (p_buffer->size - p_buffer->head);

        if (packet_len_with_header <= space_before_end)
        {
            status = PAN_SUCCESS;
        }
        else if (packet_len_with_header <= p_buffer->tail)
        {
            /* There's space at the beginning, pad the rest of the buffer */
            if (sizeof(packet_buffer_packet_t) <= space_before_end)
            {
                m_get_packet(p_buffer, p_buffer->head)->packet_state = PACKET_BUFFER_MEM_STATE_PADDING;
            }
            p_buffer->head = 0;
            status = PAN_SUCCESS;
        }
        else
        {
            status = PAN_ERROR_NO_MEM;
        }
    }
    else /* head == tail */
    {
        if (m_get_packet(p_buffer, p_buffer->tail)->packet_state == PACKET_BUFFER_MEM_STATE_FREE)
        {
            /* Buffer is empty */
            m_reset_buffer(p_buffer);
            status = PAN_SUCCESS;
        }
        else
        {
            /* Completely full */
            status = PAN_ERROR_NO_MEM;
        }
    }

    return status;
}

/*******************************                  *******************************
******************************** Public functions *******************************
********************************                  *******************************/
uint16_t packet_buffer_max_packet_len_get(const packet_buffer_t * const p_buffer)
{
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(p_buffer->size > sizeof(packet_buffer_packet_t));

    return m_max_packet_len_get(p_buffer);
}


void packet_buffer_init(packet_buffer_t * p_buffer, void * const p_pool, const uint16_t pool_size)
{
    MESH_ASSERT(NULL != p_pool);
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(IS_VALID_RAM_ADDR(p_pool));
    MESH_ASSERT(IS_VALID_RAM_ADDR( (uint8_t *) p_pool + pool_size - 1));
    MESH_ASSERT(pool_size > sizeof(packet_buffer_packet_t) );

    p_buffer->size = pool_size;
    p_buffer->head = 0;
    p_buffer->tail = 0;
    p_buffer->buffer = (uint8_t*) p_pool;
    packet_buffer_packet_t * p_first_packet = m_get_packet(p_buffer, 0);
    p_first_packet->size = p_buffer->size - sizeof(packet_buffer_packet_t);
    p_first_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
}

void packet_buffer_flush(packet_buffer_t * p_buffer)
{
    MESH_ASSERT(p_buffer != NULL);
    /* Can't flush while a packet is reserved: */
    MESH_ASSERT(m_get_packet(p_buffer, p_buffer->head)->packet_state !=
                    PACKET_BUFFER_MEM_STATE_RESERVED);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    /* We're altering the popping here, and risk asserting if someone comes in and pops a packet
     * while we're flushing. :( */
    
    __disable_irq();
    bool has_popped_packet = (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED);

    if (has_popped_packet)
    {
        /* Move head to the next packet after the popped packet.  */
        p_packet = m_get_next_packet(p_buffer, p_packet);
        p_buffer->head         = m_get_packet_buffer_index(p_buffer, p_packet);
        p_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }
    else
    {
        m_reset_buffer(p_buffer);
    }

    __enable_irq();
}

uint32_t packet_buffer_reserve(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet, uint16_t length)
{
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(NULL != pp_packet);
    MESH_ASSERT(m_get_packet(p_buffer, p_buffer->head)->packet_state !=
                    PACKET_BUFFER_MEM_STATE_RESERVED);
    uint32_t status = PAN_SUCCESS;
    if (length == 0 || length > m_max_packet_len_get(p_buffer))
    {
        status = PAN_ERROR_INVALID_LENGTH;
    }
    else
    {
        /* Check if the packet buffer has enough space for the requested packet. */
        status = m_prepare_for_reserve(p_buffer, length);
        if (PAN_SUCCESS == status)
        {
            *pp_packet = m_reserve_packet(p_buffer, length);
        }
    }

    return status;
}

void packet_buffer_commit(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet, uint16_t length)
{
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(NULL != p_packet);
    MESH_ASSERT(PACKET_BUFFER_MEM_STATE_RESERVED == p_packet->packet_state);
    MESH_ASSERT(p_packet->size >= length);
    MESH_ASSERT(0 < length);

    p_packet->size         = length;
    p_packet->packet_state = PACKET_BUFFER_MEM_STATE_COMMITTED;

    /* Move the head to the next available slot */
    m_index_increment(p_buffer, &p_buffer->head);

    if (p_buffer->head != p_buffer->tail)
    {
        /* Mark the current head as available */
        packet_buffer_packet_t * p_next_packet = m_get_packet(p_buffer, p_buffer->head);
        p_next_packet->packet_state = PACKET_BUFFER_MEM_STATE_FREE;
    }

#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif
}

uint32_t packet_buffer_pop(packet_buffer_t * const p_buffer, packet_buffer_packet_t ** pp_packet)
{
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(NULL != pp_packet);

    uint32_t status = PAN_SUCCESS;
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);

    if (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        m_index_increment(p_buffer, &p_buffer->tail);
        p_packet = m_get_packet(p_buffer, p_buffer->tail);
    }

    switch (p_packet->packet_state)
    {
        case PACKET_BUFFER_MEM_STATE_COMMITTED:
            p_packet->packet_state = PACKET_BUFFER_MEM_STATE_POPPED;
            *pp_packet             = p_packet;
            break;
        case PACKET_BUFFER_MEM_STATE_POPPED:
            MESH_ASSERT(false);
            break;
        default:
            status = PAN_ERROR_NOT_FOUND;
            break;
    }

    return status;
}

bool packet_buffer_can_pop(packet_buffer_t * p_buffer)
{
    MESH_ASSERT(NULL != p_buffer);

    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    if (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_packet = m_get_next_packet(p_buffer, p_packet);
    }
    return (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);
}

bool packet_buffer_packets_ready_to_pop(packet_buffer_t * p_buffer)
{
    MESH_ASSERT(NULL != p_buffer);
    /* get first non-popped packet */
    packet_buffer_packet_t * p_packet = m_get_packet(p_buffer, p_buffer->tail);
    while (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_POPPED ||
           p_packet->packet_state == PACKET_BUFFER_MEM_STATE_PADDING)
    {
        p_packet = m_get_next_packet(p_buffer, p_packet);
    }
    return (p_packet->packet_state == PACKET_BUFFER_MEM_STATE_COMMITTED);
}

void packet_buffer_free(packet_buffer_t * const p_buffer, packet_buffer_packet_t * const p_packet)
{
    MESH_ASSERT(NULL != p_buffer);
    MESH_ASSERT(NULL != p_packet);

    switch (p_packet->packet_state)
    {
        case PACKET_BUFFER_MEM_STATE_POPPED:
            m_free_popped_packet(p_buffer, p_packet);
            break;
        case PACKET_BUFFER_MEM_STATE_RESERVED:
            m_free_reserved_packet(p_buffer, p_packet);
            break;
        default:
            /* Only POPPED packets and RESERVED packets can be freed. */
            MESH_ASSERT(false);
    }

#if PACKET_BUFFER_DEBUG_MODE
    _GET_LR(p_packet->last_caller);
#endif
}

