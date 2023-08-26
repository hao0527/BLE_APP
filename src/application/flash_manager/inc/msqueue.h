#ifndef MSQUEUE_H__
#define MSQUEUE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup MULTI_STAGE_QUEUE Multi stage queue module
 * @ingroup MESH_CORE
 * Generic multi-stage FIFO queue for fixed-size objects.
 *
 * The multi-stage queue operates very similarly to the @c fifo_t queue, with three distictions:
 * - It does not do any IRQ locking, and assumes that each "stage" in the queue is operated from a
 *   single context.
 * - It does not do any copying, but instead provides pointers directly into the queue buffer.
 * - The multistage queue operates in N stages, where each stage is an index in the queue. This
 *   allows the user to process elements in a pipeline-like manner, where each stage processes an
 *   element, before making it available to the next. Stage 0 is the head of the pipeline, while
 *   the last stage is the tail.
 *
 * Example usage:
 * @code{c}
    void set_value(void)
    {
        uint32_t * p_integer = msq_get(&m_queue, 0);
        *p_integer = 42;
        msq_move(&queue, 0);
    }

    uint32_t get_value(void)
    {
        uint32_t * p_integer = msq_get(&m_queue, 1);
        uint32_t retval = *p_integer;
        msq_move(&queue, 1);
        return retval;
    }
 * @endcode
 * @{
 */


/**
 * Single queue instance.
 */
typedef struct
{
   uint8_t stage_count; /**< The number of stages in the @c p_stages array. */
   uint8_t elem_size;   /**< Size of a single element in bytes. */
   uint8_t elem_count;  /**< Number of elements in the elem_array. Must be a power of two. */
   uint8_t * p_stages;  /**< Array used for keeping track of the different stages of the queue. */
   void * p_elem_array; /**< Element array of the elements operated on. */
} msq_t;

/**
 * Initializes a message queue instance.
 *
 * All member variables will be checked, and the queue will be flushed.
 *
 * @param[in,out] p_queue The queue to initialize.
 */
void msq_init(msq_t * p_queue);

/**
 * Gets the element in a specified stage.
 *
 * @param[in] p_queue Queue to get from.
 * @param[in] stage   Stage in the queue to get from.
 *
 * @returns A pointer to the element at the specified stage, or NULL if no element has reached
 *          this stage.
 */
void * msq_get(const msq_t * p_queue, uint8_t stage);

/**
 * Moves a stage one element, marking the completion of this stage for the current element.
 *
 * @param[in,out] p_queue Queue to move in.
 * @param[in]     stage   Stage to move.
 */
void msq_move(msq_t * p_queue, uint8_t stage);

/**
 * Flushes all stages in the queue.
 *
 * @param[in,out] p_queue Queue to flush.
 */
void msq_reset(msq_t * p_queue);

/**
 * Gets the number of available elements at the given stage.
 *
 * @param[in] p_queue Queue to check.
 * @param[in] stage   Stage in the queue to check.
 *
 * @returns The number of elements available in the given stage.
 */
uint8_t msq_available(const msq_t * p_queue, uint8_t stage);

/** @} */

#endif /* MSQUEUE_H__ */
