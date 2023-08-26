#include "msqueue.h"
#include "utils.h"
#include "mesh_error.h"
/******************************************************************************
* Static functions
******************************************************************************/
 inline void * get_stage(const msq_t * p_queue, uint8_t stage)
{
    /* The index of the stage is the stage value modulo the number of elements in the array.
     * This way, the stage can be a running counter, and we only truncate it when using it for
     * array access. With this mechanism, having the first and last stage point to the same
     * value isn't ambigiuos, as they'll still be different numbers. */
    uint32_t index = (p_queue->p_stages[stage] & (p_queue->elem_count - 1));

    return (void *) ((uint8_t *) p_queue->p_elem_array + (index * p_queue->elem_size));
}

 inline uint8_t stage_get_available(const msq_t * p_queue, uint8_t stage)
{
    if (stage == 0)
    {
        /* The first stage will be out of space when it's `elem_count` indexes ahead of the last */
        return ((p_queue->p_stages[p_queue->stage_count - 1] + p_queue->elem_count) - p_queue->p_stages[0]);
    }
    else
    {
        return (p_queue->p_stages[stage - 1] - p_queue->p_stages[stage]);
    }
}
/******************************************************************************
* Interface functions
******************************************************************************/
void msq_init(msq_t * p_queue)
{
    MESH_ASSERT(p_queue->elem_size != 0);
    MESH_ASSERT(p_queue->stage_count > 1);
    MESH_ASSERT(p_queue->p_elem_array != NULL);
    MESH_ASSERT(p_queue->p_stages != NULL);
    //MESH_ASSERT(is_power_of_two(p_queue->elem_count));
    msq_reset(p_queue);
}

void * msq_get(const msq_t * p_queue, uint8_t stage)
{
    MESH_ASSERT(p_queue != NULL);
    MESH_ASSERT(stage < p_queue->stage_count);
    if (stage_get_available(p_queue, stage) != 0)
    {
        return get_stage(p_queue, stage);
    }
    else
    {
        return NULL;
    }
}

void msq_move(msq_t * p_queue, uint8_t stage)
{
    MESH_ASSERT(p_queue != NULL);
    MESH_ASSERT(stage < p_queue->stage_count);
    if (stage_get_available(p_queue, stage) != 0)
    {
        p_queue->p_stages[stage]++;
    }
}

void msq_reset(msq_t * p_queue)
{
    MESH_ASSERT(p_queue != NULL);
    for (uint32_t i = 0; i < p_queue->stage_count; i++)
    {
        p_queue->p_stages[i] = 0;
    }
}

uint8_t msq_available(const msq_t * p_queue, uint8_t stage)
{
    MESH_ASSERT(p_queue != NULL);
    MESH_ASSERT(stage < p_queue->stage_count);
    return stage_get_available(p_queue, stage);
}
