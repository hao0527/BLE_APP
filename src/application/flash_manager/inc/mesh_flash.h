#ifndef MESH_FLASH_H__
#define MESH_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

/** Flash operation types. */
typedef enum
{
    FLASH_OP_TYPE_NONE  = 0x00, /**< No operation type. */
    FLASH_OP_TYPE_WRITE = 0x01, /**< Flash write operation. */
    FLASH_OP_TYPE_ERASE = 0x02, /**< Flash erase operation. */
    FLASH_OP_TYPE_ALL   = 0x03  /**< All flash operations. */
} flash_op_type_t;

/**
 * @defgroup MESH_FLASH Mesh flash handler
 * @ingroup MESH_CORE
 * @{
 */

/** Index of mesh flash handler users, used to index their context internally
 * to the module */
typedef enum
{
    MESH_FLASH_USER_MESH, /**< User reserved to storing mesh state. */
    MESH_FLASH_USER_APP,  /**< User available for application specific purposes. */
#ifdef UNIT_TEST
    MESH_FLASH_USER_TEST, /**< User reserved for unit testing. */
#endif

    MESH_FLASH_USERS      /**< Number of flash users, does not represent a valid user index. */
} mesh_flash_user_t;

/**
 * @defgroup FLASH_OPERATION_PARAMS Flash operation parameter structures
 * @{
 */

/** Parameters for write flash operation */
typedef struct
{
    uint32_t * p_start_addr; /**< The start address to write to. */
    uint32_t length; /**< Number of bytes to write. */
    uint32_t * p_data; /**< The data to write. */
} flash_operation_params_write_t;

/** Parameters for erase flash operation */
typedef struct
{
    uint32_t * p_start_addr; /**< The start address to erase. */
    uint32_t length; /**< Number of bytes to erase. */
} flash_operation_params_erase_t;

/** @} */

/** Single flash operation. */
typedef struct
{
    flash_op_type_t type;     /**< Type of flash operation. */
    /** Parameters for the flash operation. */
    union
    {
        flash_operation_params_write_t write; /**< Parameters for write operations. */
        flash_operation_params_erase_t erase; /**< Parameters for erase operations. */
    } params;
} flash_operation_t;

/**
 * Flash operation callback type. Used to notify user of an ended flash
 * operation.
 *
 * @param[in] user User that completed the operation.
 * @param[in] p_op Completed operation, or a special operation with type @ref FLASH_OP_TYPE_ALL
 * and invalid @c params if all operations have been completed.
 * @param[in] token Token returned in the op_push function's @p p_token parameter.
 */
typedef void (*mesh_flash_op_cb_t)(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token);

/**
 * Initialize the mesh flash module.
 */
void mesh_flash_init(void);

/**
 * Set the end-callback of the given user.
 *
 * @note The callback is NULL by default, which means that the user won't be
 * notified when the operation ended.
 * @note Will assert if the user doesn't exist.
 */
void mesh_flash_user_callback_set(mesh_flash_user_t user, mesh_flash_op_cb_t cb);

/**
 * Push a single flash operation to the flash queue.
 *
 * @param[in] user User pushing the operation.
 * @param[in] p_op The flash operation to push.
 * @param[out] p_token An optional token that will be passed back in the end callback, or NULL if
 * the token will not be used.
 *
 * @retval PAN_SUCCESS The flash operation was successfully enqueued.
 * @retval PAN_ERROR_NOT_FOUND Couldn't find the given user.
 * @retval PAN_ERROR_NULL The @c p_op parameter was NULL.
 * @retval PAN_ERROR_INVALID_ADDR The flash operation's addresses weren't properly aligned.
 * @retval PAN_ERROR_INVALID_LENGTH The flash operation's length parameter wasn't word-aligned.
 * @retval PAN_ERROR_INVALID_PARAM The flash operation type isn't valid.
 * @retval PAN_ERROR_NO_MEM There was no more space in the given user's flash
 * queue, and the operation will not be performed.
 */
uint32_t mesh_flash_op_push(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t * p_token);

/**
 * Get the number of flash operation slots available.
 *
 * @param[in] user User to check for.
 *
 * @returns The number of slots available in the flash write queue.
 */
uint32_t mesh_flash_op_available_slots(mesh_flash_user_t user);

/**
 * Check whether a flash operation is currently in progress.
 *
 * @returns Whether a flash operation is currently being executed.
 */
bool mesh_flash_in_progress(void);

/**
 * Suspend or unsuspend flash operations.
 *
 * @note Safe for multiple users.
 *
 * @param[in] suspend Whether to suspend the flash operations.
 */
void mesh_flash_set_suspended(bool suspend);

/** @} */

#endif /* MESH_FLASH_H__ */
