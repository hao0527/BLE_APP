/**
  @defgroup pan_error SoftDevice Global Error Codes
  @{

  @brief Global Error definitions
*/

/* Header guard */
#ifndef MESH_ERROR_H__
#define MESH_ERROR_H__

#ifdef __cplusplus
extern "C" {
#endif
    

/** @defgroup PAN_ERRORS_BASE Error Codes Base number definitions
 * @{ */
#define PAN_ERROR_BASE_NUM      (0x0)       ///< Global error base
#define PAN_ERROR_SDM_BASE_NUM  (0x1000)    ///< SDM error base
#define PAN_ERROR_SOC_BASE_NUM  (0x2000)    ///< SoC error base
#define PAN_ERROR_STK_BASE_NUM  (0x3000)    ///< STK error base
/** @} */

#define PAN_SUCCESS                           (PAN_ERROR_BASE_NUM + 0)  ///< Successful command
#define PAN_ERROR_SVC_HANDLER_MISSING         (PAN_ERROR_BASE_NUM + 1)  ///< SVC handler is missing
#define PAN_ERROR_SOFTDEVICE_NOT_ENABLED      (PAN_ERROR_BASE_NUM + 2)  ///< SoftDevice has not been enabled
#define PAN_ERROR_INTERNAL                    (PAN_ERROR_BASE_NUM + 3)  ///< Internal Error
#define PAN_ERROR_NO_MEM                      (PAN_ERROR_BASE_NUM + 4)  ///< No Memory for operation
#define PAN_ERROR_NOT_FOUND                   (PAN_ERROR_BASE_NUM + 5)  ///< Not found
#define PAN_ERROR_NOT_SUPPORTED               (PAN_ERROR_BASE_NUM + 6)  ///< Not supported
#define PAN_ERROR_INVALID_PARAM               (PAN_ERROR_BASE_NUM + 7)  ///< Invalid Parameter
#define PAN_ERROR_INVALID_STATE               (PAN_ERROR_BASE_NUM + 8)  ///< Invalid state, operation disallowed in this state
#define PAN_ERROR_INVALID_LENGTH              (PAN_ERROR_BASE_NUM + 9)  ///< Invalid Length
#define PAN_ERROR_INVALID_FLAGS               (PAN_ERROR_BASE_NUM + 10) ///< Invalid Flags
#define PAN_ERROR_INVALID_DATA                (PAN_ERROR_BASE_NUM + 11) ///< Invalid Data
#define PAN_ERROR_DATA_SIZE                   (PAN_ERROR_BASE_NUM + 12) ///< Invalid Data size
#define PAN_ERROR_TIMEOUT                     (PAN_ERROR_BASE_NUM + 13) ///< Operation timed out
#define PAN_ERROR_NULL                        (PAN_ERROR_BASE_NUM + 14) ///< Null Pointer
#define PAN_ERROR_FORBIDDEN                   (PAN_ERROR_BASE_NUM + 15) ///< Forbidden Operation
#define PAN_ERROR_INVALID_ADDR                (PAN_ERROR_BASE_NUM + 16) ///< Bad Memory Address
#define PAN_ERROR_BUSY                        (PAN_ERROR_BASE_NUM + 17) ///< Busy
#define PAN_ERROR_CONN_COUNT                  (PAN_ERROR_BASE_NUM + 18) ///< Maximum connection count exceeded.
#define PAN_ERROR_RESOURCES                   (PAN_ERROR_BASE_NUM + 19) ///< Not enough resources for operation

#ifdef __cplusplus
}
#endif
#endif // PAN_ERROR_H__

/**
  @}
*/

