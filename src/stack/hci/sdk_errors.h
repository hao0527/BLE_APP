/**
 ****************************************************************************************
 * @file    sdk_errors.h
 * @brief   
 
 * @version 0.01
 * @date    2018/10/18
 * @history 
 * @note	   
 * detailed description 
 
 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */
#include "panble.h"

#ifndef __SDK_ERRORS_H__
#define __SDK_ERRORS_H__


typedef enum 
{
    SDK_SUCCESS                 = ( uint32 ) 0x00000000,
    SDK_ERROR_INVALID_PARAM     = ( uint32 ) 0x00000001,
    SDK_ERROR_INVALID_STATE     = ( uint32 ) 0x00000002,
    SDK_ERROR_NO_MEMORY         = ( uint32 ) 0x00000010,
} sdk_errors_t;



#endif /* __SDK_ERRORS_H__ */

