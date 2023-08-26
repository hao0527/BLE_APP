

#ifndef _PANAPP_CONFIG_H_
#define _PANAPP_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup app
 * @brief Application configuration definition
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */


#if (USER_SKYWORTH || USER_HBTV)
#define CFG_APP_DIS
#define CFG_APP_HID
#define CFG_APP_BATT
#define CFG_APP_FINDME
#endif

#if (USER_TESTER)
#define CFG_APP_TMP
#endif

#if (USER_HQTANK)
#define CFG_APP_HQTANK
#endif

#if (USER_RGB_LIGHT)
#define CFG_APP_RGB1
#define CFG_APP_RGB2
#endif


#if (USER_GUN_PLAY)
#define CFG_APP_DIS
#define CFG_APP_BATT
#define CFG_APP_GUN_PLAY

#if (USER_PROJ_TEMPLATE || USER_PROJ_WECHAT || PROJ_THROUGHPUT || PROJ_MULTIROLE)
#define CFG_PRF_PROJ_TEMPLATE 
#endif
#endif
/******************************************************************************************/
/* -------------------------   BLE APPLICATION SETTINGS      -----------------------------*/
/******************************************************************************************/

#if (USER_PROJ_TEMPLATE || USER_PROJ_WECHAT || PROJ_THROUGHPUT || PROJ_MULTIROLE)
#define BLE_APP_PROJ_TEMPLATE    1
#else
#define BLE_APP_PROJ_TEMPLATE     0
#endif

/// @} panapp_config

#endif /* _PANAPP_CONFIG_H_ */
