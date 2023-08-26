

#ifndef _PANPRF_CONFIG_H_
#define _PANPRF_CONFIG_H_



/**
 ****************************************************************************************
 * @addtogroup PRF_CONFIG
 * @ingroup PROFILE
 * @brief Profile configuration
 *
 * @{
 ****************************************************************************************
 */

//ATT DB,Testing and Qualification related flags
#if (BLE_CENTRAL || BLE_PERIPHERAL)
#if (USER_PROJ_TEMPLATE || USER_PROJ_WECHAT || PROJ_THROUGHPUT || PROJ_MULTIROLE)
	#define BLE_PROJ_TEMPLATE_SERVER         1
#else
	#define BLE_PROJ_TEMPLATE_SERVER         0
#endif

#if (PROJ_HID_SELFIE || PROJ_HID_KEYBOARD)
#define CFG_PRF_DISS    
#define CFG_PRF_HOGPD
#define CFG_PRF_BASS
#define CFG_PRF_FMPT
#define BLE_DIS_SERVER              1
#define BLE_BATT_SERVER             1
#define BLE_HID_DEVICE              1
#else
#define BLE_DIS_SERVER              0
#define BLE_BATT_SERVER             0
#define BLE_HID_DEVICE              0
#endif
#if (USER_RGB_LIGHT)
#define CFG_PRF_RGB_SERVER1		
#define CFG_PRF_RGB_SERVER2		
#define BLE_RGB_SERVER1			1
#define BLE_RGB_SERVER2			1
#else
#define BLE_RGB_SERVER1			0
#define BLE_RGB_SERVER2			0
#endif
#if defined(USER_PROJ_CENTRAL)
	#define BLE_TMP_CLIENT			1
#else
	#define BLE_TMP_CLIENT			0
#endif 
#if (PROJ_OTA)
    #define BLE_OTA                 1
#else
    #define BLE_OTA                 0
#endif

#define CFG_NB_PRF                  BLE_PROJ_TEMPLATE_SERVER + BLE_DIS_SERVER + BLE_BATT_SERVER \
                                    + BLE_HID_DEVICE + BLE_RGB_SERVER1 + BLE_RGB_SERVER2 \
                                    + BLE_TMP_CLIENT + BLE_OTA


#if (1) 
#define BLE_CLIENT_PRF          1
#else
#define BLE_CLIENT_PRF          0
#endif //(BLE_PROX_MONITOR || BLE_FINDME_LOCATOR ...)


#if (1)        
#define BLE_SERVER_PRF          1
#else
#define BLE_SERVER_PRF          0
#endif //(BLE_PROX_REPORTER || BLE_FINDME_TARGET ...)


#elif (BLE_OBSERVER || BLE_BROADCASTER)

/// Proximity Profile Monitor Role
#define BLE_PROX_MONITOR            0
/// Proximity Profile Reporter Role
#define BLE_PROX_REPORTER           0
///Find Me Profile Locator role
#define BLE_FINDME_LOCATOR          0
///Find Me Profile Target role
#define BLE_FINDME_TARGET           0
///Health Thermometer Profile Collector Role
#define BLE_HT_COLLECTOR            0
///Health Thermometer Profile Thermometer Role
#define BLE_HT_THERMOM              0
///Blood Pressure Profile Collector Role
#define BLE_BP_COLLECTOR            0
///Blood Pressure Profile Sensor Role
#define BLE_BP_SENSOR               0
///Heart Rate Profile Collector Role
#define BLE_HR_COLLECTOR            0
///Heart Rate Profile Sensor Role
#define BLE_HR_SENSOR               0
///Time Profile Client Role
#define BLE_TIP_CLIENT              0
///Time Profile Server Role
#define BLE_TIP_SERVER              0
/// Device Information Service Client Role
#define BLE_DIS_CLIENT              0
/// Device Information Service Server Role
#define BLE_DIS_SERVER              0
/// Scan Parameter Profile Client Role
#define BLE_SP_CLIENT               0
/// Scan Parameter Profile Server Role
#define BLE_SP_SERVER               0
/// Battery Service Client Role
#define BLE_BATT_CLIENT             0
/// Battery Service Server Role
#define BLE_BATT_SERVER             0
/// HID Device Role
#define BLE_HID_DEVICE              0
/// HID Boot Host Role
#define BLE_HID_BOOT_HOST           0
/// HID Report Host Role
#define BLE_HID_REPORT_HOST         0
/// Glucose Profile Collector Role
#define BLE_GL_COLLECTOR            0
/// Glucose Profile Sensor Role
#define BLE_GL_SENSOR               0
/// Running Speed and Cadence Collector Role
#define BLE_RSC_COLLECTOR           0
/// Running Speed and Cadence Server Role
#define BLE_RSC_SENSOR              0
/// Cycling Speed and Cadence Collector Role
#define BLE_CSC_COLLECTOR           0
/// Cycling Speed and Cadence Server Role
#define BLE_CSC_SENSOR              0
/// Cycling Power Collector Role
#define BLE_CP_COLLECTOR            0
/// Cycling Power Server Role
#define BLE_CP_SENSOR               0
/// Location and Navigation Collector Role
#define BLE_LN_COLLECTOR            0
/// Location and Navigation Server Role
#define BLE_LN_SENSOR               0
/// Alert Notification Client Role
#define BLE_AN_CLIENT               0
/// Alert Notification Server Role
#define BLE_AN_SERVER               0
/// Phone Alert Status Client Role
#define BLE_PAS_CLIENT              0
/// Phone Alert Status Server Role
#define BLE_PAS_SERVER              0
/// Internet Protocol Support Profile Server Role
#define BLE_IPS_SERVER              0
/// Internet Protocol Support Profile Client Role
#define BLE_IPS_CLIENT              0
/// Environmental Sensing Profile Server Role
#define BLE_ENV_SERVER              0
/// Environmental Sensing Profile Client Role
#define BLE_ENV_CLIENT              0
/// Weight Scale Profile Server Role
#define BLE_WSC_SERVER              0
/// Weight Scale Profile Client Role
#define BLE_WSC_CLIENT              0
/// Body Composition Profile Client Role 
#define BLE_BCS_CLIENT          0
/// Body Composition Profile Server Role
#define BLE_BCS_SERVER          0
/// User Data Service Server Role
#define BLE_UDS_SERVER              0
/// User Data Service Client Role
#define BLE_UDS_CLIENT              0

//Force ATT parts to 0
/// External database management
#define BLE_EXT_ATTS_DB             0
/// Profile Server
#define BLE_SERVER_PRF              0
/// Profile Client
#define BLE_CLIENT_PRF              0

#define CFG_NB_PRF	(0)

#endif //(BLE_OBSERVER || BLE_BROADCASTER)


/// @} PRF_CONFIG

#endif /* _PANPRF_CONFIG_H_ */
