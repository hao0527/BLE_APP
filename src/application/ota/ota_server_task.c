/**
 ****************************************************************************************
 * @file    ota_server_task.c
 * @brief

 * @version 0.01
 * @date    2018/11/27
 * @history
 * @note
 * detailed description

 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup OTA_SERVER_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "panble_config.h"
#include "co_utils.h"
#include "prf_utils.h"
#include "ota_server_task.h"
#include "ota_server.h"
#include "stack_svc_api.h"

#include "gattc.h"
#include "app.h"
#include "section_cfg.h"

#if (PROJ_OTA)
#define DBG_OTA_SRV_TASK(x)         //printf x
#define DBG_OTA_SRV_TASK_FUNC()     //printf("%s\n", __func__)
#define DBG_OTA_SRC_TASK_MEM(mem32, len)  do { \
    int i = 0; for ( ; i < len/4; i++) { printf("0x%08X ", mem32[i]); } printf("\n");\
} while ( 0 )

typedef enum OTA_CTRL_HANDLE
{
    OTA_FW_VERSION = 1,
    OTA_START      = 2,
    OTA_VALIDATE   = 3,  
    OTA_RSP_OPCODE = 0x80,
} OTA_CTRL_HANDLE_ENUM;


typedef enum 
{
    OTA_UNINIT  = 0,
    OTA_RUNNING = 1,
    OTA_SUCCESS = 2,
} OTA_STATUS_ENUM;

typedef enum 
{
    OTA_APP  = 1,
    OTA_SOFTDEVICE = 2,
} OTA_TYPE;

typedef enum 
{
    SIZE_ERROR  = 1,
    VERSION_ERROR = 2,
} OTA_ERROR;

uint8_t  m_ota_status;

uint32_t m_ota_data_buffer[FLASH_PAGE_SIZE_DWORD * 2] = { 0 };
uint8_t *mp_ota_data    = (uint8_t *)m_ota_data_buffer;
uint16_t m_ota_data_idx = 0;
uint32_t m_ota_data_recv = 0;

uint32_t m_ota_flash_size	 = 0;
uint32_t m_ota_flash_address = 0;
uint32_t m_ota_flash_offset  = 0;

uint32_t m_section_info[BYTE_TO_DWORD(FLASH_ADDR_INFO_SIZE)] = { 0 };
section_info_t mp_info_new; //= (section_info_t *)&m_section_info[BYTE_TO_DWORD(FLASH_OFFSET_INFO_DFU)];
section_info_t mp_info_current; //= (section_info_t *)&m_section_info[BYTE_TO_DWORD(FLASH_OFFSET_INFO_APP)];

void app_flash_erase ( uint32_t page_address );
void app_flash_read ( uint32_t address, uint32_t *p_buffer, uint32_t size );
void app_flash_write ( uint32_t address, uint32_t *p_buffer, uint32_t size );
void app_flash_get_crc32 ( uint32_t address, uint32_t size, uint32_t *p_crc32 );
void system_reset ( void );

static bool is_ota_data_write_enable ( void )
{
    return ( ( mp_info_new.size > 0 ) && ( mp_info_new.size < m_ota_flash_size ));
}

/*
pbuff:输入要解密的数据，输出要解密的数据
size：解密数据word长度
key:解密秘钥
*/

#define DECODE_KEY 12345678
void ota_decode_data(uint32_t *pbuff, uint32_t size, uint32_t key)
{
	unsigned int DecodeKey = key;
	unsigned int data;
	int i = 0;

	if (pbuff && size > 0) 
	{
		for (i = 0; i < size; i++) 
		{
			data = pbuff[i] & 0xff;
			data <<= 8;
			data += ((pbuff[i] >> 8) & 0xff);
			data <<= 8;
			data += ((pbuff[i] >> 16) & 0xff);
			data <<= 8;
			data += ((pbuff[i] >> 24) & 0xff);

			data ^= DecodeKey;

			pbuff[i] = data & 0xff;
			pbuff[i] <<= 8;
			pbuff[i] += ((data >> 8) & 0xff);
			pbuff[i] <<= 8;
			pbuff[i] += ((data >> 16) & 0xff);
			pbuff[i] <<= 8;
			pbuff[i] += ((data >> 24) & 0xff);

			if (DecodeKey & 1) 
			{
				DecodeKey >>= 1;
				DecodeKey |= 0x80000000;
			}
			else 
			{
				DecodeKey >>= 1;
			}
		}
	}
}

void app_flash_erase ( uint32_t page_address ) 
{
//    DBG_OTA_SRV_TASK ( ( "flash_earse 0x%08X, %d\n", 
//        page_address, ( (page_address & (FLASH_PAGE_SIZE - 1)) == 0 ) ) ); 

    SYS_UnlockReg();
	FMC_Open();
	FMC_ENABLE_AP_UPDATE();

    FMC_Erase( page_address );
    
    FMC_Close();
	SYS_LockReg();
}

void app_flash_read ( uint32_t address, uint32_t *p_buffer, uint32_t size ) 
{
    DBG_OTA_SRV_TASK ( ( "flash_read 0x%08X, %d\n", address, size ) );
    
    uint32_t i;
    
    SYS_UnlockReg();
	FMC_Open();
    
    for ( i = 0; i < BYTE_TO_DWORD(size); i++ )
    {
        p_buffer[i] = FMC_Read(address + 4 * i);
        //p_buffer[i] = *((volatile uint32_t *)(address + 4 * i));
        //DBG_OTA_SRV_TASK ( ( "[0x%08X] 0x%08X\n", address + 4 * i, p_buffer[i] ) );
    }
    
    FMC_Close();
	SYS_LockReg();
    //DBG_OTA_SRC_TASK_MEM ( p_buffer, size );
}

void app_flash_write ( uint32_t address, uint32_t *p_buffer, uint32_t size ) 
{
//    DBG_OTA_SRV_TASK ( ( "flash_write 0x%08X, %d\n", address, size ) );
//    DBG_OTA_SRC_TASK_MEM ( p_buffer, size );
    
    uint32_t i;

    SYS_UnlockReg();
	FMC_Open();
	FMC_ENABLE_AP_UPDATE();
    
    for ( i = 0; i < BYTE_TO_DWORD(size); i++ )
    {
        FMC_Write(address + 4 * i, p_buffer[i]);
    }
    
    FMC_Close();
	SYS_LockReg();
}

void app_flash_get_crc32 ( uint32_t address, uint32_t size, uint32_t *p_crc32 ) 
{
    SYS_UnlockReg();
	FMC_Open();
	FMC_ENABLE_AP_UPDATE();
    
    FMC_GetCRC32Sum(address, size, p_crc32);
    
    FMC_Close();
	SYS_LockReg();
}


void system_reset ( void )
{
    NVIC_SystemReset();
}

void bootloader_start ( void )
{
//    uint32_t dfu_flag = 0xFFFFFFFF;
//    
//    app_flash_read ( FLASH_ADDR_INFO_BACKUP + FLASH_OFFSET_DFU_FLAG, &dfu_flag, FLASH_SIZE_DFU_FLAG );
	memset ( m_section_info, 0, FLASH_ADDR_INFO_SIZE );
    app_flash_read ( FLASH_ADDR_INFO_BACKUP, m_section_info, FLASH_ADDR_INFO_SIZE );
    
    //dfu_flag >>= 16;
    if ( ( SECTION_SOFTDEVICE_FLAG == m_section_info[3] ) || ( SECTION_APP_FLAG == m_section_info[3] ) )
    {
		
        // TODO:: set FMC Config, run bootloader
        SYS_UnlockReg();
        FMC_Open();
        
        uint32_t Config_Read[4];
        FMC_ReadConfig(Config_Read,4);
        // 修改启动地址方式为 LD (Load ROM) 启动
        Config_Read[0] = 0xFFFFFF3F;
        FMC_WriteConfig( Config_Read, 4 );
        
        // reset
        system_reset();
    }
}

void dbg_section_info (section_info_t p_info)
{
    DBG_OTA_SRV_TASK ( ( "Size    0x%08X (%d Bytes)\n", p_info.size, p_info.size ) );
    DBG_OTA_SRV_TASK ( ( "CRC     0x%08X\n", p_info.crc ) );
    DBG_OTA_SRV_TASK ( ( "SoftDevice 0x%04X\n", p_info.version.softdevice ) );
    DBG_OTA_SRV_TASK ( ( "Appliction 0x%04X\n", p_info.version.appliction ) );
	DBG_OTA_SRV_TASK ( ( "project	 0x%04X\n", p_info.version.appliction ) );
    DBG_OTA_SRV_TASK ( ( "Flag    0x%04X\n", p_info.flag ) );
	DBG_OTA_SRV_TASK ( ( "dfu addr   0x%08X\n", m_ota_flash_address ) );
	DBG_OTA_SRV_TASK ( ( "dfu size    0x%08X (%d Bytes)\n", m_ota_flash_size,m_ota_flash_size ) );
}

static void ota_param_init ( void )
{
    DBG_OTA_SRV_TASK_FUNC();
    
    m_ota_data_recv = 0;
	
	memset(&mp_info_new,0,sizeof(section_info_t));
	memset(&mp_info_current,0,sizeof(section_info_t));

    app_flash_read ( FLASH_ADDR_INFO, m_section_info, FLASH_ADDR_INFO_SIZE );
	
	mp_info_current.version.appliction = (uint16_t)m_section_info[10];
	mp_info_current.version.project	= (uint16_t)(m_section_info[10] >> 16);
	mp_info_current.version.softdevice = (uint16_t)m_section_info[6];
	mp_info_current.size 				= m_section_info[0];
	mp_info_current.crc 				= m_section_info[1];
	mp_info_current.flag 				= (uint16_t)m_section_info[3];
	
    //m_ota_flash_address = m_section_info[BYTE_TO_DWORD(FLASH_OFFSET_DFU_SOURCE)];
    
	DBG_OTA_SRV_TASK ( ( "\nCurrent Information\n" ) );
    dbg_section_info ( mp_info_current );
}

static void ota_info_update ( void )
{
    DBG_OTA_SRV_TASK_FUNC();
    
	memset ( m_section_info, 0, FLASH_ADDR_INFO_SIZE );
	m_section_info[0] = mp_info_new.size;
	m_section_info[1] = mp_info_new.crc;
	m_section_info[3] = mp_info_new.flag;
	m_section_info[4] = m_ota_flash_size;
	m_section_info[5] = m_ota_flash_address;
	m_section_info[6] = mp_info_new.version.softdevice;
	m_section_info[10] = (mp_info_new.version.appliction | (mp_info_new.version.project << 16));
	
    app_flash_erase ( FLASH_ADDR_INFO_BACKUP );
    app_flash_write ( FLASH_ADDR_INFO_BACKUP, m_section_info, FLASH_ADDR_INFO_SIZE );
    
    //app_flash_write ( FLASH_ADDR_INFO, m_section_info, FLASH_ADDR_INFO_SIZE );
    memset ( m_section_info, 0, FLASH_ADDR_INFO_SIZE );
    app_flash_read ( FLASH_ADDR_INFO_BACKUP, m_section_info, FLASH_ADDR_INFO_SIZE );
    
    DBG_OTA_SRV_TASK ( ( "\nOTA Information\n" ) );
    dbg_section_info ( mp_info_new );
}

static bool is_crc_match ( void )
{
    uint32_t crc = 0;

    app_flash_get_crc32 ( m_ota_flash_address, mp_info_new.size, &crc );
    
    DBG_OTA_SRV_TASK ( ( "flash crc [0x%08X]-[0x%08X] %d\n", m_ota_flash_address, m_ota_flash_address + mp_info_new.size, mp_info_new.size ) );
    DBG_OTA_SRV_TASK ( ( "0x%08X\n", crc ) );
    
    if ( mp_info_new.crc == crc )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


static uint32_t ota_write_data ( const uint8_t *p_data, uint16_t length )
{
    memcpy ( mp_ota_data + m_ota_data_idx, p_data, length );
    m_ota_data_idx += length;
    m_ota_data_recv += length;
    
    if ( !is_ota_data_write_enable() )
    {
        return 1;
    }
    
    if ( m_ota_data_idx >= FLASH_PAGE_SIZE )
    {
		ota_decode_data(m_ota_data_buffer,FLASH_PAGE_SIZE / 4,DECODE_KEY);
		
        app_flash_erase ( m_ota_flash_address + m_ota_flash_offset );
        app_flash_write ( m_ota_flash_address + m_ota_flash_offset, m_ota_data_buffer, FLASH_PAGE_SIZE );
        
        m_ota_flash_offset += FLASH_PAGE_SIZE;

        m_ota_data_idx -= FLASH_PAGE_SIZE;
        memcpy ( mp_ota_data, mp_ota_data + FLASH_PAGE_SIZE, FLASH_PAGE_SIZE );
        
//        DBG_OTA_SRV_TASK ( ( "data_buffer %d\n", m_ota_data_idx ) );
//        DBG_OTA_SRV_TASK ( ( "data_recv  %d\n", m_ota_data_recv ) );
//        DBG_OTA_SRV_TASK ( ( "data_write %d\n", m_ota_flash_offset ) );
    }
    
    if ( m_ota_data_recv >= mp_info_new.size )
    {
        uint16_t len = m_ota_data_recv - m_ota_flash_offset;
        
//        DBG_OTA_SRV_TASK ( ( "data_recv  %d\n", m_ota_data_recv ) );
//        DBG_OTA_SRV_TASK ( ( "data_write %d\n", m_ota_flash_offset ) );
        
        if ( m_ota_data_recv > mp_info_new.size )
        {
            return 2;   // Failed SIZE
        }
        
        if ( len > 0 )
        {
			ota_decode_data(m_ota_data_buffer,len / 4,DECODE_KEY);
			
            app_flash_erase ( m_ota_flash_address + m_ota_flash_offset );
            app_flash_write ( m_ota_flash_address + m_ota_flash_offset, m_ota_data_buffer, len );
        }
        
        return 0; // Success
    }

    return 3; // Continue
}

__STATIC int ota_server_enable_req_handler ( ke_msg_id_t const msgid,
                                             struct ota_server_enable_req const* param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id )
{
    DBG_OTA_SRV_TASK_FUNC();
    
    int     msg_status = KE_MSG_CONSUMED;
    uint8_t state      = ((ke_state_get_handler)SVC_ke_state_get) ( dest_id );
    int     status     = 0;

    // check state of the task
    if ( state == OTA_SVS_IDLE )
    {
        struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );

        // Check provided values
        if ( ( param->conidx > BLE_CONNECTION_MAX )
                || ( ((gapc_get_conhdl_handler)SVC_gapc_get_conhdl) ( param->conidx ) == GAP_INVALID_CONHDL ) )
        {
            status = ( param->conidx > BLE_CONNECTION_MAX ) ? GAP_ERR_INVALID_PARAM : PRF_ERR_REQ_DISALLOWED;
        }
        else
        {
            status = GAP_ERR_NO_ERROR;
        }

        struct ota_server_enable_rsp* rsp = KE_MSG_ALLOC ( OTA_SERVER_ENABLE_RSP, src_id,
                                                           dest_id, ota_server_enable_rsp );
        rsp->conidx = param->conidx;
        rsp->status = status;

        ((ke_msg_send_handler)SVC_ke_msg_send) ( rsp );
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_req_ind_handler ( ke_msg_id_t const msgid,
                                              struct gattc_att_info_req_ind* param,
                                              ke_task_id_t const dest_id,
                                              ke_task_id_t const src_id )
{
    DBG_OTA_SRV_TASK_FUNC();
    
    struct gattc_att_info_cfm* cfm;

    uint8_t svc_idx = 0;
    uint8_t att_idx = 0;
    
    // retrieve handle information
    uint8_t status  = ota_server_get_att_idx ( param->handle, &svc_idx, &att_idx );
    
    //Send write response
    cfm = KE_MSG_ALLOC ( GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm );
    cfm->handle = param->handle;
    
    //ASSERT_ERR(0);

    if ( status == GAP_ERR_NO_ERROR )
    {
        cfm->length = 2;
    }

    cfm->status = status;
    ((ke_msg_send_handler)SVC_ke_msg_send) ( cfm );
    
    return ( KE_MSG_CONSUMED );
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
//peer write data
__STATIC int gattc_write_req_ind_handler ( ke_msg_id_t const msgid,
                                           struct gattc_write_req_ind const* param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id )
{
    struct gattc_write_cfm* cfm;

    uint8_t svc_idx = 0;
    uint8_t att_idx = 0;
    uint8_t conidx  = KE_IDX_GET ( src_id );

    // retrieve handle information
    uint8_t status = ota_server_get_att_idx ( param->handle, &svc_idx, &att_idx );
    
    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if ( status == GAP_ERR_NO_ERROR )
    {
        struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );
		
        switch ( att_idx )
        {
            case OTA_IDX_CTRL_CLIENT_CFG:
            {
                p_env->client_cfg_ctrl[conidx] = co_read16p ( &param->value[0] );
            }
            break;
            
            case OTA_IDX_DATA_VAL:
            {
//                DBG_OTA_SRV_TASK ( ( "ota_write_data len %d\n", param->length ) );
				
                ota_write_data ( &param->value[param->offset + 2], (param->length - 2) );
				
				
				struct ota_server_send_to_peer* req = KE_MSG_ALLOC_DYN ( OTA_SERVER_SEND_TO_PEER,
                                                                             p_env->prf_env.prf_task,
                                                                             p_env->prf_env.prf_task,
                                                                             ota_server_send_to_peer,
                                                                             2 );
                
                
				req->conidx = conidx;
				req->att_idx = att_idx;
				req->write_type  = NOTIFICATION;
				req->length  = 2;
				req->value[0] = OTA_RSP_OPCODE;
				req->value[1] = 0x04;
				
				((ke_msg_send_handler)SVC_ke_msg_send) ( req );
//				ota_write_data ( &param->value[param->offset], param->length );
            }
            break;
            
            case OTA_IDX_CTRL_VAL:
            {
                uint8_t opcode = param->value[0];
                
                if ( OTA_FW_VERSION == opcode )
                {
                    ota_param_init();
                    
					app_var.default_sleep_en = 0;
                    mp_info_new.size    = 0;
                    mp_info_new.crc     = 0;
                    mp_info_new.version.appliction = co_read16p ( &param->value[1] );
                    mp_info_new.version.softdevice = co_read16p ( &param->value[3] );
					mp_info_new.version.project 	= co_read16p ( &param->value[5] );
					mp_info_new.flag    = 0;
                    
                    DBG_OTA_SRV_TASK ( ( "ota version\n" ) );
                    DBG_OTA_SRV_TASK ( ( "softdevice 0x%04X\n", mp_info_new.version.softdevice ) );
                    DBG_OTA_SRV_TASK ( ( "appliction 0x%04X\n", mp_info_new.version.appliction) );
					DBG_OTA_SRV_TASK ( ( "project	 0x%04X\n", mp_info_new.version.project) );
                    
                    struct ota_server_send_to_peer* req = KE_MSG_ALLOC_DYN ( OTA_SERVER_SEND_TO_PEER,
                                                                             p_env->prf_env.prf_task,
                                                                             p_env->prf_env.prf_task,
                                                                             ota_server_send_to_peer,
                                                                             8 );
                
                    
                    req->conidx = conidx;
                    req->att_idx = att_idx;
                    req->write_type  = NOTIFICATION;
                    req->length  = 8;
                    req->value[0] = OTA_RSP_OPCODE;
                    req->value[1] = OTA_FW_VERSION;
                    req->value[2] = mp_info_current.version.appliction;
                    req->value[3] = mp_info_current.version.appliction >> 8;
                    req->value[4] = mp_info_current.version.softdevice;
                    req->value[5] = mp_info_current.version.softdevice >> 8;
					req->value[6] = mp_info_current.version.project;
                    req->value[7] = mp_info_current.version.project >> 8;
                    
                    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );
					
					struct gapc_conn_param conn_param;		//update conn param
					conn_param.intv_min = 6;		
					conn_param.intv_max = 6;
					conn_param.latency	= 0;
					conn_param.time_out = 100;
					appm_update_param(&conn_param);
                }
                else if ( OTA_START == opcode )
                {
                    m_ota_data_idx  = 0;
                    m_ota_data_recv = 0;
                    m_ota_flash_offset = 0;
                    
					#if 1
					if(param->value[1] == OTA_APP)
					{
						 mp_info_new.flag    = SECTION_APP_FLAG;
					}
					else if(param->value[1] == OTA_SOFTDEVICE)
					{
						 mp_info_new.flag    = SECTION_SOFTDEVICE_FLAG;
					}
                    mp_info_new.size = co_read32p ( &param->value[2] );

                    if(mp_info_new.flag == SECTION_SOFTDEVICE_FLAG)		//stack，dfu addr 0x00023000,dfu size 0x19000
					{																					
						m_ota_flash_address = 0x00023000;
						m_ota_flash_size 	= 0x19000;
					}
					else if(mp_info_new.flag == SECTION_APP_FLAG)		//app，dfu addr 0x00029400,dfu size 0x12C00
					{
						m_ota_flash_address = 0x00029400;
						m_ota_flash_size 	= 0x12C00;
					}
                    DBG_OTA_SRV_TASK ( ( "dfu addr %08x,image size %d Bytes\n",m_ota_flash_address, mp_info_new.size ) );
                    #else
                    mp_info_new.size = co_read32p ( &param->value[1] );
					mp_info_new.flag = SECTION_SOFTDEVICE_FLAG;
					DBG_OTA_SRV_TASK ( ( "ota start %d\n", mp_info_new.flag ));
                    DBG_OTA_SRV_TASK ( ( "image size %d Bytes\n", mp_info_new.size ) );
					#endif
					
                    struct ota_server_send_to_peer* req = KE_MSG_ALLOC_DYN ( OTA_SERVER_SEND_TO_PEER,
                                                                             p_env->prf_env.prf_task,
                                                                             p_env->prf_env.prf_task,
                                                                             ota_server_send_to_peer,
                                                                             3 );
                
                
                    req->conidx = conidx;
                    req->att_idx = att_idx;
                    req->write_type  = NOTIFICATION;
                    req->length  = 3;
                    req->value[0] = OTA_RSP_OPCODE;
                    req->value[1] = OTA_START;
                    req->value[2] = 0x00;
                    
                    if ( mp_info_new.size > m_ota_flash_size )
                    {
                        req->value[2] = SIZE_ERROR;   // failed
                    }
                    else
                    {
                        m_ota_status = OTA_RUNNING;
                    }
                    
                    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );
                }
                else if ( OTA_VALIDATE == opcode )
                {
                    struct ota_server_send_to_peer* req = KE_MSG_ALLOC_DYN ( OTA_SERVER_SEND_TO_PEER,
                                                                             p_env->prf_env.prf_task,
                                                                             p_env->prf_env.prf_task,
                                                                             ota_server_send_to_peer,
                                                                             3 );
                
                
                    req->conidx = conidx;
                    req->att_idx = att_idx;
                    req->write_type  = NOTIFICATION;
                    req->length  = 3;
                    req->value[0] = OTA_RSP_OPCODE;
                    req->value[1] = OTA_VALIDATE;
                    req->value[2] = 0x00;
                    
                    mp_info_new.crc = co_read32p ( &param->value[1] );
                    DBG_OTA_SRV_TASK ( ( "ota validate\n" ) );
                    DBG_OTA_SRV_TASK ( ( "crc code %08X\n", mp_info_new.crc ) );
                    
                    if ( is_crc_match() )
                    {
                        ota_info_update();
                        
                        m_ota_status = OTA_SUCCESS;
                    }
                    else
                    {
                        req->value[2] = 0x01;			//crc err
                    }
                    
                    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );
                }
            }
            break;
            
            default:
            {
                status = ATT_ERR_APP_ERROR;
            }
            break;
        }
    }

    //Send write response
    cfm = KE_MSG_ALLOC ( GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm );
    cfm->handle = param->handle;
    cfm->status = status;
    ((ke_msg_send_handler)SVC_ke_msg_send) ( cfm );

    return ( KE_MSG_CONSUMED );
}

void app_ota_check_status ( void )
{
	DBG_OTA_SRV_TASK ( ( "ota status %d\n", m_ota_status) );
    if ( OTA_SUCCESS == m_ota_status )
    {
        m_ota_status = OTA_UNINIT;
        
        system_reset();
    }
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler ( ke_msg_id_t const msgid, struct gattc_read_req_ind const* param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id )
{
    DBG_OTA_SRV_TASK_FUNC();
    
    struct gattc_read_cfm* cfm;

    uint8_t svc_idx = 0;
    uint8_t att_idx = 0;
    uint8_t conidx  = KE_IDX_GET ( src_id );
    uint16_t length = 0;

    // retrieve handle information
    uint8_t status = ota_server_get_att_idx ( param->handle, &svc_idx, &att_idx );

    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if ( status == GAP_ERR_NO_ERROR )
    {
        switch ( att_idx )
        {
            case OTA_IDX_CTRL_CLIENT_CFG:
            {
                length = OTA_CTRL_CLIENT_CFG_LEN;
            }
            break;

            default:
            {
                status = PRF_APP_ERROR;
            }
        }
    }

    //Send read response
    cfm = KE_MSG_ALLOC_DYN ( GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, length );
    cfm->handle = param->handle;
    cfm->status = status;
    cfm->length = length;

    // If the attribute has been found, status is GAP_ERR_NO_ERROR
    if ( status == GAP_ERR_NO_ERROR )
    {
        switch ( att_idx )
        {
            case OTA_IDX_CTRL_CLIENT_CFG:
            {
                memcpy ( cfm->value, &p_env->client_cfg_ctrl[conidx], OTA_CTRL_CLIENT_CFG_LEN );
            }
            break;

            default:
            {
                cfm->status = ATT_ERR_APP_ERROR;
            }
            break;
        }
    }

    ((ke_msg_send_handler)SVC_ke_msg_send) ( cfm );

    DBG_OTA_SRV_TASK ( ( "srv_ota read_req 0x%04X, (%d) \n", p_env->client_cfg_ctrl[conidx], cfm->status ) );
    
    return ( KE_MSG_CONSUMED );
}

__STATIC int ota_server_send_to_peer_handler ( ke_msg_id_t const                     msgid,
                                               struct ota_server_send_to_peer const* param,
                                               ke_task_id_t const                    dest_id,
                                               ke_task_id_t const                    src_id )

{
    uint8_t conidx    = KE_IDX_GET ( src_id );
    uint8_t operation = GATTC_NO_OP;
    
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );
    
    if ( (  NOTIFICATION == param->write_type ) && ( PRF_CLI_START_NTF & p_env->client_cfg_ctrl[conidx] ) )
    {
        operation = GATTC_NOTIFY;
    }
    else if ( ( param->write_type == INDICATION ) && ( PRF_CLI_START_IND & p_env->client_cfg_ctrl[conidx] ) )
    {
        operation = GATTC_INDICATE;
    }
    else 
    {
        return KE_MSG_CONSUMED;
    }
    
    struct gattc_send_evt_cmd* req = KE_MSG_ALLOC_DYN ( GATTC_SEND_EVT_CMD,
                                                        KE_BUILD_ID ( TASK_GATTC, param->conidx ),
                                                        p_env->prf_env.prf_task,
                                                        gattc_send_evt_cmd,
                                                        param->length );
    
    // Fill in the parameter structure
    req->operation = operation;
    req->seq_num   = 0x3;
    req->handle    = ota_server_get_att_handle ( 0, OTA_IDX_CTRL_VAL );
    req->length    = param->length;

    memcpy ( req->value, param->value, req->length );

    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );

    return ( KE_MSG_CONSUMED );
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY message meaning that Measurement
 * notification has been correctly sent to peer device (but not confirmed by peer device).
 * *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler ( ke_msg_id_t const msgid,  struct gattc_cmp_evt const* param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id )
{
    if ( param->operation == GATTC_NOTIFY )
    {

    }
    return ( KE_MSG_CONSUMED );
}

/// Default State handlers definition
static const struct ke_msg_handler ota_server_msg_handler_tab[] =
{
    {OTA_SERVER_ENABLE_REQ,                 ( ke_msg_func_t ) ota_server_enable_req_handler},
    {GATTC_ATT_INFO_REQ_IND,                ( ke_msg_func_t ) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND,                   ( ke_msg_func_t ) gattc_write_req_ind_handler},
    {GATTC_READ_REQ_IND,                    ( ke_msg_func_t ) gattc_read_req_ind_handler},
    {OTA_SERVER_SEND_TO_PEER,               ( ke_msg_func_t ) ota_server_send_to_peer_handler},
    {GATTC_CMP_EVT,                         ( ke_msg_func_t ) gattc_cmp_evt_handler},
};

const struct ke_state_handler task_ota_server_msg_default_handler = KE_STATE_HANDLER ( ota_server_msg_handler_tab );


void ota_server_task_init ( struct ke_task_desc* task_desc )
{
    // Get the address of the environment
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );

    task_desc->state_handler   = NULL;
    task_desc->default_handler = &task_ota_server_msg_default_handler;
    task_desc->msg_cnt         = ARRAY_LEN ( ota_server_msg_handler_tab );
    task_desc->state           = p_env->state;
    task_desc->idx_max         = OTA_SERVER_IDX_MAX;
    
    ota_param_init();
}
#endif
