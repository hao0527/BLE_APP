#ifndef __PROJ_PROJ_TEMPLATE_H__
#define __PROJ_PROJ_TEMPLATE_H__

#include "gapc_task.h"
#include "gap.h"

#define APP_WHITE_LIST       0

typedef struct sotre_info
{
    //stored ltk
	struct gapc_ltk     stored_ltk;
    //stored peer addr
	struct gap_bdaddr   stored_peer_addr;
    //stored keyboard ntf
	uint16_t            ntf_cfg[3];
    //stroed bonded
	bool                bonded;
} sotre_info_t;

struct proj_template_env_tag
{	
	sotre_info_t info;
};

enum proj_template_sotred_type
{
	PROJ_TEMPLATE_STORE_TYPE_LTK,
	PROJ_TEMPLATE_STORE_TYPE_NTF,
	PROJ_TEMPLATE_STORE_TYPE_BONDED,
	PROJ_TEMPLATE_STORE_TYPE_PEER_BD_ADDRESS,
};

void proj_template_ini(void);
void proj_template_start_advertising(void);
void proj_template_start_advertising_direct_host(void);
void proj_template_clr_store_info(void);
void proj_template_get_flash_cmd(void);
void proj_template_store_flash_data(void);
void proj_template_store_info(void *buf, uint8_t store_type);
void *proj_template_get_store_info(uint8_t store_type);
#endif


