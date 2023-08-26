#include "stdint.h"
#include <stdbool.h>         // standard boolean

#include "panip_config.h"     // stack configuration
#include "compiler.h"        // compiler defines, INLINE
#include "ke_msg.h"          // kernel message defines
#include "lld_evt.h"
#include "attm.h"
#include "gapm_task.h"
#include "panip.h"
#include "hci_uart_task.h"
#include "peripherals.h"

#define STACK_FUN_ADDR					0x00016600

//app fun register
#define SVC_dbg_sys_write_register (*(volatile uint32_t *)(STACK_FUN_ADDR))
typedef void (*dbg_sys_write_register_handler)(void (*call)(uint16_t address, uint16_t data));

#define SVC_appm_init_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 4))
typedef void (*appm_init_register_handler)(void (*call)(void));

#define SVC_hci_init_register  (*(volatile uint32_t *)(STACK_FUN_ADDR + 8))
typedef void (*hci_init_register_handler)(void (*call)(bool reset));

#define SVC_hci_send_2_controller_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 12))
typedef void (*hci_send_2_controller_register_handler)(void (*call)(void *param));

#define SVC_attm_svc_create_db_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 16))
typedef void (*attm_svc_create_db_register_handler)(uint8_t (*call)(uint16_t *shdl, uint16_t uuid, uint8_t *cfg_flag, uint8_t max_nb_att,
                           uint8_t *att_tbl, ke_task_id_t const dest_id,const struct attm_desc *att_db, uint8_t svc_perm));

#define SVC_gattc_con_enable_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 20))
typedef void (*gattc_con_enable_register_handler)(void (*call)(uint8_t conidx));

#define SVC_gattm_cleanup_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 24))
typedef void (*gattm_cleanup_register_handler)(void (*call)(uint8_t conidx));

#define SVC_gattm_create_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 28))
typedef void (*gattm_create_register_handler)(void (*call)(uint8_t conidx));

#define SVC_prf_cleanup_register  (*(volatile uint32_t *)(STACK_FUN_ADDR + 32))
typedef void (*prf_cleanup_register_handler)(void (*call)(uint8_t conidx, uint8_t reason));

#define SVC_prf_create_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 36))
typedef void (*prf_create_register_handler)(void (*call)(uint8_t conidx));

#define SVC_prf_get_id_from_task_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 40))
typedef void (*prf_get_id_from_task_register_handler)(ke_task_id_t (*call)(ke_msg_id_t task));

#define SVC_prf_get_task_from_id_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 44))
typedef void (*prf_get_task_from_id_register_handler)(ke_task_id_t (*call)(ke_msg_id_t id));

#define SVC_prf_init_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 48))
typedef void (*prf_init_register_handler)(void (*call)(bool reset));

#define SVC_attm_att_update_perm_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 52))
typedef void (*attm_att_update_perm_register_handler)(uint8_t (*call)(uint16_t handle, uint16_t access_mask, uint16_t perm));

#define SVC_gattm_init_attr_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 56))
typedef void (*gattm_init_attr_register_handler)(uint8_t (*call)(uint16_t start_hdl, bool svc_chg_en));

#define SVC_hci_basic_cmd_send_2_controller_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 60))
typedef void (*hci_basic_cmd_send_2_controller_register_handler)(void (*call)(uint16_t opcode));

#define SVC_prf_add_profile_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 64))
typedef void (*prf_add_profile_register_handler)(uint8_t (*call)(struct gapm_profile_task_add_cmd * params, ke_task_id_t *prf_task));

#define SVC_gattc_get_mtu_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 68))
typedef void (*gattc_get_mtu_register_handler)(uint16_t (*call)(uint8_t idx));

#define SVC_attm_init_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 72))
typedef void (*attm_init_register_handler)(void (*call)(bool reset));

#define SVC_gattm_init_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 76))
typedef void (*gattm_init_register_handler)(void (*call)(bool reset));

#define SVC_hci_send_2_host_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 80))
typedef void (*hci_send_2_host_register_handler)(void (*call)(void *param));

#define SVC_attmdb_destroy_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 84))
typedef void (*attmdb_destroy_register_handler)(void (*call)(void));


// interrupt
#define SVC_interrupt_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 88))
typedef void (*interrupt_register_handler)(tIRQ_TYPE sig,void (*call)(void));

//stack var init
#define SVC_stack_clear_global_var (*(volatile uint32_t *)(STACK_FUN_ADDR + 92))
typedef void (*stack_clear_global_var_handler)(void);


//ke
#define SVC_ke_event_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 96))
typedef void (*ke_event_set_handler)(uint8_t event_type);

#define SVC_ke_check_malloc (*(volatile uint32_t *)(STACK_FUN_ADDR + 100))
typedef bool (*ke_check_malloc_handler)(uint32_t size, uint8_t type);

#define SVC_ke_malloc (*(volatile uint32_t *)(STACK_FUN_ADDR + 104))
typedef void* (*ke_malloc_handler)(uint32_t size, uint8_t type);

#define SVC_ke_free (*(volatile uint32_t *)(STACK_FUN_ADDR + 108))
typedef void (*ke_free_handler)(void* mem_ptr);

#define SVC_ke_msg_alloc (*(volatile uint32_t *)(STACK_FUN_ADDR + 112))
typedef void* (*ke_msg_alloc_handler)(ke_msg_id_t const id, ke_task_id_t const dest_id,ke_task_id_t const src_id, uint16_t const param_len);

#define SVC_ke_msg_send (*(volatile uint32_t *)(STACK_FUN_ADDR + 116))
typedef void (*ke_msg_send_handler)(void const *param_ptr);

#define SVC_ke_msg_send_basic (*(volatile uint32_t *)(STACK_FUN_ADDR + 120))
typedef void (*ke_msg_send_basic_handler)(ke_msg_id_t const id, ke_task_id_t const dest_id, ke_task_id_t const src_id);

#define SVC_ke_msg_forward (*(volatile uint32_t *)(STACK_FUN_ADDR + 124))
typedef void (*ke_msg_forward_handler)(void const *param_ptr, ke_task_id_t const dest_id, ke_task_id_t const src_id);

#define SVC_ke_msg_free (*(volatile uint32_t *)(STACK_FUN_ADDR + 128))
typedef void (*ke_msg_free_handler)(struct ke_msg const *msg);

#define SVC_ke_msg_src_id_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 132))
typedef ke_msg_id_t (*ke_msg_src_id_get_handler)(void const *param_ptr);

#define SVC_ke_msg_in_queue (*(volatile uint32_t *)(STACK_FUN_ADDR + 136))
typedef bool (*ke_msg_in_queue_handler)(void const *param_ptr);

#define SVC_ke_task_create (*(volatile uint32_t *)(STACK_FUN_ADDR + 140))
typedef void (*ke_task_create_handler)(uint8_t task_type, struct ke_task_desc const * p_task_desc);

#define SVC_ke_state_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 144))
typedef void (*ke_state_set_handler)(ke_task_id_t const id, ke_state_t const state_id);

#define SVC_ke_state_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 148))
typedef ke_state_t (*ke_state_get_handler)(ke_task_id_t const id);

#define SVC_ke_task_msg_flush (*(volatile uint32_t *)(STACK_FUN_ADDR + 152))
typedef void (*ke_task_msg_flush_handler)(ke_task_id_t task);

#define SVC_ke_timer_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 156))
typedef void (*ke_timer_set_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay);

#define SVC_ke_timer_clear (*(volatile uint32_t *)(STACK_FUN_ADDR + 160))
typedef void (*ke_timer_clear_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id);

#define SVC_ke_timer_active (*(volatile uint32_t *)(STACK_FUN_ADDR + 164))
typedef bool (*ke_timer_active_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id);


//rf
#define SVC_rf_freq_dev_cal_result (*(volatile uint32_t *)(STACK_FUN_ADDR + 168))
typedef void (*rf_freq_dev_cal_result)(uint8_t *r_da_gain,uint8_t *r_da_verf_lb,uint8_t *r_da_verf_mb,uint8_t *r_guass_scale);

#define SVC_rf_dev_cal_init_handler_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 172))
typedef void (*rf_dev_cal_init_handler_register)(void (*call)(void));

#define SVC_ble_rx_handler_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 176))
typedef void (*ble_rx_handler_register)(void (*call)(void));

#define SVC_bleip_rf_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 180))					//rf_init
typedef void (*bleip_rf_init_handler)(void);

#define SVC_rc_internal_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 184))
typedef void (*rc_internal_init_handler)(void);

#define SVC_rc_check_period_calib (*(volatile uint32_t *)(STACK_FUN_ADDR + 188))
typedef void (*rc_check_period_calib_handler)(void);

#define SVC_bleip_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 192))
typedef void (*bleip_init_handler)(void);

#define SVC_bleip_schedule (*(volatile uint32_t *)(STACK_FUN_ADDR + 196))
typedef void (*bleip_schedule_handler)(void);

#define SVC_bleip_sleep (*(volatile uint32_t *)(STACK_FUN_ADDR + 200))
typedef bool (*bleip_sleep_handler)(void);

#define SVC_bleip_prevent_sleep_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 204))
typedef void (*bleip_prevent_sleep_set_handler)(uint16_t prv_slp_bit);

#define SVC_bleip_prevent_sleep_clear (*(volatile uint32_t *)(STACK_FUN_ADDR + 208))
typedef void (*bleip_prevent_sleep_clear_handler)(uint16_t prv_slp_bit);

#define SVC_ble_int_cfg (*(volatile uint32_t *)(STACK_FUN_ADDR + 212))
typedef void (*ble_int_cfg_handler)(void);


//co_list
#define SVC_co_list_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 216))
typedef void (*co_list_init_handler)(struct co_list *list);

#define SVC_co_list_push_back (*(volatile uint32_t *)(STACK_FUN_ADDR + 220))
typedef void (*co_list_push_back_handler)(struct co_list *list,struct co_list_hdr *list_hdr);

#define SVC_co_list_push_front (*(volatile uint32_t *)(STACK_FUN_ADDR + 224))
typedef void (*co_list_push_front_handler)(struct co_list *list,struct co_list_hdr *list_hdr);

#define SVC_co_list_pop_front (*(volatile uint32_t *)(STACK_FUN_ADDR + 228))
typedef struct co_list_hdr * (*co_list_pop_front_handler)(struct co_list *list);

#define SVC_co_list_extract (*(volatile uint32_t *)(STACK_FUN_ADDR + 232))
typedef void (*co_list_extract_handler)(struct co_list *list,struct co_list_hdr *list_hdr);

#define SVC_co_list_insert_before (*(volatile uint32_t *)(STACK_FUN_ADDR + 236))
typedef void (*co_list_insert_before_handler)(struct co_list *list,struct co_list_hdr *elt_ref_hdr, struct co_list_hdr *elt_to_add_hdr);

#define SVC_co_list_size (*(volatile uint32_t *)(STACK_FUN_ADDR + 240))
typedef uint16_t (*co_list_size_handler)(struct co_list *list);


//gap
#define SVC_gapc_get_conhdl (*(volatile uint32_t *)(STACK_FUN_ADDR + 244))
typedef uint16_t (*gapc_get_conhdl_handler)(uint8_t conidx);

#define SVC_gapc_is_sec_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 248))
typedef bool (*gapc_is_sec_set_handler)(uint8_t conidx, uint8_t sec_req);

#define SVC_gapc_lk_sec_lvl_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 252))
typedef uint8_t (*gapc_lk_sec_lvl_get_handler)(uint8_t conidx);

#define SVC_gapc_enc_keysize_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 256))
typedef uint8_t (*gapc_enc_keysize_get_handler)(uint8_t conidx);

#define SVC_gapc_svc_chg_ccc_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 260))
typedef bool (*gapc_svc_chg_ccc_get_handler)(uint8_t conidx);

#define SVC_gapc_svc_chg_ccc_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 264))
typedef void (*gapc_svc_chg_ccc_set_handler)(uint8_t conidx, bool enable);

#define SVC_gapc_is_disc_connection (*(volatile uint32_t *)(STACK_FUN_ADDR + 268))
typedef bool (*gapc_is_disc_connection_handler)(uint8_t conidx);

#define SVC_gapm_get_id_from_task (*(volatile uint32_t *)(STACK_FUN_ADDR + 272))
typedef ke_task_id_t (*gapm_get_id_from_task_handler)(ke_msg_id_t task);

#define SVC_gapm_get_max_mtu (*(volatile uint32_t *)(STACK_FUN_ADDR + 276))
typedef uint16_t (*gapm_get_max_mtu_handler)(void);

#define SVC_gapm_svc_chg_en (*(volatile uint32_t *)(STACK_FUN_ADDR + 280))
typedef bool (*gapm_svc_chg_en_handler)(void);


//l2cc
#define SVC_l2cc_pdu_alloc (*(volatile uint32_t *)(STACK_FUN_ADDR + 284))
typedef void* (*l2cc_pdu_alloc_handler)(uint8_t conidx, uint16_t cid, uint8_t code,  ke_task_id_t src_id, uint16_t length);

#define SVC_l2cc_pdu_send (*(volatile uint32_t *)(STACK_FUN_ADDR + 288))
typedef void (*l2cc_pdu_send_handler)(void* pdu);


//lld
#define SVC_lld_enable_cscnt_slp_int (*(volatile uint32_t *)(STACK_FUN_ADDR + 292))
typedef void (*lld_enable_cscnt_slp_int_handler)(void);

#define SVC_lld_sleep_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 296))
typedef void (*lld_sleep_init_handler)(uint16_t wext,uint16_t wosc,uint16_t wrm);

#define SVC_sleep_handler_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 300))
typedef void (*sleep_handler_register)(void (*call)(void));

#define SVC_rf_pa_ramp_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 304))
typedef void (*rf_pa_ramp_init_handler)(void);

#define SVC_ke_event_callback_set (*(volatile uint32_t *)(STACK_FUN_ADDR + 308))
typedef uint8_t (*ke_event_callback_set_handler)(uint8_t event_type, void (*p_callback)(void));

#define SVC_ke_event_clear (*(volatile uint32_t *)(STACK_FUN_ADDR + 312))
typedef void (*ke_event_clear_handler)(uint8_t event_type);

#define SVC_llm_set_test_mode (*(volatile uint32_t *)(STACK_FUN_ADDR + 316))
typedef void (*llm_set_test_mode) ( uint8_t mode );

#define SVC_llm_set_test_mode_tx_payload (*(volatile uint32_t *)(STACK_FUN_ADDR + 320))
typedef void (*llm_set_test_mode_tx_payload) ( uint8_t *payload, uint8_t length );

#define SVC_rf_power_map (*(volatile uint32_t *)(STACK_FUN_ADDR + 324))
typedef void (*rf_power_map)(uint8_t xdbm);

#define SVC_hci_dtm_init_evt_handler (*(volatile uint32_t *)(STACK_FUN_ADDR + 328))
typedef uint32_t (*hci_dtm_init_evt_handler) ( hci_dtm_evt_handler_t p_evt_handler );

#define SVC_lld_test_tx_count_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 332))
typedef void (*lld_test_tx_count_init) ( uint16_t count );

#define SVC_rx_data_report_evt_handler_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 336))
typedef void (*rx_data_report_evt_handler_init) ( rx_data_report_evt_handler_t evt_handler );

#define SVC_gapc_get_role (*(volatile uint32_t *)(STACK_FUN_ADDR + 340))
typedef uint8_t (*gapc_get_role) ( uint8_t conidx );

#define SVC_lld_con_param_get (*(volatile uint32_t *)(STACK_FUN_ADDR + 344))
typedef struct lld_con_param (*lld_con_param_get) ( void );

#define SVC_ke_get_mem_usage (*(volatile uint32_t *)(STACK_FUN_ADDR + 348))
typedef uint16_t (*ke_get_mem_usage) ( uint8_t type );

#define SVC_gapc_get_bdaddr (*(volatile uint32_t *)(STACK_FUN_ADDR + 352))
typedef struct gap_bdaddr* (*gapc_get_bdaddr) (uint8_t conidx, uint8_t src);

#define SVC_ble_event_handler_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 356))
typedef void (*ble_event_handler_register)(void (*call)(void));

#define SVC_rf_init_handler_register (*(volatile uint32_t *)(STACK_FUN_ADDR + 360))
typedef void (*rf_init_handler_register)(void (*call)(void));

#define SVC_ke_init (*(volatile uint32_t *)(STACK_FUN_ADDR + 364))
typedef void (*ke_init)(void);


