
#ifndef APP_HID_KEYBOARD_H_
#define APP_HID_KEYBOARD_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief HID Application Module entry point
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"     // SW configuration

#if (BLE_APP_HID)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition
#include "app_hid.h"

#if (PS2_SUPPORT)
#include "ps2.h"             // PS2 Mouse Driver
#endif //(PS2_SUPPORT)



/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

typedef enum
{
    HID_KEYBOARD_RESERVED      = 0,   /// 0x00 - Reserved (no event indicated)
    HID_KEYBOARD_ERR_ROLL_OVER = 1,   /// 0x01 - Keyboard ErrorRollOver
	HID_KAYBOARD_POST_FAIL     = 2,   /// 0x02 - Keyboard POSTFail
    HID_KEYBOARD_ERR_UNDEFINED = 3,   /// 0x03 - Keyboard ErrorUndefined
    HID_KEYBOARD_A             = 4,   /// 0x04 - Keyboard a and A
    HID_KEYBOARD_B             = 5,   /// 0x05 - Keyboard b and B
    HID_KEYBOARD_C             = 6,   /// 0x06 - Keyboard c and C
    HID_KEYBOARD_D             = 7,   /// 0x07 - Keyboard d and D
    HID_KEYBOARD_E             = 8,   /// 0x08 - Keyboard e and E
    HID_KEYBOARD_F             = 9,   /// 0x09 - Keyboard f and F
    HID_KEYBOARD_G             = 10,  /// 0x0A - Keyboard g and G
    HID_KEYBOARD_H             = 11,  /// 0x0B - Keyboard h and H
    HID_KEYBOARD_I             = 12,  /// 0x0C - Keyboard i and I
    HID_KEYBOARD_J             = 13,  /// 0x0D - Keyboard j and J
    HID_KEYBOARD_K             = 14,  /// 0x0E - Keyboard k and K
    HID_KEYBOARD_L             = 15,  /// 0x0F - Keyboard l and L
    HID_KEYBOARD_M             = 16,  /// 0x10 - Keyboard m and M
    HID_KEYBOARD_N             = 17,  /// 0x11 - Keyboard n and N
    HID_KEYBOARD_O             = 18,  /// 0x12 - Keyboard o and O
    HID_KEYBOARD_P             = 19,  /// 0x13 - Keyboard p and p
    HID_KEYBOARD_Q             = 20,  /// 0x14 - Keyboard q and Q
    HID_KEYBOARD_R             = 21,  /// 0x15 - Keyboard r and R
    HID_KEYBOARD_S             = 22,  /// 0x16 - Keyboard s and S
    HID_KEYBOARD_T             = 23,  /// 0x17 - Keyboard t and T
    HID_KEYBOARD_U             = 24,  /// 0x18 - Keyboard u and U
    HID_KEYBOARD_V             = 25,  /// 0x19 - Keyboard v and V
    HID_KEYBOARD_W             = 26,  /// 0x1A - Keyboard w and W
    HID_KEYBOARD_X             = 27,  /// 0x1B - Keyboard x and X
    HID_KEYBOARD_Y             = 28,  /// 0x1C - Keyboard y and Y
    HID_KEYBOARD_Z             = 29,  /// 0x1D - Keyboard z and Z
    HID_KEYBOARD_1             = 30,  /// 0x1E - Keyboard 1 and !
    HID_KEYBOARD_2             = 31,  /// 0x1F - Keyboard 2 and @
    HID_KEYBOARD_3             = 32,  /// 0x20 - Keyboard 3 and #
    HID_KEYBOARD_4             = 33,  /// 0x21 - Keyboard 4 and %
    HID_KEYBOARD_5             = 34,  /// 0x22 - Keyboard 5 and %
    HID_KEYBOARD_6             = 35,  /// 0x23 - Keyboard 6 and ^
    HID_KEYBOARD_7             = 36,  /// 0x24 - Keyboard 7 and &
    HID_KEYBOARD_8             = 37,  /// 0x25 - Keyboard 8 and *
    HID_KEYBOARD_9             = 38,  /// 0x26 - Keyboard 9 and (
    HID_KEYBOARD_0             = 39,  /// 0x27 - Keyboard 0 and )
    HID_KEYBOARD_ENTER         = 40,  /// 0x28 - Keyboard Return (ENTER)
    HID_KEYBOARD_ESCAPE        = 41,  /// 0x29 - Keyboard ESCAPE
    HID_KEYBOARD_DELETE        = 42,  /// 0x2A - Keyboard DELETE (Backspace)
    HID_KEYBOARD_TAB           = 43,  /// 0x2B - Keyboard Tab
    HID_KEYBOARD_SPACEBAR      = 44,  /// 0x2C - Keyboard Spacebar
    HID_KEYBOARD_MINUS         = 45,  /// 0x2D - Keyboard - and (underscore)
    HID_KEYBOARD_EQUAL         = 46,  /// 0x2E - Keyboard = and +
    HID_KEYBOARD_LEFT_BRKT     = 47,  /// 0x2F - Keyboard [ and {
    HID_KEYBOARD_RIGHT_BRKT    = 48,  /// 0x30 - Keyboard ] and }
    HID_KEYBOARD_BACK_SLASH    = 49,  /// 0x31 - Keyboard \ and |
    HID_KEYBOARD_SEMI_COLON    = 51,  /// 0x33 - Keyboard ; and :
    HID_KEYBOARD_SGL_QUOTE     = 52,  /// 0x34 - Keyboard ' and "
    HID_KEYBOARD_GRV_ACCENT    = 53,  /// 0x35 - Keyboard Grave Accent and Tilde
    HID_KEYBOARD_COMMA         = 54,  /// 0x36 - Keyboard , and <
    HID_KEYBOARD_DOT           = 55,  /// 0x37 - Keyboard . and >
    HID_KEYBOARD_FWD_SLASH     = 56,  /// 0x38 - Keyboard / and ?
    HID_KEYBOARD_CAPS_LOCK     = 57,  /// 0x39 - Keyboard Caps Lock
    HID_KEYBOARD_F1            = 58,  /// 0x3A - Keyboard F1
    HID_KEYBOARD_F2            = 59,  /// 0x3B - Keyboard F2
    HID_KEYBOARD_F3            = 60,  /// 0x3C - Keyboard F3
    HID_KEYBOARD_F4            = 61,  /// 0x3D - Keyboard F4
    HID_KEYBOARD_F5            = 62,  /// 0x3E - Keyboard F5
    HID_KEYBOARD_F6            = 63,  /// 0x3F - Keyboard F6
    HID_KEYBOARD_F7            = 64,  /// 0x40 - Keyboard F7
    HID_KEYBOARD_F8            = 65,  /// 0x41 - Keyboard F8
    HID_KEYBOARD_F9            = 66,  /// 0x42 - Keyboard F9
    HID_KEYBOARD_F10           = 67,  /// 0x43 - Keyboard F10
    HID_KEYBOARD_F11           = 68,  /// 0x44 - Keyboard F11
    HID_KEYBOARD_F12           = 69,  /// 0x45 - Keyboard F12
    HID_KEYBOARD_PRNT_SCREEN   = 70,  /// 0x46 - Keyboard Print Screen
    HID_KEYBOARD_SCROLL_LOCK   = 71,  /// 0x47 - Keyboard Scroll Lock
    HID_KEYBOARD_PAUSE         = 72,  /// 0x48 - Keyboard Pause
    HID_KEYBOARD_INSERT        = 73,  /// 0x49 - Keyboard Insert
    HID_KEYBOARD_HOME          = 74,  /// 0x4A - Keyboard Home
    HID_KEYBOARD_PAGE_UP       = 75,  /// 0x4B - Keyboard PageUp
    HID_KEYBOARD_DELETE_FWD    = 76,  /// 0x4C - Keyboard Delete Forward
    HID_KEYBOARD_END           = 77,  /// 0x4D - Keyboard End
    HID_KEYBOARD_PAGE_DOWN     = 78,  /// 0x4E - Keyboard PageDown
    HID_KEYBOARD_RIGHT_ARROW   = 79,  /// 0x4F - Keyboard RightArrow
    HID_KEYBOARD_LEFT_ARROW    = 80,  /// 0x50 - Keyboard LeftArrow
    HID_KEYBOARD_DOWN_ARROW    = 81,  /// 0x51 - Keyboard DownArrow
    HID_KEYBOARD_UP_ARROW      = 82,  /// 0x52 - Keyboard UpArrow
    HID_KEYBPAD_NUM_LOCK       = 83,  /// 0x53 - Keypad Num Lock and Clear
    HID_KEYBPAD_DIVIDE         = 84,  /// 0x54 - Keypad /
    HID_KEYBOARD_MULTIPLY      = 85,  /// 0x55 - Keypad *
    HID_KEYBOARD_SUBTRACT      = 86,  /// 0x56 - Keypad -
    HID_KEYBPAD_ADD            = 87,  /// 0x57 - Keypad +
    HID_KEYBPAD_ENTER          = 88,  /// 0x58 - Keypad ENTER
    HID_KEYBPAD_1              = 89,  /// 0x59 - Keypad 1 and End
    HID_KEYBPAD_2              = 90,  /// 0x5A - Keypad 2 and Down Arrow
    HID_KEYBPAD_3              = 91,  /// 0x5B - Keypad 3 and PageDn
    HID_KEYBPAD_4              = 92,  /// 0x5C - Keypad 4 and Lfet Arrow
    HID_KEYBPAD_5              = 93,  /// 0x5D - Keypad 5
    HID_KEYBPAD_6              = 94,  /// 0x5E - Keypad 6 and Right Arrow
    HID_KEYBPAD_7              = 95,  /// 0x5F - Keypad 7 and Home
    HID_KEYBPAD_8              = 96,  /// 0x60 - Keypad 8 and Up Arrow
    HID_KEYBPAD_9              = 97,  /// 0x61 - Keypad 9 and PageUp
    HID_KEYBPAD_0              = 98,  /// 0x62 - Keypad 0 and Insert
    HID_KEYBPAD_DOT            = 99,  /// 0x63 - Keypad . and Delete
    HID_KEYBPAD_MENU           = 101, /// 0x65 - Keypad Menu
    HID_KEYBPAD_POWER          = 102, /// 0x66 - Keypad Power
    HID_KEYBOARD_MUTE          = 127, /// 0x7F - Keyboard Mute
    HID_KEYBOARD_VOLUME_UP     = 128, /// 0x80 - Keyboard Volume up
    HID_KEYBOARD_VOLUME_DOWN   = 129, /// 0x81 - Keyboard Volume down
    HID_KEYBOARD_LEFT_CTRL     = 224, /// 0xE0 - Keyboard LeftContorl
    HID_KEYBOARD_LEFT_SHIFT    = 225, /// 0xE1 - Keyboard LeftShift
    HID_KEYBOARD_LEFT_ALT      = 226, /// 0xE2 - Keyboard LeftAlt
    HID_KEYBOARD_LEFT_GUI      = 227, /// 0xE3 - Keyboard LeftGUI
    HID_KEYBOARD_RIGHT_CTRL    = 228, /// 0xE4 - Keyboard LeftContorl
    HID_KEYBOARD_RIGHT_SHIFT   = 229, /// 0xE5 - Keyboard LeftShift
    HID_KEYBOARD_RIGHT_ALT     = 230, /// 0xE6 - Keyboard LeftAlt
    HID_KEYBOARD_RIGHT_GUI     = 231, /// 0xE7 - Keyboard RightGUI
    HID_KEYBOARD_BACK          = 241, /// 0xF1 - Keyboard BACK
} hid_keyboard_value_t;

typedef enum
{
    HID_MOUSE_BUTTON_LEFT       = 253,
    HID_MOUSE_BUTTON_MIDDLE     = 254,
    HID_MOUSE_BUTTON_RIGHT      = 255,
} hid_mouse_value_t;

// HID Consumer Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
//typedef enum
//{

#define HID_CONSUMER_POWER          48  // 0x30 - Power
#define HID_CONSUMER_RESET          49  // 0x31 - Reset
#define HID_CONSUMER_SLEEP          50  // 0x32 - Sleep

#define HID_CONSUMER_MENU           64  // 0x40 - Menu

#define HID_CONSUMER_BRIGHTNESS_UP  111 // 0x6f
#define HID_CONSUMER_BRIGHTNESS_DOWN  112 // 0x70

#define HID_CONSUMER_SELECTION      128 // 0x80 - Selection
#define HID_CONSUMER_ASSIGN_SEL     129 // 0x81 - Assign Selection
#define HID_CONSUMER_MODE_STEP      130 // 0x82 - Mode Step
#define HID_CONSUMER_RECALL_LAST    131 // 0x83 - Recall Last
#define HID_CONSUMER_QUIT           148 // 0x94 - Quit
#define HID_CONSUMER_HELP           149 // 0x95 - Help
#define HID_CONSUMER_CHANNEL_UP     156 // 0x9C - Channel Increment
#define HID_CONSUMER_CHANNEL_DOWN   157 // 0x9D - Channel Decrement

#define HID_CONSUMER_PLAY           176 // 0xB0 - Play
#define HID_CONSUMER_PAUSE          177 // 0xB1 - Pause
#define HID_CONSUMER_RECORD         178 // 0xB2 - Record
#define HID_CONSUMER_FAST_FORWARD   179 // 0xB3 - Fast Forward
#define HID_CONSUMER_REWIND         180 // 0xB4 - Rewind
#define HID_CONSUMER_SCAN_NEXT_TRK  181 // 0xB5 - Scan Next Track
#define HID_CONSUMER_SCAN_PREV_TRK  182 // 0xB6 - Scan Previous Track
#define HID_CONSUMER_STOP           183 // 0xB7 - Stop
#define HID_CONSUMER_EJECT          184 // 0xB8 - Eject
#define HID_CONSUMER_RANDOM_PLAY    185 // 0xB9 - Random Play
#define HID_CONSUMER_SELECT_DISC    186 // 0xBA - Select Disk
#define HID_CONSUMER_ENTER_DISC     187 // 0xBB - Enter Disc
#define HID_CONSUMER_REPEAT         188 // 0xBC - Repeat
#define HID_CONSUMER_STOP_EJECT     204 // 0xCC - Stop/Eject
#define HID_CONSUMER_PLAY_PAUSE     205 // 0xCD - Play/Pause
#define HID_CONSUMER_PLAY_SKIP      206 // 0xCE - Play/Skip

#define HID_CONSUMER_VOLUME         224 // 0xE0 - Volume
#define HID_CONSUMER_BALANCE        225 // 0xE1 - Balance
#define HID_CONSUMER_MUTE           226 // 0xE2 - Mute
#define HID_CONSUMER_BASS           227 // 0xE3 - Bass
#define HID_CONSUMER_VOLUME_UP      233 // 0xE9 - Volume Increment
#define HID_CONSUMER_VOLUME_DOWN    234 // 0xEA - Volume Decrement
//} hid_consumer_value_t;

//report key input byte0
#define HID_BIT_KEY_INPUT_NONE            (0x00)
#define HID_BIT_KEY_INPUT_L_CTRL          (0x01)
#define HID_BIT_KEY_INPUT_L_SHIFT         (0x02)
#define HID_BIT_KEY_INPUT_L_ATL           (0x04)
#define HID_BIT_KEY_INPUT_L_GUI           (0x08)
#define HID_BIT_KEY_INPUT_R_CTRL          (0x10)
#define HID_BIT_KEY_INPUT_R_SHIFT         (0x20)
#define HID_BIT_KEY_INPUT_R_ATL           (0x40)
#define HID_BIT_KEY_INPUT_R_GUI           (0x80)

//report led output
#define HID_BIT_LED_OUTPUT_CAPSLOCK       (0x02)

//report consumer byte0
#define HID_BIT_CONSUMER_HOME             (0x0001)
#define HID_BIT_CONSUMER_BRIGHTNESS_DOWN  (0x0002)
#define HID_BIT_CONSUMER_BRIGHTNESS_UP    (0x0004)
#define HID_BIT_CONSUMER_KEYBOARD         (0x0008)
#define HID_BIT_CONSUMER_SEARCH           (0x0010)
//#define HID_BIT_CONSUMER_COPY             (0x0020)
//#define HID_BIT_CONSUMER_PASTE            (0x0040)

//report consumer byte1
#define HID_BIT_CONSUMER_SCAN_PRE         (0x0100)
#define HID_BIT_CONSUMER_PLAY_PAUSE       (0x0200)
#define HID_BIT_CONSUMER_SCAN_NEXT        (0x0400)
#define HID_BIT_CONSUMER_MUTE             (0x0800)
#define HID_BIT_CONSUMER_VOL_DOWN         (0x1000)
#define HID_BIT_CONSUMER_VOL_UP           (0x2000)
#define HID_BIT_CONSUMER_POWER            (0x4000)

#define HID_REPORT_KEY_INPUT_LEN                (8)
#define HID_REPORT_KEY_INPUT_KEYCODE_START_POS  (2)
#define HID_REPORT_LED_OUTPUT_LEN               (1)
#define HID_REPORT_CONSUMER_LEN                 (2)


/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// Table of message handlers
extern const struct ke_state_handler app_hid_table_handler;

/*
 * GLOBAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * Health Thermometer Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize HID Application Module
 ****************************************************************************************
 */
void app_hid_init(void);

/**
 ****************************************************************************************
 * @brief Initialize the PS2 mouse driver so that it can be used by the application
 ****************************************************************************************
 */
void app_hid_start_mouse(void);

/**
 ****************************************************************************************
 * @brief Add a HID Service instance in the DB
 ****************************************************************************************
 */
void app_hid_add_hids(void);

/**
 ****************************************************************************************
 * @brief Enable the HID Over GATT Profile device role
 *
 * @param[in]:  conhdl - Connection handle for the connection
 ****************************************************************************************
 */
void app_hid_enable_prf(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send a mouse report to the peer device
 *
 * @param[in]:  report - Mouse report sent by the PS2 driver
 ****************************************************************************************
 */
//void app_hid_send_mouse_report(struct ps2_mouse_msg report);

//DYC_added
void app_hid_send_value_report(uint8_t rpt_idx,uint8_t *buf,uint8_t len);

extern  int app_conn_update_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id);
#endif //(BLE_APP_HID)

/// @} APP

#endif // APP_HID_KEYBOARD_H_
