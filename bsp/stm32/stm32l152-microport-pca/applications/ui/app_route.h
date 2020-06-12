/************************************************
 * DESCRIPTION:
 *
 * 	route for screen show
 *
 ************************************************/
#pragma once

//#include "ssz_def.h"
#include "GUI.h"
#include "rtthread.h"
#include "uiDisp.h"
#include "WM.h"

/************************************************
 * Declaration
 ************************************************/


#ifdef __cplusplus
extern "C" {
#endif

typedef WM_HWIN (*ScreenCreateFunc)(type_MsgBody4UICtrlMsg *);
typedef int (*ScreenUpdateFunc)(WM_HWIN , type_MsgBody4UICtrlMsg *);
typedef void (*ScrennDestroyFunc)(WM_HWIN);


//Screen information: id、creatfun、updatefun、destroyfun
typedef struct {
	uint8_t uiclass_id;
	uint8_t screen_id[5];
	ScreenCreateFunc create_func;
	ScreenUpdateFunc update_fun;
	ScrennDestroyFunc destroy_fun;
} ScreenInfo;

#define SCREEN_NUMBER_TOTAL		50
extern const ScreenInfo g_scr_routes[SCREEN_NUMBER_TOTAL];

#define  UD_CHILD_MAX 10
typedef struct
{
	WM_HWIN win_id;
	uint8_t last_cursor;
	uint8_t cur_cursor; 
	uint8_t opcode;
	uint8_t uiid[5];
	uint8_t classid;        
	uint8_t flag_upate_once;
	WM_HWIN child_id[UD_CHILD_MAX];
}UI_ScreenCommon;
	

extern UI_ScreenCommon g_ui_common_param;

//void ert_uiDisp(uint8_t UIOpCode, type_MsgBody4UICtrlMsg *UIDispData);
void ert_uiDispService(uint8_T UIClassID,uiClassIDArray UIIDArray,curDescriptor *CursorDept,
					   dItemDescriptor* DItemDept, sItemDescriptor* SItemDept);

WM_HWIN ui_get_current_hwin(void);

#ifdef __cplusplus
}
#endif














