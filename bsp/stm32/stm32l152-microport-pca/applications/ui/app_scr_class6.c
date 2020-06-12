/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

#include "DIALOG.h"
#include "app_scr_class6.h"
#include "GUI.h"
//#include "ui_common.h"
#include "image_data.h"
#include "string_data.h"
#include "font_data.h"
#include <stdbool.h>

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0     		(GUI_ID_USER + 0x00)
#define EXHAUST_ID_IMG_MENU     (GUI_ID_USER + 0x01)
#define EXHAUST_ID_IMG_SELECT   (GUI_ID_USER + 0x02)
#define EXHAUST_ID_STR_TITLE    (GUI_ID_USER + 0x03)
#define TIME_ID				    (GUI_ID_USER + 0x04)
#define BATTERY_LEVEL_ID        (GUI_ID_USER + 0x05)

#define MAIN_MENU_ICON_X(x)     ((64 - x)/2)
#define MAIN_MENU_ICON_Y(y)     ((64 - y)/2)

#define MAIN_MENU_ICON_XSIZE	64
#define MAIN_MENU_ICON_YSIZE	64

#define SELECT_ICON_SUM_LEN			(8*9)
#define SELECT_ICON_X(index)    ((DISPLAY_WIDTH-SELECT_ICON_SUM_LEN)/2 + (index*16))
#define SELECT_ICON_Y           (DISPLAY_HEIGHT-16)

/*********************************************************************
*
*       Static data
*
***********************************************************************/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "app_main_menu_11000", ID_WINDOW_0, 0, 0, 256, 64, 0, 0x0, 0 },
};

/*********************************************************************
*
*       Static code
*
***********************************************************************/

static void put_menu_info(uint16_t cursor_index)
{
	const GUI_BITMAP *pbm;
	const char *pstr;
	int x;
	
	switch(cursor_index)
	{
		case 0://exhaust
		{
			pbm = get_image(kImgExhaust);
			pstr = get_string(kStrClearAir);
			break;
		}

		case 1://BKGD
		{
			pbm = get_image(kImgNewTask);
			//pstr = get_string(kStrNewInfusionTask);
			pstr =  "BKGD" ;
			break;
		}

		case 2://PCA
		{
			pbm = get_image(kImgStopInfusion);
			//pstr = get_string(kStrStopInfusion);
			pstr =  "PCA" ;
			break;
		}
		case 3://autoCycle
		{
			pbm = get_image(kImgHistoryLog);
			//pstr = get_string(kStrHistoryLog);
			pstr =  "autoCycle" ;
			break;
		}
		
		case 4://histroy log
		{
			pbm = get_image(kImgHistoryLog);
			pstr = get_string(kStrHistoryLog);
			break;
		}

		case 5://sysytem setting
		{
			pbm = get_image(kImgSystemSetting);
			pstr = get_string(kStrSetting);
			break;
		}

		default:
	  		return;
	}
	
	x = SELECT_ICON_X(cursor_index);

	IMAGE_SetBitmap(WM_GetDialogItem(ui_get_current_hwin(), EXHAUST_ID_IMG_MENU), pbm);
	WM_MoveChildTo(WM_GetDialogItem(ui_get_current_hwin(), EXHAUST_ID_IMG_SELECT), x, SELECT_ICON_Y);
	TEXT_SetText(WM_GetDialogItem(ui_get_current_hwin(), EXHAUST_ID_STR_TITLE), pstr);
}

static void put_time(uint8_t hour,uint8_t min)
{
	char *p;
	p = get_dynamic_string(kStrDynamic3);
	sprintf(p, "%02d:%02d", hour,min);	

	TEXT_SetText(WM_GetDialogItem(g_ui_common_param.win_id, TIME_ID), p);
}

static void put_battery_level(uint8_t level)
{
	const GUI_BITMAP *bitmap;
	const char *ptr;

	switch(level)
	{
	case 0:
		bitmap = get_image(kImgBatteryEmpty);
		break;

	case 1:
		bitmap = get_image(kImgBatteryOne);
		break;

	case 2:
		bitmap = get_image(kImgBatteryTwo);
		break;

	case 3:
		bitmap = get_image(kImgBatteryThree);
		break;

	case 4:
		bitmap = get_image(kImgBatteryFull);
		break;
        }
        
    IMAGE_SetBitmap(WM_GetDialogItem(g_ui_common_param.win_id, BATTERY_LEVEL_ID), bitmap);
}

static void _cbDialog(WM_MESSAGE * pMsg) 
{
	uint8_t data;
	GUI_RECT rect = {193, 48, 256, 64}; 
	WM_HWIN hWin;
	GUI_CONST_STORAGE GUI_BITMAP* bitmap;
	switch (pMsg->MsgId) 
	{
		case WM_INIT_DIALOG:
			WINDOW_SetBkColor(pMsg->hWin, GUI_BLACK);

			//menu image
			bitmap = get_image(kImgNewTask);
			hWin = IMAGE_CreateEx(MAIN_MENU_ICON_X(bitmap->XSize), MAIN_MENU_ICON_Y(bitmap->YSize), bitmap->XSize, bitmap->YSize, pMsg->hWin, 
				                  WM_CF_SHOW,  IMAGE_CF_AUTOSIZE, EXHAUST_ID_IMG_MENU);
			IMAGE_SetBitmap(hWin, bitmap);
			
			//menu tile
			hWin = TEXT_CreateAsChild(MAIN_MENU_ICON_XSIZE, (DISPLAY_HEIGHT-14)/2, DISPLAY_WIDTH - 128, 14, pMsg->hWin, EXHAUST_ID_STR_TITLE, 
				                      WM_CF_SHOW, get_string(kStrNewInfusionTask), GUI_TA_VCENTER|GUI_TA_HCENTER);
			TEXT_SetFont(hWin, get_font(14));
			TEXT_SetBkColor(hWin, GUI_BLACK);
			TEXT_SetTextColor(hWin, GUI_WHITE);			

			//select
			bitmap = get_image(kImgSelectedPointer);
			hWin = IMAGE_CreateEx(SELECT_ICON_X(0), SELECT_ICON_Y, bitmap->XSize, bitmap->YSize, pMsg->hWin, 
				                  WM_CF_SHOW,  IMAGE_CF_AUTOSIZE, EXHAUST_ID_IMG_SELECT);
			IMAGE_SetBitmap(hWin, bitmap);

			//time 
			hWin = TEXT_CreateEx(DISPLAY_WIDTH-34-GUI_GetStringDistX("08:30")-5, 0, GUI_GetStringDistX("08:30"), 13, pMsg->hWin, 
				                 WM_CF_SHOW, GUI_TA_VCENTER|GUI_TA_LEFT,  TIME_ID, "08:30");
			TEXT_SetFont(hWin, get_font(14));
			TEXT_SetBkColor(hWin, GUI_BLACK);
			TEXT_SetTextColor(hWin, GUI_WHITE);

			//剩余电量
			hWin = IMAGE_CreateEx(222, 2, 33, 14, pMsg->hWin, WM_CF_SHOW,  IMAGE_CF_AUTOSIZE, BATTERY_LEVEL_ID);
			IMAGE_SetBitmap(hWin, get_image(kImgBatteryFull));

			break;
	
		case WM_PAINT:
			GUI_SetColor(GUI_WHITE);
			GUI_SetFont(get_font(14));
			GUI_DispStringInRect(get_string(kStrEnter), &rect, GUI_TA_RIGHT|GUI_TA_VCENTER);
			for(int i=0; i<6; i++)
			{
				GUI_DrawBitmap(get_image(kImgUnselectedPointer), SELECT_ICON_X(i), SELECT_ICON_Y+7);
			}
			break;  
		default:
			  WM_DefaultProc(pMsg);
			  break;
	}
}

/*********************************************************************
*
*       Public code
*
***********************************************************************/

WM_HWIN app_scr_class6_create(type_MsgBody4UICtrlMsg *msg) 
{
	WM_HWIN hWin;	
	hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
	return hWin;
}


int app_scr_class6_update(WM_HWIN hwin, type_MsgBody4UICtrlMsg *msg)
{	
	//设置时间
	put_time(msg->SItem.DataValArray[3],msg->SItem.DataValArray[2]);
	
	//设置电量
	put_battery_level(msg->SItem.DataValArray[1]);

	//菜单选项
	put_menu_info(msg->SItem.DataValArray[4]);

 	return 0;
}

void app_scr_class6_destroy(WM_HWIN win_id) 
{
	GUI_EndDialog(win_id, 0);
}


/*************************** End of file ****************************/