/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "GUI.h"
#include "WM.h"
#include "app_route.h"
#include "font_data.h"

void wm_bk_callback(WM_MESSAGE *pMsg)
{
	switch(pMsg->MsgId)
	{
		case WM_PAINT:
			//WINDOW_SetBkColor(pMsg->hWin, GUI_WHITE);
            GUI_Clear();
			break;

		default:
			WM_DefaultProc(pMsg);
			break;

	}
}

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 1)
type_MsgBody4UICtrlMsg disp_msg;
int main(void)
{
    int count = 1;
		
		

		//ssz_locale_set_lang(kSszLangSimplifiedChinese);
		//ssz_locale_set_country_region(kSszCountryRegionChina);

		font_data_select_by_lang(kSszLangSimplifiedChinese);
		string_data_select_by_lang(kSszLangSimplifiedChinese);

		//init emWin component
		if( GUI_Init() != 0)
		{
			while(1);
		}
		GUI_UC_SetEncodeUTF8();
		GUI_SetBkColor(GUI_BLACK);
		WM_SetCallback(WM_HBKWIN, wm_bk_callback);
		GUI_SetColor(GUI_WHITE);
		
		//GUI_DispString("Hello world!");
		
		GUI_SetFont(get_font(14));
		
		disp_msg.SItem.DataValArray[1] = 1 ;
		ert_uiDispService( 1 ,disp_msg.UIID, &disp_msg.CursorIndex, &disp_msg.ditem, &disp_msg.SItem);
		//GUI_DispString("Hello world!Hello world!Hello world!Hello world!");
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
			//ert_uiDispService( 1 ,disp_msg.UIID, &disp_msg.CursorIndex, &disp_msg.ditem, &disp_msg.SItem);
					
			GUI_Exec();
//    rt_pin_write(LED0_PIN, PIN_HIGH);
      rt_thread_mdelay(500);
//        rt_pin_write(LED0_PIN, PIN_LOW);
//        rt_thread_mdelay(500);
			
    }

    return RT_EOK;
}

#include <finsh.h>

void display_UI( uint8_t i)
{
	
	disp_msg.SItem.DataValArray[1] = i ;
	ert_uiDispService( 1 ,disp_msg.UIID, &disp_msg.CursorIndex, &disp_msg.ditem, &disp_msg.SItem);

}
FINSH_FUNCTION_EXPORT(display_UI, test dispaly UI class1);


