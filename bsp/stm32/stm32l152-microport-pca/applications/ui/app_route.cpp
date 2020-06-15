/************************************************
 * DESCRIPTION:
 *
 * 	route for screen show
 *
 ************************************************/
#include "string.h"

#include "app_route.h"
//#include "config.h" 
#include "WM.h"

//add by deming, 2020/05/27
#include "app_scr_class1.h"
#include "app_scr_class2.h"
#include "app_scr_class3.h"
#include "app_scr_class4.h"
#include "app_scr_class5.h"
#include "app_scr_class6.h"

#include "app_scr_class8.h"
#include "app_scr_class9.h"
#include "app_scr_class10.h"
#include "app_scr_class11.h"
#include "app_scr_class12.h"
#include "app_scr_class13.h"
#include "app_scr_class14.h"
#include "app_scr_class15.h"
#include "app_scr_class16.h"
#include "app_scr_class17.h"
#include "app_scr_class18.h"
#include "app_scr_class19.h"
#include "app_scr_class20.h"

/************************************************
 * Declaration
 ************************************************/
 UI_ScreenCommon g_ui_common_param;

const ScreenInfo g_scr_routes[SCREEN_NUMBER_TOTAL]= {

	{
		.uiclass_id 	= 1,
		//.screen_id 		= {},
		.create_func 	= app_scr_class1_create,
		.update_fun 	= app_scr_class1_update,
		.destroy_fun 	= app_scr_class1_destroy,
	},
	{
		.uiclass_id 	= 2,
		//.screen_id 		= {},
		.create_func 	= app_scr_class2_create,
		.update_fun 	= app_scr_class2_update,
		.destroy_fun 	= app_scr_class2_destroy,
	},
	{
		.uiclass_id 	= 3,
		//.screen_id 		= {},
		.create_func 	= app_scr_class3_create,
		.update_fun 	= app_scr_class3_update,
		.destroy_fun 	= app_scr_class3_destroy,
	},
	{
		.uiclass_id 	= 4,
		//.screen_id 		= {},
		.create_func 	= app_scr_class4_create,
		.update_fun 	= app_scr_class4_update,
		.destroy_fun 	= app_scr_class4_destroy,
	},
	{
		.uiclass_id 	= 5,
		//.screen_id 	= {},
		.create_func 	= app_scr_class5_create,
		.update_fun 	= app_scr_class5_update,
		.destroy_fun 	= app_scr_class5_destroy,
	},
	{
		.uiclass_id 	= 6,
		//.screen_id 		= {},
		.create_func 	= app_scr_class6_create,
		.update_fun 	= app_scr_class6_update,
		.destroy_fun 	= app_scr_class6_destroy,
	},
	{
		.uiclass_id 	= 7,
		//.screen_id 		= {},
		.create_func 	= NULL,
		.update_fun 	= NULL,
		.destroy_fun 	= NULL,
	},
	{
		.uiclass_id 	= 8,
		//.screen_id 		= {},
		.create_func 	= app_scr_class8_create,
		.update_fun 	= app_scr_class8_update,
		.destroy_fun 	= app_scr_class8_destroy,
	},
	{
		.uiclass_id 	= 9,
		//.screen_id 		= {},
		.create_func 	= app_scr_class9_create,
		.update_fun 	= app_scr_class9_update,
		.destroy_fun 	= app_scr_class9_destroy,
	},
	{
		.uiclass_id 	= 10,
		//.screen_id 		= {},
		.create_func 	= app_scr_class10_create,
		.update_fun 	= app_scr_class10_update,
		.destroy_fun 	= app_scr_class10_destroy,
	},
	{
		.uiclass_id 	= 11,
		//.screen_id 		= {},
		.create_func 	= app_scr_class11_create,
		.update_fun 	= app_scr_class11_update,
		.destroy_fun 	= app_scr_class11_destroy,
	},
	{
		.uiclass_id 	= 12,
		//.screen_id 		= {},
		.create_func 	= app_scr_class12_create,
		.update_fun 	= app_scr_class12_update,
		.destroy_fun 	= app_scr_class12_destroy,
	},
	{
		.uiclass_id 	= 13,
		//.screen_id 		= {},
		.create_func 	= app_scr_class13_create,
		.update_fun 	= app_scr_class13_update,
		.destroy_fun 	= app_scr_class13_destroy,
	},
	{
		.uiclass_id 	= 14,
		//.screen_id 		= {},
		.create_func 	= app_scr_class14_create,
		.update_fun 	= app_scr_class14_update,
		.destroy_fun 	= app_scr_class14_destroy,
	},
	{
		.uiclass_id 	= 15,
		//.screen_id 		= {},
		.create_func 	= app_scr_class15_create,
		.update_fun 	= app_scr_class15_update,
		.destroy_fun 	= app_scr_class15_destroy,
	},
	{
		.uiclass_id 	= 16,
		//.screen_id 		= {},
		.create_func 	= app_scr_class16_create,
		.update_fun 	= app_scr_class16_update,
		.destroy_fun 	= app_scr_class16_destroy,
	},
	{
		.uiclass_id 	= 17,
		//.screen_id 		= {},
		.create_func 	= app_scr_class17_create,
		.update_fun 	= app_scr_class17_update,
		.destroy_fun 	= app_scr_class17_destroy,
	},
	{
		.uiclass_id 	= 18,
		//.screen_id 		= {},
		.create_func 	= app_scr_class18_create,
		.update_fun 	= app_scr_class18_update,
		.destroy_fun 	= app_scr_class18_destroy,
	},
	{
		.uiclass_id 	= 19,
		//.screen_id 		= {},
		.create_func 	= app_scr_class19_create,
		.update_fun 	= app_scr_class19_update,
		.destroy_fun 	= app_scr_class19_destroy,
	},
	{
		.uiclass_id 	= 20,
		//.screen_id 		= {},
		.create_func 	= app_scr_class20_create,
		.update_fun 	= app_scr_class20_update,
		.destroy_fun 	= app_scr_class20_destroy,
	},
         
    

};
 
WM_HWIN ui_get_current_hwin(void)
{
	return g_ui_common_param.win_id;
}

void ert_uiDispService(uint8_T UIClassID,uiClassIDArray UIIDArray,
        curDescriptor* CursorDept,dItemDescriptor* DItemDept,
        sItemDescriptor* SItemDept)
{
  	int i;
	ScreenCreateFunc createfunc;
	ScreenUpdateFunc updatefun;
	ScrennDestroyFunc destroyfun;

	type_MsgBody4UICtrlMsg UIDispData;
	memset((uint8_t *)&UIDispData, 0, sizeof(type_MsgBody4UICtrlMsg));
	
        //copy data 
	memcpy((uint8_t *)&UIDispData.CursorIndex, (uint8_t *)CursorDept, sizeof(curDescriptor));
	memcpy((uint8_t *)&UIDispData.ditem, (uint8_t *)DItemDept, sizeof(dItemDescriptor));
	memcpy((uint8_t *)&UIDispData.SItem, (uint8_t *)SItemDept, sizeof(sItemDescriptor));
	memcpy(UIDispData.UIID, UIIDArray, 5);
        UIDispData.ClassID = UIClassID ;

        
	if(g_scr_routes[UIClassID-1].create_func        == NULL ||  
           g_scr_routes[UIClassID-1].update_fun          == NULL ||  
           g_scr_routes[UIClassID-1].destroy_fun         ==NULL)
	{//any api is null renturn
		return;
	}
        createfunc = g_scr_routes[UIClassID-1].create_func;
        updatefun = g_scr_routes[UIClassID-1].update_fun;
        destroyfun = g_scr_routes[UIClassID-1].destroy_fun;

        //check this screen if has been created
	if( g_ui_common_param.classid != UIDispData.ClassID )
	{
		if(g_ui_common_param.win_id != 0)
		{
			destroyfun(g_ui_common_param.win_id);	
		}
		g_ui_common_param.win_id = createfunc(&UIDispData);	
                g_ui_common_param.classid = UIDispData.ClassID;
		updatefun(g_ui_common_param.win_id, &UIDispData);
	}
	else
	{
		updatefun(g_ui_common_param.win_id, &UIDispData);
	}
}


/*
	初始化olcd的显示任务

*/
#include "font_data.h"

type_MsgBody4UICtrlMsg disp_msg;
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

int init_emWin_Env()
{
	//ssz_locale_set_lang(kSszLangSimplifiedChinese);
	//ssz_locale_set_country_region(kSszCountryRegionChina);

	font_data_select_by_lang(kSszLangSimplifiedChinese);
	string_data_select_by_lang(kSszLangSimplifiedChinese);

	//init emWin component
	if( GUI_Init() != 0)
	{
	while(1);
	}

 }
INIT_ENV_EXPORT(init_emWin_Env);

static void display_UI(void *parameter)
{
	uint8_t index;
	uint8_t UIID = 0;
	
	GUI_UC_SetEncodeUTF8();
	GUI_SetBkColor(GUI_BLACK);
	WM_SetCallback(WM_HBKWIN, wm_bk_callback);
	GUI_SetColor(GUI_WHITE);

	GUI_SetFont(get_font(14));

	while(1)
	{
		index = (index + 1)%7;
		if(  index >= 7)
			index = 1 ;
		disp_msg.SItem.DataValArray[1] = index;      
		disp_msg.SItem.DataValArray[2] = index;
		disp_msg.SItem.DataValArray[3] = index;
		disp_msg.SItem.DataValArray[4] = index;

		disp_msg.ditem.DataValArray[1] = index;
		disp_msg.ditem.DataValArray[2] = index;
		disp_msg.ditem.DataValArray[3] = index;      
		disp_msg.ditem.DataValArray[4] = index;
		disp_msg.ditem.DataValArray[5] = index;

		disp_msg.CursorIndex.CursorType = index;

		disp_msg.SItem.DataValArray[5] = index;
		disp_msg.SItem.DataValArray[6] = index;
		disp_msg.SItem.DataValArray[7] = index;
		disp_msg.SItem.DataValArray[8] = index;
		disp_msg.SItem.DataValArray[9] = index;
		disp_msg.SItem.DataValArray[10] = index;
		disp_msg.SItem.DataValArray[11] = index;
		disp_msg.SItem.DataValArray[12] = index;
		disp_msg.SItem.DataValArray[13] = index;
		disp_msg.SItem.DataValArray[14] = index;
		disp_msg.SItem.DataValArray[15] = index;
		disp_msg.SItem.DataValArray[16] = index;
		disp_msg.SItem.DataValArray[21] = index;

		disp_msg.ditem.DataValArray[1] = index;
		disp_msg.ditem.DataValArray[2] = index;
		disp_msg.ditem.DataValArray[3] = index;
		disp_msg.CursorIndex.CursorIndex = index;
			
		UIID = (UIID+1)%3;
		if( UIID == 0 )
			UIID = 1 ;
		rt_kprintf("display ui, UIID = %d , Index = %d \n",UIID,index);
		ert_uiDispService( 3 ,disp_msg.UIID, &disp_msg.CursorIndex, &disp_msg.ditem, &disp_msg.SItem);
		
		rt_thread_mdelay(2000);
	}
}

static void enWin_server(void *parameter)
{

	while(1)
	{
		GUI_Exec();
		rt_thread_mdelay(10);
	}
}
	
static char thread_dispaly_stack[1024];
static struct rt_thread thread_display;
static char thread_emWin_server_stack[1024];
static struct rt_thread thread_emWin_server;

int display_thread_init( void )
{
	rt_thread_init(
				&thread_display,
				"display task",
				display_UI,
				RT_NULL,
				&thread_dispaly_stack[0],
				sizeof(thread_dispaly_stack),
				20,5);

	rt_thread_init(
				&thread_emWin_server,
				"enWin server task",
				enWin_server,
				RT_NULL,
				&thread_emWin_server_stack[0],
				sizeof(thread_emWin_server_stack),
				18,5);

	rt_thread_startup(&thread_emWin_server);
	rt_thread_startup(&thread_display);
				
	rt_kprintf("thread startup thread_display\n");

	return 0;
}
INIT_APP_EXPORT(display_thread_init);






