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
static UI_ScreenCommon g_ui_common_param;

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


































