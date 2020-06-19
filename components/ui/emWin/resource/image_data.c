/************************************************
*
* NAME: image_data.c
*
* DESCRIPTION:
*
*
* REVISION HISTORY:
*   Rev 1.0 2017-3-2 ryan
* Initial revision.
*
************************************************/
#include "image_data.h"
#include "string.h"

/************************************************
* Declaration
************************************************/
typedef struct
{
	ImgID id;
	GUI_CONST_STORAGE GUI_BITMAP* data;
}ImgData;


/************************************************
* Variable 
************************************************/
//#include "generated_image_data.h"
const static ImgData g_image_data[] =
{
  
	{kImgAc, &bmAC},
	{kImgBatteryAc, &bmbattery_AC},
	{kImgBatteryEmpty, &bmbattery_empty},
	{kImgBatteryFull, &bmbattery_full},
	{kImgBatteryNo, &bmbattery_no},
	{kImgBatteryOne, &bmbattery_one},
	{kImgBatteryThree, &bmbattery_three},
	{kImgBatteryTwo, &bmbattery_two},
	{kImgError, &bmerror},
	{kImgExhaust, &bmexhaust},
	{kImgExhaustPause, &bmexhaust_pause},
	{kImgExhaustRun, &bmexhaust_run},
	{kImgFlushMaxDoseSelected, &bmflush_max_dose_selected},
	{kImgFlushMaxDoseUnselected, &bmflush_max_dose_unselected},
	{kImgHistoryLog, &bmhistory_log},
	{kImgInfoSelect, &bminfo_select},
	{kImgInfoUnselect, &bminfo_unselect},
	{kImgInfusionPause, &bminfusion_pause},
	{kImgInfusionRun, &bminfusion_run},
	{kImgInfusionStop, &bminfusion_stop},
	{kImgLeftArrow, &bmleft_arrow},
	{kImgLock, &bmlock},
	{kImgLogo, &bmlogo},
	{kImgModeSelected, &bmmode_selected},
	{kImgModeUnselected, &bmmode_unselected},
	{kImgMute, &bmmute},
	{kImgMuteSmall, &bmmute_small},
	{kImgNewTask, &bmnew_task},
	{kImgPeakSetSelect, &bmpeak_set_select},
	{kImgPeakSetUnselect, &bmpeak_set_unselect},
	{kImgRightArrow, &bmright_arrow},
	{kImgSelectedPointer, &bmselected_pointer},
	{kImgSelect1, &bmselect_1},
	{kImgSelect2, &bmselect_2},
	{kImgSetArrow, &bmset_arrow},
	{kImgSoundPause, &bmsound_pause},
	{kImgStopInfusion, &bmstop_infusion},
	{kImgSystemSetting, &bmsystem_setting},
	{kImgTimeSelect, &bmtime_select},
	{kImgTimeUnselect, &bmtime_unselect},
	{kImgUnselectedPointer, &bmunselected_pointer},
	{kImgWarning, &bmwarning},
	{kImgWarningCommon, &bmwarning_common},

	//add by deming, 2020/05/27

//	{kImgsetmenu_version, &bmsetmenu_version},
//	{kImgAutoCycle, &bmAutoCycle},
//	{kImgAutoCycle_breaktime, &bmAutoCycle_breaktime},
//	{kImgAutoCycle_effectivetime, &bmAutoCycle_effectivetime},
//	{kImgAutoCycle_rate, &bmAutoCycle_rate}, 
//	{kImgAutoCycle_worktime, &bmAutoCycle_worktime},
//	{kImgBKGD, &bmBKGD},
//	{kImgBKGD_KVORate, &bmBKGD_KVORate},
//	{kImgBKGD_Rate, &bmBKGD_Rate},
//	{kImgBKGD_VOL, &bmBKGD_VOL},
//	{kImgPCA, &bmPCA},
//	{kImgPCA_first, &bmPCA_first},
//	{kImgPCA_regular, &bmPCA_regular},
//	{kImgPCA_time, &bmPCA_time},

//	{kImgsetmenu_unit, &bmsetmenu_unit},
//	{kImgpasswordchange, &bmpasswordchange},
//	{kImgpasswordnotchange, &bmpasswordnotchange},	
//	{kImgsetmenu_passwordn, &bmsetmenu_passwordn},
//	{kImgsetmenu_time, &bmsetmenu_time},
//	{kImglog_alarm, &bmlog_alarm},
//	{kImglog_opeartion, &bmlog_opeartion},
//	{kImgpoweroffcheck, &bmpoweroffcheck},
//	{kImgunlockfail, &bmunlockfail},     
};


/************************************************
* Function 
************************************************/

//get image from the image id
GUI_CONST_STORAGE GUI_BITMAP* get_image(ImgID img_id)
{
    if (img_id >= kImgIDMax)
    {
        return NULL;
    }
    return g_image_data[img_id].data;
}


