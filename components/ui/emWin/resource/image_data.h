#pragma once
//#include "ssz_def.h"
#include "GUI.h"

/************************************************
* Declaration
************************************************/


#ifdef __cplusplus
extern "C" {
#endif
extern GUI_CONST_STORAGE GUI_BITMAP bmAC;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_AC;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_empty;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_full;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_no;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_one;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_three;
extern GUI_CONST_STORAGE GUI_BITMAP bmbattery_two;
extern GUI_CONST_STORAGE GUI_BITMAP bmerror;
extern GUI_CONST_STORAGE GUI_BITMAP bmexhaust;
extern GUI_CONST_STORAGE GUI_BITMAP bmexhaust_pause;
extern GUI_CONST_STORAGE GUI_BITMAP bmexhaust_run;
extern GUI_CONST_STORAGE GUI_BITMAP bmflush_max_dose_selected;
extern GUI_CONST_STORAGE GUI_BITMAP bmflush_max_dose_unselected;
extern GUI_CONST_STORAGE GUI_BITMAP bmhistory_log;
extern GUI_CONST_STORAGE GUI_BITMAP bminfo_select;
extern GUI_CONST_STORAGE GUI_BITMAP bminfo_unselect;
extern GUI_CONST_STORAGE GUI_BITMAP bminfusion_pause;
extern GUI_CONST_STORAGE GUI_BITMAP bminfusion_run;
extern GUI_CONST_STORAGE GUI_BITMAP bminfusion_stop;
extern GUI_CONST_STORAGE GUI_BITMAP bmleft_arrow;
extern GUI_CONST_STORAGE GUI_BITMAP bmlock;
extern GUI_CONST_STORAGE GUI_BITMAP bmlogo;
extern GUI_CONST_STORAGE GUI_BITMAP bmmode_selected;
extern GUI_CONST_STORAGE GUI_BITMAP bmmode_unselected;
extern GUI_CONST_STORAGE GUI_BITMAP bmmute;
extern GUI_CONST_STORAGE GUI_BITMAP bmmute_small;
extern GUI_CONST_STORAGE GUI_BITMAP bmnew_task;
extern GUI_CONST_STORAGE GUI_BITMAP bmpeak_set_select;
extern GUI_CONST_STORAGE GUI_BITMAP bmpeak_set_unselect;
extern GUI_CONST_STORAGE GUI_BITMAP bmright_arrow;
extern GUI_CONST_STORAGE GUI_BITMAP bmselected_pointer;
extern GUI_CONST_STORAGE GUI_BITMAP bmselect_1;
extern GUI_CONST_STORAGE GUI_BITMAP bmselect_2;
extern GUI_CONST_STORAGE GUI_BITMAP bmset_arrow;
extern GUI_CONST_STORAGE GUI_BITMAP bmsound_pause;
extern GUI_CONST_STORAGE GUI_BITMAP bmstop_infusion;
extern GUI_CONST_STORAGE GUI_BITMAP bmsystem_setting;
extern GUI_CONST_STORAGE GUI_BITMAP bmtime_select;
extern GUI_CONST_STORAGE GUI_BITMAP bmtime_unselect;
extern GUI_CONST_STORAGE GUI_BITMAP bmunselected_pointer;
extern GUI_CONST_STORAGE GUI_BITMAP bmwarning;
extern GUI_CONST_STORAGE GUI_BITMAP bmwarning_common;

//add by deming, 2020/05/27
//extern GUI_CONST_STORAGE GUI_BITMAP bmsetmenu_unit;
extern GUI_CONST_STORAGE GUI_BITMAP bmsetmenu_version;
//extern GUI_CONST_STORAGE GUI_BITMAP bmunlockfail;
//extern GUI_CONST_STORAGE GUI_BITMAP bmworktime;
//extern GUI_CONST_STORAGE GUI_BITMAP bmAutoCycle;
//extern GUI_CONST_STORAGE GUI_BITMAP bmAutoCycle_breaktime;
//extern GUI_CONST_STORAGE GUI_BITMAP bmAutoCycle_effectivetime;
//extern GUI_CONST_STORAGE GUI_BITMAP bmAutoCycle_rate;
//extern GUI_CONST_STORAGE GUI_BITMAP bmAutoCycle_worktime;
//extern GUI_CONST_STORAGE GUI_BITMAP bmBKGD;
//extern GUI_CONST_STORAGE GUI_BITMAP bmBKGD_KVORate;
//extern GUI_CONST_STORAGE GUI_BITMAP bmBKGD_Rate;
//extern GUI_CONST_STORAGE GUI_BITMAP bmBKGD_VOL;
////extern GUI_CONST_STORAGE GUI_BITMAP bmlog_alarm;
////extern GUI_CONST_STORAGE GUI_BITMAP bmlog_opeartion;
////extern GUI_CONST_STORAGE GUI_BITMAP bmpasswordchange;
////extern GUI_CONST_STORAGE GUI_BITMAP bmpasswordnotchange;
//extern GUI_CONST_STORAGE GUI_BITMAP bmPCA;
//extern GUI_CONST_STORAGE GUI_BITMAP bmPCA_first;
//extern GUI_CONST_STORAGE GUI_BITMAP bmPCA_regular;
//extern GUI_CONST_STORAGE GUI_BITMAP bmPCA_time;
////extern GUI_CONST_STORAGE GUI_BITMAP bmpoweroffcheck;
////extern GUI_CONST_STORAGE GUI_BITMAP bmsetmenu_passwordn;
//extern GUI_CONST_STORAGE GUI_BITMAP bmsetmenu_time;


typedef enum
{
	kImgAc,
	kImgBatteryAc,
	kImgBatteryEmpty,
	kImgBatteryFull,
	kImgBatteryNo,
	kImgBatteryOne,
	kImgBatteryThree,
	kImgBatteryTwo,
	kImgError,
	kImgExhaust,
	kImgExhaustPause,
	kImgExhaustRun,
	kImgFlushMaxDoseSelected,
	kImgFlushMaxDoseUnselected,
	kImgHistoryLog,
	kImgInfoSelect,
	kImgInfoUnselect,
	kImgInfusionPause,
	kImgInfusionRun,
	kImgInfusionStop,
	kImgLeftArrow,
	kImgLock,
	kImgLogo,
	kImgModeSelected,
	kImgModeUnselected,
	kImgMute,
	kImgMuteSmall,
	kImgNewTask,
	kImgPeakSetSelect,
	kImgPeakSetUnselect,
	kImgRightArrow,
	kImgSelectedPointer,
	kImgSelect1,
	kImgSelect2,
	kImgSetArrow,
	kImgSoundPause,
	kImgStopInfusion,
	kImgSystemSetting,
	kImgTimeSelect,
	kImgTimeUnselect,
	kImgUnselectedPointer,
	kImgWarning,
	kImgWarningCommon,

	//add by deming, 2020/05/27
	
	kImgsetmenu_version,
	kImgAutoCycle,
	kImgAutoCycle_breaktime,
	kImgAutoCycle_effectivetime,
	kImgAutoCycle_rate,
	kImgAutoCycle_worktime,
	kImgBKGD,
	kImgBKGD_KVORate,
	kImgBKGD_Rate,
	kImgBKGD_VOL,
	kImgPCA,
	kImgPCA_first,
	kImgPCA_regular,
	kImgPCA_time,
        
        
//	kImgpasswordchange,
//	kImgpasswordnotchange,	
        //kImgsetmenu_passwordn,
	//kImgsetmenu_time,
	//kImglog_alarm,
	//kImglog_opeartion,
	//kImgpoweroffcheck,
	//kImgunlockfail,   
        //kImgsetmenu_unit,
        
        
	kImgIDMax,
 }ImgID;

//get image from the image id
GUI_CONST_STORAGE GUI_BITMAP* get_image(ImgID img_id);

#ifdef __cplusplus
}
#endif

