#pragma once
#include<rtwtypes.h>
//#include "rtdef.h"
//#include "rtthread.h"


#ifndef DEFINED_TYPEDEF_FOR_Type_MsgBody4UICtrlMsg_
#define DEFINED_TYPEDEF_FOR_Type_MsgBody4UICtrlMsg_




/*
    界面代号索引值，共五位。
    usage：36010
*/
typedef uint8_T* uiClassIDArray;  

/* 
    光标操作数据结构
    CursorType：光标类型，
        枚举值分别为：InverseSelect（反选）、Flash（下标闪烁）、none（无光标）
    CursorIndex：表示有光标操作时对应哪个动态控件；
    CurIndexMode：驱动层不适用。
 */
enum CursorTypeNum
{
    none = 0 ,
    InverseSelect,
    Flash,

    CursorTypeNumMax,
};
	
typedef struct _CursorDescriptor
{
    uint8_T CursorType;
    uint8_T CursorIndex;
    uint8_T CurIndexMod;
} curDescriptor;

/* 
    动态控件操作数据结构
    NumEffective：对应界面类型有几个有效数据。
    DataValArray[10]:结合NumEffective使用，每个数据对应动态空间显示的数值。
    DataModArray[10]：驱动层不适用。
 */
typedef struct _DItemDescriptor
{
    uint8_T NumEffective;
    uint8_T DataValArray[10];
    uint8_T DataModArray[10];
} dItemDescriptor;

/* 
    静态控件操作数据结构
    NumEffective：对应界面类型有几个有效数据。
    DataValArray[60]:结合NumEffective使用，每个数据对应静态控件的图标或者文字。
 */
typedef struct _SItemDescriptor
{
    uint8_T NumEffective;
    uint8_T DataValArray[60];
} sItemDescriptor;


typedef struct {
  uint8_T ClassID;
  uint8_T UIID[5];
  sItemDescriptor SItem;
  dItemDescriptor ditem;
  curDescriptor CursorIndex;
  uint8_T Reserved;
} Type_MsgBody4UICtrlMsg;

typedef Type_MsgBody4UICtrlMsg type_MsgBody4UICtrlMsg;

#endif

#ifdef ERT
    #define uiDisp(ptrData1,ptrData2)    ert_uiDisp(ptrData1,ptrData2)
    #include "ert_uiDisp.h"
#else
    #define uiDisp(ptrData1,ptrData2)    sim_uiDisp(ptrData1,ptrData2)  
    void sim_uiDisp(uint8_T UIOpCode, Type_MsgBody4UICtrlMsg* UIDispData);
#endif


