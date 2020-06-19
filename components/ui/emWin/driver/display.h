/************************************************
* DESCRIPTION:
*
*
* REVISION HISTORY:
*   Rev 1.0 2017-08-04 xqzhao
*   Rev 1.1 2020-06-11  deming
* Initial revision.
*
************************************************/
#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED


#include "rtthread.h"
#include "rtdef.h"

/************************************************
* Declaration
************************************************/
#ifdef __cplusplus
extern "C" {
#endif

	
#define DISPLAY_WIDTH 256 
#define DISPLAY_HEIGHT 64 

#define ssz_assert //  
	
#define ui_rect_xsize(rect) ((rect).x1-(rect).x0 + 1)
#define ui_rect_ysize(rect) ((rect).y1-(rect).y0 + 1)	
#define ui_rect_init_by_size(rect,x,y,xsize,ysize) {(rect).x0=(x);(rect).y0=(y); \
	(rect).x1=(x)+(xsize)-1;(rect).y1=(y)+(ysize)-1;}

	
//flush the display cache to lcd
void display_flush(int x0, int y0, int x1, int y1);
//flush the dirty area to lcd
//void display_flush_dirty(void);
//is exist dirty area
rt_bool_t display_is_dirty(void);
//set dirty area
void display_set_dirty(int x, int y, int width, int height);

//open display for show, when need show, it will auto open
void display_open(void);

//close display for reduce power
void display_close(void);


uint8_t display_data_get(int x, int y);
void display_data_at(int x, int y, const uint8_t *pixels, int pixels_count);
int display_init(void);


#ifdef __cplusplus
}
#endif

#endif

