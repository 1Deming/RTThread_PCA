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
#include "display.h"
#include "string.h"
#include "spi_olcd.h"
#include "rtthread.h"

/************************************************
* Declaration
************************************************/
typedef struct  {
	int16_t x0;
	int16_t y0;
	int16_t x1;
	int16_t y1;
}Rect;

/************************************************
* Variable 
************************************************/
static uint8_t g_display_cache[64][128];//64 row,128*2 column
static Rect g_display_dirty_rect;
static rt_bool_t g_display_is_dirty = RT_FALSE;
static rt_bool_t g_display_is_opened = RT_FALSE;
static rt_sem_t olcd_flush_sem = RT_NULL;

/************************************************
* Function 
************************************************/
//get the merged rect which can contain two rect
void ui_rect_merge(Rect* dest, const Rect* src)
{
	if (dest->x0 > src->x0) {
		dest->x0 = src->x0;
	}
	if (dest->y0 > src->y0) {
		dest->y0 = src->y0;
	}
	if (dest->x1 < src->x1) {
		dest->x1 = src->x1;
	}
	if (dest->y1 < src->y1) {
		dest->y1 = src->y1;
	}
}

//flush the display cache to lcd
void display_flush(int x0, int y0, int x1, int y1)
{
//	if (!g_display_is_opened) {
//		display_open();
//	}
    int x_index_start;
    int x_index_end;
	int x_pos;

	if (x0<0) {
		x0 = 0;
	}
	if (y0 < 0) {
		y0 = 0;
	}
	if (x1 > DISPLAY_WIDTH - 1) {
		x1 = DISPLAY_WIDTH - 1;
	}
	if (y1 > DISPLAY_HEIGHT- 1) {
		y1 = DISPLAY_HEIGHT - 1;
	}

	//start pos must at edge of 4
	x_pos = x0 / 4;
	x0 = x_pos*4;
	
    x_index_start = x0/2;
    x_index_end = x1/2;

	//must write 4 pixel as one unit, so need increase it if need
	if ((x_index_end-x_index_start)%2!=1) {
		x_index_end++;
	}

    for(int y=y0; y<=y1; y++)
    {
        //DrvLcdSetPos(x, y);
				drv_oled_set_pos(x_pos, y);
				drv_oled_write_RAM_en();
				
				drv_oled_write_bytes(&g_display_cache[y][x_index_start], (x_index_end-x_index_start+1));
    }
    
}
//flush the dirty area to lcd
static void display_flush_dirty(void *parameter)
{
	while(1)
	{
		//获取信号量
		rt_sem_take(olcd_flush_sem,RT_WAITING_FOREVER);

		if(g_display_is_dirty)
		{
			g_display_is_dirty = RT_FALSE;
			display_flush(g_display_dirty_rect.x0, g_display_dirty_rect.y0, 
				g_display_dirty_rect.x1, g_display_dirty_rect.y1);
		}
	}
}
//is exist dirty area
rt_bool_t display_is_dirty()
{
    return g_display_is_dirty;
}
//set dirty area
void display_set_dirty(int x, int y, int width, int height)
{
	if (width<=0 || height<=0) {
		return;
	}
    if(!g_display_is_dirty)
    {
        g_display_is_dirty = RT_TRUE;
		ui_rect_init_by_size(g_display_dirty_rect, x, y, width, height);
		//发送信号量，触发olcd刷新任务
		rt_sem_release(olcd_flush_sem);
    }
    else
    {
		Rect tmp;
		ui_rect_init_by_size(tmp, x, y, width, height);
		ui_rect_merge(&g_display_dirty_rect, &tmp);
    }
}

//open display for show, when need show, it will auto open
void display_open() {
	if (!g_display_is_opened) {
		//drv_oled_mcu_io_as_gpio();		
		g_display_is_opened = RT_TRUE;
		drv_oled_pwr_en();
		drv_oled_init();		
		drv_oled_sleep_off();
	}
}

//close display for reduce power
void display_close() {
	if (g_display_is_opened) {
		g_display_is_opened = RT_FALSE;
		drv_oled_sleep_on();
		drv_oled_pwr_dis();
		//drv_oled_mcu_io_as_analog();		
	}
}

void display_data_standard(int x, int y, const uint8_t * pixels, int pixels_count) {
	int pixel_index;
	int pixel_x_index_at_cache;
	uint8_t one_pixel;
	for (int i = 0; i < pixels_count; i++) {
		pixel_index = i / 2;
		if (i % 2 == 0) {
			//get 4 high bit
			one_pixel = (pixels[pixel_index] & 0xF0) >> 4;
		}
		else {
			//get 4 low bit
			one_pixel = pixels[pixel_index] & 0x0F;
		}
		pixel_x_index_at_cache = (x + i) / 2;
		if ((x + i) % 2 == 0) {
			//clear 4 high bit as 0
			g_display_cache[y][pixel_x_index_at_cache] &= 0x0F;
			g_display_cache[y][pixel_x_index_at_cache] |= (one_pixel << 4);
		}
		else {
			//clear 4 low bit as 0
			g_display_cache[y][pixel_x_index_at_cache] &= 0xF0;
			g_display_cache[y][pixel_x_index_at_cache] |= (one_pixel);
		}
	}
}
int display_data_quick(int x, int y, const uint8_t * pixels, int pixels_count) {
	//can copy directly
	memcpy(&g_display_cache[y][x], pixels, pixels_count / 2);
	return pixels_count - pixels_count % 2;
}

uint8_t display_data_get(int x, int y)
{
	uint8_t data = 0;
	int pixel_x_index_at_cache;

	pixel_x_index_at_cache = x / 2;
	if ((x % 2) == 0) {
		//clear 4 high bit as 0
		data = (g_display_cache[y][pixel_x_index_at_cache]>>4)&0x0F;
	}
	else {
		//clear 4 low bit as 0
		data = g_display_cache[y][pixel_x_index_at_cache]&0x0F;
	}
	return data;
}


void display_data_at(int x, int y, const uint8_t * pixels, int pixels_count)
{
	uint8_t data ;
	
	if( *pixels > 0 )
		data = 0x0F;
	else
		data = 0 ;
	
	if( (x%2) == 0 )
	{
		//高四位
			g_display_cache[y][x/2] &= 0x0F;
			g_display_cache[y][x/2] |= ( data << 4 );
	}
	else
	{
		//低四位
			g_display_cache[y][x/2] &= 0xF0;
			g_display_cache[y][x/2] |= ( data );
	}
	
	display_set_dirty(x, y, pixels_count, 1);
}

int display_init()
{
	//初始化信号量
	olcd_flush_sem = rt_sem_create("uiFlushSem",0,RT_IPC_FLAG_PRIO);
	if( olcd_flush_sem == RT_NULL )
	{
		rt_kprintf("create olcd flush sem failed!\n");
	}
	else
	{
		rt_kprintf("create olcd flush sem finsh!\n");
	}
	
	return 0;
}
INIT_COMPONENT_EXPORT(display_init);

static char thread_olcd_flush_stack[512];
static struct rt_thread thread_olcd;
int olcd_flush_thread_init( void )
{
	rt_thread_init(
				&thread_olcd,
				"olcd flush task",
				display_flush_dirty,
				RT_NULL,
				&thread_olcd_flush_stack[0],
				sizeof(thread_olcd_flush_stack),
				25,5);

	rt_thread_startup(&thread_olcd);

	rt_kprintf("thread startup thread_olcd\n");

	return 0;
}
INIT_APP_EXPORT(olcd_flush_thread_init);

//添加fish调试指令
#ifdef RT_USING_FINSH
#include <finsh.h>

static rt_err_t lcd_flush(void)
{
	rt_err_t result;
    
	//display_flush(0,0,256,64);
   display_flush_dirty(0);

   return result;
}
FINSH_FUNCTION_EXPORT(lcd_flush, flush all screen data into hardware.);

static rt_err_t display_data(uint8_t range)
{
	rt_err_t result;
	uint8_t display_cache[128];
	
	memset(display_cache,0,sizeof(display_cache));
	for( int j = 0 ; j < 128 ; j = j + range )
	{
		display_cache[j] = 0x0F;
	}


	for( int i = 0 ; i < 64 ; i = i + 1 )
	{
	//	for( int j = 0 ; j < 128 ; j = j + 1 )
	//	{
				display_data_at(0,i,display_cache,256);
	//	}
	}	
   return result;
}
FINSH_FUNCTION_EXPORT(display_data, set data at x-y);

void printfdata()
{
	rt_kprintf("[enWin]: x,y,PixelIndex:\n");
	for(int y = 0 ; y < 64; y ++)
	{
		for(int x = 0 ; x < 128; x ++ )
		{
			if(g_display_cache[y][x] > 0)
				rt_kprintf("x");
			else
				rt_kprintf(".");
		}
		rt_kprintf("\n");
	}
	
}
FINSH_FUNCTION_EXPORT(printfdata, display emWin data);



#endif // RT_USING_FINSH
