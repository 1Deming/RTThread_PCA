/*
 * COPYRIGHT (C) 2018, Real-Thread Information Technology Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-06-08     deming       the first version
 */

#include <rtthread.h>
#include <drivers/spi.h>
#include <drv_spi.h>

#define OLCD_DEBUG_ON

#ifdef OLCD_DEBUG_ON
#define OLCD_DEBUG         rt_kprintf("[OLCD]");rt_kprintf
//#define SPI_DEBUG         rt_kprintf("[SPI] ");rt_kprintf
#define SPI_DEBUG(...)
#else
#define OLCD_DEBUG(...)
#define SPI_DEBUG(...)
#endif 

/********************************* OLCD DRIVER **************************************/
#include "spi_olcd.h"
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>

#define MAX_ADDR_LEN 20
const char olcd_spi_name[50] = { "olcd_spi_device"} ;
#define SPI_MODE_SELECT  1
#define SPI_HARDWARE  SPI_MODE_SELECT


struct spi_olcd
{
    uint8_t is_init;
    struct rt_spi_device *rt_spi_device;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];  /* hw address   */
    rt_uint8_t  active;

};
static struct spi_olcd spi_olcd_device;
//static struct rt_spi_device olcd_spi_device;

#define OLCD_MODE_PIN    GET_PIN(D, 5)
#define OLCD_PWR_PIN    GET_PIN(D, 2)
#define OLCD_RESET_PIN    GET_PIN(B, 6)

void drv_oled_write_data_mode()
{
    rt_pin_mode(OLCD_MODE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_MODE_PIN, PIN_HIGH);
}
void drv_oled_write_command_mode()
{
    rt_pin_mode(OLCD_MODE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_MODE_PIN, PIN_LOW);
}

void drv_oled_pwr_en()
{
    rt_pin_mode(OLCD_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_PWR_PIN, PIN_HIGH);
    //ssz_gpio_set(OLED_PWR_EN_PORT,OLED_PWR_EN_PIN);
}
void drv_oled_pwr_dis()
{
    rt_pin_mode(OLCD_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_PWR_PIN, PIN_LOW);
    //ssz_gpio_clear(OLED_PWR_EN_PORT,OLED_PWR_EN_PIN);
}
void drv_oled_rest_en()
{
    rt_pin_mode(OLCD_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_RESET_PIN, PIN_LOW);
    //ssz_gpio_clear(OLED_RES_PORT,OLED_RES_PIN);
}
void drv_oled_rest_dis()
{
    rt_pin_mode(OLCD_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(OLCD_RESET_PIN, PIN_HIGH);
    //ssz_gpio_set(OLED_RES_PORT,OLED_RES_PIN);
}

//spi olcd设备最底层的驱动
rt_bool_t drv_oled_spi_write(void* buff, int buff_size)
{
    uint8_t *buffdata = buff;
//    rt_kprintf("spi write data :\n");
    // for(int i = 0 ; i < buff_size ; i ++)
    // {
    //     rt_kprintf(" %d,",buffdata[i]);
    // }
  //  rt_kprintf(" \n\nsend data over.\n");

    if( rt_spi_send(spi_olcd_device.rt_spi_device,buff,buff_size) == 0 )
        {
            OLCD_DEBUG("rt_spi_send failed!\n");
            return RT_FALSE;
        }

    return RT_TRUE;
}
void drv_oled_write_command(uint8_t command, uint8_t spi_mode)
{
    if(spi_mode == SPI_HARDWARE)
    {
        //drv_oled_cs_en();
        drv_oled_write_command_mode();
        __NOP();
        __NOP();
        drv_oled_spi_write(&command,1);
        __NOP();
        __NOP();
        //drv_oled_cs_dis();
    }
}
void drv_oled_write_data(uint8_t data, uint8_t spi_mode)
{
    if(spi_mode == SPI_HARDWARE)
    {
        //drv_oled_cs_en();
        drv_oled_write_data_mode();
        __NOP();
        __NOP();
        drv_oled_spi_write(&data,1);
        __NOP();
        __NOP();
        //drv_oled_cs_dis();
    }   
}
void drv_oled_write_datas(uint8_t *data, int data_size, uint8_t spi_mode)
{
	if (spi_mode == SPI_HARDWARE) {
		//drv_oled_cs_en();
		drv_oled_write_data_mode();
		drv_oled_spi_write(data,data_size);
		//drv_oled_cs_dis();
	}
}
// buff = 0x12 unlock; buff = 0x16 lock
void drv_oled_command_lock_mode(uint8_t buff)
{
    drv_oled_write_command(0xfd, SPI_MODE_SELECT);
    drv_oled_write_data(buff, SPI_MODE_SELECT);
}
//解锁命令
void drv_oled_conmmand_unlock()
{
    drv_oled_command_lock_mode(0x12);
}
//锁定之后只能接收解锁命令
void drv_oled_command_lock()
{
    drv_oled_command_lock_mode(0x16);
}
//buff = 0xae sleep on; buff = 0xaf sleep off
void drv_oled_sleep_mode(uint8_t buff)
{
    drv_oled_write_command(buff, SPI_MODE_SELECT);
}
void drv_oled_sleep_on()
{
    drv_oled_sleep_mode(0xAE);
}
void drv_oled_sleep_off()
{
    drv_oled_sleep_mode(0xAF);
}

//start = 0; end = 0xff
void drv_oled_set_column_start_end_addr(uint8_t start_addr, uint8_t end_addr)
{
    drv_oled_write_command(0x15, SPI_MODE_SELECT);
    drv_oled_write_data(start_addr, SPI_MODE_SELECT);
    drv_oled_write_data(end_addr, SPI_MODE_SELECT);
}
void drv_oled_set_row_start_end_addr(uint8_t start_addr, uint8_t end_addr)
{
    drv_oled_write_command(0x75, SPI_MODE_SELECT);
    drv_oled_write_data(start_addr, SPI_MODE_SELECT);
    drv_oled_write_data(end_addr, SPI_MODE_SELECT);
}

/**************************************************************
    re_map = A ;dual_com = B

    A[0]=0b, Horizontal address increment [reset]
    A[0]=1b, Vertical address increment
    
    A[1]=0b, Disable Column Address Re-map [reset]
    A[1]=1b, Enable Column Address Re-map
    
    A[2]=0b, Disable Nibble Re-map [reset]
    A[2]=1b, Enable Nibble Re-map
    
    A[4]=0b, Scan from COM0 to COM[N –1] [reset]
    A[4]=1b, Scan from COM[N-1] to COM0, where N is the Multiplex ratio
    
    A[5]=0b, Disable COM Split Odd Even [reset]
    A[5]=1b, Enable COM Split Odd Even

    B[4], Enable / disable Dual COM Line mode
    0b, Disable Dual COM mode [reset]
    1b, Enable Dual COM mode (MUX < 63)
*****************************************************************/
void drv_oled_set_Re_map_and_Dual_COM_line_mode(uint8_t re_map, uint8_t dual_com)
{
    drv_oled_write_command(0xA0, SPI_MODE_SELECT);
    drv_oled_write_data(re_map, SPI_MODE_SELECT);
    drv_oled_write_data(dual_com, SPI_MODE_SELECT);
}

void drv_oled_set_display_start_line(uint8_t start_line)
{
    drv_oled_write_command(0xA1, SPI_MODE_SELECT);
    drv_oled_write_data(start_line, SPI_MODE_SELECT);
}

void drv_oled_set_display_offset(uint8_t offset)
{
    drv_oled_write_command(0xA2, SPI_MODE_SELECT);
    drv_oled_write_data(offset, SPI_MODE_SELECT);
}

//function=0b, Select external VDD
//function=1b, Enable internal VDD regulator [reset]
void drv_oled_function_selection(uint8_t function)
{
    drv_oled_write_command(0xAB, SPI_MODE_SELECT);
    drv_oled_write_data(function, SPI_MODE_SELECT);
}

void drv_oled_set_phase_length(uint8_t phase_length)
{
    drv_oled_write_command(0xB1, SPI_MODE_SELECT);
    drv_oled_write_data(phase_length, SPI_MODE_SELECT);
}

void drv_oled_set_front_clock_divider_or_oscillator_frequency(uint8_t divset)
{
    drv_oled_write_command(0xB3, SPI_MODE_SELECT);
    drv_oled_write_data(divset, SPI_MODE_SELECT);
}

void drv_oled_set_display_enhancement_A(uint8_t enhanc_1, uint8_t enhanc_2)
{
    drv_oled_write_command(0xB4, SPI_MODE_SELECT);
    drv_oled_write_data(enhanc_1, SPI_MODE_SELECT);
    drv_oled_write_data(enhanc_2, SPI_MODE_SELECT);
}

void drv_oled_set_gpio(uint8_t gpio)
{
    drv_oled_write_command(0xB5, SPI_MODE_SELECT);
    drv_oled_write_data(gpio, SPI_MODE_SELECT);
}

void drv_oled_set_second_precharge_period(uint8_t period)
{
    drv_oled_write_command(0xB6, SPI_MODE_SELECT);
    drv_oled_write_data(period, SPI_MODE_SELECT);
}

void drv_oled_select_default_linear_gray_scale_table()
{
    drv_oled_write_command(0xB9, SPI_MODE_SELECT);
}

void drv_oled_set_pre_charge_voltage(uint8_t vol)
{
    drv_oled_write_command(0xBB, SPI_MODE_SELECT);
    drv_oled_write_data(vol, SPI_MODE_SELECT);
}

void drv_oled_set_Vcomh(uint8_t vol_level)
{
    drv_oled_write_command(0xBE, SPI_MODE_SELECT);
    drv_oled_write_data(vol_level, SPI_MODE_SELECT);
}

void drv_oled_set_contrast_current(uint8_t cur)
{
    drv_oled_write_command(0xC1, SPI_MODE_SELECT);
    drv_oled_write_data(cur, SPI_MODE_SELECT);
}

void drv_oled_set_master_contrast_current_control(uint8_t master_cur)
{
    drv_oled_write_command(0xC7, SPI_MODE_SELECT);
    drv_oled_write_data(master_cur, SPI_MODE_SELECT);
}

void drv_oled_set_MUX_ratio(uint8_t ratio)
{
    drv_oled_write_command(0xCA, SPI_MODE_SELECT);
    drv_oled_write_data(ratio, SPI_MODE_SELECT);
}

void drv_oled_set_display_enhancement_B(uint8_t enhanc_1, uint8_t enhanc_2)
{
    drv_oled_write_command(0xD1, SPI_MODE_SELECT);
    drv_oled_write_data(enhanc_1, SPI_MODE_SELECT);
    drv_oled_write_data(enhanc_2, SPI_MODE_SELECT);
}

/****************************************************
mode = A4h = Entire Display OFF, all pixels turns OFF in GS level 0
mode = A5h = Entire Display ON, all pixels turns ON in GS level 15
mode = A6h = Normal Display [reset]
mode = A7h = Inverse Display 
****************************************************/
void drv_oled_set_display_mode(uint8_t mode)
{
    drv_oled_write_command(mode, SPI_MODE_SELECT);
}

void drv_oled_init()
{
    OLCD_DEBUG("drv olcd init start!\n");

	if(spi_olcd_device.is_init)
		return ;

	spi_olcd_device.is_init = RT_TRUE;
    
    OLCD_DEBUG("register spi olcd device\n");
    //挂载spi设备
    if( rt_hw_spi_device_attach("spi2",olcd_spi_name,GPIOB,GPIO_PIN_12) != RT_EOK )
		{
			 OLCD_DEBUG("olcd spi device can not be find!\n");
		}
    else
    {
			spi_olcd_device.rt_spi_device = (struct rt_spi_device*)rt_device_find(olcd_spi_name);
         
    }
    OLCD_DEBUG("start init olcd hardware\n");  

hardware:   
    OLCD_DEBUG("config spi bus hardware\n");  
    //配置SPI外设
    struct rt_spi_configuration  spi_configuration =		
    {
        .mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB,
        .data_width = 8,
        .max_hz = 16 * 1000 * 1000 ,   //16M
    };
    rt_spi_configure(spi_olcd_device.rt_spi_device,&spi_configuration);

    //初始化spi硬件设备
    drv_oled_pwr_en();
    drv_oled_rest_en();
    rt_thread_mdelay(200);
    drv_oled_rest_dis();
    rt_thread_mdelay(200);
    drv_oled_conmmand_unlock();
    //drv_oled_sleep_on();
    drv_oled_set_column_start_end_addr(0x1c, 0x5b);
    drv_oled_set_row_start_end_addr(0x00, 0x3f);
    drv_oled_set_Re_map_and_Dual_COM_line_mode(0x14, 0x11);
    drv_oled_set_display_start_line(0x00);
    drv_oled_set_display_offset(0x00);
    drv_oled_function_selection(0x01);//enable internal VDD 00:disable
    drv_oled_set_phase_length(0xE2);
    drv_oled_set_front_clock_divider_or_oscillator_frequency(0x91);
    drv_oled_set_display_enhancement_A(0xA0, 0xFD);
    //drv_oled_set_gpio(0x00);
    drv_oled_set_second_precharge_period(0x0f);
    drv_oled_select_default_linear_gray_scale_table();
    drv_oled_set_pre_charge_voltage(0x1F);
    drv_oled_set_Vcomh(0x07);
    drv_oled_set_contrast_current(0x20); //  0xFf
    drv_oled_set_master_contrast_current_control(0x0f);
    drv_oled_set_MUX_ratio(0x3f);
    drv_oled_set_display_enhancement_B(0x82, 0x20);
    drv_oled_set_display_mode(0xA6);
    drv_oled_clear_all_screen();
    drv_oled_sleep_off();

    OLCD_DEBUG("init olcd hardware finish\n");  
}
void drv_oled_deinit()
{
	spi_olcd_device.is_init  = RT_FALSE;
}

//直接全屏数据到硬件中
void drv_oled_clear_all_screen()
{
    int i,j;
    drv_oled_set_column_start_end_addr(0x1c, 0x5b);
    drv_oled_set_row_start_end_addr(0x00, 0x3f);
    drv_oled_write_command(0x5C, SPI_MODE_SELECT);

    for(i = 0; i < 64; i++){
        for(j = 0; j < 128; j++){
            drv_oled_write_data(0x00, SPI_MODE_SELECT);
        }
    }
}
void drv_oled_display_all_screen()
{
    int i,j;

    drv_oled_set_column_start_end_addr(0x1c, 0x5b);
    drv_oled_set_row_start_end_addr(0x00, 0x3f);
    drv_oled_write_command(0x5C, SPI_MODE_SELECT);

    for(i = 0; i < 64; i++){
        for(j = 0; j < 128; j++){
            drv_oled_write_data(0xff, SPI_MODE_SELECT);
        }
    }
}


//列地址增加的方式，每次点亮两个点，共128(256/2)个
//x代表行，y代表列
void drv_oled_display_text_at_pos(int col, int row, uint8_t* str, int size)
{
    int i;
    
    drv_oled_set_column_start_end_addr(col+0x1c, 0x5b);//COLUMN_MAX
    drv_oled_set_row_start_end_addr(row, ROW_MAX);
    drv_oled_write_command(0x5C, SPI_MODE_SELECT);

    for(i = 0; i < size; i++){
        drv_oled_write_data(str[i], SPI_MODE_SELECT);
    }
}

//set start and end addr
void drv_oled_set_pos(int col, int row)
{
	//set column start address
	drv_oled_write_command(0x15, SPI_MODE_SELECT);
	drv_oled_write_data(col+0x1C, SPI_MODE_SELECT);

	//set row start address
	drv_oled_write_command(0x75, SPI_MODE_SELECT);
	drv_oled_write_data(row, SPI_MODE_SELECT);

    //drv_oled_set_column_start_end_addr(col+0x1c, 0x5b);
    //drv_oled_set_row_start_end_addr(row, ROW_MAX);
}

//enable write RAM 
void drv_oled_write_RAM_en()
{
    drv_oled_write_command(0x5C, SPI_MODE_SELECT);
}

//write the data
void drv_oled_write_byte(uint8_t data)
{
    drv_oled_write_data(data, SPI_MODE_SELECT);
}

void drv_oled_write_bytes(uint8_t * data, int data_size)
{
	drv_oled_write_datas(data, data_size, SPI_MODE_SELECT);
}

void drv_oled_all_pixels_on()
{
    drv_oled_write_command(0xAF, SPI_MODE_SELECT); //display on
    drv_oled_write_command(0xA5, SPI_MODE_SELECT); //display all pixels
}

void drv_oled_all_pixels_off()
{
    drv_oled_write_command(0xA4, SPI_MODE_SELECT); //all pixels off
    //drv_oled_write_command(0xAE, SPI_MODE_SELECT); //display off
    drv_oled_set_display_mode(0xA6); //set to normal mode
}

 
// /********************************* RT-Thread OLCD interface begin **************************************/
// static rt_err_t spi_olcd_init(rt_device_t dev)
// {
//     OLCD_DEBUG("call spi_olcd_init function.\n");
//     return RT_EOK;
// }

// static rt_err_t spi_olcd_open(rt_device_t dev, rt_uint16_t oflag)
// {
//     OLCD_DEBUG("call spi_olcd_open function.\n");
//     return RT_EOK;
// }

// static rt_err_t spi_olcd_close(rt_device_t dev)
// {
//     OLCD_DEBUG("call spi_olcd_close function.\n");
//     return RT_EOK;
// }

// static rt_size_t spi_olcd_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
// {
//     OLCD_DEBUG("call spi_olcd_read function.\n");
//     //rt_set_errno(-RT_ENOSYS);
//     return 0;
// }

// static rt_size_t spi_olcd_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
// {
//     OLCD_DEBUG("call spi_olcd_write function.\n");
//     //rt_set_errno(-RT_ENOSYS);
//     return 0;
// }

// static rt_err_t spi_olcd_control(rt_device_t dev, int cmd, void *args)
// {
//     OLCD_DEBUG("call spi_olcd_control function.\n");
//     //struct rw009_wifi *wifi_device = (struct rw009_wifi *)dev;
//     rt_err_t result = RT_EOK;

//     return result;
// }
// /********************************* RT-Thread OLCD interface end **************************************/

// static void spi_wifi_data_thread_entry(void *parameter)
// {
    
//     //rt_err_t result;

//     while (1)
//     {
// 			;
// 		}
// }

// #ifdef RT_USING_DEVICE_OPS
// const static struct rt_device_ops spi_olcd_ops =
// {
//     spi_olcd_init,
//     spi_olcd_open,
//     spi_olcd_close,
//     spi_olcd_read,
//     spi_olcd_write,
//     spi_olcd_control
// };
// #endif 
// 
/*
    将spi设备挂载在总线上（spi总线设备在spi总线注册时已经初始化完成）。
    初始化spi设备外设。
    开始工作。
*/
// rt_err_t rt_hw_olcd_init(const char *spi_device_name)
// {
 

//     return RT_EOK;
// }

//添加fish调试指令
#ifdef RT_USING_FINSH
#include <finsh.h>

static rt_err_t spi_lcd_flush(void)
{
	rt_err_t result;

   return result;
}
//FINSH_FUNCTION_EXPORT(spi_lcd_flush, flush all screen data into hardware.);
FINSH_FUNCTION_EXPORT(drv_oled_init, init olcd spi device and hardware device.);
FINSH_FUNCTION_EXPORT_ALIAS(drv_oled_display_all_screen,display_allUI,display all screen.);
FINSH_FUNCTION_EXPORT_ALIAS(drv_oled_clear_all_screen,clear_allUI,clear all screen.);

#endif // RT_USING_FINSH
