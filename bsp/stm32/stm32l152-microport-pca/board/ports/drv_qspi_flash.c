/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     zylx         first version
 */
 
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

#ifdef BSP_USING_QSPI_FLASH

#include "spi_flash.h"
#include "spi_flash_sfud.h"

#define FLASH_DEBUG_ON

#ifdef FLASH_DEBUG_ON
#define FLASH_DEBUG         rt_kprintf("[FLASH]");rt_kprintf
#else
#define FLASH_DEBUG(...)   

#endif 

char w25qxx_read_status_register2(struct rt_spi_device *device)
{
    /* 0x35 read status register2 */
    char instruction = 0x35, status;

    rt_spi_send_then_recv(device, &instruction, 1, &status, 1);

    return status;
}

void w25qxx_write_enable(struct rt_spi_device *device)
{
    /* 0x06 write enable */
    char instruction = 0x06;

    rt_spi_send(device, &instruction, 1);
}

void w25qxx_enter_qspi_mode(struct rt_spi_device *device)
{
    char status = 0;
    /* 0x38 enter qspi mode */
    char instruction = 0x38;
    char write_status2_buf[2] = {0};

    /* 0x31 write status register2 */
    write_status2_buf[0] = 0x31;

    status = w25qxx_read_status_register2(device);
    if (!(status & 0x02))
    {
        status |= 1 << 1;
        w25qxx_write_enable(device);
        write_status2_buf[1] = status;
        rt_spi_send(device, &write_status2_buf, 2);
        rt_spi_send(device, &instruction, 1);
        rt_kprintf("flash already enter qspi mode\n");
        rt_thread_mdelay(10);
    }
}

#define FLASH_PWR_EN    GET_PIN(A, 12)
#define FLASH_CE    		GET_PIN(D, 0)
#define FLASH_WP    		GET_PIN(D, 1)

static int rt_hw_spi_flash_with_sfud_init(void)
{
		FLASH_DEBUG("spi_flash_with_sfud_init start\n");
	
		rt_pin_mode(FLASH_PWR_EN, PIN_MODE_OUTPUT);
		rt_pin_mode(FLASH_CE, PIN_MODE_OUTPUT);
		rt_pin_mode(FLASH_WP, PIN_MODE_OUTPUT);
		
		rt_pin_write(FLASH_PWR_EN, PIN_HIGH);
		rt_pin_write(FLASH_CE, PIN_HIGH);
		rt_pin_write(FLASH_WP, PIN_HIGH);
    //register spi device
    if( rt_hw_spi_device_attach("spi3","spi30",GPIOA,GPIO_PIN_15) != RT_EOK )
		{
			 FLASH_DEBUG("flash spi device can not be find!\n");
		}
		
    /* init W25Q256 */
    if (RT_NULL == rt_sfud_flash_probe("SST25VF020", "spi30"))
    {
			FLASH_DEBUG("SST25VF020 flash probe fail\n");
        return -RT_ERROR;
    }
		
		FLASH_DEBUG("spi_flash_with_sfud_init Finsh\n");
		
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_spi_flash_with_sfud_init);

#endif/* BSP_USING_QSPI_FLASH */
