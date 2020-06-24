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
#include "dfs_fs.h"

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(E, 11)

#define FS_PARTITION_NAME     "SST25VF020"

int main(void)
{
    int count = 1;
		/* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    
    /* ?? elm ????? / ??,???????????????????????? */
    if (dfs_mount(FS_PARTITION_NAME, "/", "elm", 0, 0) == 0)
    {
			rt_kprintf("Filesystem initialized!\n");
    }
    else
    {
        /* ??????,?????????????????? */
        dfs_mkfs("elm", FS_PARTITION_NAME);
        /* ?????????? */
        if (dfs_mount(FS_PARTITION_NAME, "/", "elm", 0, 0) == 0)
        {
            /* ??????????,????????? */
            rt_kprintf("Failed to initialize filesystem!");
        }
    }

    rt_thread_mdelay(100);	
	
	while (count++)
    {	
			rt_pin_write(LED0_PIN, PIN_HIGH);
			rt_thread_mdelay(500);
			rt_pin_write(LED0_PIN, PIN_LOW);
			rt_thread_mdelay(500);		
    }

    return RT_EOK;
}

void sf_mount()
{
	if (dfs_mount(FS_PARTITION_NAME, "/", "elm", 0, 0) == 0)
	{
			rt_kprintf("Failed to initialize filesystem!");
	}
}
FINSH_FUNCTION_EXPORT(sf_mount, mount file system\n);


void sf_mkfs()
{
	dfs_mkfs("elm", FS_PARTITION_NAME);
}
FINSH_FUNCTION_EXPORT(sf_mkfs, mkfs file system\n);
