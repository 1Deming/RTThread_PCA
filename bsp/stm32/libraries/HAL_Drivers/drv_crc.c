/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-06-11     deming        first version
 */

#include "drv_crc.h"
#include "stm32l1xx_hal.h"
#include "rtdef.h"

CRC_HandleTypeDef hcrc;

int stm32_crc_init()
{
    hcrc.Instance = CRC;
    if( HAL_CRC_Init( &hcrc ) != HAL_OK )
			return 1;
		
		return 0;
}

INIT_BOARD_EXPORT(stm32_crc_init);
