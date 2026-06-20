/**
 *******************************************************************************
 * @file  template/source/main.h
 * @brief This file contains the including files of main routine.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include "hc32_ll.h"

/*=============================================================================
 * 硬件板本选择 (BOARD_VERSION)
 *  0 = 原HB_chuchai板 (PA04电压, 110k分压, GPIO电机, 霍尔电流传感器, PB14 RS485 DIR)
 *  1 = 整合板 (PA06电压, 150k分压, PWM电机, 差分运放电流传感器, PA3 RS485 DIR)
 *============================================================================*/
#define BOARD_VERSION   1

#endif /* __MAIN_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
