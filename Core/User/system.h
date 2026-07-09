/**
 * @file system.h
 * @brief 用户代码统一头文件入口。
 *
 * 模块用途：集中包含 HAL 生成头文件与全部用户模块头文件。
 * GPIO 引脚映射：无直接 GPIO 引脚，各模块映射见对应模块说明。
 * 依赖的外设和 CubeIDE 配置：依赖 CubeMX 生成的 main.h 和 spi.h。
 * 初始化方法：HAL 与 MX_* 初始化完成后调用 system_init()。
 * 调用方法：main.c 及其他用户 .c 文件仅包含本头文件。
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "main.h"
#include "spi.h"
#include "ads8688.h"
#include "ads8688_storage.h"

/**
 * @brief 初始化全部用户模块。
 * @param 无。
 * @return 无。
 * @note 必须在 CubeMX 生成的外设初始化完成后调用。
 */
void system_init(void);

#endif /* SYSTEM_H */
