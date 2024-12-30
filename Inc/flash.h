/*
 * flash.h
 *
 * Created: 2016-04-22
 * Author: zhanglifu
 */
#ifndef _flash_H_
#define _flash_H_


#include "stm32f1xx_hal.h"


/*********************************************************************/
//                        变量定义
/*********************************************************************/
//-- 用途划分
// 0x0800FC00-0x0800FFFF -- 使用最后4k 个字节用来存放电机信息
#define FMC_FLASH_BASE      0x08010000   // FLASH的起始地址
#define APP_MAX_SIZE        0x00020000   // 应用程序最大大小 (128KB)

// 根据最大可能支持的Flash容量调整结束地址
#define FMC_FLASH_END       0x08020000   // FLASH的结束地址 (假设128KB容量)

// 设备信息和日志地址调整
#define DEVICE_INFO_ADDRESS 0x0801C000   //（假设128KB容量：结束地址 - 16KB）
#define DEVICE_LOG_ADDRESS  0x0801E000   //（假设128KB容量：结束地址 - 8KB）

// Flash大小调整为128KB
#define FMC_FLASH_SIZE 128               // 定义Flash大小，单位KB

// 根据Flash容量选择扇区大小和其他参数
#if FMC_FLASH_SIZE < 256
#define FMC_SECTOR_SIZE 1024             // 扇区大小为1KB
#define MOD_SECTOR_SIZE 0x3FF
#define PAGE_COUNT_BY16 512
#else
#define FMC_SECTOR_SIZE 2048             // 扇区大小为2KB
#define MOD_SECTOR_SIZE 0x7FF
#define PAGE_COUNT_BY16 1024
#endif





/*********************************************************************/
//                        函数声明
/*********************************************************************/
void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);




#endif
