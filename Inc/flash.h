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
//                        ��������
/*********************************************************************/
//-- ��;����
// 0x0800FC00-0x0800FFFF -- ʹ�����4k ���ֽ�������ŵ����Ϣ
#define FMC_FLASH_BASE      0x08010000   // FLASH����ʼ��ַ
#define APP_MAX_SIZE        0x00020000   // Ӧ�ó�������С (128KB)

// ����������֧�ֵ�Flash��������������ַ
#define FMC_FLASH_END       0x08020000   // FLASH�Ľ�����ַ (����128KB����)

// �豸��Ϣ����־��ַ����
#define DEVICE_INFO_ADDRESS 0x0801C000   //������128KB������������ַ - 16KB��
#define DEVICE_LOG_ADDRESS  0x0801E000   //������128KB������������ַ - 8KB��

// Flash��С����Ϊ128KB
#define FMC_FLASH_SIZE 128               // ����Flash��С����λKB

// ����Flash����ѡ��������С����������
#if FMC_FLASH_SIZE < 256
#define FMC_SECTOR_SIZE 1024             // ������СΪ1KB
#define MOD_SECTOR_SIZE 0x3FF
#define PAGE_COUNT_BY16 512
#else
#define FMC_SECTOR_SIZE 2048             // ������СΪ2KB
#define MOD_SECTOR_SIZE 0x7FF
#define PAGE_COUNT_BY16 1024
#endif





/*********************************************************************/
//                        ��������
/*********************************************************************/
void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);




#endif
