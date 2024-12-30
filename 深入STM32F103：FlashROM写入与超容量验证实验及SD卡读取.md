

@[TOC](文章目录)

---

# 前言
`本文将完成：`

一、Flash地址空间的数据读取及内存验证。

stm32f103c8t6只有20KB 内存（RAM）供程序代码和数组变量存放，因此，针对内部Flash的总计64KB存储空间(地址从0x08000000开始），运行一次写入8KB数据，总计复位运行代码8次，将64KB数据写入Flash，并验证写入数据的正确性和读写速率。此外，继续往后续地址写入数据，检验stm32f103c8t6 实际FlashROM是否超过64KB。
二、掌握SD卡协议原理，用STM32F103完成对SD卡的数据读取（fat文件模式）。

---

`提示：以下是本篇文章正文内容，下面案例可供参考`

# 一、Flash地址空间的数据读取
## 1.1 Flash是什么？
**Flash**是一种**非易失性存储器（Non-Volatile Memory）**，即使断电也能保存数据。它被广泛应用于嵌入式系统、固态硬盘、U盘等设备中。

在STM32中，Flash主要分为两种用途：
1. **代码存储**：在STM32芯片中，程序代码通常存储在Flash中（如STM32F103的内置64KB FlashROM）。
2. **数据存储**：在嵌入式开发中，也可以利用未被代码占用的Flash区域存储重要的用户数据。

---

## 1.2 Flash的原理

Flash存储器的基本原理是通过**电荷捕获和释放**来存储数据。每个存储单元由浮动栅极的晶体管组成，通过控制浮动栅极的电荷状态来表示0或1。

### 1. **Flash的存储单元**
- 每个单元存储1位数据（0或1）。
- 数据存储基于电荷状态：  
  - **0**：存储单元中有电荷（表示逻辑0）。  
  - **1**：存储单元中没有电荷（表示逻辑1）。

### 2. **Flash的读写操作**
**读取**和**写入**是Flash存储的基本操作：

#### **（1）读取原理**
- Flash读取操作是**非破坏性操作**。
- 通过将一个小电流施加到存储单元上，检测单元的电荷状态，从而判断存储的是0还是1。

#### **（2）写入原理**
- 写入操作将单元的电荷状态改变为0或1，通常是通过以下过程实现：
  - 施加高电压，往浮动栅极注入或移除电子。
  - 写入的数据是以**页（Page）**为单位，STM32的Flash页大小通常是1KB或2KB。

#### **（3）擦除原理**
- Flash不能直接覆盖写入，只能先**擦除后再写入**。
- 擦除是以**扇区（Sector）**为单位，通常擦除一个扇区会将其中的所有数据重置为1。

---

## 1.3 Flash读取的流程

Flash读取的流程通常是直接从指定地址获取数据，STM32的内部Flash读取采用**内存映射机制**，这意味着Flash中的数据地址与内存地址空间一致，读取时的具体流程如下：

1. **定位地址**：通过给定的Flash存储器地址（例如，STM32F103的Flash地址从`0x08000000`开始），CPU会直接读取对应存储单元的数据。
2. **数据提取**：通过存储单元的电荷状态，判断是逻辑0还是逻辑1。
3. **返回结果**：数据通过总线返回到CPU寄存器中，供程序使用。

读取Flash的速度相对较快，因为它不涉及高电压操作，也不需要改变存储单元状态。

---

## 1.4 Flash读取的特点

1. **优点**：
   - 读取速度快：Flash的读取速度通常可以达到接近RAM的速度。
   - 数据可靠：读取是非破坏性操作，不会影响已有数据。
   - 支持内存映射：STM32可以直接像操作内存一样访问Flash数据。

2. **缺点**：
   - 固定容量：Flash的存储容量有限（STM32F103C8T6内部为64KB）。
   - 不支持频繁写入：Flash的擦写次数有限（通常10万次左右）。
   - 不能随机擦写：Flash需要以扇区为单位擦除后再写入。

---
## 1.5 实验实现Flash的读取
###  1. CubeMx配置
1. 选择STM32C8T6芯片
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/7fda0c8fa2b24744a387a3568440417f.png)
2. 配置RCC
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ee9451157ea84a7abe33a32f3ea31136.png)
3. 配置SYS
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/e29955138b9345838b7149c2dd58e01b.png)
添加了一个LED指示灯，在进行串口通信时，可以通过这个指示灯判断是否在进行通信传输，引脚选择A4，GPIO配置如下：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c5b40c546cd2432683484963d02f522b.png)

4. 时钟配置
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/1c26362583a34452afaaf0a50a9cf722.png)
5. 生成代码
设置堆栈大小为4K或2K，然后正常生成代码即可
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/e472b0df761640d7832bab0aab634875.png)

###  2. Keil编写
1. 先在工程文件中创建flash.c和flash.h，然后在keil中同样创立两个新文件，flash.c和flash.h
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/f5e167f00f5740d18e1c40aae8b35935.png)
其中，flash.c的内容如下：

```cpp
/*
 * flash.c
 *
 * Created: 2018-01-29
 * Author: zhanglifu
 */
 
/*********************************************************************/
//                        头文件
/*********************************************************************/
#include "flash.h"


// 不检查的写入
// WriteAddr:起始地址
// pBuffer:  数据指针
// NumToWrite:字节数数
void FlashWriteNoCheck( uint32_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite )
{
    uint16_t i;

    for( i=0; i<NumToWrite; i+=4 )
    {
        while( HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr+i,*(uint32_t *)(pBuffer+i) ) );
    }

}

extern void FLASH_PageErase(uint32_t PageAddress);
void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length )
{
    uint16_t i;
    uint8_t FlashBuff[FMC_SECTOR_SIZE];
    uint32_t StartAddress = destination_address - destination_address%FMC_SECTOR_SIZE;
    uint16_t Offset = destination_address - StartAddress;
    uint8_t *pBuf = buffer;
    uint32_t remaindNum = length;

    HAL_StatusTypeDef status = HAL_ERROR;

    // 地址检查
    if( (destination_address < FMC_FLASH_BASE) || ( destination_address + length >= FMC_FLASH_END) || (length <= 0) )
        return;

    HAL_FLASH_Unlock();	// 解锁
    do
    {
        // 读出一页数据
        for(i=0; i < FMC_SECTOR_SIZE; i += 4 )
            *(uint32_t *)(FlashBuff+i) = *(uint32_t *)(StartAddress+i);

        // 修改要改入的数据
        for ( i=0; (i+Offset)<FMC_SECTOR_SIZE && i< remaindNum; i++ )
            *(FlashBuff+Offset+i) = *(pBuf+i);


        // 擦除一ROW数据
        FLASH_PageErase( StartAddress );

        // HAL库 FLASH_PageErase有BUFF,要加上下面三行代码
        while( status != HAL_OK )
            status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
        CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

        // 写入数据
        FlashWriteNoCheck( StartAddress,FlashBuff,FMC_SECTOR_SIZE );

        // 为下一页做准备
        StartAddress +=  FMC_SECTOR_SIZE;
        remaindNum -= i;
        pBuf += i;
        Offset = 0;

    } while( remaindNum > 0 );

    HAL_FLASH_Lock();  // 上锁
		
}



// 从FLASH中读指定长度数据
void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length)
{
    uint16_t i;
    uint8_t Offset = 0;
    uint32_t StartAddress = source_address;
    uint16_t data;

    // 地址检测
    if( source_address + length > FMC_FLASH_END ) return;

    // 如果没有对16齐
    if( source_address & 1 )
    {
        Offset = 1;
        StartAddress = source_address-1;
    }

    // flash的操作要求16对齐 最小读写操作16个比特
    if ( Offset )
    {
        data = *(uint16_t *)(StartAddress);
        buffer[0] = data >> 8;
        StartAddress += 2;
    }

    for ( i = 0; i < (length-Offset); i += 2)
    {
        data = *(uint16_t *)(StartAddress+i);
        buffer[i+Offset] = (data & 0xFF);
        if ( (i+Offset) < (length - 1) )
            buffer[i + Offset + 1] = (data >> 8);
    }

}




```
flash.h代码：

```cpp
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
#define FMC_FLASH_BASE      0x08000000 	// FLASH的起始地址
#define APP_MAX_SIZE        0x00010000  // 


#define FMC_FLASH_END        0x08010000  // FLASH的结束地址 256
#define DEVICE_INFO_ADDRESS  0x0800C000  //（STM32_FLASH_END - DEVICE_INFO_SIZE）   // 设备信息起始地址
#define DEVICE_LOG_ADDRESS   0x0800E000  //（STM32_FLASH_END - 2*DEVICE_INFO_SIZE） // 设备日志起始地址



#define FMC_FLASH_SIZE 64			          // 定义Flash大小，单位KB


#if FMC_FLASH_SIZE < 256
#define FMC_SECTOR_SIZE 1024            // 字节
#define MOD_SECTOR_SIZE 0X3FF
#define PAGE_COUNT_BY16 512
#else
#define FMC_SECTOR_SIZE	2048
#define MOD_SECTOR_SIZE 0X7FF
#define PAGE_COUNT_BY16 1024
#endif





/*********************************************************************/
//                        函数声明
/*********************************************************************/
void FlashWriteBuff( const uint32_t destination_address, uint8_t *const buffer,uint32_t length );
void FlashReadBuff(const uint32_t source_address,uint8_t *const buffer,uint16_t length);




#endif

```
在mian函数下添加相关代码：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/b09018a835d74721bf99998fb804943f.png)
main.c：

```cpp
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t FlashWBuff [255];
uint8_t FlashRBuff [255];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i;
	uint8_t FlashTest[] = "Hello This is SUNYUAN Flash Test DEMO";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	FlashWriteBuff( DEVICE_INFO_ADDRESS, FlashTest,sizeof(FlashTest) );        // 写入数据到Flash
	
	for(i=0;i<255;i++)
		FlashWBuff[i] = i;
	
  FlashWriteBuff( DEVICE_INFO_ADDRESS + sizeof(FlashTest), FlashWBuff,255 );  // 写入数据到Flash
	FlashReadBuff(  DEVICE_INFO_ADDRESS + sizeof(FlashTest),FlashRBuff,255  );  // 从Flash中读取数
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

```
### 3. 程序调试
1. 插上ST_Link,点击调试
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/d27bbe6415b641ec9afe8e12df4a093d.png)
2. 点击View->memory windows->memory 1打开内存观察窗口，View->Watch windows->Watch 1打开一个变量观察窗口，将变量FlashWBuff 和 FlashRBuff加入到 Watch 1 观察窗口，View->Periodic Windows Update开启变量自动更新
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/72a7ce4fe4f54f719c27e6bfc7eace40.png)
3. 视频


[video(video-IZNyYAFW-1735547969185)(type-csdn)(url-https://live.csdn.net/v/embed/441250)(image-https://i-blog.csdnimg.cn/img_convert/7c9393f1ebd1ac14442f0d69d880eeff.jpeg)(title-stm32_Flash测试)]
# 二、Flash内存验证
## 2.1 相关参数配置
一般来说flash地址从0x08000000开始,但是我们为了验证是否有128K的Flash，修改的相关参数如下：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/3aa7959d4aeb4418a22954f813f47db5.png)

flash.h代码：

```cpp
#define FMC_FLASH_BASE      0x08000000   // FLASH的起始地址
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

```
通过循环的方式，从起始地址到结束地址进行写入和读取验证，后发现确实有128K的Flash容量：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/d95298e8acd647be9f5dbabf63b503d7.png)

## 2.2 Flash有128K原因
1 STM32F103C8T6和STM32F103CBT6 引脚相同，唯一的区别是前者为64kflash（0x8000000~0x800FFFF） 后者为128kflash（0x8000000~0x801FFFF)；
2 已经发现STM32Ff103C8T6 在 00x8010000~0x801FFFF是可读写的；
ST 技术人员对此回答：
STM32F103C8T6和STM32F103CBT6 是在同一圆晶上制造的，但测试时，只测试手册提供的参数范围所包含的内容，也就意味这 STM32F103C8T6也是128k Flash，只不过只测试了前面64k，然后打上 STM32F103C8T6标注（当然内部就写上 C8T6 的 ID）；


---
# 三、SD卡的数据读取
## 3.1 SD卡原理
### 1. SD卡物理结构
SD Host Controller Simplified Specification（以下简称：主机协议）用来标准化SD主机控制器，针对的是SD卡主机控制器厂商。这个协议不是强制的，在我们阅读SD驱动代码的时候，如果涉及到SD卡主机控制的代码，我们可能需要翻一下这篇文档，或者查阅SD卡主机控制器厂商提供给我们的文档（一般都是各大cpu芯片厂商提供给我们开发者文档）。
SD卡（Secure Digital Memory Card）在我们的生活中已经非常普遍了，控制器对SD卡进行读写通信操作一般有两种通信接口可选，一种是 SPI接口，另外一种就是 SDIO接口。SDIO 全称是 安全数字输入/输出接口，多媒体卡(MMC)、SD卡、SD I/O卡 都有 SDIO接口。STM32F103系列控制器有一个 SDIO主机接口，它可以与 MMC卡、SD卡、SD I/O卡 以及 CE-ATA 设备进行数据传输。
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/e8485ab31f0a4ce89507a011589bac51.png)
（1）存储单元是存储数据部件，存储单元通过存储单元接口与卡控制单元进行数据传输；
（2）电源检测单元保证SD卡工作在合适的电压下，如出现掉电或上状态时，它会使控制单元和存储单元接口复位；
（3）卡及接口控制单元控制SD卡的运行状态，它包括有8个寄存器； 接口驱动器控制SD卡引脚的输入输出
### 2. SD卡操作模式
（1）SD卡一般都支持 SDIO 和 SPI 这两种接口。
（2）其中SD卡模式的信号线有：CLK、CMD、DAT0-DAT3，6根线。
（3）SPI模式的信号线有：CS、CLK、MISO（DATAOUT）、MOSI(DATAIN),4根线。
（4）SD卡的命令格式：命令CMD0就是0，CMD16就是16，以此类推。
（5）SD卡的命令总共有12类，下表为几个比较重要的命令：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c75b409ebfc441b8981c41c3ec7dacfd.png)
### 3. SD卡读取与写入(SPI模式)
（1）发送CMD17；
（2）接收卡响应R1；
（3）接收数据起始令牌0XFE；
（4）接收数据；
（5）接收2个字节的CRC，如果不使用CRC，这两个字节在读取后可以丢掉。
（6）禁止片选之后，发多8个CLK；
## 3.2 CubeMx配置
1. 选择STM32C8T6芯片
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/7fda0c8fa2b24744a387a3568440417f.png)
2. 配置RCC
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ee9451157ea84a7abe33a32f3ea31136.png)
3. 配置SYS
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/e29955138b9345838b7149c2dd58e01b.png)
添加了一个LED指示灯，在进行串口通信时，可以通过这个指示灯判断是否在进行通信传输，引脚选择A4，GPIO配置如下：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/c5b40c546cd2432683484963d02f522b.png)
4. 时钟配置
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/1c26362583a34452afaaf0a50a9cf722.png)
5. 配置SPI
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/2bb19498b08d478ba97e34a96139f8dd.png)
6.配置串口
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/1fc7eca5006e48d8963501c9a4fd2600.png)


## 3.3 工程编写
用keil编写代码：
main.c

```cpp
int main(void)
{
  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */
  
 
  /* MCU Configuration--------------------------------------------------------*/
 
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
 
  /* USER CODE BEGIN Init */
 
  /* USER CODE END Init */
 
  /* Configure the system clock */
  SystemClock_Config();
 
  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */
 
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart1,&aRxBuffer1,1); 	//enable uart	
 
	printf(" main \r\n");
 
	Get_SDCard_Capacity();	//µÃµ½Ê¹ÓÃÄÚ´æ²¢Ñ¡Ôñ¸ñÊ½»¯
 
 
 
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
		
		WritetoSD(WriteBuffer,sizeof(WriteBuffer));		
 
		
		
		HAL_Delay(500);
		WriteBuffer[0] = WriteBuffer[0] +10;
		WriteBuffer[1] = WriteBuffer[1] +10;
		write_cnt ++;
		
		while(write_cnt > 10)
		{	
			printf(" while \r\n");
			HAL_Delay(500);
		}		
    /* USER CODE END WHILE */
 
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```
写入函数WritetoSD:

```cpp
void WritetoSD(BYTE write_buff[],uint8_t bufSize)
{
	FATFS fs;
	FIL file;
	uint8_t res=0;
	UINT Bw;	
	res = SD_init();		//SD卡初始化
	if(res == 1)
	{
		printf("SD卡初始化失败! \r\n");		
	}
	else
	{
		printf("SD卡初始化成功！ \r\n");		
	}
	res=f_mount(&fs,"0:",1);		//挂载
//	if(test_sd == 0)		//用于测试格式化
	if(res == FR_NO_FILESYSTEM)		//没有文件系统，格式化
	{
//		test_sd =1;				//用于测试格式化
		printf("没有文件系统! \r\n");		
		res = f_mkfs("", 0, 0);		//格式化sd卡
		if(res == FR_OK)
		{
			printf("格式化成功! \r\n");		
			res = f_mount(NULL,"0:",1); 		//格式化后先取消挂载
			res = f_mount(&fs,"0:",1);			//重新挂载	
			if(res == FR_OK)
			{
				printf("SD卡已经成功挂载，可以进进行文件写入测试!\r\n");
			}	
		}
		else
		{
			printf("格式化失败! \r\n");		
		}
	}
	else if(res == FR_OK)
	{
		printf("挂载成功! \r\n");		
	}
	else
	{
		printf("挂载失败! \r\n");
	}	
	res = f_open(&file,SD_FileName,FA_OPEN_ALWAYS |FA_WRITE);
	if((res & FR_DENIED) == FR_DENIED)
	{
		printf("卡存储已满，写入失败!\r\n");		
	}
	f_lseek(&file, f_size(&file));//确保写词写入不会覆盖之前的数据
	if(res == FR_OK)
	{
		printf("打开成功/创建文件成功！ \r\n");		
		res = f_write(&file,write_buff,bufSize,&Bw);		//写数据到SD卡
		if(res == FR_OK)
		{
			printf("文件写入成功！ \r\n");			
		}
		else
		{
			printf("文件写入失败！ \r\n");
		}		
	}
	else
	{
		printf("打开文件失败!\r\n");
	}	
	f_close(&file);						//关闭文件		
	f_mount(NULL,"0:",1);		 //取消挂载
}
```
## 3.4 测试
首先需要正确连接SD卡，引脚连接见下图：
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/f7ee1a9dd51e45b9b5b936e3ee099a1a.png)
然后打开串口助手，进行调试即可，注意波特率、停止位、数据位、校验位的设置:
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/80636ca51d0e4928ad88bbc17cf8c4df.png)

# 总结

## **1. Flash验证成果**
通过本文对STM32F103C8T6的Flash进行全面验证，发现以下关键点：
1. **Flash的实际容量**：
   - 虽然官方标注STM32F103C8T6的Flash容量为64KB，但实验表明，其实际容量为128KB（地址范围：`0x08000000`至`0x08020000`）。
   - 这归因于STM32F103C8T6和STM32F103CBT6使用了相同的晶圆，只是在测试时仅标定了前64KB的Flash。

2. **Flash操作的实现**：
   - 实现了通过HAL库对Flash进行分段写入和读取的完整操作。
   - 验证数据完整性，确保写入和读取的一致性。

3. **CubeMX配置与Keil代码实现**：
   - 通过CubeMX生成基础初始化代码，包括时钟、GPIO等配置。
   - 编写了完整的`flash.c`和`flash.h`文件，用于封装Flash操作函数，支持按扇区擦除、写入和读取。

4. **数据验证方法**：
   - 通过ST-Link观察内存和变量值，结合循环写入与读取，验证了整个Flash空间的可用性。
   - 在128KB地址范围内数据写入和读取均无异常。

---

## **2. SD卡操作成果**
1. **SD卡读取与写入**：
   - 配置了SPI接口与SD卡通信，通过FATFS文件系统实现对SD卡文件的读写操作。
   - 成功完成对SD卡的初始化、挂载、文件创建、文件写入和读取操作。

2. **SPI通信配置**：
   - 使用SPI接口进行SD卡通信，掌握了CMD命令与SD卡数据传输协议。
   - 测试通过SD卡写入文件的完整流程，并验证文件内容的正确性。

3. **调试与优化**：
   - 在调试过程中结合串口打印输出状态信息，便于快速定位问题。
   - 使用Keil的内存观察工具实时监控SD卡与Flash的操作结果。


---


通过本文的实验和代码实现，读者可以完整掌握STM32F103的Flash与SD卡存储操作，为嵌入式开发积累宝贵经验。
