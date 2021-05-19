# 前言
本文主要参考文章：[新建基于STM32F103ZET6的工程-HAL库版本](https://www.cnblogs.com/h1019384803/p/10925909.html)
因为 [Mars-King](https://www.cnblogs.com/h1019384803/) 大佬文章写的是1.7.0的版本，我现在没有找到此版本，官方GitHub最低也是1.8.0版本。所以针对新版本与旧版本的偏差做一下补充，偏差较少，请大家以大佬的文章为主。
工程下载：[github](https://github.com/Ikaros-521/STM32F103ZET6_HAL_demo) [码云](https://gitee.com/ikaros-521/STM32F103ZET6_HAL_demo)
# 正文
## 搭建
官网地址：[https://www.st.com](https://www.st.com)
方便直接下载，我直接提供下载的页面地址：[https://www.st.com/content/st_com/zh/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef1.html#get-software](https://www.st.com/content/st_com/zh/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef1.html#get-software)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517164435330.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
GitHub仓库：[https://github.com/STMicroelectronics/STM32CubeF1](https://github.com/STMicroelectronics/STM32CubeF1)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517164516643.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
本文使用的是1.8.3版本
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517164548471.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
下载后解压，内容如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517164744853.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
ok，然后参考文章：[新建基于STM32F103ZET6的工程-HAL库版本](https://www.cnblogs.com/h1019384803/p/10925909.html)进行项目构建。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517165259712.png)
大部分步骤都是适用的，直到**CORTEX**文件夹的内容，`STM32CubeF1-1.8.3\Drivers\CMSIS\Include`和`STM32CubeF1-1.8.3\Drivers\CMSIS\Device\ST\STM32F1xx\Source\Templates\arm`
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517165418715.png)
`MDK_PRO`
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517165748624.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
`MDK_PRO\Inc`来自`STM32CubeF1-1.8.3\Drivers\CMSIS\Device\ST\STM32F1xx\Include`和`STM32CubeF1-1.8.3\Projects\STM32F103RB-Nucleo\Templates\Inc`
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517165915873.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
`MDK_PRO\Src`来自`STM32CubeF1-1.8.3\Projects\STM32F103RB-Nucleo\Templates\Src`
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517170432306.png)
`STM32F1xx_HAL_Driver` 来自 `STM32CubeF1-1.8.3\Drivers\STM32F1xx_HAL_Driver`，Src下所有的xxx_template文件都删掉（共3个，因为之后编译会报错重定义）
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517170531129.png)
准备好后，Keil5创建工程，就正常工程类似步骤。具体参考文章：[新建基于STM32F103ZET6的工程-HAL库版本](https://www.cnblogs.com/h1019384803/p/10925909.html)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517170957518.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
`main.h`中，`stm32f1xx_nucleo.h`报错不存在，可以注释掉（文件是在`STM32CubeF1-1.8.3\Drivers\BSP\STM32F1xx_Nucleo`下，不过例程用不到）
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517171204442.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)
最后效果如下：
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210517170807474.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lrYXJvc181MjE=,size_16,color_FFFFFF,t_70)

## 测试代码
### 效果
请以实际效果为准，图片有失真和加速

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210519111109132.gif#pic_center)

### main.c

```c
/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

    /* STM32F103xB HAL library initialization:
         - Configure the Flash prefetch
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization
       */
    HAL_Init();

    /* Configure the system clock to 64 MHz */
    SystemClock_Config();


    /* Add your application code here
       */
	// 使能时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	// GPIO初始化
	GPIO_InitTypeDef GPIO_Init = {GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH};
	// void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
	HAL_GPIO_Init(GPIOB, &GPIO_Init);
	HAL_GPIO_Init(GPIOE, &GPIO_Init);
	
	// 配置引脚的初始化电平 GPIO_PIN_SET则为高电平 GPIO_PIN_RESET则为低电平
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);

    /* Infinite loop */
    while (1)
    {
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};

    /* Configure PLL ------------------------------------------------------*/
    /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
    /* Enable HSI and activate PLL with HSi_DIV2 as source */
    oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
    oscinitstruct.HSEState        = RCC_HSE_OFF;
    oscinitstruct.LSEState        = RCC_LSE_OFF;
    oscinitstruct.HSIState        = RCC_HSI_ON;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
    oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

```

