/**
  ******************************************************************************
  * @file    stm32f4xx_hal_rcc.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-June-2014
  * @brief   Header file of RCC HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_RCC_H
#define __STM32F4xx_HAL_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/** @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  */
#define __GPIOA_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN))
#define __GPIOB_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN))
#define __GPIOC_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN))
#define __GPIOD_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN))
#define __GPIOE_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOEEN))
#define __GPIOH_CLK_ENABLE()         (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOHEN))
#define __DMA1_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN))
#define __DMA2_CLK_ENABLE()          (RCC->AHB1ENR |= (RCC_AHB1ENR_DMA2EN))


/** @brief  Enable or disable the AHB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  */
#define __USB_OTG_FS_CLK_ENABLE()  do {(RCC->AHB2ENR |= (RCC_AHB2ENR_OTGFSEN));\
                                       __SYSCFG_CLK_ENABLE();\
                                    }while(0)
                                        

/** @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  */
#define __TIM2_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN))
#define __TIM3_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN))
#define __TIM4_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN))
#define __TIM5_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_TIM5EN))
#define __WWDG_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_WWDGEN))
#define __SPI2_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_SPI2EN))
#define __SPI3_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_SPI3EN))
#define __USART2_CLK_ENABLE()  (RCC->APB1ENR |= (RCC_APB1ENR_USART2EN))
#define __I2C1_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN))
#define __I2C2_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN))
#define __I2C3_CLK_ENABLE()    (RCC->APB1ENR |= (RCC_APB1ENR_I2C3EN))
#define __PWR_CLK_ENABLE()     (RCC->APB1ENR |= (RCC_APB1ENR_PWREN))

/** @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  */
#define __TIM1_CLK_ENABLE()    (RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN))
#define __USART1_CLK_ENABLE()  (RCC->APB2ENR |= (RCC_APB2ENR_USART1EN))
#define __USART6_CLK_ENABLE()  (RCC->APB2ENR |= (RCC_APB2ENR_USART6EN))
#define __ADC1_CLK_ENABLE()    (RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN))
#define __SPI1_CLK_ENABLE()    (RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN))
#define __SYSCFG_CLK_ENABLE()  (RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN))
#define __TIM9_CLK_ENABLE()    (RCC->APB2ENR |= (RCC_APB2ENR_TIM9EN))
#define __TIM10_CLK_ENABLE()   (RCC->APB2ENR |= (RCC_APB2ENR_TIM10EN))
#define __TIM11_CLK_ENABLE()   (RCC->APB2ENR |= (RCC_APB2ENR_TIM11EN))

/** @brief  Force or release AHB1 peripheral reset.
  */  
#define __AHB1_FORCE_RESET()    (RCC->AHB1RSTR = 0xFFFFFFFF)
#define __GPIOA_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOARST))
#define __GPIOB_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOBRST))
#define __GPIOC_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOCRST))
#define __GPIOD_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIODRST))
#define __GPIOE_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOERST))
#define __GPIOH_FORCE_RESET()   (RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOHRST))
#define __DMA1_FORCE_RESET()    (RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA1RST))
#define __DMA2_FORCE_RESET()    (RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA2RST))

#define __AHB1_RELEASE_RESET()  (RCC->AHB1RSTR = 0x00)
#define __GPIOA_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST))
#define __GPIOB_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOBRST))
#define __GPIOC_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOCRST))
#define __GPIOD_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIODRST))
#define __GPIOE_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOERST))
#define __GPIOF_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOFRST))
#define __GPIOG_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOGRST))
#define __GPIOH_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOHRST))
#define __GPIOI_RELEASE_RESET() (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOIRST))
#define __DMA1_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA1RST))
#define __DMA2_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA2RST))

/** @brief  Force or release AHB2 peripheral reset.
  */
#define __AHB2_FORCE_RESET()    (RCC->AHB2RSTR = 0xFFFFFFFF) 
#define __USB_OTG_FS_FORCE_RESET()   (RCC->AHB2RSTR |= (RCC_AHB2RSTR_OTGFSRST))

#define __AHB2_RELEASE_RESET()  (RCC->AHB2RSTR = 0x00)
#define __USB_OTG_FS_RELEASE_RESET() (RCC->AHB2RSTR &= ~(RCC_AHB2RSTR_OTGFSRST))

/* alias define maintained for legacy */
#define __OTGFS_FORCE_RESET    __USB_OTG_FS_FORCE_RESET
#define __OTGFS_RELEASE_RESET  __USB_OTG_FS_RELEASE_RESET                                      
                                      
/** @brief  Force or release APB1 peripheral reset.
  */
#define __APB1_FORCE_RESET()     (RCC->APB1RSTR = 0xFFFFFFFF)  
#define __TIM2_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM2RST))
#define __TIM3_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM3RST))
#define __TIM4_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM4RST))
#define __TIM5_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_TIM5RST))
#define __WWDG_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_WWDGRST))
#define __SPI2_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_SPI2RST))
#define __SPI3_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_SPI3RST))
#define __USART2_FORCE_RESET()   (RCC->APB1RSTR |= (RCC_APB1RSTR_USART2RST))
#define __USART3_FORCE_RESET()   (RCC->APB1RSTR |= (RCC_APB1RSTR_USART3RST))
#define __UART5_FORCE_RESET()    (RCC->APB1RSTR |= (RCC_APB1RSTR_UART5RST))
#define __I2C1_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST))
#define __I2C2_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C2RST))
#define __I2C3_FORCE_RESET()     (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C3RST))
#define __PWR_FORCE_RESET()      (RCC->APB1RSTR |= (RCC_APB1RSTR_PWRRST))

#define __APB1_RELEASE_RESET()   (RCC->APB1RSTR = 0x00) 
#define __TIM2_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST))
#define __TIM3_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST))
#define __TIM4_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM4RST))
#define __TIM5_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM5RST))
#define __WWDG_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_WWDGRST))
#define __SPI2_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI2RST))
#define __SPI3_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_SPI3RST))
#define __USART2_RELEASE_RESET() (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART2RST))
#define __USART3_RELEASE_RESET() (RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST))
#define __UART5_RELEASE_RESET()  (RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART5RST))
#define __I2C1_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST))
#define __I2C2_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C2RST))
#define __I2C3_RELEASE_RESET()   (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C3RST))
#define __PWR_RELEASE_RESET()    (RCC->APB1RSTR &= ~(RCC_APB1RSTR_PWRRST))

/** @brief  Force or release APB2 peripheral reset.
  */
#define __APB2_FORCE_RESET()     (RCC->APB2RSTR = 0xFFFFFFFF)  
#define __TIM1_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM1RST))
#define __USART1_FORCE_RESET()   (RCC->APB2RSTR |= (RCC_APB2RSTR_USART1RST))
#define __USART6_FORCE_RESET()   (RCC->APB2RSTR |= (RCC_APB2RSTR_USART6RST))
#define __ADC_FORCE_RESET()      (RCC->APB2RSTR |= (RCC_APB2RSTR_ADCRST))
#define __SDIO_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_SDIORST))
#define __SPI1_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST))
#define __SPI4_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_SPI4RST))
#define __SYSCFG_FORCE_RESET()   (RCC->APB2RSTR |= (RCC_APB2RSTR_SYSCFGRST))
#define __TIM9_FORCE_RESET()     (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM9RST))
#define __TIM10_FORCE_RESET()    (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM10RST))
#define __TIM11_FORCE_RESET()    (RCC->APB2RSTR |= (RCC_APB2RSTR_TIM11RST))

#define __APB2_RELEASE_RESET()   (RCC->APB2RSTR = 0x00)
#define __TIM1_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST))
#define __USART1_RELEASE_RESET() (RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST))
#define __USART6_RELEASE_RESET() (RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART6RST))
#define __ADC_RELEASE_RESET()    (RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST))
#define __SDIO_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SDIORST))
#define __SPI1_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST))
#define __SPI4_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI4RST))
#define __SYSCFG_RELEASE_RESET() (RCC->APB2RSTR &= ~(RCC_APB2RSTR_SYSCFGRST))
#define __TIM9_RELEASE_RESET()   (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM9RST))
#define __TIM10_RELEASE_RESET()  (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM10RST))
#define __TIM11_RELEASE_RESET()  (RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM11RST))

/** @brief  Force or release AHB3 peripheral reset.
  */  
#define __AHB3_FORCE_RESET() (RCC->AHB3RSTR = 0xFFFFFFFF)
#define __AHB3_RELEASE_RESET() (RCC->AHB3RSTR = 0x00) 

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_RCC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
