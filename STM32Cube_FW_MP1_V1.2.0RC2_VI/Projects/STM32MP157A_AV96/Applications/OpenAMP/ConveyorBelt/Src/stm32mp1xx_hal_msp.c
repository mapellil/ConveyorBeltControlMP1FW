/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/ConveyorBelt/Src/stm32mp1xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   This file provides code for the MSP Initialization 
  *          and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "x_nucleo_ihm03a1_stm32mp15xx.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//extern volatile uint8_t EmergencyStop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */
extern void BSP_MotorControl_BusyInterruptHandler(void);
extern void BSP_MotorControl_FlagInterruptHandler(void);
extern volatile uint8_t EmergencyStop;

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief IPCC MSP Initialization
* This function configures the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspInit(IPCC_HandleTypeDef* hipcc)
{

  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspInit 0 */

  /* USER CODE END IPCC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_IPCC_CLK_ENABLE();
  /* IPCC interrupt Init */
    HAL_NVIC_SetPriority(IPCC_RX1_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(IPCC_RX1_IRQn);
  /* USER CODE BEGIN IPCC_MspInit 1 */

  /* USER CODE END IPCC_MspInit 1 */
}

}

/**
* @brief IPCC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/

void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* hipcc)
{

  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspDeInit 0 */

  /* USER CODE END IPCC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_IPCC_CLK_DISABLE();

    /* IPCC interrupt DeInit */
    HAL_NVIC_DisableIRQ(IPCC_RX1_IRQn);
  /* USER CODE BEGIN IPCC_MspDeInit 1 */

  /* USER CODE END IPCC_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

GPIO_InitTypeDef GPIO_InitStruct;
  if (hi2c->Instance==I2C1)
  {
    if(IS_ENGINEERING_BOOT_MODE())
	{
      RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
      /*##-1- Configure the I2C clock source #*/
      RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C35;
      RCC_PeriphCLKInitStruct.I2c35ClockSelection = RCC_I2C35CLKSOURCE_HSI;
      HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
    }

    /**I2C1 GPIO Configuration
    PF14     ------> I2C1_SCL
    PF15     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  //GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_I2C1;
    PERIPH_LOCK(GPIOF);
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    PERIPH_UNLOCK(GPIOF);
    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, DEFAULT_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C5_MspDeInit 0 */

  /* USER CODE END I2C5_MspDeInit 0 */

  /* Peripheral clock disable */
  __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PF14     ------> I2C1_SCL
    PF15     ------> I2C1_SDA
    */
    PERIPH_LOCK(GPIOF);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14|GPIO_PIN_15);
    PERIPH_UNLOCK(GPIOF);

    /* I2C1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */
  /* USER CODE END I2C1_MspDeInit 1 */
  }
}


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {
	if(IS_ENGINEERING_BOOT_MODE())
	{
	  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

	  /*##-1- Configure the SPI clock source #*/
	  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI45; /** FIXME should be SPI2x ? **/
	  RCC_PeriphCLKInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_HSI;
	  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
	}
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    SPIx_SCK_GPIO_CLK_ENABLE();    // Enable step clock
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    PERIPH_LOCK(SPIx_SCK_GPIO_PORT);
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
    PERIPH_UNLOCK(SPIx_SCK_GPIO_PORT);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    PERIPH_LOCK(SPIx_MISO_GPIO_PORT);
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);
    PERIPH_UNLOCK(SPIx_MISO_GPIO_PORT);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    PERIPH_LOCK(SPIx_MOSI_GPIO_PORT);
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
    PERIPH_UNLOCK(SPIx_MOSI_GPIO_PORT);
  }
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param[in] hspi SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPIx)
  {
    /*##-1- Reset peripherals ##################################################*/
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();
    SPIx_CLK_DISABLE();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    PERIPH_LOCK(SPIx_SCK_GPIO_PORT);
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
    PERIPH_UNLOCK(SPIx_SCK_GPIO_PORT);

    /* Configure SPI MISO as alternate function  */
    PERIPH_LOCK(SPIx_MISO_GPIO_PORT);
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
    PERIPH_UNLOCK(SPIx_MISO_GPIO_PORT);

    /* Configure SPI MOSI as alternate function  */
    PERIPH_LOCK(SPIx_MOSI_GPIO_PORT);
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
    PERIPH_UNLOCK(SPIx_MOSI_GPIO_PORT);
  }
}

/**
  * @brief PWM MSP Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_STEP_CLOCK)
  {
	log_info("===>>>  HAL_TIM_PWM_MspInit\n");
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_CLCK_ENABLE();

    /* GPIO configuration */
    GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_STEP_CLOCK;
    PERIPH_LOCK(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT);
    HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT, &GPIO_InitStruct);
    PERIPH_UNLOCK(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT);

    /* Set Interrupt Group Priority of Timer Interrupt*/
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_IRQn, 4, 0);

    /* Enable the timer global Interrupt */
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_IRQn);
  }
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_TIMER_STEP_CLOCK)
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_CLCK_DISABLE();

    /* GPIO Deconfiguration */
    PERIPH_LOCK(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT);
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT, BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PIN);
    PERIPH_UNLOCK(BSP_MOTOR_CONTROL_BOARD_STEP_CLOCK_PORT);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_BUSY_PIN)
  {
    BSP_MotorControl_BusyInterruptHandler();
  }

  if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_FLAG_PIN)
  {
    BSP_MotorControl_FlagInterruptHandler();
  }

  if (GPIO_Pin == USER_BUTTON_PIN) {
    EmergencyStop = 1;
  }
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
