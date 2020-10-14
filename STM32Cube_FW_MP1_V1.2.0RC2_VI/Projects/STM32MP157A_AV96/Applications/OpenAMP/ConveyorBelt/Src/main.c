/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OpenAMP/ConveyorBelt/Inc/main.c
  * @author  MCD Application Team
  * @brief   Main program body.
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

#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
#define i2c_bus      (&hi2c1)
#define def_i2c_time_out 100
#define MAX_DEV 4					 // max num of 6180 devs
#define BOTTOM_DEV 0				 // bottom 6180 dev
#define RIGHT_DEV  1
#define MOTOR_SPEED          34000   // [Steps/sec]

#define SENSE_ITEM_DISTANCE_mm 100   // [mm]
#define MASTER_CMD_SIZE_B       80   // [char]
#define RX_BUFFER_SIZE_B        80   // [char]
#define MAX_ANSW_MSG_LEN        40   // [char]
#define DISCARD_TIME_ms       1000   // [ms]
#define MOTOR_SPEED          34000   // [Steps/sec]
//#define UNKNOWN_CMD_ECHO           // uncomment to send back to A7 Linux master the unknown cmds (to see the log: cat /sys/kernel/debug/remoteproc/remoteproc0/trace0)
#define NELEMS(x) (sizeof(x) / sizeof((x)[0]))
#define DigitDisplay_ms          1   // ms each digit is kept on

#define BSP_BP_PORT GPIOC            // GPIO used to discard defective pieces
#define BSP_BP_PIN  GPIO_PIN_13

IPCC_HandleTypeDef hipcc;

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;
static int PresentDevIds[MAX_DEV];
static VL6180xDev_t dev;
//VL6180xDev_t theVL6180xDev;
static struct MyVL6180Dev_t BoardDevs[4] = { [0]= { .DevID = 0 }, [1]= { .DevID = 1 }, [2]= { .DevID = 2 }, [3]= { .DevID = 3 }, };
//static VL6180xDev_t theVL6180xDev = &BoardDevs[0];
volatile uint8_t EmergencyStop;
/***************  DISPLAY PUBLIC *********************/
static const char *DISP_NextString;
/***************  DISPLAY PRIVATE *********************/
static char DISP_CurString[10];
static int DISP_Loop = 0;
static I2C_HandleTypeDef hi2c1;
VIRT_UART_HandleTypeDef huart0;
VIRT_UART_HandleTypeDef huart1;
static __IO FlagStatus VirtUart0RxMsg = RESET;
static __IO FlagStatus VirtUart1RxMsg = RESET;
static char VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
static uint16_t VirtUart0ChannelRxSize = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IPCC_Init(void);
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
static void MX_GPIO_Init(void);

void DISP_ExecLoopBody(void) {
    if (DISP_NextString != NULL) {
        strncpy(DISP_CurString, DISP_NextString, sizeof(DISP_CurString) - 1);
        DISP_CurString[sizeof(DISP_CurString) - 1] = 0;
        DISP_NextString = NULL;
    }
    XNUCLEO6180XA1_DisplayString(DISP_CurString, DigitDisplay_ms);
    DISP_Loop++;
}

int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Transmit(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status? -1 : 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Receive(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }

    return status? -1 : 0;
}

void WaitMilliSec(int ms) {
    HAL_Delay(ms); /* it's milli sec  cos we do set systick to 1KHz */
}

/**
 * platform and application specific for XNUCLEO6180XA1 Expansion Board
 */
void XNUCLEO6180XA1_WaitMilliSec(int n) {
    WaitMilliSec(n);
}

volatile int IntrFired = 0;

void XNUCLEO6180XA1_UserIntHandler(void) {
    IntrFired++;
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the command GET_STATUS_CMD */
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check BUSY flag: if not set, a command is under execution */
  if ((statusRegister & POWERSTEP01_STATUS_BUSY) == 0)
  {
    // BUSY
  }

  /* Check SW_F flag: if not set, the SW input is opened */
  if ((statusRegister & POWERSTEP01_STATUS_SW_F ) == 0)
  {
     // SW OPEN
  }
  else
  {
    // SW CLOSED
  }
  /* Check SW_EN bit */
  if ((statusRegister & POWERSTEP01_STATUS_SW_EVN) == POWERSTEP01_STATUS_SW_EVN)
  {
    // switch turn_on event
  }
  /* Check direction bit */
  if ((statusRegister & POWERSTEP01_STATUS_DIR) == 0)
  {
    // BACKWARD
  }
  else
  {
    // FORWARD
  }
  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_STOPPED )
  {
       // MOTOR STOPPED_MSG
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION )
  {
           // MOTOR ACCELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_DECELERATION )
  {
           // MOTOR DECELERATION
  }
  else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS) == POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD )
  {
       // MOTOR RUNNING AT CONSTANT SPEED
  }

  /* Check Command Error flag: if set, the command received by SPI can't be */
  /* performed. This occurs for instance when a move command is sent to the */
  /* Powerstep01 while it is already running */
  if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR) == POWERSTEP01_STATUS_CMD_ERROR)
  {
       // Command Error
  }

  /* Check Step mode clock flag: if set, the device is working in step clock mode */
  if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD) == POWERSTEP01_STATUS_STCK_MOD)
  {
     //Step clock mode enabled
  }

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out
  }

  /* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC) == 0)
  {
     //ADC undervoltage lock-out
  }

  /* Check thermal STATUS flags: if  set, the thermal status is not normal */
  if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS) != 0)
  {
    //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
  }

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection
  }

  /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_A) == 0)
  {
    //overcurrent detection
  }

  /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
  if ((statusRegister & POWERSTEP01_STATUS_STALL_B) == 0)
  {
    //overcurrent detection
  }

}

/**
  * @brief  This function is the User handler for the busy interrupt
  * @param  None
  * @retval None
  */
void MyBusyInterruptHandler(void)
{

   if (BSP_MotorControl_CheckBusyHw())
   {
      /* Busy pin is low, so at list one Powerstep01 chip is busy */
     /* To be customized (for example Switch on a LED) */
   }
   else
   {
     /* To be customized (for example Switch off a LED) */
   }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param[in] error Number of the error
  * @retval None
  */
void MyErrorHandler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;

  /* Infinite loop */
  while(1)
  {
  }
}

void RemovePiece_GPIO_Init() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the GPIO Clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* Configure GPIO pin as output */
  GPIO_InitStruct.Pin = GPIO_PIN_8;  // PC8, PIN2 on morpho connector
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC , &GPIO_InitStruct);
}

typedef enum BeltFSM_state {
  BELT_STOPPED_STATE = 0,
  BELT_RUNNING_FWD_STATE,
  BELT_RUNNING_BWD_STATE
} BeltFSM_state_t;
BeltFSM_state_t belt_fsm_state = BELT_STOPPED_STATE;

typedef enum MasterCmd {
    RUN_FWD_BELT_CMD = 0,
    RUN_BWD_BELT_CMD,
    STOP_BELT_CMD,
    GET_STATUS_CMD,
    DISCARD_CMD,
    GET_LUX_CMD,
    UNKNOWN_CMD
} MasterCmd_t;
char* MasterCmdString[] = {"run_fwd;", "run_bwd;", "stop;", "get_status;", "discard;", "get_lux;"};
int nCmds = NELEMS(MasterCmdString);

MasterCmd_t ParseMasterCmd(char * cmd) {
  int i;
  for (i=0; i<nCmds; i++) {
	  log_info("---> CMD: <%s>\r", cmd);
//	    log_info("MasterCmdString []:%s\r", MasterCmdString [i]);
    if (strcmp (MasterCmdString [i], cmd) == 0)
      return (MasterCmd_t) i;
  }
  return (MasterCmd_t) i;
}

typedef enum SlaveAnsw {
    READY_MSG = 0,
    RUNNING_FWD_MSG,
    RUNNING_BWD_MSG,
    MAN_STOPPED_MSG,
    SENS_STOPPED_MSG,
    EMERGENCY_STOP_MSG,
    REFUSED_MSG,
    DISCARDED_PIECE_MSG,
    FAILED_MSG,
    MEASURED_LUX_MSG,
    UNKNOWN_MSG
} SlaveAnsw_t;
char* SlaveAnswString[] = {"ready;", "running_forward;", "running_backward;", "man_stopped;", "sens_stopped;", "emergency_stop;", "refused;", "discarded;", "failed;", "measured_lux_xxxx;", "unknown;"};

void SendSlaveAnswer(int nargs, ...) {
  assert(nargs >= 1);
  va_list vl;
  va_start(vl, nargs);
  SlaveAnsw_t answ = (SlaveAnsw_t)va_arg(vl, int);

  switch (answ) {
  case RUNNING_FWD_MSG:
  case RUNNING_BWD_MSG:
  case MAN_STOPPED_MSG:
  case REFUSED_MSG:
  case DISCARDED_PIECE_MSG:
  case FAILED_MSG:
  case UNKNOWN_MSG:
    if (VIRT_UART_Transmit(&huart0, (unsigned char *)SlaveAnswString[answ], strlen(SlaveAnswString[answ])) != VIRT_UART_OK ) {
      log_info("Error VIRT_UART_Transmit uart0 answ: %s\n", SlaveAnswString[answ]);
    }
    break;
  case MEASURED_LUX_MSG:
    assert(nargs == 2);
    char strlux[5];
    snprintf (strlux, 5, "%04d", va_arg(vl, int));
    char whole_msg[MAX_ANSW_MSG_LEN];
    strcpy (whole_msg, SlaveAnswString[answ]);
    strncpy(strrchr(whole_msg, '_')+1, strlux, 4);
    if (VIRT_UART_Transmit(&huart0, (unsigned char *)whole_msg, strlen(whole_msg)) != VIRT_UART_OK ) {
      log_info("Error VIRT_UART_Transmit uart0\n");
    }
    break;
  case READY_MSG:        // these are spontaneous notification to be sent on /dev/ttyRPMSG1
  case SENS_STOPPED_MSG:
  case EMERGENCY_STOP_MSG:
    if (VIRT_UART_Transmit(&huart1, (unsigned char *)SlaveAnswString[answ], strlen(SlaveAnswString[answ])) != VIRT_UART_OK ) {
      log_info("Error VIRT_UART_Transmit uart1 answ: <%s>, len: <%d>\n", SlaveAnswString[answ], strlen(SlaveAnswString[answ]));
    }
    break;
  default:
    break;
  }
  va_end(vl);
}

void SendUnknownCmdEcho(char * cmd) {
#ifdef UNKNOWN_CMD_ECHO
	log_info("Entered unknown cmd:%s len:%d", cmd, strlen(cmd));
#endif
}

typedef enum MotorStopType {
  HARD_STOP = 0,
  SOFT_STOP
}MotorStopType_t;

void StopBelt(MotorStopType_t stop) {
  if (stop == HARD_STOP)
    BSP_MotorControl_CmdHardHiZ(0);
  else
    BSP_MotorControl_CmdSoftHiZ(0);
  BSP_MotorControl_WaitWhileActive(0);
}

void RunBeltForward(void) {
  BSP_MotorControl_CmdRun(0, FORWARD, MOTOR_SPEED);
  BSP_MotorControl_WaitWhileActive(0);
}

void RunBeltBackward(void) {
  BSP_MotorControl_CmdRun(0, BACKWARD, MOTOR_SPEED);
  BSP_MotorControl_WaitWhileActive(0);
}


int VuartReceiveString(char ** ppMasterCmdRx) {
  int ret = 0;
  OPENAMP_check_for_message();
  if (VirtUart0RxMsg) {
	  *ppMasterCmdRx = VirtUart0ChannelBuffRx;
	  ret = VirtUart0ChannelRxSize;
	  VirtUart0RxMsg = RESET;
  }
  return ret;
}

void RunBeltFSM() {
  char *MasterCmdRx = NULL;
  int status;
  VL6180x_RangeData_t RangeBottomDev;
  VL6180x_AlsData_t AlsData;

  SendSlaveAnswer(1, READY_MSG);

  do {
    switch (belt_fsm_state) {
      case BELT_STOPPED_STATE:
        EmergencyStop = 0;
        if (VuartReceiveString(&MasterCmdRx) != 0) {

          switch (ParseMasterCmd(MasterCmdRx)) {
            case RUN_FWD_BELT_CMD:
              RunBeltForward();
              SendSlaveAnswer(1, RUNNING_FWD_MSG);
              // read range to move away the previous piece if present
              do {
                do {
                  if (EmergencyStop) {
                    StopBelt(HARD_STOP);
                    SendSlaveAnswer(1, EMERGENCY_STOP_MSG);
                    EmergencyStop = 0;
                    belt_fsm_state = BELT_STOPPED_STATE;
                    break;
                  }
                  dev = BoardDevs + PresentDevIds[BOTTOM_DEV];
                  status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
                  if ( status != 0 )
                    continue;
                } while(RangeBottomDev.errorStatus == DataNotReady);
              } while(RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm);
              belt_fsm_state = BELT_RUNNING_FWD_STATE;
              break;

            case RUN_BWD_BELT_CMD:
              RunBeltBackward();
              SendSlaveAnswer(1, RUNNING_BWD_MSG);
              // read range to move away the previous piece if present
              do {
                do {
                  if (EmergencyStop) {
                    StopBelt(HARD_STOP);
                    SendSlaveAnswer(1, EMERGENCY_STOP_MSG);
                    EmergencyStop = 0;
                    belt_fsm_state = BELT_STOPPED_STATE;
                    break;
                  }
                  dev = BoardDevs + PresentDevIds[BOTTOM_DEV];
                  status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
                  if ( status != 0 )
                    continue;
                } while(RangeBottomDev.errorStatus == DataNotReady);
              } while(RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm);
              belt_fsm_state = BELT_RUNNING_BWD_STATE;
              break;

            case GET_STATUS_CMD:
              SendSlaveAnswer(1, MAN_STOPPED_MSG);
              break;

            case GET_LUX_CMD:
              /* read ALS */
              AlsData.lux = 0;
              if (VL6180x_AlsPollMeasurement(dev, &AlsData) == 0) {
                 SendSlaveAnswer(2, MEASURED_LUX_MSG, AlsData.lux);
              }
              break;

            case DISCARD_CMD:
              // activate GPIO PC8 to remove piece from belt
              dev =  BoardDevs + PresentDevIds[BOTTOM_DEV];
              status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
              if (RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm ) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
                HAL_Delay(DISCARD_TIME_ms);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
                status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
                if (RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm )
                  SendSlaveAnswer(1, FAILED_MSG);
                else
                  SendSlaveAnswer(1, DISCARDED_PIECE_MSG);
              } else { // piece not present: nothing to discard
                 SendSlaveAnswer(1, REFUSED_MSG);
              }
              break;

            case UNKNOWN_CMD:
              SendUnknownCmdEcho(MasterCmdRx);
              SendSlaveAnswer(1, UNKNOWN_MSG);
              break;

            default:
              SendSlaveAnswer(1, REFUSED_MSG);
              break;
          }
//          memset(&MasterCmdRx[0], '\0', MASTER_CMD_SIZE_B);
        }
        break;

      case BELT_RUNNING_FWD_STATE:
        if (VuartReceiveString(&MasterCmdRx) != 0) {
          switch (ParseMasterCmd(MasterCmdRx)) {
            case STOP_BELT_CMD:
              StopBelt(SOFT_STOP);
              SendSlaveAnswer(1, MAN_STOPPED_MSG);
              belt_fsm_state = BELT_STOPPED_STATE;
              break;

            case GET_STATUS_CMD:
              SendSlaveAnswer(1, RUNNING_FWD_MSG);
              break;

            case GET_LUX_CMD:
              /* read ALS */
              AlsData.lux = 0;
              if (VL6180x_AlsPollMeasurement(dev, &AlsData) == 0) {
                 SendSlaveAnswer(2, MEASURED_LUX_MSG, AlsData.lux);
              }
              break;

            case UNKNOWN_CMD:
              SendUnknownCmdEcho(MasterCmdRx);
              SendSlaveAnswer(1, UNKNOWN_MSG);
              break;

            default:
              SendSlaveAnswer(1, REFUSED_MSG);
              break;
          }
//          memset(&MasterCmdRx[0], '\0', MASTER_CMD_SIZE_B);
        } else {
          // read 6180 range
          dev =  BoardDevs + PresentDevIds[BOTTOM_DEV];
          status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
          if (RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm || EmergencyStop) {
            if (EmergencyStop) {
              StopBelt(HARD_STOP);
              SendSlaveAnswer(1, EMERGENCY_STOP_MSG);
            } else {
              StopBelt(SOFT_STOP);
              SendSlaveAnswer(1, SENS_STOPPED_MSG);
            }
            EmergencyStop = 0;
            belt_fsm_state = BELT_STOPPED_STATE;
            break;
          }
        }
        break;

      case BELT_RUNNING_BWD_STATE:
        if (VuartReceiveString(&MasterCmdRx) != 0) {
          switch (ParseMasterCmd(MasterCmdRx)) {
            case STOP_BELT_CMD:
              StopBelt(SOFT_STOP);
              SendSlaveAnswer(1, MAN_STOPPED_MSG);
              belt_fsm_state = BELT_STOPPED_STATE;
              break;

            case GET_STATUS_CMD:
              SendSlaveAnswer(1, RUNNING_BWD_MSG);
              break;

            case GET_LUX_CMD:
              /* read ALS */
              AlsData.lux = 0;
              if (VL6180x_AlsPollMeasurement(dev, &AlsData) == 0) {
                 SendSlaveAnswer(2, MEASURED_LUX_MSG, AlsData.lux);
              }
              break;

            case UNKNOWN_CMD:
              SendUnknownCmdEcho(MasterCmdRx);
              SendSlaveAnswer(1, UNKNOWN_MSG);
              break;

            default:
              SendSlaveAnswer(1, REFUSED_MSG);
              break;
          }
//          memset(&MasterCmdRx[0], '\0', MASTER_CMD_SIZE_B);
        } else {
          // read 6180 range
          dev =  BoardDevs + PresentDevIds[BOTTOM_DEV];
          status = VL6180x_RangeGetMeasurementIfReady(dev, &RangeBottomDev);
          if (RangeBottomDev.range_mm < SENSE_ITEM_DISTANCE_mm || EmergencyStop) {
            if (EmergencyStop) {
              StopBelt(HARD_STOP);
              SendSlaveAnswer(1, EMERGENCY_STOP_MSG);
            } else {
              StopBelt(SOFT_STOP);
              SendSlaveAnswer(1, SENS_STOPPED_MSG);
            }
            EmergencyStop = 0;
            belt_fsm_state = BELT_STOPPED_STATE;
            break;
          }
        }
        break;
    }
  } while (1);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int i, n_dev, PresentDevMask, nPresentDevs, status;
  char DisplayStr[5];

  /* Reset of all peripherals, Initialize the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  log_info("Cortex-M4 boot successful with STM32Cube FW version: v%ld.%ld.%ld \r\n",
                                            ((HAL_GetHalVersion() >> 24) & 0x000000FF),
                                            ((HAL_GetHalVersion() >> 16) & 0x000000FF),
                                            ((HAL_GetHalVersion() >> 8) & 0x000000FF));
  /* USER CODE END Init */

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* IPCC initialisation */
   MX_IPCC_Init();
  /* OpenAmp initialisation ---------------------------------*/
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  //XNUCLEO6180XA1_GPIO_Init();
  XNUCLEO6180XA1_I2C1_Init(&hi2c1);

  /* SysTick end of count event each 1ms */
//  SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000);
  if (!XNUCLEO6180XA1_IsV2()) {
      log_err("V2 expansion board is required");
      n_dev=1;
  } else {
	  log_info("V2 expansion board detected \r\n");
      n_dev = 4;
  }

  /* put all 6180 devices under reset */
  for (int i = 0; i < n_dev; i++) {
      /* put all under reset */
      XNUCLEO6180XA1_ResetId(0, i);
  }

  /* detect presence and initialize devices i2c address  */
  /*set device i2c address for dev[i] = 0x52+(i+1)*2 */
  PresentDevMask = 0;
  nPresentDevs = 0;
  strcpy(DisplayStr,"TLBR");
  for (i = n_dev - 1; i >= 0; i--) {
      int FinalI2cAddr;
      uint8_t id;

      /* unreset device that wake up at default i2c addres 0x52 */
      XNUCLEO6180XA1_ResetId(1, i);
      WaitMilliSec(2);    /* at least 400usec before to acces device */
      BoardDevs[i].I2cAddr = 0x52;
      /* to detect device presence try to read it's dev id */
      status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
      if (status) {
          /* these device is not present skip init and clear it's letter on string */
          BoardDevs[i].Present = 0;
          DisplayStr[i]=' ';
          continue;
      }
      /* device present only */
      if (i == 2 || i == 3) { /* Licio init bottom 2 dev  and right 3 only */
        BoardDevs[i].Present = 1;
        PresentDevMask |= 1 << i;
        PresentDevIds[nPresentDevs]=i;
        nPresentDevs++;
        status = VL6180x_InitData(&BoardDevs[i]);

        FinalI2cAddr = 0x52 + ((i+1) * 2);
        if (FinalI2cAddr != 0x52) {
          status = VL6180x_SetI2CAddress(&BoardDevs[i], FinalI2cAddr);
          if( status ){
            log_err("VL6180x_SetI2CAddress fail");
          }
          BoardDevs[i].I2cAddr = FinalI2cAddr;
        }

        WaitMilliSec(1);
        status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
        WaitMilliSec(1);
        status= VL6180x_Prepare(&BoardDevs[i]);
        if( status<0 ){
          log_err("VL6180x_Prepare fail");
        }
        /* Disable Dmax computation */
        VL6180x_DMaxSetState(&BoardDevs[i], 0);
      }
  }
  DisplayStr[4]=0;
  for( i=0; i<nPresentDevs; i++){
    dev =  BoardDevs + PresentDevIds[i];
    status = VL6180x_RangeStartContinuousMode(dev);
    if( status<0 ){
      log_err("VL6180x_RangeStartContinuousMode fail");
    }
    dev->Ready=0;
  }
//===============================================================================

  //----- Init of the Powerstep01 library
    /* Set the Powerstep01 library to use 1 device */
    BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, 1);
    /* When BSP_MotorControl_Init is called with NULL pointer,                  */
    /* the Powerstep01 registers are set with the predefined values from file   */
    /* powerstep01_target_config.h, otherwise the registers are set using the   */
    /* powerstep01_Init_u relevant union of structure values.                   */
    /* The first call to BSP_MotorControl_Init initializes the first device     */
    /* whose Id is 0.                                                           */
    /* The nth call to BSP_MotorControl_Init initializes the nth device         */
    /* whose Id is n-1.                                                         */
    /* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
    /* device with the union declared in the the main.c file and comment the    */
    /* subsequent call having the NULL pointer                                  */
    //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, &initDeviceParameters);
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_POWERSTEP01, NULL);

    /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
    BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

    /* Attach the function MyBusyInterruptHandler (defined below) to the busy interrupt */
    BSP_MotorControl_AttachBusyInterrupt(MyBusyInterruptHandler);

    /* Attach the function Error_Handler (defined below) to the error Handler*/
    BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

    /* Move device 0 of 16000 steps in the FORWARD direction*/
//    while(1) {
//    	BSP_MotorControl_Move(0, FORWARD, 160);
//    	BSP_MotorControl_WaitWhileActive(0);
//    }
   /* Create Virtual UART device
   * defined by a rpmsg channel attached to the remote device
   */
  log_info("Virtual UART0 (/dev/ttyRPMSG0) OpenAMP-rpmsg cmd channel creation\r\n");
  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART0 failed.\r\n");
    Error_Handler();
  }

  /*Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
   Error_Handler();
  }

  log_info("Virtual UART1 (/dev/ttyRPMSG1) OpenAMP-rpmsg answ channel creation\r\n");
  if (VIRT_UART_Init(&huart1) != VIRT_UART_OK) {
    log_err("VIRT_UART_Init UART1 failed.\r\n");
    Error_Handler();
  }

  if(VIRT_UART_RegisterCallback(&huart1, VIRT_UART_RXCPLT_CB_ID, VIRT_UART1_RxCpltCallback) != VIRT_UART_OK)
  {
    Error_Handler();
  }

  /* Infinite loop run FSM */
  while (1)
  {
    RunBeltFSM();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

  /**PLL1 Config
  */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
  RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL2 Config
    */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL3 Config
    */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL4 Config
    */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  Error_Handler();
  }
  /**RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
  Error_Handler();
  }

  /**Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief IPPC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
     Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    log_info("Msg received on VIRTUAL UART0 channel: <%s>\tSize: %d\n\r", (char *) huart->pRxBuffPtr, huart->RxXferSize);
    /* copy received msg in a variable to sent it back to master processor in main infinite loop*/
    VirtUart0ChannelRxSize = huart->RxXferSize < MAX_BUFFER_SIZE? huart->RxXferSize : MAX_BUFFER_SIZE-1;
    memcpy(VirtUart0ChannelBuffRx, huart->pRxBuffPtr, VirtUart0ChannelRxSize);
    char* p = strchr(VirtUart0ChannelBuffRx, ((int) ';'));
    if (p != NULL) {
		*(p + 1) = '\0';
	} else {
		VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize] = ';';
		VirtUart0ChannelBuffRx[VirtUart0ChannelRxSize + 1] = '\0';
    }
    VirtUart0RxMsg = SET;
}

void VIRT_UART1_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
    log_info("Msg received on VIRTUAL UART1 channel: <%s>\tSize: %d\n\r", (char *) huart->pRxBuffPtr, huart->RxXferSize);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  log_err("Error_Handler");
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  log_err("OOOps: file %s, line %d\r\n", __FILE__, __LINE__);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
