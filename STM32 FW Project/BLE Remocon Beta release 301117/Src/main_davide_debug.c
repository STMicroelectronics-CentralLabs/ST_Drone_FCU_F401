/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "debug.h"
#include "config_drone.h"
#include "timer.h"
#include "rc.h"
#include "motor.h"
//#include "stm32f4xx_nucleo.h"
//#include "board.h"
#include "steval_fcu001_v1.h"
#include "steval_fcu001_v1_accelero.h"
#include "steval_fcu001_v1_gyro.h"
#include "steval_fcu001_v1_magneto.h"
#include "steval_fcu001_v1_pressure.h"

#include "sensor_data.h"
#include "quaternion.h"
#include "ahrs.h"
#include "flight_control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;
//static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
//static void *LPS22HB_T_0_handle = NULL; 


/* USER CODE BEGIN PV */
extern Queue_TypeDef que;
extern volatile tUserTimer tim;
extern char rc_connection_flag;
extern int16_t gAIL, gELE, gTHR, gRUD;
int32_t rc_cal_flag = 0;
int32_t rc_enable_motor = 0;
int32_t rc_cal_cnt = 0;
int32_t fly_ready = 0;
unsigned char ch, ch_flag;

uint32_t tim9_event_flag = 0, tim9_cnt = 0, tim9_cnt2 = 0;
float tmp_euler_z = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);

static void initializeAllSensors( void );
void enableAllSensors( void );

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
P_PI_PIDControlTypeDef pid;
EulerAngleTypeDef euler_rc, euler_ahrs, euler_rc_fil, euler_rc_y_pre[4], euler_rc_x_pre[4];
AxesRaw_TypeDef acc, gyro, mag, acc_fil_int, gyro_fil_int, mag_fil_int;
AxesRaw_TypeDef_Float acc_fil, acc_y_pre[4], acc_x_pre[4], acc_ahrs_FIFO[FIFO_Order], acc_FIFO[FIFO_Order], acc_ahrs;
AxesRaw_TypeDef_Float gyro_fil, gyro_y_pre[4], gyro_x_pre[4], gyro_ahrs_FIFO[FIFO_Order], gyro_FIFO[FIFO_Order], gyro_ahrs;
AxesRaw_TypeDef_Float mag_fil;
AxesRaw_TypeDef acc_off_calc, gyro_off_calc, acc_offset, gyro_offset;
EulerAngleTypeDef euler_ahrs_offset;
int sensor_init_cali = 0, sensor_init_cali_count = 0;
int gyro_cali_count = 0;

typedef struct
{
  int16_t X_Degree;
  int16_t Y_Degree;
  int16_t Z_Degree;
}Attitude_Degree;

typedef struct
{
  float a1, a2, b0, b1, b2;
}IIR_Coeff;

//rc filter
//1hz
//IIR_Coeff euler_rc_fil_coeff = {3.1806385488747191,  -3.8611943489942142, 2.1121553551109691,  -0.43826514226197993, 0.00041659920440659937, 0.0016663968176263975 , 0.0024995952264395961 , 0.0016663968176263975 , 0.00041659920440659937};
//2hz
//IIR_Coeff euler_rc_fil_coeff = {2.3695130071820381,  -2.3139884144158804, 1.0546654058785678,  -0.18737949236818496, 0.0048243433577162273, 0.019297373430864909 , 0.028946060146297362 , 0.019297373430864909 , 0.0048243433577162273};
//4hz
//IIR_Coeff euler_rc_fil_coeff = {0.78209519802333793,  -0.6799785269162999, 0.1826756977530325,  -0.03011887504316928, 0.046582906636443676, 0.1863316265457747, 0.27949743981866204 , 0.1863316265457747  , 0.046582906636443676};

//sensor filter
//7hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.922286512869545,  -0.92519529534950118, 0.00072719561998898304, 0.0014543912399779661, 0.00072719561998898304};

//15hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.8337326589246479,  -0.84653197479202391, 0.003199828966843966, 0.0063996579336879321, 0.003199828966843966};

//30hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.66920314293119312,  -0.71663387350415764, 0.011857682643241156, 0.023715365286482312, 0.011857682643241156};

//60hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.3489677452527946 ,  -0.51398189421967566, 0.041253537241720303, 0.082507074483440607, 0.041253537241720303};

//100hz, 800hz
IIR_Coeff gyro_fil_coeff = {0.94280904158206336,  -0.33333333333333343, 0.09763107293781749 , 0.19526214587563498 , 0.09763107293781749 };

Attitude_Degree  Fly, Fly_offset, Fly_origin;
Gyro_Rad gyro_rad, gyro_degree, gyro_cali_degree;
MotorControlTypeDef motor_pwm;
int count1 = 0, count2 = 0;
AHRS_State_TypeDef ahrs;
float pre;

uint32_t VBAT_Sense;
float VBAT = 0;
/* USER CODE END 0 */


 int main(void)
{

  /* USER CODE BEGIN 1 */
  int16_t idx, value, pid_interval, i, j;
  int ret;
  float tmp_f;

  int mytimcnt = 0;
  acc_fil.AXIS_X = 0;
  acc_fil.AXIS_Y = 0;
  acc_fil.AXIS_Z = 0;
  mag_fil.AXIS_X = 0;
  mag_fil.AXIS_Y = 0;
  mag_fil.AXIS_Z = 0;
  gyro_fil.AXIS_X = 0;
  gyro_fil.AXIS_Y = 0;
  gyro_fil.AXIS_Z = 0;
  euler_rc_fil.thx = 0;
  euler_rc_fil.thy = 0;
  euler_rc_fil.thz = 0;
  acc_off_calc.AXIS_X = 0;
  acc_off_calc.AXIS_Y = 0;
  acc_off_calc.AXIS_Z = 0;
  gyro_off_calc.AXIS_X = 0;
  gyro_off_calc.AXIS_Y = 0;
  gyro_off_calc.AXIS_Z = 0;
  acc_offset.AXIS_X = 0;
  acc_offset.AXIS_Y = 0;
  acc_offset.AXIS_Z = 1000;
  gyro_offset.AXIS_X = 0;
  gyro_offset.AXIS_Y = 0;
  gyro_offset.AXIS_Z = 0;
  euler_rc.thz = euler_ahrs.thz;
  euler_ahrs_offset.thx = 0;
  euler_ahrs_offset.thy = 0;

  for(i=0;i<4;i++)
  {
    acc_y_pre[i].AXIS_X = 0;
    acc_y_pre[i].AXIS_Y = 0;
    acc_y_pre[i].AXIS_Z = 0;
    acc_x_pre[i].AXIS_X = 0;
    acc_x_pre[i].AXIS_Y = 0;
    acc_x_pre[i].AXIS_Z = 0;
    gyro_y_pre[i].AXIS_X = 0;
    gyro_y_pre[i].AXIS_Y = 0;
    gyro_y_pre[i].AXIS_Z = 0;
    gyro_x_pre[i].AXIS_X = 0;
    gyro_x_pre[i].AXIS_Y = 0;
    gyro_x_pre[i].AXIS_Z = 0;
    euler_rc_y_pre[i].thx = 0;
    euler_rc_y_pre[i].thy = 0;
    euler_rc_y_pre[i].thz = 0;
    euler_rc_x_pre[i].thx = 0;
    euler_rc_x_pre[i].thy = 0;
    euler_rc_x_pre[i].thz = 0;
  }

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  HAL_Delay(1000);
  
  while(1);
  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_ADC1_Init();  
//  //MX_SPI1_Init();
//  //MX_SPI2_Init();
//  MX_TIM2_Init();
//  MX_TIM4_Init();
//  MX_TIM9_Init();
//  MX_USART1_UART_Init();
  //MX_USB_DEVICE_Init();
 
  /* USER CODE BEGIN 2 */

//  PRINTF("FCU F401 rev.1.1 - 2016\n\n");
  
//  // Initialize Onboard LLED
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
  BSP_LED_On(LED1);
  BSP_LED_On(LED2);
  

  /* Update - 31/01/17 - New sensors initialization */
  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  

  /* Initialize and Enable the available sensors */
  initializeAllSensors();
  enableAllSensors();
  

  
  /* DEBUG Sensor data read */
//  while (1){
//    BSP_PRESSURE_Get_Press(LPS22HB_P_0_handle, &pre);
//  }
//  
  while (1)
  {
    ReadSensorRawData(LSM6DSL_X_0_handle, LSM6DSL_G_0_handle, LSM303AGR_M_0_handle, LPS22HB_P_0_handle, &acc, &gyro, &mag, &pre);
  }
  
//  ret = BSP_IMU_6AXES_Init();
//  if (!ret)
//    PRINTF("> BSP_IMU_6AXES_Init() done\n");
//  else
//    PRINTF("*** BSP_IMU_6AXES_Init() error\n");
//
//  ret = BSP_MAGNETO_Init();
//  if (!ret)
//    PRINTF("> BSP_MAGNETO_Init() done\n");
//  else
//    PRINTF("*** BSP_MAGNETO_Init() error\n");
//
//  ret = BSP_PRESSURE_Init();
//  if (!ret)
//    PRINTF("> BSP_PRESSURE_Init() done\n");
//  else
//    PRINTF("*** BSP_PRESSURE_Init() error\n");

  init_remote_control();

  PRINTF("Test...\n");
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim9);

  PIDControlInit(&pid);
  set_motor_pwm_zero(&motor_pwm);

  // Setup a timer with 5ms interval
  pid_interval = PID_SAMPLING_TIME*1000;
  SetupTimer(&tim, pid_interval);

  // Start timer
  StartTimer(&tim);
  ch = 0;
  ch_flag = 0;
  
  uint16_t pwm;
  pwm =0;
  int16_t gTHR_last = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (tim9_event_flag == 1)
    {     // Timer9 event: frequency 50Hz
      tim9_event_flag = 0;

      count1++;
           
      acc_ahrs.AXIS_X = 0;
      acc_ahrs.AXIS_Y = 0;
      acc_ahrs.AXIS_Z = 0;
      gyro_ahrs.AXIS_X = 0;
      gyro_ahrs.AXIS_Y = 0;
      gyro_ahrs.AXIS_Z = 0;

      for(i=0;i<FIFO_Order;i++)
      {
        acc_ahrs.AXIS_X += acc_ahrs_FIFO[i].AXIS_X;
        acc_ahrs.AXIS_Y += acc_ahrs_FIFO[i].AXIS_Y;
        acc_ahrs.AXIS_Z += acc_ahrs_FIFO[i].AXIS_Z;
        gyro_ahrs.AXIS_X += gyro_ahrs_FIFO[i].AXIS_X;
        gyro_ahrs.AXIS_Y += gyro_ahrs_FIFO[i].AXIS_Y;
        gyro_ahrs.AXIS_Z += gyro_ahrs_FIFO[i].AXIS_Z;
      }

      acc_ahrs.AXIS_X *=FIFO_Order_Recip;
      acc_ahrs.AXIS_Y *=FIFO_Order_Recip;
      acc_ahrs.AXIS_Z *=FIFO_Order_Recip;
      gyro_ahrs.AXIS_X *=FIFO_Order_Recip;
      gyro_ahrs.AXIS_Y *=FIFO_Order_Recip;
      gyro_ahrs.AXIS_Z *=FIFO_Order_Recip;

      acc_fil_int.AXIS_X = acc_ahrs.AXIS_X;
      acc_fil_int.AXIS_Y = acc_ahrs.AXIS_Y;
      acc_fil_int.AXIS_Z = acc_ahrs.AXIS_Z;
      gyro_fil_int.AXIS_X = gyro_ahrs.AXIS_X;
      gyro_fil_int.AXIS_Y = gyro_ahrs.AXIS_Y;
      gyro_fil_int.AXIS_Z = gyro_ahrs.AXIS_Z;


      //PRINTF("%f %f %f %f\n", acc_ahrs.AXIS_X, acc_ahrs.AXIS_Y, gyro_ahrs.AXIS_X, gyro_ahrs.AXIS_Y);

      // AHRS update, quaternion & true gyro data are stored in ahrs
      ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);

      // Calculate euler angle drone
      QuaternionToEuler(&ahrs.q, &euler_ahrs);

      // Get target euler angle from remote control
      GetTargetEulerAngle(&euler_rc, &euler_ahrs);


      if(gTHR<MIN_THR)
      {
        euler_ahrs_offset.thx = 0;
        euler_ahrs_offset.thy = 0;
      }

      Fly_origin.X_Degree = (int16_t)(euler_ahrs.thx * 5730);
      Fly_origin.Y_Degree = (int16_t)(euler_ahrs.thy * 5730);
      Fly_origin.Z_Degree = (int16_t)(euler_ahrs.thz * 5730);


      if(gTHR<MIN_THR)
      {
        euler_rc.thz = 0;
        euler_ahrs.thz = 0;
      }

      euler_rc_fil.thx = euler_rc.thx;
      euler_rc_fil.thy = euler_rc.thy;
      euler_rc_fil.thz = euler_rc.thz;

      FlightControlPID_OuterLoop(&euler_rc_fil, &euler_ahrs, &ahrs, &pid);

    }

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
  {
    ch_flag = 1;
  }

  if (isTimerEventExist(&tim))    // Check if a timer event is present
  {

        ClearTimer(&tim);           // Clear current event;

        count2++;

        mytimcnt++;
        if (rc_connection_flag && rc_enable_motor)
        {
          if (mytimcnt%50 == 0)
          BSP_LED_On(LED2);
        }
        else
        {
          if (mytimcnt%50 == 0)
            BSP_LED_Toggle(LED2);
        }
    }
  
//    /* Update 04/12/16 - Added for debug on UART*/
//    PRINTF("ELE = %d\t",gELE);
//    PRINTF("AIL = %d\t",gAIL);
//    PRINTF("RUD = %d\t",gRUD);
//    PRINTF("THR = %d\n\n",gTHR);
//     
//    PRINTF("Drone_Pitch = %f\tDrone_Roll = %f\n\n", euler_ahrs.thx * 57.3, euler_ahrs.thy * 57.3);
//    
//    /* Update 05/12/16 - Added for Pressure data on UART for debug*/
//    //BSP_PRESSURE_GetPressure(PRESSURE_Value);
//    PRINTF("Pressure [atm] = %f\n\n",pre);  
//    /* Update 05/12/16 - Added for Magnetometer data on UART for debug*/
//    PRINTF("Magnetometer X = %d\tY = %d\tZ = %d\n\n", mag.AXIS_X, mag.AXIS_Y, mag.AXIS_Z);  
//    
//    /* Update 05/12/16 - Added reading of Battery voltage and data on UART */
//    //VBAT_Sense = HAL_ADC_GetValue(&hadc1);
//    HAL_ADC_Start(&hadc1);
//        if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
//        {
//            VBAT_Sense = HAL_ADC_GetValue(&hadc1);
//            VBAT = (((VBAT_Sense*3.3)/4095)*(BAT_RUP+BAT_RDW))/BAT_RDW;
//            PRINTF("Battery voltage = %fV\n\n", VBAT);
//        }
//    HAL_ADC_Stop(&hadc1);
    
    
    if (get_queue(&que, &idx, &value) != -1)
    {
        //if (idx == 0)
        //PRINTF("Channel(%d) = %d\n", idx, value);
    }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//
//  __PWR_CLK_ENABLE();
//
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 16;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//  RCC_OscInitStruct.PLL.PLLQ = 7;
//  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
//
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;  // Nucleo
  RCC_OscInitStruct.PLL.PLLM = 16; // FCU001
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  //RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

  
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  //RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
//  {
//    Error_Handler();
//  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32767;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim4.Init.Period = 1999;                                   /* Default value */
  htim4.Init.Period = 1500;                                     /* Higher freq for DC coreless motor no gear */
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 51;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
//  __GPIOC_CLK_ENABLE();
//  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

//  /*Configure GPIO pins : PC13 PC14 PC15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//  /*Configure GPIO pin : PA4 */
//  GPIO_InitStruct.Pin = GPIO_PIN_4;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PB0 PB2 PB10 PB12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  /*Configure GPIO pin : PA8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PA15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  /*Configure GPIO pin : PB5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 *  Handle Timer9 interrupt @ 800Hz
 *  Set the event flag and increase time index
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(sensor_init_cali == 0)
  {
    sensor_init_cali_count++;

    if(sensor_init_cali_count > 800)
    {
      // Read sensor data and prepare for specific coodinate system
      ReadSensorRawData(LSM6DSL_X_0_handle, LSM6DSL_G_0_handle, LSM303AGR_M_0_handle, LPS22HB_P_0_handle, &acc, &gyro, &mag, &pre);

      acc_off_calc.AXIS_X += acc.AXIS_X;
      acc_off_calc.AXIS_Y += acc.AXIS_Y;
      acc_off_calc.AXIS_Z += acc.AXIS_Z;

      gyro_off_calc.AXIS_X += gyro.AXIS_X;
      gyro_off_calc.AXIS_Y += gyro.AXIS_Y;
      gyro_off_calc.AXIS_Z += gyro.AXIS_Z;

      if (sensor_init_cali_count >= 1600)
      {
        acc_offset.AXIS_X = acc_off_calc.AXIS_X * 0.00125;
        acc_offset.AXIS_Y = acc_off_calc.AXIS_Y * 0.00125;
        acc_offset.AXIS_Z = acc_off_calc.AXIS_Z * 0.00125;

        gyro_offset.AXIS_X = gyro_off_calc.AXIS_X * 0.00125;
        gyro_offset.AXIS_Y = gyro_off_calc.AXIS_Y * 0.00125;
        gyro_offset.AXIS_Z = gyro_off_calc.AXIS_Z * 0.00125;

        acc_off_calc.AXIS_X = 0;
        acc_off_calc.AXIS_Y = 0;
        acc_off_calc.AXIS_Z = 0;
        gyro_off_calc.AXIS_X = 0;
        gyro_off_calc.AXIS_Y = 0;
        gyro_off_calc.AXIS_Z = 0;

        sensor_init_cali_count = 0;
        sensor_init_cali = 1;
      }
    }
  }

  if(sensor_init_cali == 1)
  {
    tim9_cnt++;
    tim9_cnt2++;

    // Read sensor data and prepare for specific coodinate system
    ReadSensorRawData(LSM6DSL_X_0_handle, LSM6DSL_G_0_handle, LSM303AGR_M_0_handle, LPS22HB_P_0_handle, &acc, &gyro, &mag, &pre);

    if (rc_cal_flag == 1)
    {
      acc_off_calc.AXIS_X += acc.AXIS_X;
      acc_off_calc.AXIS_Y += acc.AXIS_Y;
      acc_off_calc.AXIS_Z += acc.AXIS_Z;

      gyro_off_calc.AXIS_X += gyro.AXIS_X;
      gyro_off_calc.AXIS_Y += gyro.AXIS_Y;
      gyro_off_calc.AXIS_Z += gyro.AXIS_Z;

      rc_cal_cnt++;

      if (rc_cal_cnt >= 800)
      {
        acc_offset.AXIS_X = acc_off_calc.AXIS_X * 0.00125;
        acc_offset.AXIS_Y = acc_off_calc.AXIS_Y * 0.00125;
        acc_offset.AXIS_Z = acc_off_calc.AXIS_Z * 0.00125;

        gyro_offset.AXIS_X = gyro_off_calc.AXIS_X * 0.00125;
        gyro_offset.AXIS_Y = gyro_off_calc.AXIS_Y * 0.00125;
        gyro_offset.AXIS_Z = gyro_off_calc.AXIS_Z * 0.00125;

        acc_off_calc.AXIS_X = 0;
        acc_off_calc.AXIS_Y = 0;
        acc_off_calc.AXIS_Z = 0;
        gyro_off_calc.AXIS_X = 0;
        gyro_off_calc.AXIS_Y = 0;
        gyro_off_calc.AXIS_Z = 0;

        rc_cal_cnt = 0;
        rc_cal_flag = 0;
      }
    }

    acc.AXIS_X -= acc_offset.AXIS_X;
    acc.AXIS_Y -= acc_offset.AXIS_Y;
    acc.AXIS_Z -= (acc_offset.AXIS_Z - 1000);
    gyro.AXIS_X -= gyro_offset.AXIS_X;
    gyro.AXIS_Y -= gyro_offset.AXIS_Y;
    gyro.AXIS_Z -= gyro_offset.AXIS_Z;

    // Save filtered data to acc_FIFO
    acc_FIFO[tim9_cnt2-1].AXIS_X = acc.AXIS_X;
    acc_FIFO[tim9_cnt2-1].AXIS_Y = acc.AXIS_Y;
    acc_FIFO[tim9_cnt2-1].AXIS_Z = acc.AXIS_Z;

    // IIR Filtering on gyro
    gyro_fil.AXIS_X = gyro_fil_coeff.b0*gyro.AXIS_X + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_X + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_X
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_X + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_X;
    gyro_fil.AXIS_Y = gyro_fil_coeff.b0*gyro.AXIS_Y + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_Y + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_Y
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_Y + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_Y;
    gyro_fil.AXIS_Z = gyro_fil_coeff.b0*gyro.AXIS_Z + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_Z + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_Z
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_Z + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_Z;
    // Shift IIR filter state
    for(int i=1;i>0;i--)
    {
      gyro_x_pre[i].AXIS_X = gyro_x_pre[i-1].AXIS_X;
      gyro_x_pre[i].AXIS_Y = gyro_x_pre[i-1].AXIS_Y;
      gyro_x_pre[i].AXIS_Z = gyro_x_pre[i-1].AXIS_Z;
      gyro_y_pre[i].AXIS_X = gyro_y_pre[i-1].AXIS_X;
      gyro_y_pre[i].AXIS_Y = gyro_y_pre[i-1].AXIS_Y;
      gyro_y_pre[i].AXIS_Z = gyro_y_pre[i-1].AXIS_Z;
    }
    gyro_x_pre[0].AXIS_X = gyro.AXIS_X;
    gyro_x_pre[0].AXIS_Y = gyro.AXIS_Y;
    gyro_x_pre[0].AXIS_Z = gyro.AXIS_Z;
    gyro_y_pre[0].AXIS_X = gyro_fil.AXIS_X;
    gyro_y_pre[0].AXIS_Y = gyro_fil.AXIS_Y;
    gyro_y_pre[0].AXIS_Z = gyro_fil.AXIS_Z;

    //  Save filtered data to gyro_FIFO
    gyro_FIFO[tim9_cnt2-1].AXIS_X = gyro_fil.AXIS_X;
    gyro_FIFO[tim9_cnt2-1].AXIS_Y = gyro_fil.AXIS_Y;
    gyro_FIFO[tim9_cnt2-1].AXIS_Z = gyro_fil.AXIS_Z;
    

    if(tim9_cnt2 == FIFO_Order)
    {
      tim9_cnt2 = 0;
      tim9_event_flag = 1;
      for(int i=0;i<FIFO_Order;i++)
      {
        acc_ahrs_FIFO[i].AXIS_X = acc_FIFO[i].AXIS_X;
        acc_ahrs_FIFO[i].AXIS_Y = acc_FIFO[i].AXIS_Y;
        acc_ahrs_FIFO[i].AXIS_Z = acc_FIFO[i].AXIS_Z;
        gyro_ahrs_FIFO[i].AXIS_X = gyro_FIFO[i].AXIS_X;
        gyro_ahrs_FIFO[i].AXIS_Y = gyro_FIFO[i].AXIS_Y;
        gyro_ahrs_FIFO[i].AXIS_Z = gyro_FIFO[i].AXIS_Z;
      }
    }

      
      gyro_rad.gx = gyro_fil.AXIS_X*COE_MDPS_TO_RADPS;
      gyro_rad.gy = gyro_fil.AXIS_Y*COE_MDPS_TO_RADPS;
      gyro_rad.gz = gyro_fil.AXIS_Z*COE_MDPS_TO_RADPS;

      euler_ahrs.thz += gyro_rad.gz*PID_SAMPLING_TIME;

      if(gTHR<MIN_THR)
      {
        euler_rc.thz = 0;
        euler_ahrs.thz = 0;
      }


      if (rc_connection_flag && rc_enable_motor)
      {   // Do PID Control
        FlightControlPID_innerLoop(&euler_rc_fil, &gyro_rad, &ahrs, &pid, &motor_pwm);
      }
      else
      {
        // set motor output zero
        set_motor_pwm_zero(&motor_pwm);
      }

      if(gTHR<MIN_THR)
      {
        set_motor_pwm_zero(&motor_pwm);
      }

      set_motor_pwm(&motor_pwm);
  }
}


/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeAllSensors( void )
{
  if (BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_GYRO_Init( LSM6DSL_G_0, &LSM6DSL_G_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
//  if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
//  {
//    while(1);
//  }
//  
//  if (BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle ) != COMPONENT_OK)
//  {
//    while(1);
//  }
//  
  if (BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
//  if (BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ) != COMPONENT_OK)
//  {
//    while(1);
//  }
//  
//  if(BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle ) == COMPONENT_ERROR)
//  {
//    no_T_HTS221 = 1;
//  }
//  
//  if(BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle ) == COMPONENT_ERROR)
//  {
//    no_H_HTS221 = 1;
//  }
//  
//  /* Inialize the Gas Gauge if the battery is present */
//  if(BSP_GG_Init(&GG_handle) == COMPONENT_ERROR)
//  {
//    no_GG=1;
//  }
//  
//  if(!SendOverUSB)
//  {
//    /* Enable HW Double Tap detection */
//    BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSL_X_0_handle);
//    BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSL_X_0_handle, LSM6DSL_TAP_THRESHOLD_MID);
//  }
//  
  
}

/**
* @brief  Enable all sensors
* @param  None
* @retval None
*/
void enableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
  BSP_GYRO_Sensor_Enable( LSM6DSL_G_0_handle );
//  BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
//  BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
  BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
//  BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
//  if(!no_T_HTS221)
//  {
//    BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
//    BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
//  }
  
}



/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
