#include "board.h"
#include "lsm6ds33.h"
#include "lis3mdl.h"
#include "lps25hb.h"
#include "pressure.h"

#ifndef NULL
  #define NULL      (void *) 0
#endif
/**
 * @}
 */

extern uint32_t tim9_event_flag, tim9_cnt;

extern SPI_HandleTypeDef hspi2;
#define SENSOR_SPI_HANDLE   (&hspi2)

/* Link function for 6 Axes IMU peripheral */
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Init( void );
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Write( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite );
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Read( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead );
static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Init(void);
static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite);
static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);

MAGNETO_StatusTypeDef LIS3MDL_IO_Init(void);
MAGNETO_StatusTypeDef LIS3MDL_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite);
MAGNETO_StatusTypeDef LIS3MDL_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);
static MAGNETO_StatusTypeDef MAGNETO_IO_Init(void);
static MAGNETO_StatusTypeDef MAGNETO_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite);
static MAGNETO_StatusTypeDef MAGNETO_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);

PRESSURE_StatusTypeDef LPS25HB_IO_Init(void);
PRESSURE_StatusTypeDef LPS25HB_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite);
PRESSURE_StatusTypeDef LPS25HB_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);
static PRESSURE_StatusTypeDef PRESSURE_IO_Init(void);
static PRESSURE_StatusTypeDef PRESSURE_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite);
static PRESSURE_StatusTypeDef PRESSURE_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead);


GPIO_TypeDef* SPI_SENSOR_NSS_PORT[SENSOR_SPI_NUMBER] = { LSM6DSL_NSS_PORT, LIS2MDL_NSS_PORT, LPS22HB_NSS_PORT };
const uint16_t SPI_SENSOR_NSS_PIN[SENSOR_SPI_NUMBER] = { LSM6DSL_NSS_PIN,  LIS2MDL_NSS_PIN,  LPS22HB_NSS_PIN  };


/** @defgroup X_NUCLEO_IKS01A1_IMU_6AXES_Private_Variables X_NUCLEO_IKS01A1_IMU_6AXES_Private_Variables
 * @{
 */
static IMU_6AXES_DrvTypeDef *Imu6AxesDrv = NULL;
static uint8_t Imu6AxesInitialized = 0;
static uint8_t imu_6axes_sensor_type = 1; /* 1 activates LSM6DS33, 0 activates LSM6DS0 */

static MAGNETO_DrvTypeDef *MagnetoDrv = NULL;
static uint8_t MagnetoInitialized = 0;

static PRESSURE_DrvTypeDef *PressureDrv = NULL;
static uint8_t PressureInitialized = 0;
static uint8_t pressure_sensor_type = 1; /* 1 activates LPS25HB, 0 activates LPS25H */

/**
 * @}
 */

/** @defgroup X_NUCLEO_IKS01A1_IMU_6AXES_Exported_Functions X_NUCLEO_IKS01A1_IMU_6AXES_Exported_Functions
 * @{
 */

/**
 * @brief  Initialize the IMU 6 axes sensor
 * @param  None
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Init(void)
{
    IMU_6AXES_InitTypeDef InitStructure;
    uint8_t xg_id = 0;
    int done = 0;

    if(!Imu6AxesInitialized)
    {
      do
      {
        switch(imu_6axes_sensor_type)
        {
          case 1: /* Try to initialized LSM6DSL */
          {
            /* Initialize the six axes driver structure */
            Imu6AxesDrv = &LSM6DS33Drv;

            /* Configure sensor */
            InitStructure.G_FullScale      = LSM6DS33_G_FS_2000;
            InitStructure.G_OutputDataRate = LSM6DS33_G_ODR_833HZ;
            InitStructure.G_X_Axis         = LSM6DS33_G_XEN_ENABLE;
            InitStructure.G_Y_Axis         = LSM6DS33_G_YEN_ENABLE;
            InitStructure.G_Z_Axis         = LSM6DS33_G_ZEN_ENABLE;

            InitStructure.X_FullScale      = LSM6DS33_XL_FS_4G;
            InitStructure.X_OutputDataRate = LSM6DS33_XL_ODR_3K33HZ;
            InitStructure.X_X_Axis         = LSM6DS33_XL_XEN_ENABLE;
            InitStructure.X_Y_Axis         = LSM6DS33_XL_YEN_ENABLE;
            InitStructure.X_Z_Axis         = LSM6DS33_XL_ZEN_ENABLE;

            if( Imu6AxesDrv->Init == NULL )
            {
              Imu6AxesDrv = NULL;
              imu_6axes_sensor_type--;
              break;
            }

            if( Imu6AxesDrv->Init(&InitStructure) != IMU_6AXES_OK)
            {
              Imu6AxesDrv = NULL;
              imu_6axes_sensor_type--;
              break;
            }

            if ( Imu6AxesDrv->Read_XG_ID == NULL )
            {
              Imu6AxesDrv = NULL;
              imu_6axes_sensor_type--;
              break;
            }

            if(Imu6AxesDrv->Read_XG_ID(&xg_id) != IMU_6AXES_OK)
            {
              Imu6AxesDrv = NULL;
              imu_6axes_sensor_type--;
              break;
            }

            if(xg_id == I_AM_LSM6DS33_XG)
            {
              Imu6AxesDrv->extData = (IMU_6AXES_DrvExtTypeDef *)&LSM6DS33Drv_ext;
              Imu6AxesInitialized = 1;
              done = 1;
              break;
            }else
            {
              Imu6AxesDrv = NULL;
              imu_6axes_sensor_type--;
              break;
            }
          }
//          case 1: /* Try to initialized LSM6DS33 */
//          {
//            /* Initialize the six axes driver structure */
//            Imu6AxesDrv = &LSM6DS33Drv;
//
//            /* Configure sensor */
//            InitStructure.G_FullScale      = LSM6DS33_G_FS_2000;
//            InitStructure.G_OutputDataRate = LSM6DS33_G_ODR_833HZ;
//            InitStructure.G_X_Axis         = LSM6DS33_G_XEN_ENABLE;
//            InitStructure.G_Y_Axis         = LSM6DS33_G_YEN_ENABLE;
//            InitStructure.G_Z_Axis         = LSM6DS33_G_ZEN_ENABLE;
//
//            InitStructure.X_FullScale      = LSM6DS33_XL_FS_4G;
//            InitStructure.X_OutputDataRate = LSM6DS33_XL_ODR_3K33HZ;
//            InitStructure.X_X_Axis         = LSM6DS33_XL_XEN_ENABLE;
//            InitStructure.X_Y_Axis         = LSM6DS33_XL_YEN_ENABLE;
//            InitStructure.X_Z_Axis         = LSM6DS33_XL_ZEN_ENABLE;
//
//            if( Imu6AxesDrv->Init == NULL )
//            {
//              Imu6AxesDrv = NULL;
//              imu_6axes_sensor_type--;
//              break;
//            }
//
//            if( Imu6AxesDrv->Init(&InitStructure) != IMU_6AXES_OK)
//            {
//              Imu6AxesDrv = NULL;
//              imu_6axes_sensor_type--;
//              break;
//            }
//
//            if ( Imu6AxesDrv->Read_XG_ID == NULL )
//            {
//              Imu6AxesDrv = NULL;
//              imu_6axes_sensor_type--;
//              break;
//            }
//
//            if(Imu6AxesDrv->Read_XG_ID(&xg_id) != IMU_6AXES_OK)
//            {
//              Imu6AxesDrv = NULL;
//              imu_6axes_sensor_type--;
//              break;
//            }
//
//            if(xg_id == I_AM_LSM6DS33_XG)
//            {
//              Imu6AxesDrv->extData = (IMU_6AXES_DrvExtTypeDef *)&LSM6DS33Drv_ext;
//              Imu6AxesInitialized = 1;
//              done = 1;
//              break;
//            }else
//            {
//              Imu6AxesDrv = NULL;
//              imu_6axes_sensor_type--;
//              break;
//            }
//          }
          
          
          
//          case 0: /* Try to initialized LSM6DS0 */
//          default:
//          {
//            imu_6axes_sensor_type = 0;
//            /* Initialize the six axes driver structure */
//            Imu6AxesDrv = &LSM6DS0Drv;
//
//            /* Configure sensor */
//            InitStructure.G_FullScale       = LSM6DS0_G_FS_2000;
//            InitStructure.G_OutputDataRate  = LSM6DS0_G_ODR_119HZ;
//            InitStructure.G_X_Axis          = LSM6DS0_G_XEN_ENABLE;
//            InitStructure.G_Y_Axis          = LSM6DS0_G_YEN_ENABLE;
//            InitStructure.G_Z_Axis          = LSM6DS0_G_ZEN_ENABLE;
//
//            InitStructure.X_FullScale       = LSM6DS0_XL_FS_2G;
//            InitStructure.X_OutputDataRate  = LSM6DS0_XL_ODR_119HZ;
//            InitStructure.X_X_Axis          = LSM6DS0_XL_XEN_ENABLE;
//            InitStructure.X_Y_Axis          = LSM6DS0_XL_YEN_ENABLE;
//            InitStructure.X_Z_Axis          = LSM6DS0_XL_ZEN_ENABLE;
//
//            if( Imu6AxesDrv->Init == NULL )
//            {
//              Imu6AxesDrv = NULL;
//              return IMU_6AXES_ERROR;
//            }
//
//            if( Imu6AxesDrv->Init(&InitStructure) != IMU_6AXES_OK)
//            {
//              Imu6AxesDrv = NULL;
//              return IMU_6AXES_ERROR;
//            }
//
//            if ( Imu6AxesDrv->Read_XG_ID == NULL )
//            {
//              Imu6AxesDrv = NULL;
//              return IMU_6AXES_ERROR;
//            }
//
//            if(Imu6AxesDrv->Read_XG_ID(&xg_id) != IMU_6AXES_OK)
//            {
//              Imu6AxesDrv = NULL;
//              return IMU_6AXES_ERROR;
//            }
//
//            if(xg_id == I_AM_LSM6DS0_XG)
//            {
//              Imu6AxesDrv->extData = (IMU_6AXES_DrvExtTypeDef *)&LSM6DS0Drv_ext;
//              Imu6AxesInitialized = 1;
//              done = 1;
//              break;
//            }
//          }
        }
      } while(!done);
    }

    return IMU_6AXES_OK;
}

/**
 * @brief  Check if the IMU 6 axes sensor is initialized
 * @param  None
 * @retval 0 if the sensor is not initialized, 1 if the sensor is already initialized
 */
uint8_t BSP_IMU_6AXES_isInitialized(void)
{
    return Imu6AxesInitialized;
}


/**
 * @brief  Read the ID of the IMU 6 axes sensor
 * @param  xg_id the pointer where the who_am_i of the device is stored
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Read_XG_ID(uint8_t *xg_id)
{
    if ( Imu6AxesDrv->Read_XG_ID == NULL )
    {
      return IMU_6AXES_ERROR;
    }

    return Imu6AxesDrv->Read_XG_ID(xg_id);
}


/**
 * @brief  Check the ID of the IMU 6 axes sensor
 * @param  None
 * @retval IMU_6AXES_OK if the ID matches, IMU_6AXES_ERROR if the ID does not match or error occurs
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Check_XG_ID(void)
{
    uint8_t xg_id;

    if(BSP_IMU_6AXES_Read_XG_ID(&xg_id) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    switch(imu_6axes_sensor_type)
    {
      case 1:
      {
        if(xg_id == I_AM_LSM6DS33_XG)
        {
          return IMU_6AXES_OK;
        } else {
          return IMU_6AXES_ERROR;
        }
      }
//      case 0:
//      default:
//      {
//        if(xg_id == I_AM_LSM6DS0_XG)
//        {
//            return IMU_6AXES_OK;
//        } else {
//            return IMU_6AXES_ERROR;
//        }
//      }
    }
    return IMU_6AXES_ERROR;
}


/**
 * @brief  Get the accelerometer raw axes of the IMU 6 axes sensor
 * @param  pData the pointer where the output data are stored
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_X_GetAxesRaw(AxesRaw_TypeDef *pData)
{
    if ( Imu6AxesDrv->Get_X_Axes == NULL )
    {
      return IMU_6AXES_ERROR;
    }

    return Imu6AxesDrv->Get_X_Axes((int32_t *)pData);
}


/**
 * @brief  Get the gyroscope raw axes of the IMU 6 axes sensor
 * @param  pData the pointer where the output data are stored
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_G_GetAxesRaw(AxesRaw_TypeDef *pData)
{
    if ( Imu6AxesDrv->Get_G_Axes == NULL )
    {
      return IMU_6AXES_ERROR;
    }

    return Imu6AxesDrv->Get_G_Axes((int32_t *)pData);
}

/**
 * @brief  Get accelerometer sensitivity
 * @param  pfData the pointer where accelerometer sensitivity is stored
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_X_GetSensitivity( float *pfData )
{
    if( Imu6AxesDrv->Get_X_Sensitivity == NULL )
    {
      return IMU_6AXES_ERROR;
    }

    return Imu6AxesDrv->Get_X_Sensitivity( pfData );
}



/**
 * @brief  Get gyroscope sensitivity.
 * @param  pfData the pointer where the gyroscope sensitivity is stored
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_G_GetSensitivity( float *pfData )
{
    if( Imu6AxesDrv->Get_G_Sensitivity == NULL )
    {
      return IMU_6AXES_ERROR;
    }

    return Imu6AxesDrv->Get_G_Sensitivity( pfData );
}

/**
 * @brief  Get component type currently used.
 * @param  None
 * @retval IMU_6AXES_NONE_COMPONENT if none component is currently used, the component unique id otherwise
 */
IMU_6AXES_ComponentTypeDef BSP_IMU_6AXES_GetComponentType( void )
{
    if( Imu6AxesDrv == NULL )
    {
      return IMU_6AXES_NONE_COMPONENT;
    }

//    if( Imu6AxesDrv == &LSM6DS0Drv )
//    {
//      return IMU_6AXES_LSM6DS0_COMPONENT;
//    }

    if( Imu6AxesDrv == &LSM6DS33Drv )
    {
      return IMU_6AXES_LSM6DS33_DIL24_COMPONENT;
    }

    return IMU_6AXES_NONE_COMPONENT;
}

/**
 * @brief  Enable free fall detection (available only for LSM6DS33 sensor)
 * @param  None
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_NOT_IMPLEMENTED if the feature is not supported, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Enable_Free_Fall_Detection_Ext(void)
{
    /* At the moment this feature is only implemented for LSM6DS33 */
    if( Imu6AxesDrv->extData == NULL || Imu6AxesDrv->extData->id != IMU_6AXES_LSM6DS33_DIL24_COMPONENT || Imu6AxesDrv->extData->pData == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    if(((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Enable_Free_Fall_Detection == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    return ((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Enable_Free_Fall_Detection();
}

/**
 * @brief  Disable free fall detection (available only for LSM6DS33 sensor)
 * @param  None
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_NOT_IMPLEMENTED if the feature is not supported, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Disable_Free_Fall_Detection_Ext(void)
{
    /* At the moment this feature is only implemented for LSM6DS33 */
    if( Imu6AxesDrv->extData == NULL || Imu6AxesDrv->extData->id != IMU_6AXES_LSM6DS33_DIL24_COMPONENT || Imu6AxesDrv->extData->pData == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    if(((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Disable_Free_Fall_Detection == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    return ((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Disable_Free_Fall_Detection();
}

/**
 * @brief  Get status of free fall detection (available only for LSM6DS33 sensor)
 * @param  status the pointer where the status of free fall detection is stored; 0 means no detection, 1 means detection happened
 * @retval IMU_6AXES_OK in case of success, IMU_6AXES_NOT_IMPLEMENTED if the feature is not supported, IMU_6AXES_ERROR otherwise
 */
IMU_6AXES_StatusTypeDef BSP_IMU_6AXES_Get_Status_Free_Fall_Detection_Ext(uint8_t *status)
{
    /* At the moment this feature is only implemented for LSM6DS33 */
    if( Imu6AxesDrv->extData == NULL || Imu6AxesDrv->extData->id != IMU_6AXES_LSM6DS33_DIL24_COMPONENT || Imu6AxesDrv->extData->pData == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    if(((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Get_Status_Free_Fall_Detection == NULL)
    {
      return IMU_6AXES_NOT_IMPLEMENTED;
    }

    if(status == NULL)
    {
      return IMU_6AXES_ERROR;
    }

    return ((LSM6DS33_DrvExtTypeDef *)(Imu6AxesDrv->extData->pData))->Get_Status_Free_Fall_Detection(status);
}


/**
 * @brief  Configures LSM6DS3 I2C interface
 * @param  None
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Init( void )
{
  return IMU_6AXES_IO_Init();
}


/**
 * @brief  Writes a buffer to the LSM6DS3 sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Write(  uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite )
{
  return IMU_6AXES_IO_Write( pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite );
}


/**
 * @brief  Reads a buffer from the LSM6DS3 sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
IMU_6AXES_StatusTypeDef LSM6DS33_IO_Read( uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead )
{
  return IMU_6AXES_IO_Read( pBuffer, DeviceAddr, RegisterAddr, NumByteToRead );
}


/**
 * @brief  Configures Imu 6 axes I2C interface
 * @param  None
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Init( void )
{
//    if(I2C_SHIELDS_Init() != HAL_OK)
//    {
//      return IMU_6AXES_ERROR;
//    }
    // All initialization should be already done in main()
    Sensor_SPI_NSS_Set(SPI_LSM6DS33);
    return IMU_6AXES_OK;
}


/**
 * @brief  Writes a buffer to the IMU 6 axes sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    HAL_StatusTypeDef ret = HAL_OK;

    Sensor_SPI_NSS_Reset(SPI_LSM6DS33);         // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                      // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &RegisterAddr, 1, SPI_TIMEOUT);              // Send register address

    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite);               // Send data
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite, SPI_TIMEOUT);       // Send data
    Sensor_SPI_NSS_Set(SPI_LSM6DS33);           // Restore down NSS

    if (ret == HAL_OK)
        return IMU_6AXES_OK;
    else
        return IMU_6AXES_ERROR;
}


static IMU_6AXES_StatusTypeDef IMU_6AXES_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    uint8_t tmp;
    HAL_StatusTypeDef ret = HAL_OK;

    tmp = RegisterAddr | 0x80;
    Sensor_SPI_NSS_Reset(SPI_LSM6DS33);         // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                  // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &tmp, 1, SPI_TIMEOUT);                   // Send register address

    //ret |= HAL_SPI_Receive_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead);             // Send data
    ret |= HAL_SPI_Receive(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead, SPI_TIMEOUT);     // Send data
    Sensor_SPI_NSS_Set(SPI_LSM6DS33);           // Restore down NSS

    if (ret == HAL_OK)
        return IMU_6AXES_OK;
    else
        return IMU_6AXES_ERROR;
}



/****************************** Magneto Meter **********************************/



/** @defgroup X_NUCLEO_IKS01A1_MAGNETO_Exported_Functions X_NUCLEO_IKS01A1_MAGNETO_Exported_Functions
 * @{
 */

/**
 * @brief  Initialize the magneto sensor
 * @param  None
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 */
MAGNETO_StatusTypeDef BSP_MAGNETO_Init(void)
{
    uint8_t m_id = 0;
    MAGNETO_InitTypeDef InitStructure;

    if(!MagnetoInitialized)
    {
      /* Initialize the magneto driver structure */
      MagnetoDrv = &LIS3MDLDrv;

      /* Configure sensor */
      InitStructure.M_FullScale = LIS3MDL_M_FS_4;
      InitStructure.M_OperatingMode = LIS3MDL_M_MD_CONTINUOUS;
      InitStructure.M_XYOperativeMode = LIS3MDL_M_OM_HP;
      InitStructure.M_OutputDataRate = LIS3MDL_M_DO_80;

      /* magneto sensor init */
      if ( MagnetoDrv->Init == NULL )
      {
        MagnetoDrv = NULL;
        return MAGNETO_ERROR;
      }

      if(MagnetoDrv->Init(&InitStructure) != MAGNETO_OK)
      {
        MagnetoDrv = NULL;
        return MAGNETO_ERROR;
      }

      if ( MagnetoDrv->Read_M_ID == NULL )
      {
        MagnetoDrv = NULL;
        return MAGNETO_ERROR;
      }

      if(MagnetoDrv->Read_M_ID(&m_id) != MAGNETO_OK)
      {
        MagnetoDrv = NULL;
        return MAGNETO_ERROR;
      }

      if(m_id == I_AM_LIS3MDL_M)
      {
        MagnetoDrv->extData = (MAGNETO_DrvExtTypeDef *)&LIS3MDLDrv_ext;
        MagnetoInitialized = 1;
      }
    }

    return MAGNETO_OK;
}

/**
 * @brief  Check if the magnetic sensor is initialized
 * @param  None
 * @retval 0 if the sensor is not initialized, 1 if the sensor is already initialized
 */
uint8_t BSP_MAGNETO_isInitialized(void)
{
    return MagnetoInitialized;
}


/**
 * @brief  Read the ID of the magnetic sensor
 * @param  m_id the pointer where the who_am_i of the device is stored
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 */
MAGNETO_StatusTypeDef BSP_MAGNETO_Read_M_ID(uint8_t *m_id)
{
    if ( MagnetoDrv->Read_M_ID == NULL )
    {
      return MAGNETO_ERROR;
    }

    return MagnetoDrv->Read_M_ID(m_id);
}


/**
 * @brief  Check the ID of the magnetic sensor
 * @param  None
 * @retval MAGNETO_OK if the ID matches, MAGNETO_ERROR if the ID does not match or error occurs
 */
MAGNETO_StatusTypeDef BSP_MAGNETO_Check_M_ID(void)
{
    uint8_t m_id;

    if(BSP_MAGNETO_Read_M_ID(&m_id) != MAGNETO_OK)
    {
      return MAGNETO_ERROR;
    }

    if(m_id == I_AM_LIS3MDL_M)
    {
        return MAGNETO_OK;
    } else {
        return MAGNETO_ERROR;
    }
}


/**
 * @brief  Get the magnetic sensor raw axes
 * @param  pData the pointer where the output data are stored
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 */
MAGNETO_StatusTypeDef BSP_MAGNETO_M_GetAxesRaw(AxesRaw_TypeDef *pData)
{
    if ( MagnetoDrv->Get_M_Axes == NULL )
    {
      return MAGNETO_ERROR;
    }

    return MagnetoDrv->Get_M_Axes((int32_t *)pData);
}

/**
 * @brief  Get component type currently used.
 * @param  None
 * @retval MAGNETO_NONE_COMPONENT if none component is currently used, the component unique id otherwise
 */
MAGNETO_ComponentTypeDef BSP_MAGNETO_GetComponentType( void )
{
    if( MagnetoDrv == NULL )
    {
      return MAGNETO_NONE_COMPONENT;
    }

    if( MagnetoDrv == &LIS3MDLDrv )
    {
      return MAGNETO_LIS3MDL_COMPONENT;
    }

    return MAGNETO_NONE_COMPONENT;
}


/********************************* LINK MAGNETO *****************************/
/**
 * @brief  Configures LIS3MDL I2C interface
 * @param  None
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
MAGNETO_StatusTypeDef LIS3MDL_IO_Init(void)
{
    return MAGNETO_IO_Init();
}

/**
 * @brief  Writes a buffer to the LIS3MDL sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
MAGNETO_StatusTypeDef LIS3MDL_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    return MAGNETO_IO_Write(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
}

/**
 * @brief  Reads a buffer from the LIS3MDL sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
MAGNETO_StatusTypeDef LIS3MDL_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    return MAGNETO_IO_Read(pBuffer, DeviceAddr, RegisterAddr, NumByteToRead);
}


/**
 * @brief  Configures magneto I2C interface
 * @param  None
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
static MAGNETO_StatusTypeDef MAGNETO_IO_Init(void)
{
    Sensor_SPI_NSS_Set(SPI_LIS3MDL);
    return MAGNETO_OK;
}

/**
 * @brief  Writes a buffer to the magneto sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
static MAGNETO_StatusTypeDef MAGNETO_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{

    uint8_t tmp;
    HAL_StatusTypeDef ret = HAL_OK;

    tmp = RegisterAddr | 0x40;                 // auto increment
    Sensor_SPI_NSS_Reset(SPI_LIS3MDL);         // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                      // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &tmp, 1, SPI_TIMEOUT);              // Send register address

    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite);               // Send data
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite, SPI_TIMEOUT);       // Send data
    Sensor_SPI_NSS_Set(SPI_LIS3MDL);           // Restore down NSS

    if (ret == HAL_OK)
        return MAGNETO_OK;
    else
        return MAGNETO_ERROR;
}

/**
 * @brief  Reads a buffer from the magneto sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval MAGNETO_OK in case of success, an error code otherwise
 */
static MAGNETO_StatusTypeDef MAGNETO_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    uint8_t tmp;
    HAL_StatusTypeDef ret = HAL_OK;

    tmp = RegisterAddr | 0xc0;                  // read + auto increment
    Sensor_SPI_NSS_Reset(SPI_LIS3MDL);          // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                  // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &tmp, 1, SPI_TIMEOUT);                   // Send register address

    //ret |= HAL_SPI_Receive_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead);             // Send data
    ret |= HAL_SPI_Receive(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead, SPI_TIMEOUT);     // Send data
    Sensor_SPI_NSS_Set(SPI_LIS3MDL);           // Restore down NSS

    if (ret == HAL_OK)
        return MAGNETO_OK;
    else
        return MAGNETO_ERROR;
}



/****************************** Presure Sensor **************************************/

/**
 * @brief  Initialize the pressure sensor
 * @param  None
 * @retval PRESSURE_OK in case of success, PRESSURE_ERROR otherwise
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_Init(void)
{
    uint8_t p_id = 0;
    PRESSURE_InitTypeDef InitStructure;
    int done = 0;

    if(!PressureInitialized)
    {
      do
      {
        switch(pressure_sensor_type)
        {
          case 1:
          {
            /* Initialize the pressure driver structure */
            PressureDrv = &LPS25HBDrv;

            /* Configure sensor */
            InitStructure.OutputDataRate = LPS25HB_ODR_1Hz;
            InitStructure.BlockDataUpdate = LPS25HB_BDU_CONT;
            InitStructure.DiffEnable = LPS25HB_DIFF_ENABLE;
            //InitStructure.SPIMode = LPS25HB_SPI_SIM_3W;
            /* Update - 07/12/16 - Changed to SPI 4W */
            InitStructure.SPIMode = LPS25HB_SPI_SIM_4W;
            InitStructure.PressureResolution = LPS25HB_P_RES_AVG_32;
            InitStructure.TemperatureResolution = LPS25HB_T_RES_AVG_16;

             /* Pressure sensor init */
            if ( PressureDrv->Init == NULL )
            {
              PressureDrv = NULL;
              pressure_sensor_type--;
              break;
            }

            if(PressureDrv->Init(&InitStructure) != PRESSURE_OK)
            {
              PressureDrv = NULL;
              pressure_sensor_type--;
              break;
            }

            if ( PressureDrv->ReadID == NULL )
            {
              PressureDrv = NULL;
              pressure_sensor_type--;
              break;
            }

            if(PressureDrv->ReadID(&p_id) != PRESSURE_OK)
            {
              PressureDrv = NULL;
              pressure_sensor_type--;
              break;
            }

            if(p_id == I_AM_LPS25HB)
            {
              PressureDrv->extData = (PRESSURE_DrvExtTypeDef *)&LPS25HBDrv_ext;
              PressureInitialized = 1;
              done = 1;
              break;
            }else
            {
              PressureDrv = NULL;
//              pressure_sensor_type--;
              break;
            }
          }
//          case 0:
//          default:
//          {
//            /* Initialize the pressure driver structure */
//            PressureDrv = &LPS25HDrv;
//
//            /* Configure sensor */
//            InitStructure.OutputDataRate = LPS25H_ODR_1Hz;
//            InitStructure.BlockDataUpdate = LPS25H_BDU_CONT;
//            InitStructure.DiffEnable = LPS25H_DIFF_ENABLE;
//            InitStructure.SPIMode = LPS25H_SPI_SIM_3W;
//            InitStructure.PressureResolution = LPS25H_P_RES_AVG_32;
//            InitStructure.TemperatureResolution = LPS25H_T_RES_AVG_16;
//
//            /* Pressure sensor init */
//            if ( PressureDrv->Init == NULL )
//            {
//              PressureDrv = NULL;
//              return PRESSURE_ERROR;
//            }
//
//            if(PressureDrv->Init(&InitStructure) != PRESSURE_OK)
//            {
//              PressureDrv = NULL;
//              return PRESSURE_ERROR;
//            }
//
//            if ( PressureDrv->ReadID == NULL )
//            {
//              PressureDrv = NULL;
//              return PRESSURE_ERROR;
//            }
//
//            if(PressureDrv->ReadID(&p_id) != PRESSURE_OK)
//            {
//              PressureDrv = NULL;
//              return PRESSURE_ERROR;
//            }
//
//            if(p_id == I_AM_LPS25H)
//            {
//              PressureDrv->extData = (PRESSURE_DrvExtTypeDef *)&LPS25HDrv_ext;
//              PressureInitialized = 1;
//              done = 1;
//              break;
//            }
//          }
        }
      } while(!done);
    }

    return PRESSURE_OK;
}

/**
 * @brief  Check if the pressure sensor is initialized
 * @param  None
 * @retval 0 if the sensor is not initialized, 1 if the sensor is already initialized
 */
uint8_t BSP_PRESSURE_isInitialized(void)
{
    return PressureInitialized;
}

/**
 * @brief  Read the ID of the pressure sensor
 * @param  p_id the pointer where the who_am_i of the device is stored
 * @retval PRESSURE_OK in case of success, PRESSURE_ERROR otherwise
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_ReadID(uint8_t *p_id)
{
    if ( PressureDrv->ReadID == NULL )
    {
      return PRESSURE_ERROR;
    }

    return PressureDrv->ReadID(p_id);
}


/**
 * @brief  Check the ID of the pressure sensor
 * @param  None
 * @retval PRESSURE_OK if the ID matches, PRESSURE_ERROR if the ID does not match or error occurs
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_CheckID(void)
{
    uint8_t p_id;

    if(BSP_PRESSURE_ReadID(&p_id) != PRESSURE_OK)
    {
      return PRESSURE_ERROR;
    }

    switch(pressure_sensor_type)
    {
      case 1:
      {
        if(p_id == I_AM_LPS25HB)
        {
          return PRESSURE_OK;
        } else {
          return PRESSURE_ERROR;
        }
      }
//      case 0:
//      default:
//      {
//        if(p_id == I_AM_LPS25H)
//        {
//          return PRESSURE_OK;
//        } else {
//          return PRESSURE_ERROR;
//        }
//      }
    }
    return PRESSURE_ERROR;
}


/**
 * @brief  Reboot the memory content of the pressure sensor
 * @param  None
 * @retval PRESSURE_OK in case of success, PRESSURE_ERROR otherwise
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_Reset(void)
{
    if ( PressureDrv->Reset == NULL )
    {
      return PRESSURE_ERROR;
    }

    return PressureDrv->Reset();
}


/**
 * @brief  Get the pressure
 * @param  pfData the pointer where the output data are stored
 * @retval PRESSURE_OK in case of success, PRESSURE_ERROR otherwise
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_GetPressure(float* pfData)
{
    if ( PressureDrv->GetPressure == NULL )
    {
      return PRESSURE_ERROR;
    }

    return PressureDrv->GetPressure(pfData);
}

/**
 * @brief  Get the temperature.
 * @param  pfData the pointer where the output data are stored
 * @retval PRESSURE_OK in case of success, PRESSURE_ERROR otherwise
 */
PRESSURE_StatusTypeDef BSP_PRESSURE_GetTemperature(float* pfData)
{
    if ( PressureDrv->GetTemperature == NULL )
    {
      return PRESSURE_ERROR;
    }

    return PressureDrv->GetTemperature(pfData);
}

/**
 * @brief  Get component type currently used.
 * @param  None
 * @retval PRESSURE_NONE_COMPONENT if none component is currently used, the component unique id otherwise
 */
PRESSURE_ComponentTypeDef BSP_PRESSURE_GetComponentType( void )
{
    if( PressureDrv == NULL )
    {
      return PRESSURE_NONE_COMPONENT;
    }

//    if( PressureDrv == &LPS25HDrv )
//    {
//      return PRESSURE_LPS25H_COMPONENT;
//    }

    if( PressureDrv == &LPS25HBDrv )
    {
      return PRESSURE_LPS25HB_DIL24_COMPONENT;
    }

    return PRESSURE_NONE_COMPONENT;
}

/**
 * @brief  Configures LPS25H SPI interface
 * @param  None
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25H_IO_Init(void)
{
    return PRESSURE_IO_Init();
}

/**
 * @brief  Configures LPS25HB I2C interface
 * @param  None
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25HB_IO_Init(void)
{
    return PRESSURE_IO_Init();
}

/**
 * @brief  Configures LPS25HB interrupt lines for NUCLEO boards
 * @param  None
 * @retval None
 */
void LPS25HB_IO_ITConfig( void )
{
  /* To be implemented */
}

/**
 * @brief  Writes a buffer to the LPS25H sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25H_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    return PRESSURE_IO_Write(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
}

/**
 * @brief  Writes a buffer to the LPS25HB sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25HB_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    return PRESSURE_IO_Write(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
}

/**
 * @brief  Reads a buffer from the LPS25H sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25H_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    return PRESSURE_IO_Read(pBuffer, DeviceAddr, RegisterAddr, NumByteToRead);
}

/**
 * @brief  Reads a buffer from the LPS25HB sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
PRESSURE_StatusTypeDef LPS25HB_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    return PRESSURE_IO_Read(pBuffer, DeviceAddr, RegisterAddr, NumByteToRead);
}

/**
 * @brief  Configures pressure I2C interface
 * @param  None
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
static PRESSURE_StatusTypeDef PRESSURE_IO_Init(void)
{
    // All initialization should be already done in main()
    Sensor_SPI_NSS_Set(SPI_LPS25HB);
    return PRESSURE_OK;
}

/**
 * @brief  Writes a buffer to the pressure sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
static PRESSURE_StatusTypeDef PRESSURE_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    uint8_t tmp;
    HAL_StatusTypeDef ret = HAL_OK;

    tmp = RegisterAddr | 0x40;                 // auto increment
    Sensor_SPI_NSS_Reset(SPI_LPS25HB);         // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                      // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &tmp, 1, SPI_TIMEOUT);              // Send register address

    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite);               // Send data
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, pBuffer, NumByteToWrite, SPI_TIMEOUT);       // Send data
    Sensor_SPI_NSS_Set(SPI_LPS25HB);           // Restore down NSS

    if (ret == HAL_OK)
        return PRESSURE_OK;
    else
        return PRESSURE_ERROR;
}

/**
 * @brief  Reads a buffer from the pressure sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be read
 * @param  NumByteToRead number of bytes to be read
 * @retval PRESSURE_OK in case of success, an error code otherwise
 */
static PRESSURE_StatusTypeDef PRESSURE_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    uint8_t tmp;
    HAL_StatusTypeDef ret = HAL_OK;

    tmp = RegisterAddr | 0xc0;                  // read + auto increment
    Sensor_SPI_NSS_Reset(SPI_LPS25HB);          // Pull down NSS
    //ret |= HAL_SPI_Transmit_IT(SENSOR_SPI_HANDLE, &RegisterAddr, 1);                  // Send register address
    ret |= HAL_SPI_Transmit(SENSOR_SPI_HANDLE, &tmp, 1, SPI_TIMEOUT);                   // Send register address

    //ret |= HAL_SPI_Receive_IT(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead);             // Send data
    ret |= HAL_SPI_Receive(SENSOR_SPI_HANDLE, pBuffer, NumByteToRead, SPI_TIMEOUT);     // Send data
    Sensor_SPI_NSS_Set(SPI_LPS25HB);           // Restore down NSS

    if (ret == HAL_OK)
        return PRESSURE_OK;
    else
        return PRESSURE_ERROR;
}





void Sensor_SPI_NSS_Set(SPI_SENSOR_NSS_TypeDef sensor)
{
    if (sensor < SENSOR_SPI_NUMBER)
    {
        HAL_GPIO_WritePin(SPI_SENSOR_NSS_PORT[sensor], SPI_SENSOR_NSS_PIN[sensor], GPIO_PIN_SET);
    }
}

void Sensor_SPI_NSS_Reset(SPI_SENSOR_NSS_TypeDef sensor)
{
    if (sensor < SENSOR_SPI_NUMBER)
    {
        HAL_GPIO_WritePin(SPI_SENSOR_NSS_PORT[sensor], SPI_SENSOR_NSS_PIN[sensor], GPIO_PIN_RESET);
    }
}

uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Set all the pins before init to avoid glitch */
//  HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_X_SPI_CS_Port, SENSORTILE_LSM303AGR_X_SPI_CS_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_M_SPI_CS_Port, SENSORTILE_LSM303AGR_M_SPI_CS_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(SENSORTILE_LPS22HB_SPI_CS_Port, SENSORTILE_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIS2MDL_NSS_PORT, LIS2MDL_NSS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LPS22HB_NSS_PORT, LPS22HB_NSS_PIN, GPIO_PIN_SET);
  
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  LSM6DSL_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LSM6DSL_NSS_PIN;
  HAL_GPIO_Init(LSM6DSL_NSS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_SET);
  
  LIS2MDL_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LIS2MDL_NSS_PIN;
  HAL_GPIO_Init(LIS2MDL_NSS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LIS2MDL_NSS_PORT, LIS2MDL_NSS_PIN, GPIO_PIN_SET);
  
  LPS22HB_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LPS22HB_NSS_PIN;
  HAL_GPIO_Init(LPS22HB_NSS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LPS22HB_NSS_PORT, LPS22HB_NSS_PIN, GPIO_PIN_SET);

  //return COMPONENT_OK;
  return 0;
}


