/* Introduction for remote control module:
  This module provide interface to work with remote control receiver signals

    1. init_remote_control()
    This function initial all the variables for remote control signal processing.
    It must be called in initialization stage

    2. HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    This function process the input capture channels interrupt to calculate the
    pulse of each RC channel and save the pulse width value into global variable
    rc_t[4]. The drfault time step is 0.25us/LSB.
    It also convert the pulse width signal into real control signal stored in
    global variable: gAIL, gELE, gTHR, gRUD (by calling update_rc_data(idx))
    gAIL, gELE and gRUD are 0 centerred (+/-0.5ms with 0.25us/LSB). gTHE is 0~1ms
    with 0.25us/LSB.

    3. HAL_SYSTICK_Callback()
    This function calls the user timer processing function first. And then it will
    check if RC signal is time out or not. A time out flag will be set (rc_connection_flag)
    if no RC pulse is received within certain time

  Note
    If AUTO_CONNECTION_CENTER is set to 1, the first 4 sets of RC data during
    connection, will be averaged and used as the center of Roll, Pitch and Yaw
    control.
*/

#include "rc.h"

#define AUTO_CONNECTION_CENTER  0

const float max_pitch_rad = ((float)PI*(float)PITCH_MAX_DEG)/180.0f;
const float max_roll_rad = ((float)PI*(float)ROLL_MAX_DEG)/180.0f;
const float max_yaw_rad = ((float)PI*(float)YAW_MAX_DEG)/180.0f;
int32_t t1;


int32_t ail_center = AIL_MIDDLE;
int32_t ele_center = ELE_MIDDLE;
int32_t rud_center = RUD_MIDDLE;

int32_t rc_cnt = 0;
int32_t rc_acc[4] = {0};

int32_t rc_z_control_flag = 1;

extern int32_t rc_cal_flag;
extern int32_t rc_enable_motor;
extern int32_t fly_ready;

GPIO_TypeDef* RC_Channel_Ports[4] =
    {
      RC_CHANNEL1_PORT,
      RC_CHANNEL2_PORT,
      RC_CHANNEL3_PORT,
      RC_CHANNEL4_PORT
    };

uint16_t RC_Channel_Pins[4] =
    {
      RC_CHANNEL1_PIN,
      RC_CHANNEL2_PIN,
      RC_CHANNEL3_PIN,
      RC_CHANNEL4_PIN
    };

volatile int rc_timeout;    // R/C timeout counter
char rc_connection_flag;    // R/C connection status
char rc_flag[4];            // flag for received input capture interrupt count
/* timer data for rising and falling edge and pulse width */
int rc_t_rise[4], rc_t_fall[4], rc_t[4];
/* Global R/C data */
int16_t gAIL, gELE, gTHR, gRUD;


// A queue for testing purpose (to print R/C data in main function)
Queue_TypeDef que;
int32_t cnt;

extern uint8_t joydata[7];

// privite function
void init_rc_variables(void);

void init_remote_control(void)
{
  rc_connection_flag = 0;
  rc_timeout = 1000;

  // Initial R/C global variables
  gAIL = 0;
  gELE = 0;
  gTHR = 0;
  gRUD = 0;

  init_rc_variables();
  // queue for test purpose
  cnt = 0;
  init_queue(&que);
}

void init_rc_variables(void)
{
  uint32_t i;
  rc_connection_flag = 0;
  for (i=0;i<4;i++)
  {
    rc_flag[i] = 0;
    rc_t_rise[i] = 0;
    rc_t_fall[i] = 0;
    rc_t[i] = 0;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{


  #ifdef REMOCON_PWM
  int32_t timcnt, idx;
  
  // save the counter data
    switch (htim->Channel)
    {
      case HAL_TIM_ACTIVE_CHANNEL_1: idx = 0; timcnt = htim->Instance->CCR1; break;
      case HAL_TIM_ACTIVE_CHANNEL_2: idx = 1; timcnt = htim->Instance->CCR2; break;
      case HAL_TIM_ACTIVE_CHANNEL_3: idx = 2; timcnt = htim->Instance->CCR3; break;
      case HAL_TIM_ACTIVE_CHANNEL_4: idx = 3; timcnt = htim->Instance->CCR4; break;
      default: idx = -1;
    }

    if (idx != -1)
    {
      rc_timeout = 0;     // Reset timeout count
      if (rc_flag[idx] < 2)
        rc_flag[idx]++;
      if (HAL_GPIO_ReadPin(RC_Channel_Ports[idx], RC_Channel_Pins[idx]) == GPIO_PIN_SET)
      {   // rising edge
        rc_t_rise[idx] = timcnt;
      }
      else
      {   // falling edge
        rc_t_fall[idx] = timcnt;
        if (rc_flag[idx] >= 2)  // be sure to calculate signal width after at least a pair of interrupt
        {
          if (rc_t_fall[idx] > rc_t_rise[idx])
            rc_t[idx] = rc_t_fall[idx] - rc_t_rise[idx];
          else
            rc_t[idx] = rc_t_fall[idx] - rc_t_rise[idx] + 32768;
        }
         
        update_rc_data(idx);
  
        if (AUTO_CONNECTION_CENTER)
        {
          // Automatic use average of 4 RC data during connecting as the center of RC
          //if (rc_cnt < 16)
          if (rc_cnt < 4)  
          {
              rc_acc[idx] += rc_t[idx];
              rc_cnt++;
              //if (rc_cnt == 16)
              if (rc_cnt == 4)
              {
                  ail_center = rc_acc[0] / 4;
                  ele_center = rc_acc[1] / 4;
                  rud_center = rc_acc[3] / 4;
              }
          }
        }
        
  

        // For debug purpose
        // Add channel & width info into a queue for printing
        add_queue(&que, idx, rc_t[idx]);
      }
  }
  #endif
  #ifdef REMOCON_BLE
        
        update_rc_data(0);
      
  #endif
}


void HAL_SYSTICK_Callback(void)
{
  // Process user timer
  User_Timer_Callback();
  // Count rc_timeout up to 1s
  if (rc_timeout < 1000)
    rc_timeout++;
  if (rc_timeout > RC_TIMEOUT_VALUE)
    init_rc_variables();
  #ifdef REMOCON_PWM
    rc_connection_flag = (rc_timeout <= RC_TIMEOUT_VALUE);
  #endif
  #ifdef REMOCON_BLE
    rc_connection_flag = 1;             /* To modify and check status of BLE connection */
  #endif
}


/* Update global variables of R/C data */
void update_rc_data(int32_t idx)
{
  #ifdef REMOCON_PWM
    switch (idx)
    {
      case 0: gAIL = rc_t[0] - ail_center; break;
      case 1: gELE = rc_t[1] - ele_center; break;
      case 2: gTHR = (rc_t[2] > THR_BOTTOM) ? (rc_t[2] - THR_BOTTOM) : 0; break;
      case 3: gRUD = rc_t[3] - rud_center; break;
      default: break;
    }
  #endif
  
    
  if ( (gTHR == 0) && (gELE < - RC_CAL_THRESHOLD) && (gAIL > RC_CAL_THRESHOLD) && (gRUD < - RC_CAL_THRESHOLD))
  {
    rc_cal_flag = 1;
  }

  if ( (gTHR == 0) && (gELE < - RC_CAL_THRESHOLD) && (gAIL < - RC_CAL_THRESHOLD) && (gRUD > RC_CAL_THRESHOLD))
  {
    rc_enable_motor = 1;
    fly_ready = 1;
  }
}

/*
 * Convert RC received gAIL, gELE, gRUD
 */
void GetTargetEulerAngle(EulerAngleTypeDef *euler_rc, EulerAngleTypeDef *euler_ahrs)
{
    t1 = gELE;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc->thx = -t1 * max_pitch_rad / RC_FULLSCALE;

    t1 = gAIL;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;
    euler_rc->thy = -t1 * max_roll_rad / RC_FULLSCALE;

    t1 = gRUD;
    if (t1 > RC_FULLSCALE)
        t1 = RC_FULLSCALE;
    else if (t1 < -RC_FULLSCALE)
        t1 = - RC_FULLSCALE;

    if(rc_z_control_flag == 1)
    {
      if(t1 > EULER_Z_TH)
      {
        euler_rc->thz = euler_rc->thz + max_yaw_rad;
      }
      else if(t1 < -EULER_Z_TH)
      {
        euler_rc->thz = euler_rc->thz - max_yaw_rad;
      }
    }
    else
    {
      if(t1 > -EULER_Z_TH&&t1 < EULER_Z_TH)
      {
           rc_z_control_flag = 1;
      }
    }
}


void init_queue(Queue_TypeDef *q)
{
  int32_t i;

  q->header = 0;
  q->tail = 0;
  q->length = QUEUE_LENGTH;
  q->full = 0;
  q->empty = 1;
  for (i=0;i<QUEUE_LENGTH;i++)
  {
    q->buffer[i][0] = 0;
    q->buffer[i][1] = 0;
  }
}

void add_queue(Queue_TypeDef *q, int16_t idx, int16_t value)
{
  int h;
  cnt++;
  if (q->full==1)
  {
    if (q->header == 0)
      h = q->length - 1;
    else
      h = q->header - 1;
    q->buffer[h][0] = idx;
    q->buffer[h][1] = -1;   // set error flag
  }
  else
  {
    // insert the node
    q->buffer[q->header][0] = idx;
    q->buffer[q->header][1] = value;
    // increase the header pointer
    q->header++;
    if (q->header >= q->length)
      q->header = 0;
    // check if queue is full
    if (q->header == q->tail)
      q->full = 1;
    // it will not empty any more
    q->empty = 0;
  }
}

int32_t get_queue(Queue_TypeDef *q, int16_t *idx, int16_t *value)
{
  // get data only if the queue is not empty
  if (q->empty != 1)
  {
    // get the node data
    *idx = q->buffer[q->tail][0];
    *value = q->buffer[q->tail][1];
    // moving tail pointer
    q->tail++;
    if (q->tail >= q->length)
      q->tail = 0;
    // check if queue is empty
    if (q->header == q->tail)
      q->empty = 1;
    // It will not full any more
    q->full = 0;
    return 0;
  }
  else
    return -1;      // queue is empty
}


