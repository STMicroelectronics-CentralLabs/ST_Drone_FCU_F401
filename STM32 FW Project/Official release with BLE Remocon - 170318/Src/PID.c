/*This is the PID main body.
File name : PID.c
*/

#include                "PID.h"

/* Variables of PID structure definition;
*/
//global variables;
PID_TypeDef  Axis_x, Axis_y, Axis_z;

/* init PID;reset pid and set parameters.
*/
void PID_Init(void)
{
  //setup speed pid.
  reset_PID(&Axis_x);
  reset_PID(&Axis_y);
  reset_PID(&Axis_z);

  //Set Axis x
  parameters_PID(&Axis_x,
                 Axis_x_KP,
                 Axis_x_KI,
                 Axis_x_KD,
                 Axis_x_KC,
                 Axis_x_max_e_limit,
                 Axis_x_min_e_limit,
                 Axis_x_PID_out_max,
                 Axis_x_PID_out_min
                );

  //Set Axis y
  parameters_PID(&Axis_y,
                 Axis_y_KP,
                 Axis_y_KI,
                 Axis_y_KD,
                 Axis_y_KC,
                 Axis_y_max_e_limit,
                 Axis_y_min_e_limit,
                 Axis_y_PID_out_max,
                 Axis_y_PID_out_min
                );

  //Set Axis Z.
  parameters_PID(&Axis_z,
                 Axis_z_KP,
                 Axis_z_KI,
                 Axis_z_KD,
                 Axis_z_KC,
                 Axis_z_max_e_limit,
                 Axis_z_min_e_limit,
                 Axis_z_PID_out_max,
                 Axis_z_PID_out_min
                );
}

/* reset PID.
*/
void reset_PID(PID_TypeDef* loop)
{
  loop->reference = 600;
  loop->feedback = 0;
  loop->error_now = 0;
  loop->error_old = 0;
  loop->error_older = 0;
  loop->error_add = 0;
  loop->error_saturated = 0;
  loop->pre_output.all = 0;
  loop->output.all = 0;
}

/* PID performance set.
*/
void parameters_PID(PID_TypeDef* loop,
                    int32_t kp,
                    int32_t ki,
                    int32_t kd,
                    int32_t kc,
                    int32_t errormax,
                    int32_t errormin,
                    int32_t outputmax,
                    int32_t outputmin
                   )
{ loop->KP = kp;
  loop->KI = ki;
  loop->KC = kc;

  loop->error_max = errormax;
  loop->error_min = errormin;

  loop->error_add_max = (outputmax << 16) / loop->KI;
  loop->error_add_min = (outputmin << 16) / loop->KI;

  loop->output_max = outputmax;
  loop->output_min = outputmin;
}

/* PID body
*/
void PID_module(PID_TypeDef* loop)
{ int32_t up,ui,ud,uc;

  loop->error_now = loop->reference - loop->feedback;

  if(loop->error_now > loop->error_max)
  { loop->error_now = loop->error_max;
  }//end of if(loop->error_now > loop->error_max)
  else if(loop->error_now < loop->error_min)
  { loop->error_now = loop->error_min;
  }//end of if else if(loop->error_now < loop->error_min)

  loop->error_add += loop->error_now;

  if(loop->error_add > loop->error_add_max)
  { loop->error_add = loop->error_add_max;
  }//end of if(loop->error_add > loop->error_add_max)
  else if(loop->error_add < loop->error_add_min)
  { loop->error_add = loop->error_add_min;
  }//end of if else if(loop->error_add < loop->error_add_min)

  loop->error_older = loop->error_old;
  loop->error_old = loop->error_now;

  up = loop->KP * loop->error_now;
  ui = loop->KI * loop->error_add;
  ud = loop->KD * (loop->error_old - loop->error_older);
  uc = loop->KC * loop->error_saturated;

  loop->pre_output.all = up + ui + ud + uc;

  if(loop->pre_output.high_low.shigh > loop->output_max)
  { loop->output.high_low.shigh = loop->output_max;
  }
  else if(loop->pre_output.high_low.shigh < loop->output_min)
  { loop->output.high_low.shigh = loop->output_min;
  }
  else
  { loop->output.high_low.shigh = loop->pre_output.high_low.shigh;
  }

  loop->error_saturated = loop->output.high_low.shigh - loop->pre_output.high_low.shigh;
}

/******************************* End of file **********************************/

