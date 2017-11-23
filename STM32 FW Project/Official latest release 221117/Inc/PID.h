/*This is type define which is used in PID.c.
File name : PID.h
*/

#ifndef         __PID
#define         __PID

#include        <stdio.h>
#include        <math.h>
#include        <string.h>
#include        <stdint.h>
#include        "math.h"
#include        "def32.h"

/* PID parameters set
*/
//Axis-X PID >> Pitch
//#define         Axis_x_KP				            1000 	              //KP
//#define         Axis_x_KI				            2000                  //KI
#define         Axis_x_KP				            10000 	              //KP
#define         Axis_x_KI				            2000                  //KI
#define         Axis_x_KD				            0	                    //KD
#define         Axis_x_KC				            0	                    //KC

#define         Axis_x_max_e_limit	        (+400)	              //posi error max(r/min)
#define         Axis_x_min_e_limit	        (-400)	              //posi error min(r/min)

#define         Axis_x_PID_out_max	        (+1300)		            //posi pid output max data
#define         Axis_x_PID_out_min	        (-1300) 		          //posi pid output min data

#define         Axis_x_ref_max	            600	                  //posi error max(r/min)
#define         Axis_x_ref_min	            300	                  //posi error min(r/min)

//Axis-Y PID >> Roll
//#define         Axis_y_KP				            1000 	              //KP
//#define         Axis_y_KI				            2000                  //KI
#define         Axis_y_KP				            10000 	              //KP
#define         Axis_y_KI				            2000                 //KI
#define         Axis_y_KD				            0	                    //KD
#define         Axis_y_KC				            0	                    //KC

#define         Axis_y_max_e_limit	        (+400)	              //posi error max(r/min)
#define         Axis_y_min_e_limit	        (-400)	              //posi error min(r/min)

#define         Axis_y_PID_out_max	        (+1300)		            //posi pid output max data
#define         Axis_y_PID_out_min	        (-1300) 		          //posi pid output min data

#define         Axis_y_ref_max	            600	                  //posi error max(r/min)
#define         Axis_y_ref_min	            300	                  //posi error min(r/min)

//Axis-Z PID >> Yaw
#define         Axis_z_KP				            10000 	              //KP
#define         Axis_z_KI				            2000                  //KI
#define         Axis_z_KD				            0	                    //KD
#define         Axis_z_KC				            0	                    //KC

#define         Axis_z_max_e_limit	        (+400)	              //posi error max(r/min)
#define         Axis_z_min_e_limit	        (-400)	              //posi error min(r/min)

#define         Axis_z_PID_out_max	        (+1300)		            //posi pid output max data
#define         Axis_z_PID_out_min	        (-1300) 		          //posi pid output min data

#define         Axis_z_ref_max	            600	                  //posi error max(r/min)
#define         Axis_z_ref_min	            300	                  //posi error min(r/min)

/* PID structure define.
*/
typedef struct
{ int32_t KP;
  int32_t KI;
  int32_t KD;
  int32_t KC;

  int32_t reference;
  int32_t feedback;
  int32_t error_now;
  int32_t error_old;
  int32_t error_older;

  int32_t error_max;
  int32_t error_min;
  int32_t error_add;
  int32_t error_add_max;
  int32_t error_add_min;
  int32_t error_saturated;

  int32_t output_max;
  int32_t output_min;

  s32_s16u16 pre_output;
  s32_s16u16 output;
}PID_TypeDef;

/* Variables of PID structure definition;
*/
extern PID_TypeDef  Axis_x, Axis_y, Axis_z;

/*functions declare;
*/
void PID_Init(void);
void reset_PID(PID_TypeDef* loop);
void parameters_PID(PID_TypeDef *loop,
                    int32_t kp,
                    int32_t ki,
                    int32_t kd,
                    int32_t kc,
                    int32_t errormax,
                    int32_t errormin,
                    int32_t outputmax,
                    int32_t outputmin
                   );
void PID_module(PID_TypeDef* loop);

#endif

/******************************* End of file **********************************/

