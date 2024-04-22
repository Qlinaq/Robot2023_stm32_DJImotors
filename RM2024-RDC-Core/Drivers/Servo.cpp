#include "Servo.hpp"
#include "tim.h"

#if USE_Servo
#include "tim.h"



namespace Servo
{   
    //extern TIM_HandleTypeDef htim2;
   void ServoInit() // initializing the pwm module
   {
     HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);  //initialize tim 
     HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,60); // set to initial angle
    }
    

    void drop_first_ball () // initialization (?)
    {    
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,60);
    }
    void intermediate(){ // drop first ball
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,130);

    }
    void drop_second_ball () // drop second ball
    {    
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,220);

    }
    
    

}
#endif
