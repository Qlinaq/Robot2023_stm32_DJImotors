#include "AppConfig.h"
//#include "stdint.h"
//#include "tim.h"
//#include "main.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_tim.h"

#if USE_Servo

namespace Servo
{
    void ServoInit();
    void drop_first_ball ();
    void drop_second_ball ();
    void intermediate();
    
}
#endif
  