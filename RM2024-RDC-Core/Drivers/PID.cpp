#include "PID.hpp"
#if USE_PID
#define IntegralMax 10000
namespace Control
{

/*This is where you implement the PID algorithm*/
// embedd the output of displacement PID ring into the target of velocity PID ring, i.e. pid[1].update(pid[0].update(target,currentData),currentData)
float PID::update(float target, float measurement, float dt)
{
    /*=====================================================================*/
    // Your implementation of the PID algorithm begins here
    /*=====================================================================*/
    error = target - measurement;
    pOut = Kp * error;

    iOut += Ki * error * dt;
    
    //limit the integral output
    if (iOut > IntegralMax) {
        iOut = IntegralMax;
    } else if (iOut < -IntegralMax) {
        iOut = -IntegralMax;
    }
    

    dOut = Kd * (error - lastError) / dt;

    output = pOut + iOut + dOut; // sum up the output from 3 control

    lastError = error; //  set the current error to last error using in next tick
    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

}  // namespace Control
#endif
