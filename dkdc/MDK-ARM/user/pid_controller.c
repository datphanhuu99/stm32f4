#include "pid_controller.h"

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

void PIDInit(PIDControl *pid, float kp, float ki, float kd, 
             float sampleTimeSeconds, float minOutput, float maxOutput, 
             PIDMode mode, PIDDirection controllerDirection)     	
{
    pid->controllerDirection = controllerDirection;
    pid->mode = mode;
    pid->input = 0.0f;
    pid->lastInput = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    
    if(sampleTimeSeconds > 0.0f)
    {
        pid->sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        pid->sampleTime = 1.0f;
    }
    
    PIDOutputLimitsSet(pid, minOutput, maxOutput);
    PIDTuningsSet(pid, kp, ki, kd);
}
 void PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd)
 {
		if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }
		
		
 }


