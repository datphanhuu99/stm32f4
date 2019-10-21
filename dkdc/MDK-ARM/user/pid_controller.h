#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    MANUAL,
    AUTOMATIC
}
PIDMode;

typedef enum
{
    DIRECT,
    REVERSE
}
PIDDirection;

typedef struct
{
    float input;
    float lastInput;
    float output;
    
    float Kp; //gia tri kp
    float Ki;// gia tri ki
    float Kd;// gia tri kd
    
    float alteredKp;
    float alteredKi;
    float alteredKd;
    
    //float iTerm;
    
    float sampleTime;
    
    float outMin;
    float outMax;
    
    float setpoint;
    
    PIDDirection controllerDirection;
    
    PIDMode mode;
}
PIDControl;
extern void PIDInit(PIDControl *pid, float kp, float ki, float kd, 
                    float sampleTimeSeconds, float minOutput, float maxOutput, 
                    PIDMode mode, PIDDirection controllerDirection);     	
extern bool PIDCompute(PIDControl *pid); 
										
extern void PIDModeSet(PIDControl *pid, PIDMode mode);                                                                                                                                       

extern void PIDOutputLimitsSet(PIDControl *pid, float min, float max); 							  							  
										
extern void PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd);         	                                         

extern void PIDTuningKpSet(PIDControl *pid, float kp);

extern void PIDTuningKiSet(PIDControl *pid, float ki);

extern void PIDTuningKdSet(PIDControl *pid, float kd);

extern void PIDControllerDirectionSet(PIDControl *pid, 
                                      PIDDirection controllerDirection);	  									  									  									  


extern void PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds);                                                       									  									  									   


inline void 
PIDSetpointSet(PIDControl *pid, float setpoint) { pid->setpoint = setpoint; }
inline void 
PIDInputSet(PIDControl *pid, float input) { pid->input = input; }


inline float 
PIDOutputGet(PIDControl *pid) { return pid->output; }
inline float 
PIDKpGet(PIDControl *pid) { return pid->Kp; }						  


inline float 
PIDKiGet(PIDControl *pid) { return pid->Ki; }						  

inline float 
PIDKdGet(PIDControl *pid) { return pid->Kd; }						  


inline PIDMode 
PIDModeGet(PIDControl *pid) { return pid->mode; }						  

inline PIDDirection 
PIDDirectionGet(PIDControl *pid) { return pid->controllerDirection; }		

#ifdef __cplusplus
}
#endif

#endif 
