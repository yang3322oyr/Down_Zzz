#ifndef _WheelLeg_Chassis_
#define _WheelLeg_Chassis_

#include "main.h"
#include "Chassis.h"

int8_t Chassis_Smooth_Restand_Fun(Chassis_Control_StructTypedef *Chassis_Control);
int8_t Chassis_OFF_Slow_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct, float Smooth_Start_K);
int8_t Chassis_Jump_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct);
#endif
