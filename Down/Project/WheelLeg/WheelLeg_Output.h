#ifndef __WHEELLEG_OUTPUT_H__
#define __WHEELLEG_OUTPUT_H__

#include "main.h"

void WheelLeg_Output_Init(void);

int8_t WheelLeg_Output
(const uint8_t Control_State,

const float Leg_Left_Link_F_N_Add,
const float Leg_Left_Angle_Torque_NM_Add,
const float Leg_Right_Link_F_N_Add,
const float Leg_Right_Angle_Torque_NM_Add,

const float Leg_Left_L_Target,
const float Leg_Left_L_Feedback,
const float Leg_Left_L_Speed_Feedback,
const float Leg_Left_Total_Angle_Target,
const float Leg_Left_Total_Angle_Feedback,
const float Leg_Left_Total_Angle_Speed_Feedback,
const float Leg_Right_L_Target,
const float Leg_Right_L_Feedback,
const float Leg_Right_L_Speed_Feedback,
const float Leg_Right_Total_Angle_Target,
const float Leg_Right_Total_Angle_Feedback,
const float Leg_Right_Total_Angle_Speed_Feedback,

const float K_PID_Control_Weight,

const float Jt_Left[2][2],
const float Jt_Right[2][2],

const float Wheel_Left_Torque,
const float Wheel_Right_Torque
);


#endif