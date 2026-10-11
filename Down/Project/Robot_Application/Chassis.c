/**
 * @file    Chassis.c[底盘控制]
 * @brief   轮腿底盘控制
 * @details 统一接口,外部控制底盘数据和传感器数据统一从 Chassis_Task 传入；
 *          Chassis_Task 调用 WheelLeg 文件的上层函数实现底盘的控制；
 *          Chassis_Control_Struct中存储需要输入到WheelLeg文件任务函数的参数
 * 			底层相关的底盘数据定义在各个 WheelLeg 前缀文件中
 */

#include "Chassis.h"
#include "RoboControl.h"
#include "Power_Limit.h"
#include "CAN_Driver.h"
#include "Motor_DJI_Driver.h"
#include "PID.h"
#include "USART_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "Motor_DAMIAO_Driver.h"
#include "Referee_Unpack.h"
#include "Vofa.h"
#include "INS.h"
#include "Function.h"
#include "Buzzer.h"
#include "arm_math.h"  // CMSIS-DSP 库
#include "user_lib.h"
#include "Remote_Control.h"
#include "kalman_filter.h"
#include <math.h>   
#include <stdbool.h>


#include "WheelLeg_SpringComp.h"
#include "WheelLeg_Kinematics.h"

Chassis_Control_StructTypedef Chassis_Control_Struct = 
{
    .Leg_Left.Leg_LR  = Leg_L,
    .Leg_Right.Leg_LR = Leg_L
};


void Chassis_Task(void const * argument)
{
	osDelay(100);

    /*===| 默认模块赋值 |===*/
    Chassis_Control_Struct.Chassis_Control.Chassis_State = Chassis_OFF;
    Chassis_Control_Struct.Chassis_Control.Jump_Level = 1;
    Chassis_Control_Struct.Chassis_Control.Leg_Length_Length = Leg_Length_Max;
    DWT_GetDeltaT(Chassis_Control_Struct.Chassis_Count);

    //氮气弹簧补偿初始化
    WheelLeg_SpringComp_Init();
    //LQR运控初始化
    WheelLeg_LQR_Init();
    //电流输出初始化
    WheelLeg_Output_Init();
    for(;;)
    {
        Chassis_Control_Struct.Chassis_Dt = DWT_GetDeltaT(Chassis_Control_Struct.Chassis_Count);
        
        /*===| 获得外部数据 |===*/
        Get_Control_Data(&Chassis_Control_Struct);
        
        /*===| 底盘运动学解算 |===*/
        WheelLeg_Kinematics_Update(&Chassis_Control_Struct);

        /*===|LQR运控解算 |===*/
        WheelLeg_LQR_Output(&Chassis_Control_Struct);

        osDelay(1);
    }
}

/**
 * @brief 获得外部数据
 * @note RoboControl_Struct赋值控制数据
 * @note INS赋值状态数据
 * 
 */
void Get_Control_Data(Chassis_Control_StructTypedef *Chassis_Control_Struct)
{
/*===| RoboControl_Struct数据赋值 |===*/
    //地盘状态赋值
    Chassis_Control_Struct->Chassis_Control.Chassis_State = RoboControl_Struct.Chassis_State;
    //目标运动速度
    Chassis_Control_Struct->Chassis_Control.Vy_Target = RoboControl_Struct.Robo_Target_Vy;
    Chassis_Control_Struct->Chassis_Control.Vx_Target = RoboControl_Struct.Robo_Target_Vx;
    Chassis_Control_Struct->Chassis_Control.Wz_Speed_Target = RoboControl_Struct.Robo_Target_Wz;

    Chassis_Control_Struct->Chassis_Control.Jump_Level = RoboControl_Struct.Jump_Level;
    Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = RoboControl_Struct.Leg_Length;
    
/*===| INS数据赋值 |===*/


    
}