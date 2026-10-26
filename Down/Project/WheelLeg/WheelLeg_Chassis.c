/**
 * @file    WheelLeg_Chassis.c[底盘任务]
 * @brief   轮腿底盘任务函数封装
 * @details 
 */
#include "WheelLeg_Chassis.h"

//Chassis_Smooth_Restand_Fun翻倒自起函数相关宏定义
#define Chassis_Recover_Angle_Speed 			125.0f		//翻倒回正摆角速度(°/s)
#define Chassis_RESET_Slow_Frist_Angle_Speed 	125.0f		//缓倒地自起只动摆角时速度(°/s)
//保持比例180:1(收腿的前中段恰好不碰地，后段碰地)
#define Chassis_RESET_Slow_Second_Angle_Speed 	90.0f		//缓倒地自起收腿时摆角速度(°/s)
#define Chassis_RESET_Slow_Second_Link_Speed 	0.5f		//缓倒地自起收腿时腿长速度(m/s)
//最终角度和LQR稳定后的腿摆角相同
#define Chassis_RESET_Slow_Second_L_Total_End 	9.0f		//缓倒地自起收腿时目标总摆角(°)
#define Chassis_RESET_Slow_Second_R_Total_End 	(-9.0f)		//缓倒地自起收腿时目标总摆角(°)

//Chassis_OFF_Slow_Fun缓失能函数相关宏定义
#define Chassis_OFF_Slow_Link_Speed 			0.17f		//缓失能伸腿速度(m/s)
#define Chassis_OFF_Slow_Angle_Speed 			60.0f		//缓失能摆角速度(°/s)
#define Chassis_OFF_Slow_L_Total_End 			75.0f		//缓失能左腿目标总摆角(°)
#define Chassis_OFF_Slow_R_Total_End 			(-75.0f)	//缓失能右腿目标总摆角(°)
#define Chassis_OFF_Slow_Link_Arrive_Error 		0.01f		//缓失能腿长到位误差(m)
#define Chassis_OFF_Slow_Smooth_K_Arrive_Error 	0.01f		//缓失能平滑启动比例到位阈值

//Chassis_Jump_Fun跳跃函数相关宏定义和变量
//蓄力到位腿长阈值(m)
const float Chassis_Jump_Ready_Link_Arrive = 0.16f;
//跳跃等级对应蹬伸腿长(m)
const float  Chassis_Jump_Push_Leg_Length_1 = 0.20f;
const float  Chassis_Jump_Push_Leg_Length_2 = 0.25f;
const float  Chassis_Jump_Push_Leg_Length_3 = 0.33f;
//收腿到位腿长阈值(m)
const float Chassis_Jump_Fold_Link_Arrive = 0.17f;
//跳跃结束腿长(m)
const float Chassis_Jump_End_Leg_length     = 0.20f;
//蹬腿沿杆力最大前馈(N，伸腿方向为正)与实际前馈权重的变化速率(0->1,需要1/Chassis_Jump_Push_Slope_Up时间)
const float Chassis_Jump_Push_Link_F        = 330.0f;
const float Chassis_Jump_Push_Slope_Up      = 25.0f;
//空中收腿沿杆力最大前馈(N，收腿方向为负)与其实际前馈权重的变化速率(0->1,需要1/Chassis_Jump_Fold_Slope_Up时间)
const float Chassis_Jump_Fold_Link_F        = -50.0f;
const float Chassis_Jump_Fold_Slope_Up      = 10.0f;
//各子状态到位确认时间(s)
const float Chassis_Jump_Crouch_Tick = 0.06f;
const float Chassis_Jump_Push_Tick   = 0.03f;
const float Chassis_Jump_Fold_Tick   = 0.08f;
//空中摆角力矩权重
const float Chassis_Jump_Air_Torque_K = 0.25f;

/**
 * ===| 翻倒自起 & 检测意外翻倒 |===
 * 
 * @note 参数注意：
 * @note 车头抬高,车尾放低时pitch增大(灯条一侧为车尾)
 * @note Pitch范围:-180~180度,车水平正置时Pitch为0度
 * @note 腿在相对机体垂直、竖直向下时腿摆角总为0°+N*360°
 * @note 左腿从左看逆时针为正方向
 * @note 右腿从右看逆时针为正方向
 * 
 * @note 使用说明：
 * @note 只有当Chassis_State等于任意翻倒自起枚举时，才会有输出(改变指针)
 * @note 检测到意外翻倒：Chassis_State不等于任意翻倒自起枚举,而Pitch却不对劲时会返回-1(Error)
 * @note 正常返回1
 * 
 * @note 使用：
 * @note 在需要自起时,让Chassis_State等于Chassis_Recover
 * @note 直接将Total_Angle_Target、Leg_Length_Target带入串级PID计算即可(摆角PID参数给硬一点)
 * @note 最终Chassis_State等于Chassis_FOLLOW时完成翻倒自起
 * 
 *  @param Chassis_Control->Chassis_Dt            控制周期(s)
 *  @param Chassis_Control->Chassis_Control.Chassis_State [out]当前底盘状态
 *  @param Chassis_Control->Chassis_Control.Pitch     当前底盘Pitch(极性见使用注意)
 *  @param Chassis_Control->Leg_Left.Total_Angle_Feedback 左腿总角度反馈(°)
 *  @param Chassis_Control->Leg_Right.Total_Angle_Feedback 右腿总角度反馈(°)
 *  @param Chassis_Control->Chassis_Control.Leg_Length_Feebback 	 腿长反馈(m)
 *
 *  @param Chassis_Control->Leg_Left.Total_Angle_Target [out] 左腿总角度目标(°)
 *  @param Chassis_Control->Leg_Right.Total_Angle_Target [out] 右腿总角度目标(°)
 *  @param Chassis_Control->Chassis_Control.Leg_Length_Target    [out] 目标腿长(m)
 * 
 *=========================================================================================*/
int8_t Chassis_Smooth_Restand_Fun(Chassis_Control_StructTypedef *Chassis_Control)
{
    const float L_Angle_Err = Chassis_Control->Leg_Left.Total_Angle_Target-Chassis_Control->Leg_Left.Total_Angle_Feedback;
    const float R_Angle_Err = Chassis_Control->Leg_Right.Total_Angle_Target-Chassis_Control->Leg_Right.Total_Angle_Feedback;
    const float L_Length_Err = Chassis_Control->Chassis_Control.Leg_Length_Target-Chassis_Control->Leg_Left.Length_Feedback;
    const float R_Length_Err = Chassis_Control->Chassis_Control.Leg_Length_Target-Chassis_Control->Leg_Right.Length_Feedback;

	//储存Chassis_State值，用于比较底盘状态与赋值
	Chassis_State_EnumTypedef Chassis_Return = Chassis_Control->Chassis_Control.Chassis_State;
	Chassis_State_EnumTypedef Chassis_RESET_Slow_FLag_Return = Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag;

	//用于翻倒回正，左腿摆向，逆时针:1，顺时针:-1
	//翻倒回正:腿摆角方向相同，前翻(Pitch<0车头触地)，左腿逆时针转
	static int8_t Angle_Speed = 0;	
	//检测翻倒回正腿摆角堵转计数器，单位:s
	static float Chassis_Recover_Angle_Stop_Error_Tim = 0.0f;

	//记录目标最终摆角值用
	static Chassis_State_EnumTypedef Last_State = Chassis_OFF;
	static Chassis_State_EnumTypedef Chassis_RESET_Slow_FLag_Last = 0;
	static float L_Total_End = 0.0f, R_Total_End = 0.0f;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒自起腿摆向和速度
	//记录最终目标腿摆角Total_End
	if(Chassis_Return != Last_State||(Chassis_RESET_Slow_FLag_Last != Chassis_RESET_Slow_FLag_Return&&Chassis_Return == Chassis_RESET_Slow))
	{
		if(Chassis_Return == Chassis_Recover)
		{
			//起点 = 当前总角度反馈
			Chassis_Control->Leg_Left.Total_Angle_Target  = Chassis_Control->Leg_Left.Total_Angle_Feedback;
			Chassis_Control->Leg_Right.Total_Angle_Target = Chassis_Control->Leg_Right.Total_Angle_Feedback;
			//车头触地,左腿逆时针转
			if(Chassis_Control->Chassis_Control.Pitch<0.0f)
			{
				Angle_Speed = 1;
			}
			//车尾触地,左腿顺时针转
			else
			{
				Angle_Speed = -1;
			}
		}
		if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 2)
		{
			//起点 = 当前总角度反馈
			Chassis_Control->Leg_Left.Total_Angle_Target = Chassis_Control->Leg_Left.Total_Angle_Feedback;
			Chassis_Control->Leg_Right.Total_Angle_Target = Chassis_Control->Leg_Right.Total_Angle_Feedback;

			//把总角包到 (-180,180],只用于判断"是否落在允许区间"
			float L_A0 = Caculate_Included_Angle(0.0f, Chassis_Control->Leg_Left.Total_Angle_Feedback);
			float R_A0 = Caculate_Included_Angle(0.0f, Chassis_Control->Leg_Right.Total_Angle_Feedback);

			
			//特殊区段，顺时针方向绕远
			if(L_A0 <= 0.0f && L_A0 >= -90.0f)
			{
				L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback -(360.0f-Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, 90.0f));
			}
			else//左腿目标 +90°:走最短弧;
			{
				L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, 90.0f);
			}
			
			if(R_A0 >= 0.0f && R_A0 <= 90.0f)
			{
				R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + (360.0f+Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, -90.0f));
			}
			else
			{
				R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, -90.0f);
			}

		}
		else if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 3)
		{
			//收腿阶段:端点 = 从当前总角度走"最短弧"回到 0°
			L_Total_End = Chassis_Control->Leg_Left.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, Chassis_RESET_Slow_Second_L_Total_End);
			R_Total_End = Chassis_Control->Leg_Right.Total_Angle_Feedback + Caculate_Included_Angle(Chassis_Control->Leg_Right.Total_Angle_Feedback, Chassis_RESET_Slow_Second_R_Total_End);
		}
		Last_State = Chassis_Return;
        Chassis_RESET_Slow_FLag_Last = Chassis_RESET_Slow_FLag_Return;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//翻倒回正
	if(Chassis_Return == Chassis_Recover)
	{
		//自由腿长
		Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;

		//根据摆角夹角设置固定速度归中
		float CenterSpeed_Target = Caculate_Included_Angle(Chassis_Control->Leg_Left.Total_Angle_Feedback, -Chassis_Control->Leg_Right.Total_Angle_Feedback);
		Limit_float(&CenterSpeed_Target, 50.0f, -50.0f);
		
		//翻倒回正检测
		//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
		if(fabsf(Chassis_Control->Chassis_Control.Pitch)<25.0f)
		{
            if(Chassis_RESET_Slow_FLag_Return ==1 )
            {
                Chassis_Return = Chassis_RESET_Slow;
                Chassis_RESET_Slow_FLag_Return = 2;
            }
            else
            {
                Chassis_Return = Chassis_RESET;
                Chassis_RESET_Slow_FLag_Return = 0;
            }
			
		}
		//其他区间：不稳定平衡姿态，摆腿回正
		else
		{
			if(fabsf(L_Angle_Err)< 50.0f)
			Chassis_Control->Leg_Left.Total_Angle_Target += (Chassis_Recover_Angle_Speed * Angle_Speed + CenterSpeed_Target)*Chassis_Control->Chassis_Dt;
			if(fabsf(R_Angle_Err)< 50.0f)
			Chassis_Control->Leg_Right.Total_Angle_Target +=(-Chassis_Recover_Angle_Speed * Angle_Speed + CenterSpeed_Target)*Chassis_Control->Chassis_Dt;
			if((fabsf(L_Angle_Err)< 40.0f)&&fabsf(R_Angle_Err)< 40.0f)
			{
				Chassis_Recover_Angle_Stop_Error_Tim = 0.0f;
			}
			else
			{
				Chassis_Recover_Angle_Stop_Error_Tim += Chassis_Control->Chassis_Dt;
			}
			if(Chassis_Recover_Angle_Stop_Error_Tim>1.0f)
			{
				Angle_Speed = -Angle_Speed;
				Chassis_Recover_Angle_Stop_Error_Tim = 0.0f;
			}
		}
	}
	
	//倒地缓自起(摆腿至90度)
	else if(Chassis_Return == Chassis_RESET_Slow && Chassis_RESET_Slow_FLag_Return == 2)
	{
		//自由腿长
		Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;
		
		/*===| 摆腿阶段:双腿总角度分别缓摆至 90° / -90°|===*/
		Acc_and_Dec(L_Total_End, &Chassis_Control->Leg_Left.Total_Angle_Target, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(R_Total_End, &Chassis_Control->Leg_Right.Total_Angle_Target, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_RESET_Slow_Frist_Angle_Speed, Chassis_Control->Chassis_Dt);

		/*===| 摆腿到位:锁存当前腿长(自由腿长)并进入收腿阶段 |===*/
		if(fabsf(L_Total_End - Chassis_Control->Leg_Left.Total_Angle_Target) < 0.05f
		&& fabsf(R_Total_End - Chassis_Control->Leg_Right.Total_Angle_Target) < 0.05f
		&& fabsf(L_Angle_Err) < 5.0f
		&& fabsf(R_Angle_Err) < 5.0f)
		{
			Chassis_Return     = Chassis_RESET_Slow;
            Chassis_RESET_Slow_FLag_Return = 3;
			Chassis_Control->Chassis_Control.Leg_Length_Target = Chassis_Control->Chassis_Control.Leg_Length_Feebback;
		}
	}
	//倒地缓自起(收腿)
	else if(Chassis_Return == Chassis_RESET_Slow&&Chassis_RESET_Slow_FLag_Return == 3)
	{
		/*===| 收腿阶段:总角度回零、腿长缓收至最短(速率与原来一致:90°/s、0.5m/s) |===*/
		Acc_and_Dec(L_Total_End, &Chassis_Control->Leg_Left.Total_Angle_Target, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(R_Total_End, &Chassis_Control->Leg_Right.Total_Angle_Target, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_RESET_Slow_Second_Angle_Speed, Chassis_Control->Chassis_Dt);
		Acc_and_Dec(Leg_Length_Min, &Chassis_Control->Chassis_Control.Leg_Length_Target, Chassis_RESET_Slow_Second_Link_Speed, Chassis_RESET_Slow_Second_Link_Speed, Chassis_Control->Chassis_Dt);

		if(fabsf(L_Total_End - Chassis_Control->Leg_Left.Total_Angle_Target)  < 0.05f
		&& fabsf(R_Total_End - Chassis_Control->Leg_Right.Total_Angle_Target)  < 0.05f
		&& fabsf(L_Angle_Err)  < 5.0f
		&& fabsf(R_Angle_Err)  < 5.0f
		&& fabsf(L_Length_Err) < 0.04f
		&& fabsf(R_Length_Err) < 0.04f)
		{
			/*===| 收腿完成:允许站立 |===*/
			Chassis_Return = Chassis_FOLLOW;
            Chassis_RESET_Slow_FLag_Return = 0;
		}
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//检测意外翻倒计数器，单位:s
	static float Chassis_Pitch_Error_Tim = 0.0f;
	//翻倒检测
	//Pitch：机体正置-25~25，翻倒：>100||<-100，其他区间：不稳定平衡姿态
	if(Chassis_Return!=Chassis_Recover&&fabsf(Chassis_Control->Chassis_Control.Pitch)>100.0f)
	{	
		Chassis_Pitch_Error_Tim += Chassis_Control->Chassis_Dt;

		//仍在倒地自起
		if(Chassis_Return==Chassis_RESET_Slow)
		{
			//持续0.1s后重新翻倒回正
			Chassis_Return = Chassis_Recover;
            Chassis_RESET_Slow_FLag_Return = 1;
			Chassis_Pitch_Error_Tim = 0.0f;
		}
        else if(Chassis_Return==Chassis_RESET)
        {
			//持续0.1s后重新翻倒回正
			Chassis_Return = Chassis_Recover;
            Chassis_RESET_Slow_FLag_Return = 0;
			Chassis_Pitch_Error_Tim = 0.0f;
        }
		//意外翻倒
		else if(Chassis_Pitch_Error_Tim>0.05f)
		{
			Chassis_Pitch_Error_Tim = 0.06f;
			return -1;
		}
	}
	else
	{
		Chassis_Pitch_Error_Tim = 0.0f;
	}

	//检测底盘状态改变时间计数器，单位:s
	static float Chassis_State_Change_Tim = 0.0f;
	if(Chassis_Control->Chassis_Control.Chassis_State != Chassis_Return||Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag != Chassis_RESET_Slow_FLag_Return)
	{
		Chassis_State_Change_Tim += Chassis_Control->Chassis_Dt;
		if(Chassis_State_Change_Tim>0.1f)
		{
			Chassis_Control->Chassis_Control.Chassis_State = Chassis_Return;
			Chassis_Control->WheelLeg_Control_Flag.Chassis_RESET_Slow_FLag = Chassis_RESET_Slow_FLag_Return;
			Chassis_State_Change_Tim = 0.0f;
		}
	}
	else
	{
		Chassis_State_Change_Tim = 0.0f;
	}
	return 1;
}


// //错误状态检测
// //离地检测(跳跃不检测)
// //Pith翻倒检测(没有,Chassis_Smooth_Restand_Fun返回-1就是寄了)
// int8_t Chassis_ErrorState_Detect_Fun(void)
// {
    
// }

/**
 * ===| 跳跃 |===
 * 
 * @attention 蹬腿跳跃状态时LQR仅保留正常控制量,PID控制腿长输出为0
 * @attention 跳跃至空中收腿状态时LQR仅保留控制摆角的控制量,轮电机输出0
 * @attention 开始跳跃过程中赋值RoboControl_Struct.Leg_Length = Chassis_Jump_End_Leg_length;保证落地后腿长连续并有足够的缓冲
 * @attention 蹬腿跳跃、空中收腿不要Roll补偿，落地缓冲中不要加Roll补偿
 * 
 * @note 参数与运动过程
 * @note 进入跳跃后依次经过 Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag有4个状态:
 * @note 1:降腿蓄力 -> 2:全力蹬腿 -> 3:空中收腿 -> 4:落地缓冲（待加入）
 * @note 状态1:腿长目标压至最短(Leg_Length_Min),双腿腿长均小于蓄力到位阈值并持续 Chassis_Jump_Crouch_Tick 后蓄力完成
 * @note 状态2:蹬腿沿杆力前馈由 K_slope 斜坡(0->1)平滑淡入,双腿腿长均超过跳跃等级对应蹬伸腿长并持续 Chassis_Jump_Push_Tick 后蹬腿完成
 * @note 子状态3:腿长目标压回最短快速收腿,收腿前馈随 K_slope 淡入,轮电机输出置0、摆角力矩按 Chassis_Jump_Air_Torque_K 削弱,双腿腿长均小于收腿到位阈值并持续 Chassis_Jump_Fold_Tick 后收腿完成
 * @note 子状态4:复位状态机,腿长目标恢复至 Jump_End_Leg_length，Chassis_State改为Chassis_FOLLOW
 * 
 * @note 使用说明:
 * @note 只有当Chassis_State等于Chassis_Jump时才会有输出(改变指针)
 * @note 进入跳跃:外部在需要跳跃时将Chassis_State置为Chassis_Jump
 * @note 最终Chassis_State等于Chassis_FOLLOW时完成跳跃
 * 
 * @param Chassis_Control_Struct->Chassis_Dt                              控制周期(s)
 * @param Chassis_Control_Struct->Chassis_Control.Chassis_State           [out]当前底盘状态
 * @param Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag [out]跳跃进行中状态标志位
 * @param Chassis_Control_Struct->Chassis_Control.Jump_Level              跳跃等级1~3
 * @param Chassis_Control_Struct->Leg_Left.Length_Feedback                左腿腿长反馈(m)
 * @param Chassis_Control_Struct->Leg_Right.Length_Feedback               右腿腿长反馈(m)
 * @param Chassis_Control_Struct->Chassis_Control.Leg_Length_Target       [out]目标腿长(m)
 * @param Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]      [out]左腿沿杆力前馈(N)
 * @param Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0]     [out]右腿沿杆力前馈(N)
 * @param Chassis_Control_Struct->Leg_Left.LQR_Wheel_Torque_Target        [out]左轮电机力矩(空中收腿状态赋值0)
 * @param Chassis_Control_Struct->Leg_Right.LQR_Wheel_Torque_Target       [out]右轮电机力矩(空中收腿状态赋值0)
 * 
 * @return 1:正在跳跃(或跳跃已完成,转入Chassis_FOLLOW); 0:未进入跳跃,无输出
 * 
 * 待完善：
 * 蹬腿跳跃状态时的腿长相等起跳/Roll水平起跳的位置/角度PID
 * 落地缓冲，不同腿长的缓冲
 * 根据测试看是否需要空中Pitch/Roll姿态调整（角度量守恒）
 *=========================================================================================*/
int8_t Chassis_Jump_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct)
{
    //子状态到位时间计数器,单位:ms
    static float Jump_state_change_Tim = 0.0f;
    //斜坡函数系数(0~1)
    static float K_slope = 0.0f;
    //用于检测状态切换
    static Chassis_State_EnumTypedef Last_State = Chassis_OFF;
    //跳跃等级对应蹬伸腿长(m)
    float Jump_Leg_length;

// /*===| 非跳跃状态 |===*/
//     if(Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_Jump)
//     {
// 		//不允许存在标志位非0
//         Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 0;
//         /*===| 退出跳跃:复位状态机并清除跳跃进行中标志位 |===*/
//         if(Last_State == Chassis_Jump)
//         {
//             Jump_state_change_Tim = 0.0f;
//             K_slope = 0.0f;
//             //清除跳跃沿杆力前馈,避免中途退出后残留
//             Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = 0.0f;
//             Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = 0.0f;
//         }
//         Last_State = Chassis_Control_Struct->Chassis_Control.Chassis_State;
//         return 0;
//     }

/*===| 跳跃状态 |===*/
    /*===| 进入跳跃准备状态 |===*/
    if(Last_State != Chassis_Jump&&Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_Jump)
    {
        Last_State = Chassis_Jump;
        Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 1;
        Jump_state_change_Tim = 0.0f;
        K_slope = 0.0f;
    }

    /*===| 跳跃等级决定蹬伸腿长 |===*/
    switch(Chassis_Control_Struct->Chassis_Control.Jump_Level)
    {
        case 1:
            Jump_Leg_length = Chassis_Jump_Push_Leg_Length_1;
            break;

        case 2:
            Jump_Leg_length = Chassis_Jump_Push_Leg_Length_2;
            break;

        case 3:
            Jump_Leg_length = Chassis_Jump_Push_Leg_Length_3;
            break;

        default:
            Jump_Leg_length = Chassis_Jump_Push_Leg_Length_1;
            break;
    }

    /*===| 子状态1:降腿蓄力 |===*/
    if(Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag == 1&&Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_Jump)
    {
        //腿长目标压至最短
        Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = Leg_Length_Min;

        //蓄力阶段无额外沿杆力前馈(其他力即弹簧补偿+重力补偿+腿长速度环PID)
        Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = 0.0f;
        Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = 0.0f;

        //双腿腿长均到位并持续一定时间后转入蹬腿
        if(Chassis_Control_Struct->Leg_Left.Length_Feedback  < Chassis_Jump_Ready_Link_Arrive
        && Chassis_Control_Struct->Leg_Right.Length_Feedback < Chassis_Jump_Ready_Link_Arrive)
        {
            Jump_state_change_Tim += Chassis_Control_Struct->Chassis_Dt;
        }
        else
        {
            Jump_state_change_Tim = 0.0f;
        }
        if(Jump_state_change_Tim >= Chassis_Jump_Crouch_Tick)
        {
            Jump_state_change_Tim = 0.0f;
            Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 2;     //状态转移
        }
    }
    /*===| 子状态2:全力蹬腿 |===*/
    else if(Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag == 2&&Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_Jump)
    {
        //蹬腿前馈斜坡淡入,避免力突变
        K_slope += Chassis_Jump_Push_Slope_Up * Chassis_Control_Struct->Chassis_Dt;
        Limit_float(&K_slope,1.0f,0.0f);

        Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = K_slope * Chassis_Jump_Push_Link_F;
        Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = K_slope * Chassis_Jump_Push_Link_F;

        //双腿腿长均超过蹬伸腿长并持续一定时间后转入空中收腿
        if(Chassis_Control_Struct->Leg_Left.Length_Feedback  > Jump_Leg_length
        && Chassis_Control_Struct->Leg_Right.Length_Feedback > Jump_Leg_length)
        {
            Jump_state_change_Tim += Chassis_Control_Struct->Chassis_Dt;
        }
        else
        {
            Jump_state_change_Tim = 0.0f;
        }
        if(Jump_state_change_Tim >= Chassis_Jump_Push_Tick)
        {
            K_slope = 0.0f;
            Jump_state_change_Tim = 0.0f;
            Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 3;     //状态转移
        }
    }
    /*===| 子状态3:空中调整快速收腿 |===*/
    else if(Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag == 3&&Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_Jump)
    {
        //收腿前馈斜坡淡入
        K_slope += Chassis_Jump_Fold_Slope_Up * Chassis_Control_Struct->Chassis_Dt;
        Limit_float(&K_slope,1.0f,0.0f);

        
        //腿长目标压回最短,快速收腿
        Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = K_slope*Leg_Length_Min+(1-K_slope)*Leg_Length_Max;

        Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = K_slope * Chassis_Jump_Fold_Link_F;
        Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = K_slope * Chassis_Jump_Fold_Link_F;

        //空中轮电机输出置0,避免轮子在空中乱转
        Chassis_Control_Struct->Leg_Left.LQR_Wheel_Torque_Target  = 0.0f;
        Chassis_Control_Struct->Leg_Right.LQR_Wheel_Torque_Target = 0.0f;

        //双腿腿长均收至可起立长度并持续一定时间后结束跳跃
        if(Chassis_Control_Struct->Leg_Left.Length_Feedback  < Chassis_Jump_Fold_Link_Arrive
        && Chassis_Control_Struct->Leg_Right.Length_Feedback < Chassis_Jump_Fold_Link_Arrive)
        {
            Jump_state_change_Tim += Chassis_Control_Struct->Chassis_Dt;
        }
        else
        {
            Jump_state_change_Tim = 0.0f;
        }
        if(Jump_state_change_Tim >= Chassis_Jump_Fold_Tick)
        {
            K_slope = 0.0f;
            Jump_state_change_Tim = 0.0f;
            Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 4;     //状态转移
        }
    }
    /*===| 子状态4:落地缓冲 |===*/
    else if(Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag == 4&&Chassis_Control_Struct->Chassis_Control.Chassis_State == Chassis_Jump)
    {
        Jump_state_change_Tim = 0.0f;
        K_slope = 0.0f;

        //清除跳跃沿杆力前馈,避免残留到后续跟随控制
        Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = 0.0f;
        Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = 0.0f;

        //腿长目标恢复,记得同步RoboControl腿长,避免回到跟随瞬间腿长目标跳变
        Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = Chassis_Jump_End_Leg_length;
        // RoboControl_Struct.Leg_Length = Chassis_Jump_End_Leg_length;

        //底盘跟随并清除跳跃进行中标志位
        Chassis_Control_Struct->Chassis_Control.Chassis_State = Chassis_FOLLOW;
        Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 0;
    }
	/*===| 子状态0:跳跃完成回到Chassis_FOLLOW或Chassis_Jump_FLag=0 |===*/

	VOFA_JustFloat_AddValue(0,Chassis_Control_Struct->Chassis_Control.Chassis_State);
	VOFA_JustFloat_AddValue(1,Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag);
	VOFA_JustFloat_AddValue(2,Chassis_Control_Struct->Chassis_Control.Leg_Length_Target);
	VOFA_JustFloat_AddValue(3,Chassis_Control_Struct->Leg_Left.Length_Feedback);
	VOFA_JustFloat_AddValue(4,Chassis_Control_Struct->Leg_Right.Length_Feedback);
	VOFA_JustFloat_AddValue(5,Chassis_Control_Struct->Leg_Left.Length_Speed);
	VOFA_JustFloat_AddValue(6,Chassis_Control_Struct->Leg_Right.Length_Speed);
	VOFA_JustFloat_AddValue(7,Jump_state_change_Tim);
	VOFA_JustFloat_AddValue(8,K_slope);

/*===| 非跳跃状态 |===*/
    if(Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_Jump)
    {
		//不允许存在标志位非0
        Chassis_Control_Struct->WheelLeg_Control_Flag.Chassis_Jump_FLag = 0;
        /*===| 退出跳跃:复位状态机并清除跳跃进行中标志位 |===*/
        if(Last_State == Chassis_Jump)
        {
            Jump_state_change_Tim = 0.0f;
            K_slope = 0.0f;
            //清除跳跃沿杆力前馈,避免中途退出后残留
            Chassis_Control_Struct->Leg_Left.LQR_Link_Torque_Target[0]  = 0.0f;
            Chassis_Control_Struct->Leg_Right.LQR_Link_Torque_Target[0] = 0.0f;
        }
        Last_State = Chassis_Control_Struct->Chassis_Control.Chassis_State;
        return 0;
    }

    return 1;
}

/**
 * ===| 缓失能 |===
 * 
 * @attention 仅测试使用，比赛禁用(也是非常相信机械好吧)
 * 
 * @note 参数与运动过程(与 yang_Chassis_OFF_Slow 一致):
 * @note 进入缓失能瞬间,腿长目标、双腿总摆角目标锁存为当前反馈值作为起点,
 * @note 随后腿长按 Chassis_OFF_Slow_Link_Speed 缓伸至最长(Leg_Length_Max),
 * @note 双腿总摆角按 Chassis_OFF_Slow_L_Total_End 缓摆至 Chassis_OFF_Slow_L/R_Total_End,
 * @note 当平滑启动比例 Smooth_Start_K 归零 或 腿长已伸至最长时,转入 Chassis_OFF 完成失能
 * 
 * @note 使用说明:
 * @note 只有当Chassis_State等于Chassis_OFF_Slow时才会有输出(改变指针)
 * @note 进入缓失能:外部在需要缓失能时将Chassis_State置为Chassis_OFF_Slow
 * @note 最终Chassis_State等于Chassis_OFF时完成缓失能
 * 
 * @param Chassis_Control_Struct->Chassis_Dt                        控制周期(s)
 * @param Chassis_Control_Struct->Chassis_Control.Chassis_State     [out]当前底盘状态
 * @param Chassis_Control_Struct->Chassis_Control.Leg_Length_Feebback 腿长反馈(m)
 * @param Chassis_Control_Struct->Chassis_Control.Leg_Length_Target [out]目标腿长(m)
 * @param Chassis_Control_Struct->Leg_Left.Total_Angle_Feedback     左腿总角度反馈(°)
 * @param Chassis_Control_Struct->Leg_Right.Total_Angle_Feedback    右腿总角度反馈(°)
 * @param Chassis_Control_Struct->Leg_Left.Total_Angle_Target       [out]左腿总角度目标(°)
 * @param Chassis_Control_Struct->Leg_Right.Total_Angle_Target      [out]右腿总角度目标(°)
 * @param Smooth_Start_K RoboControl_Struct.Smooth_Start_K赋值
 * 
 * @return 1:正在缓失能或缓失能已完成(转入Chassis_OFF); 0:未进入缓失能,无输出
 * 
 *=========================================================================================*/
int8_t Chassis_OFF_Slow_Fun(Chassis_Control_StructTypedef *Chassis_Control_Struct, float Smooth_Start_K)
{
    //缓失能终点:双腿总摆角(°)
    static float L_Total_End = 0.0f, R_Total_End = 0.0f;
    //用于检测状态切换(进入缓失能确认起点)
    static Chassis_State_EnumTypedef Last_State = Chassis_OFF;

    /*===| 只有缓失能状态才会有输出 |===*/
    if(Chassis_Control_Struct->Chassis_Control.Chassis_State != Chassis_OFF_Slow)
    {
        Last_State = Chassis_Control_Struct->Chassis_Control.Chassis_State;
        return 0;
    }

    /*===| 进入缓失能:确认起点并计算摆角终点 |===*/
    if(Last_State != Chassis_OFF_Slow)
    {
        //起点 = 当前腿长反馈(自由腿长)
        Chassis_Control_Struct->Chassis_Control.Leg_Length_Target = Chassis_Control_Struct->Chassis_Control.Leg_Length_Feebback;
        //起点 = 当前总角度反馈
        Chassis_Control_Struct->Leg_Left.Total_Angle_Target  = Chassis_Control_Struct->Leg_Left.Total_Angle_Feedback;
        Chassis_Control_Struct->Leg_Right.Total_Angle_Target = Chassis_Control_Struct->Leg_Right.Total_Angle_Feedback;
		//终点 = 从当前总角度走"最短弧"到 +90° / -90°
        L_Total_End = Chassis_Control_Struct->Leg_Left.Total_Angle_Feedback
                    + Caculate_Included_Angle(Chassis_Control_Struct->Leg_Left.Total_Angle_Feedback, Chassis_OFF_Slow_L_Total_End);
        R_Total_End = Chassis_Control_Struct->Leg_Right.Total_Angle_Feedback
                    + Caculate_Included_Angle(Chassis_Control_Struct->Leg_Right.Total_Angle_Feedback, Chassis_OFF_Slow_R_Total_End);


        Last_State = Chassis_OFF_Slow;
    }

    /*===| 缓失能过程:腿长缓伸至最长 + 双腿总摆角缓摆至 +90°/-90° |===*/
    Acc_and_Dec(Leg_Length_Max, &Chassis_Control_Struct->Chassis_Control.Leg_Length_Target,
                Chassis_OFF_Slow_Link_Speed, Chassis_OFF_Slow_Link_Speed, Chassis_Control_Struct->Chassis_Dt);
    Acc_and_Dec(L_Total_End, &Chassis_Control_Struct->Leg_Left.Total_Angle_Target,
                Chassis_OFF_Slow_Angle_Speed, Chassis_OFF_Slow_Angle_Speed, Chassis_Control_Struct->Chassis_Dt);
    Acc_and_Dec(R_Total_End, &Chassis_Control_Struct->Leg_Right.Total_Angle_Target,
                Chassis_OFF_Slow_Angle_Speed, Chassis_OFF_Slow_Angle_Speed, Chassis_Control_Struct->Chassis_Dt);

    /*===| 缓失能完成:平滑启动比例归零 或 腿长已伸至最长 → 完全失能 |===*/
    if(Smooth_Start_K < Chassis_OFF_Slow_Smooth_K_Arrive_Error
    || Chassis_Control_Struct->Chassis_Control.Leg_Length_Feebback >= Leg_Length_Max)
    {
        Chassis_Control_Struct->Chassis_Control.Chassis_State = Chassis_OFF;
        Last_State = Chassis_OFF;
    }
    return 1;
}