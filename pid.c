#include"pid.h"
#include"stdio.h"
#include <stdarg.h>

#ifdef __cplusplus
extern "C"
{
#endif
void _PID_Init(PID_T* pid);
void PID_hander_parallerl(float thispid_out, PID_T* this_pid, float cycle);
void PID_hander_cascade(float thispid_out, PID_T* this_pid, float cycle);

/////////////////////////////////////////////////////////
#if EXAMPLE
//以下为pid初始化实例和解析
PID_T pid={0};
PID_T next_pid={0};
void example(void)
{
	//first PID结构体应定义为全局变量以便调用
	//second 对结构全赋值为0以保证安全
	//thired 选择必要的成员进行初始化
//ps;对必要的成员采用 在行后面加 /// 经行标注
	/*
	该变量要存储两个模式
	1.串行 or 并行 or NULL
	THISPID_PARALLERL THISPID_CASCADE NULL
	2.位置式 or 增量式
	THISPID_POSITION THISPID_INCREMENT

	eg：pid->flag.thispid_mode =(THISPID_PARALLERL & THISPID_POSITION);
		pid->flag.thispid_mode =(NULL & THISPID_POSITION);
	*/
	pid.flag.thispid_mode;///
	/*

	#define DIFFERENTIAL_COMPLETE			//完全微分
	#define DIFFERENTIAL_PART				//不完全微分
	#define DIFFERENTIAL_PREVIOUS   		//微分先行
	以上为differential_way可选功能
	*/
	pid.flag.differential_way;///
	/*
	#define INTEGRAL_NORMAL						//普通积分
	#define INTEGRAL_SEPARATION					//积分分离
	#define INTEGRAL_SATURATION 				//抗饱和积分
	#define INTEGRAL_SPEED						//变速积分
	以上为积分的不可进行或运算的功能
	#define INTEGRAL_TRAPEZIOD					//梯形积分
	此功能能与其他积分功能进行或运算，即同时使用
	*/
	pid.flag.integral_way;///

	pid.flag.use_predict;//用于使用pid预测功能ENABLE DISABLE 

	pid.next_pid;//下一个pid的指针，没有时置NULL///
	pid.parameter.k;//并行pid相加系数          -- 用于并行pid


	pid.parameter.a;//低通滤波器参数          -- 用于带一阶低通的微分
	//p   i   d  系数
	pid.parameter.kp;///
	pid.parameter.ki;///
	pid.parameter.kd;///

	pid.parameter.kd_lpf;//不完全微分系数
	pid.parameter.kd_pre;//微分先行系数

	pid.parameter.kf;//前馈系数

	pid.parameter.k_pre;//预测系数 
	pid.parameter.target;//期望值///
	pid.parameter.present;//当前值///
	pid.parameter.predict;//预测值，要用到预测不可将置0///
	//以下功能不用时置为负数即可
	pid.parameter.target_limit = -1; //目标值限幅///
	pid.parameter.bias_limit = -1;   //误差限幅///
	pid.parameter.bias_dead_zone = -1;//小于这个值将不进行PID操作(死区)///
	pid.parameter.bias_for_integral = -1;//开始积分的误差	--	用于积分分离///
	pid.parameter.integral_limit = -1;//积分限幅		--	用于抗积分饱和///
	pid.parameter.out_limit = -1;  //输出限幅///

	pid->variable;//如用变速积分且不用默认函数，则该节点必须完成一个变速函数!!
	//经行多pid级联，注意，并行pid需多设置个系数k用于多pid相加
	pid.next_pid = &next_pid;
	pid.flag.thispid_mode = (THISPID_PARALLERL & THISPID_POSITION);//pid与next_pid串行
	pid.flag.thispid_mode = (THISPID_CASCADE & THISPID_POSITION);//pid与next_pid并行

}

#endif



//一个可变参数
//当使用并行pid时要初始化一个系数k（float）
void PID_SET_PIDMode(PID_T* pid, PID_Mode mode, ...)
{
	va_list ap;//声明一个va_list变量
	switch (mode)
	{
	case PID_POSITION_NULL:
		pid->flag.thispid_mode = THISPID_POSITION;
		break;	
	case PID_POSITION_PARALLERL:
		pid->flag.thispid_mode = (THISPID_POSITION & THISPID_PARALLERL);

		break;	
	case PID_POSITION_CASCADE:
		pid->flag.thispid_mode = (THISPID_POSITION & THISPID_CASCADE);
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.k = va_arg(ap, float);
		va_end(ap);
		break;	
	case PID_INCREMENT_NULL:
		pid->flag.thispid_mode = (THISPID_INCREMENT);

		break;	
	case PID_INCREMENT_PARALLERL:
		pid->flag.thispid_mode = (THISPID_INCREMENT & THISPID_PARALLERL);

		break;	
	case PID_INCREMENT_CASCADE:
		pid->flag.thispid_mode = (THISPID_INCREMENT & THISPID_CASCADE);
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.k = va_arg(ap, float);
		va_end(ap);
		break;
	default:
		pid->flag.thispid_mode = THISPID_POSITION;
		break;
	}
}
//更具不同的功能，可能要传入一个参数做特别的初始化float 或函数指针
void PID_SET_I_Function(PID_T* pid, PID_I_Function imode ,...)
{
	va_list ap;//声明一个va_list变量
	pid->flag.integral_way = imode;
	switch (imode)
	{
	case PID_INTEGRAL_NORMAL:
		break;
	case PID_INTEGRAL_SEPARATION:
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.bias_for_integral = va_arg(ap, float);
		va_end(ap);
		break;
	case PID_INTEGRAL_SATURATION:
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.integral_limit = va_arg(ap, float);
		va_end(ap);
		break;
	case PID_INTEGRAL_SPEED:
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->variable = va_arg(ap, void*);
		va_end(ap);
		break;
	case PID_INTEGRAL_TRAPEZIOD:
		break;
	default:
		break;
	}
}
//更具不同的功能，可能要传入一个参数做特别的初始化float 或函数指针
void PID_SET_D_Function(PID_T* pid, PID_D_Function dmode, ...)
{
	va_list ap;//声明一个va_list变量
	pid->flag.differential_way = dmode;
	switch (dmode)
	{
	case PID_DIFFERENTIAL_COMPLETE :	//完全微分
		break;
	case PID_DIFFERENTIAL_PART:					//不完全微分
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.kd_lpf = va_arg(ap, float);
		va_end(ap);
		break;
	case PID_DIFFERENTIAL_PREVIOUS :			//微分先行
		va_start(ap, 1);   //初始化，第二个参数为最后一个确定的形参
		pid->parameter.kd_pre = va_arg(ap, float);
		va_end(ap);
		break;
	default:
		break;
	}
}

void PID_Node_Connection(PID_T* pid, PID_T* next_pid)
{
	pid->next_pid = next_pid;
}

void PID_Sst_Target_Limit(PID_T* pid, float value)
{
	pid->parameter.target_limit = value;
}
void PID_Sst_Bias_Limit(PID_T* pid, float value)
{
	pid->parameter.bias_limit = value;
}
void PID_Sst_Bias_Dead_Zone(PID_T* pid, float value)
{
	pid->parameter.bias_dead_zone;
}
void PID_Sst_Integral_Limit(PID_T* pid, float value)
{
	pid->parameter.integral_limit = value;
}
void PID_Sst_Out_Limit(PID_T* pid, float value)
{
	pid->parameter.out_limit = value;
}
void PID_Sst_Feedforward(PID_T* pid, float value)
{
	pid->parameter.kf = value;
}
void PID_Sst_KPre(PID_T* pid, float value)
{
	pid->flag.use_predict = 1;
	pid->parameter.k_pre = value;
}
void PID_Sst_Target(PID_T* pid, float value)
{
	pid->parameter.target = value;
}
void PID_Sst_Present(PID_T* pid, float value)
{
	pid->parameter.present = value;
}
void PID_Sst_Predict(PID_T* pid, float value)
{
	pid->parameter.predict = value;
}


//return a * X(n) + (1 - a) * Y(n - 1)
float FOLowPassFilter(float In, float LastOut, float a)
{
	return a * In + (1 - a) * LastOut;
}

//！！！！！！本函数除第一个参数仅可传PID_T类型！！！！！！！
void PID_Base_Init(int count, ...)
{
	//count 表示可变参数个数
	va_list ap;           //声明一个va_list变量
	va_start(ap, count);   //初始化，第二个参数为最后一个确定的形参

	for (int i = 0; i < count; i++)
	{

		_PID_Init(va_arg(ap, PID_T*)); //读取可变参数，的二个参数为可变参数的类型


	}
	va_end(ap);                 //清理工作 
}

void PID_Disable(PID_T* pid)
{
	pid->flag.run = SHOUTDOWN;
}
void PID_Enable(PID_T* pid)
{
	pid->flag.run = RUN;
}
void _PID_Init(PID_T* pid)
{

	if ((pid->next_pid == NULL) && (pid->flag.thispid_mode & 0x30))//使用了串级或并行却未更改指针
		while (1);
	pid->flag.run = RUN;

	pid->parameter.target = 0;
	pid->parameter.present = 0;
	pid->parameter.predict = 0;

	pid->parameter.kp = 0;
	pid->parameter.ki = 0;
	pid->parameter.kd = 0;
	pid->parameter.kf = 0;
	pid->parameter.kd_lpf = 0;
	pid->parameter.kd_pre = 0;
	pid->parameter.k_pre = 0;

	pid->parameter.target_limit = -1;
	pid->parameter.bias_limit = -1;
	pid->parameter.bias_dead_zone = -1;
	pid->parameter.bias_for_integral = -1;
	pid->parameter.integral_limit = -1;
	pid->parameter.out_limit = -1;

	pid->parameter.out = 0;
	pid->parameter.all_out = 0;

	pid->process.bias = 0;
	pid->process.differential_bias = 0;
	pid->process.lpf_differential_bias = 0;
	pid->process.feedforward = 0;
	pid->process.integral_bias = 0;
	pid->process.last_bias = 0;
	pid->process.lastlast_bias = 0;
	pid->process.last_target = 0;
}

void PID_Hander(PID_T* start_pid, float cycle)
{
	PID_hander_cascade(0, start_pid, cycle);
}

void PID_hander_parallerl(float thispid_out,PID_T* this_pid, float cycle)
{
	this_pid->parameter.present = thispid_out;
	if (this_pid->flag.thispid_mode & THISPID_INCREMENT)
		_PID_Hander_Increment(this_pid, cycle);
	else
		_PID_Hander_Position(this_pid, cycle);
	if ((this_pid->flag.thispid_mode & THISPID_PARALLERL) && (this_pid->next_pid != NULL))//串级
	{
		PID_T* next_pid = (PID_T*)this_pid->next_pid;
		next_pid->parameter.target = this_pid->parameter.out;
		PID_hander_parallerl(this_pid->parameter.out,this_pid->next_pid, cycle);
	}
	else if ((this_pid->flag.thispid_mode & THISPID_CASCADE) && (this_pid->next_pid != NULL))//并行
	{
		this_pid->parameter.all_out = (this_pid->parameter.out * this_pid->parameter.k);
		PID_hander_cascade(this_pid->parameter.all_out, this_pid->next_pid, cycle);
		//this_pid->parameter.all_out = 0;//清除，为下一次使用做初始化
	}

}

void PID_hander_cascade( float thispid_out,PID_T *this_pid,float cycle)
{
	if (this_pid->flag.thispid_mode & THISPID_INCREMENT)
		_PID_Hander_Increment(this_pid, cycle);
	else
		_PID_Hander_Position(this_pid, cycle);
	
	this_pid->parameter.all_out = thispid_out + this_pid->parameter.out * this_pid->parameter.k;

	if ((this_pid->flag.thispid_mode & THISPID_PARALLERL) && (this_pid->next_pid != NULL))//串级
	{
		//this_pid->next_pid->PID_Parameter->present = this_pid->parameter.all_out;
		PID_hander_parallerl(this_pid->parameter.out,this_pid->next_pid, cycle);
	}
	else if ((this_pid->flag.thispid_mode & THISPID_CASCADE) && (this_pid->next_pid != NULL))//并行
		PID_hander_cascade(this_pid->parameter.all_out, this_pid->next_pid, cycle);
	//next_pid->parameter.all_out += (next_pid->parameter.out + thispid_out);



}

void _PID_Hander_Increment(PID_T* pid, float cycle)
{
	if (pid->flag.run == SHOUTDOWN)
	{
		pid->parameter.out = pid->parameter.present;
		pid->process.bias = 0;
		pid->process.differential_bias = 0;
		pid->process.lpf_differential_bias = 0;
		pid->process.feedforward = 0;
		pid->process.integral_bias = 0;
		pid->process.last_bias = 0;
		pid->process.last_target = 0;
		pid->process.lastlast_bias = 0;
		pid->parameter.present = 0;
		pid->parameter.predict = 0;
		return;
	}
	////////////////////////////////期望值限幅
	if (pid->parameter.target_limit >= 0)
	{
		if ((pid->parameter.target) > pid->parameter.target_limit)
			(pid->parameter.target) = pid->parameter.target_limit;
		else if ((pid->parameter.target) < -pid->parameter.target_limit)
			(pid->parameter.target) = -pid->parameter.target_limit;
	}

	////////////////////////////////前馈操作--前馈值直接加入输出
	pid->process.feedforward = pid->parameter.kf * (pid->parameter.target);
	/////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////预测操作--预测值直接加入偏差	
	if (pid->flag.use_predict == 0 || pid->parameter.predict == 0)
		pid->process.predict = 0;
	else
		pid->process.predict = pid->parameter.k_pre * pid->parameter.predict * ABS(pid->parameter.predict);
	/////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////偏差操作	
		//第一部分为期望
		//第二部分为反馈
		//第三部分为预测（一般为内环反馈,可不设置）
	float temp_bias = (pid->parameter.target) -
		(pid->parameter.present) -
		pid->process.predict;

	if (pid->parameter.bias_dead_zone >= 0)
		temp_bias = (temp_bias < pid->parameter.bias_dead_zone&&
			temp_bias > -pid->parameter.bias_dead_zone) ? 0 : temp_bias;	//误差死区判断--不用该功能时可以将其置-1

	if (pid->parameter.bias_limit >= 0)
	{
		temp_bias = (temp_bias > pid->parameter.bias_limit) ? pid->parameter.bias_limit : temp_bias;//误差限幅--不用该功能时可以将其置-1
		temp_bias = (temp_bias < -pid->parameter.bias_limit) ? -pid->parameter.bias_limit : temp_bias;
	}
	pid->process.bias = temp_bias;		//计算误差
	/////////////////////////////////////////////////////////////////////////////////	

	////////////////////////////////积分操作
	if (pid->parameter.ki == 0)
		pid->process.integral_bias = 0;

	else
	{

		switch (pid->flag.integral_way)
		{
		case INTEGRAL_NORMAL:
			//普通积分
			pid->process.integral_bias = pid->process.bias * cycle;
			pid->process.integral_bias = pid->process.bias * cycle;

			break;

		case INTEGRAL_SEPARATION:
			//积分分离
			if (pid->process.bias > pid->parameter.bias_for_integral ||
				pid->process.bias < -pid->parameter.bias_for_integral)
				break;

			pid->process.integral_bias = pid->process.bias * cycle;
			break;

		case INTEGRAL_SATURATION:
			//抗积分饱和
			if (pid->process.integral_bias * pid->parameter.ki > pid->parameter.integral_limit)
				pid->process.integral_bias = pid->parameter.integral_limit / pid->parameter.ki;

			else if (pid->process.integral_bias * pid->parameter.ki < -pid->parameter.integral_limit)
				pid->process.integral_bias = -pid->parameter.integral_limit / pid->parameter.ki;

			else
				pid->process.integral_bias = pid->process.bias * cycle;
			break;

		case INTEGRAL_SPEED:
			//变速积分 -- 可自行添加函数或处理算法
			pid->process.integral_bias = (pid->process.bias + pid->process.last_bias) / (2.0f * cycle);
			break;

		default:
			//默认为普通积分
			pid->process.integral_bias = pid->process.bias * cycle;
			break;
		}


		if (pid->flag.integral_way & INTEGRAL_TRAPEZIOD)
			pid->process.integral_bias -= (pid->process.bias - pid->process.last_bias) * cycle;
		/////////////////////////////////////////////////////////////////////////////////	

		////////////////////////////////微分操作	
		switch (pid->flag.differential_way)
		{//微分D : e(k) - 2e(k - 1) + e(k - 2)   当前误差 - 2 * 上次误差 + 上上次误差

		case DIFFERENTIAL_COMPLETE:
			//直接求微分
			pid->process.lpf_differential_bias
				= pid->process.differential_bias
				= pid->process.bias - 2 * pid->process.last_bias + pid->process.lastlast_bias;
			break;

		case DIFFERENTIAL_PART:
			//求微分，再低通滤波
			pid->process.differential_bias = pid->process.bias - 2 * pid->process.last_bias + pid->process.lastlast_bias;//本行进行d的计算
			pid->process.differential_bias = FOLowPassFilter(pid->process.differential_bias, pid->process.lpf_differential_bias, pid->parameter.kd_lpf);
			pid->process.lpf_differential_bias = pid->process.differential_bias;
			//pid->process.lpf_differential_bias = pid->parameter.kd_lpf * 
			//	(3.1415926f * cycle * (pid->process.differential_bias - pid->process.lpf_differential_bias));//本行进行低通滤波 输入输出为d
			//	//(3.1415926f * cycle * (pid->process.bias - 2*pid->process.last_bias + pid->process.lastlast_bias));//本行进行低通滤波 输入输出为d
			break;

		case DIFFERENTIAL_PREVIOUS:
			//微分先行
			pid->process.lpf_differential_bias
				= pid->process.differential_bias
				= (pid->process.bias - 2 * pid->process.last_bias + pid->process.lastlast_bias) - pid->parameter.kd_pre * (pid->parameter.target - pid->process.last_target);
			break;

		default:
			//直接求微分
			pid->process.lpf_differential_bias
				= pid->process.differential_bias
				= pid->process.bias - 2 * pid->process.last_bias + pid->process.lastlast_bias;
			break;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////	

	////////////////////////////////输出合成

#if USE_HOOK_FIRST
	pid->parameter.user_hook_first(pid);
#endif
	//计算输出
	pid->parameter.out = pid->parameter.kp * pid->process.bias +
		pid->parameter.ki * pid->process.integral_bias +
		pid->parameter.kd * pid->process.lpf_differential_bias / cycle + 	//可以将cycle注释
		pid->process.feedforward;
#if USE_HOOK_OUT
	pid->parameter.user_hook_end(pid);
#endif
	//输出限幅
	if (pid->parameter.out_limit >= 0)
	{
		if (pid->parameter.out > pid->parameter.out_limit)
			pid->parameter.out = pid->parameter.out_limit;

		else if (pid->parameter.out < -pid->parameter.out_limit)
			pid->parameter.out = -pid->parameter.out_limit;
	}
	//存储过去值
	pid->process.last_target = (pid->parameter.target);
	pid->process.lastlast_bias = pid->process.last_bias;
	pid->process.last_bias = pid->process.bias;
}

void _PID_Hander_Position(PID_T* pid, float cycle)
{
	if (pid->flag.run == SHOUTDOWN)
	{
		pid->parameter.out = pid->parameter.present;
		pid->process.bias = 0;
		pid->process.differential_bias = 0;
		pid->process.lpf_differential_bias = 0;
		pid->process.feedforward = 0;
		pid->process.integral_bias = 0;
		pid->process.last_bias = 0;
		pid->process.last_target = 0;
		pid->process.lastlast_bias = 0;
		pid->parameter.present = 0;
		pid->parameter.predict = 0;
		return;
	}
	////////////////////////////////期望值限幅
	if (pid->parameter.target_limit >= 0)
	{
		if ((pid->parameter.target) > pid->parameter.target_limit)
			(pid->parameter.target) = pid->parameter.target_limit;
		else if ((pid->parameter.target) < -pid->parameter.target_limit)
			(pid->parameter.target) = -pid->parameter.target_limit;
	}

	////////////////////////////////前馈操作--前馈值直接加入输出
	pid->process.feedforward = pid->parameter.kf *(pid->parameter.target);
	/////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////预测操作--预测值直接加入偏差	
	if (pid->flag.use_predict == 0 || pid->parameter.predict == 0)//似乎可能存在predict真的计算为0的bug，待验证**
		pid->process.predict = 0;
	else
		pid->process.predict = pid->parameter.k_pre * pid->parameter.predict * ABS(pid->parameter.predict);
	/////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////偏差操作	
		//第一部分为期望
		//第二部分为反馈
		//第三部分为预测（一般为内环反馈,可不设置）
	float temp_bias = (pid->parameter.target) -
		(pid->parameter.present) -
		pid->process.predict;

	if (pid->parameter.bias_dead_zone >= 0)
		temp_bias = (temp_bias < pid->parameter.bias_dead_zone&&
			temp_bias > -pid->parameter.bias_dead_zone) ? 0 : temp_bias;	//误差死区判断--不用该功能时可以将其置-1

	if (pid->parameter.bias_limit >= 0)
	{
		temp_bias = (temp_bias > pid->parameter.bias_limit) ? pid->parameter.bias_limit : temp_bias;//误差限幅--不用该功能时可以将其置-1
		temp_bias = (temp_bias < -pid->parameter.bias_limit) ? -pid->parameter.bias_limit : temp_bias;
	}
	pid->process.bias = temp_bias;		//计算误差
	/////////////////////////////////////////////////////////////////////////////////	

	////////////////////////////////积分操作
	if (pid->parameter.ki == 0)
		pid->process.integral_bias = 0;

	else

	{

		switch (pid->flag.integral_way)
		{
		case INTEGRAL_NORMAL:
			//普通积分
			pid->process.integral_bias += pid->process.bias * cycle;
			pid->process.integral_bias += pid->process.bias * cycle;

			break;

		case INTEGRAL_SEPARATION:
			//积分分离
			if (pid->process.bias > pid->parameter.bias_for_integral || pid->process.bias < -pid->parameter.bias_for_integral)
				break;

			pid->process.integral_bias += pid->process.bias * cycle;
			break;

		case INTEGRAL_SATURATION:
			//抗积分饱和
			if (pid->process.integral_bias * pid->parameter.ki > pid->parameter.integral_limit)
				pid->process.integral_bias = pid->parameter.integral_limit / pid->parameter.ki;

			else if (pid->process.integral_bias * pid->parameter.ki < -pid->parameter.integral_limit)
				pid->process.integral_bias = -pid->parameter.integral_limit / pid->parameter.ki;

			else
				pid->process.integral_bias += pid->process.bias * cycle;
			break;

		case INTEGRAL_SPEED:
			//变速积分 -- 可自行添加函数或处理算法
			pid->process.integral_bias += (pid->process.bias + pid->process.last_bias) / (2.0f * cycle);
			break;

		default:
			//默认为普通积分
			pid->process.integral_bias += pid->process.bias * cycle;
			break;
		}


		if (pid->flag.integral_way & INTEGRAL_TRAPEZIOD)
			pid->process.integral_bias -= (pid->process.bias - pid->process.last_bias) * cycle;
	}
	/////////////////////////////////////////////////////////////////////////////////	

	////////////////////////////////微分操作	
	switch (pid->flag.differential_way)
	{
	case DIFFERENTIAL_COMPLETE:
		//直接求微分
		pid->process.lpf_differential_bias
			= pid->process.differential_bias
			= pid->process.bias - pid->process.last_bias;
		break;

	case DIFFERENTIAL_PART:
		//求微分，再低通滤波
		pid->process.differential_bias = pid->process.bias - pid->process.last_bias;
		pid->process.differential_bias = FOLowPassFilter(pid->process.differential_bias, pid->process.lpf_differential_bias,pid->parameter.kd_lpf);
		pid->process.lpf_differential_bias = pid->process.differential_bias;
		/*pid->process.lpf_differential_bias += pid->parameter.kd_lpf * 3.1415926f * cycle
			* (pid->process.differential_bias - pid->process.lpf_differential_bias);*/
		break;

	case DIFFERENTIAL_PREVIOUS:
		//微分先行
		pid->process.lpf_differential_bias
			= pid->process.differential_bias
			= (pid->process.bias - pid->process.last_bias) - pid->parameter.kd_pre * (pid->parameter.target - pid->process.last_target);
		break;

	default:
		//直接求微分
		pid->process.lpf_differential_bias
			= pid->process.differential_bias
			= pid->process.bias - pid->process.last_bias;
		break;
	}
	/////////////////////////////////////////////////////////////////////////////////	

	////////////////////////////////输出合成

#if USE_HOOK_FIRST
	pid->parameter.user_hook_first(pid);
#endif
	//计算输出
	pid->parameter.out = pid->parameter.kp * pid->process.bias +
		pid->parameter.ki * pid->process.integral_bias +
		pid->parameter.kd * pid->process.lpf_differential_bias / cycle + 	//可以将cycle注释
		pid->process.feedforward;
#if USE_HOOK_OUT
	pid->parameter.user_hook_end(pid);
#endif
	//输出限幅
	if (pid->parameter.out_limit >= 0)
	{
		if (pid->parameter.out > pid->parameter.out_limit)
			pid->parameter.out = pid->parameter.out_limit;

		else if (pid->parameter.out < -pid->parameter.out_limit)
			pid->parameter.out = -pid->parameter.out_limit;
	}
	//存储过去值
	pid->process.last_target = (pid->parameter.target);
	pid->process.last_bias = pid->process.bias;
}

/*

pid->flag.run = 0;
pid->flag.integral_way = INTEGRAL_NORMAL;
pid->flag.differential_way = DIFFERENTIAL_COMPLETE;
pid->flag.use_predict = 0;

pid->parameter.target = 0;
pid->parameter.present = 0;
pid->parameter.predict = 0;

pid->parameter.kp = 0;
pid->parameter.ki = 0;
pid->parameter.kd = 0;
pid->parameter.kf = 0;
pid->parameter.kd_lpf = 0;
pid->parameter.kd_pre = 0;
pid->parameter.k_pre = 0;

pid->parameter.target_limit = -1;
pid->parameter.bias_limit = -1;
pid->parameter.bias_dead_zone = -1;
pid->parameter.bias_for_integral = -1;
pid->parameter.integral_limit = -1;
pid->parameter.out_limit = -1;

pid->parameter.out = 0;
pid->parameter.a = 1;

pid->process.bias = 0;
pid->process.differential_bias = 0;
pid->process.lpf_differential_bias = 0;
pid->process.feedforward = 0;
pid->process.integral_bias = 0;
pid->process.last_bias = 0;
pid->process.lastlast_bias = 0;
pid->process.last_target = 0;
*/

