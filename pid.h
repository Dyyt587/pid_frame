//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@文件名		：pid.h
//@描述			：pid模块库，可实现pid的计算
#ifndef _PID_H
#define _PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef u8
#define u8 unsigned char

#endif
#define ENABLE 1
#define DISABLE 0

#define INTEGRAL_NORMAL				0x00			//普通积分
#define INTEGRAL_SEPARATION		0x01			//积分分离
#define INTEGRAL_SATURATION 	0x02			//抗饱和积分
#define INTEGRAL_SPEED				0x04			//变速积分	
#define INTEGRAL_TRAPEZIOD				0x08			//梯形积分	
#define INTEGRAL_USE_SWITCH 1 //使用switch程序结构，更快但以上方案仅可梯形积分与其他方案或运算
//不建议在任何条件下对除梯形积分与其他方案或运算 以外运算经行或操作
//因为在程序上仅通过ifelse判断后就是简单的多次运算相加
//此功能还在开发中
//强烈建议#define INTEGRAL_USE_SWITCH 为1！！！！！！！！

#define DIFFERENTIAL_COMPLETE	0			//完全微分
#define DIFFERENTIAL_PART			1			//不完全微分
#define DIFFERENTIAL_PREVIOUS 2			//微分先行


////一阶低通滤波
//#define FO_LOW_PASS_FILTER_SENSITIVE_a                   0.8f                        //一阶低通滤波系数取值范围为(0,1)。值越小越稳定，越大越灵敏，二者难兼顾。
//#define FO_LOW_PASS_FILTER_STEADY_a                      0.15f
////输入Adc采样值，输出低通滤波值
////优点：调节精细，稳定度和灵敏度偏向分明。缺点：带有浮点运算
//#define FOLowPassFilter(In,LastIn, a)                                                                                                       \
//{                                                                                                                                           \
//    g_u16FOLowPassFilterResult = (uint16_t)(a * AdcSample + (1 - a) * g_u16FOLowPassFilterResultL);                                         \
//    g_u16FOLowPassFilterResultL = g_u16FOLowPassFilterResult;                                                                               \
//}


//注意，钩子函数对所有pid节点都有效，但每个节点都可自行编写属于自己的函数
#define USE_HOOK_FIRST 0//使用钩子函数，自行编写
#define USE_HOOK_END 0
#define USE_VARIABLE_DEFAULT 1 //变数积分使用默认函数
#define EXAMPLE 0 

#define ABS(x) ((x > 0) ? x : -x)

#define THISPID_PARALLERL 0x10 //串级
#define THISPID_CASCADE 0x20  //并行

#define THISPID_POSITION 0x00 //位置
#define THISPID_INCREMENT 0x01 //增量

#define SHOUTDOWN 0
#define RUN 1

typedef enum PID_Mode
{
	PID_POSITION_NULL = 0,
	PID_POSITION_PARALLERL,
	PID_POSITION_CASCADE,
	PID_INCREMENT_NULL,
	PID_INCREMENT_PARALLERL,
	PID_INCREMENT_CASCADE
}PID_Mode;

typedef enum PID_I_Function
{
	PID_INTEGRAL_NORMAL=0,
	PID_INTEGRAL_SEPARATION,
	PID_INTEGRAL_SATURATION,
	PID_INTEGRAL_SPEED,
	PID_INTEGRAL_TRAPEZIOD
}PID_I_Function;

typedef enum PID_D_Function
{
 PID_DIFFERENTIAL_COMPLETE =0,			//完全微分
 PID_DIFFERENTIAL_PART,					//不完全微分
 PID_DIFFERENTIAL_PREVIOUS 			//微分先行
}PID_D_Function;

typedef struct PID_Flag
{
	u8 run;
	u8 integral_way;
	u8 differential_way;
	u8 use_predict;
	u8 thispid_mode;//判定对下个pid为并行或串级
}PID_Flag;

typedef struct PID_Parameter
{
	float kp;								//比例系数
	float ki;								//积分系数
	float kd;								//微分系数
	float kf;								//前馈系数

	float kd_lpf;						//不完全微分系数
	float kd_pre;						//微分先行系数

	float k_pre;						//预测系数

	float target_limit;				//目标值限幅
	float bias_limit;					//误差限幅
	float bias_dead_zone;			//小于这个值将不进行PID操作
	float bias_for_integral;	//开始积分的误差	--	用于积分分离
	float integral_limit;			//积分限幅				--	用于抗积分饱和
	float out_limit;					//输出限幅
	//float a;               //低通滤波器参数          -- 用于带一阶低通的微分
	float k;               //并行pid相加系数          -- 用于并行pid

	float out; //此节点pid输出
	float all_out; //同一级pid输出，并行就累加有最后一个pid传入下一个节点，串级就等于out

	float target; //期望值
	float present;//当前值
	float predict;//预测值

#if USE_HOOK_FIRST
	void (*user_hook_first)(PID_T* pid);//钩子函数，在计算result之前，其他必要操作之后
#endif
#if USE_HOOK_FIRST
	void (*user_hook_out)(PID_T* pid);//钩子函数，在计算result之后，限幅之前
#endif
}PID_Parameter;

//增量式pid
// n
//比例P:    e(k) - e(k - 1)   当前误差 - 上次误差
//
//积分I : e(i)     当前误差
//
//微分D : e(k) - 2e(k - 1) + e(k - 2)   当前误差 - 2 * 上次误差 + 上上次误差

//位置式
//
//e(k) : 用户设定的值（目标值） - 控制对象的当前的状态值
//
//比例P : e(k)
//
//积分I : ∑e(i)     误差的累加(包括e(k))
//
//微分D : e(k) - e(k - 1)  这次误差 - 上次误差

typedef struct PID_Process
{
	float bias; //总误差
	float integral_bias;//i误差
	float differential_bias;//d误差
	float lpf_differential_bias;//上次d误差

	float feedforward; //前馈值
	float predict;  //预测值

	float last_target; //上次期望值
	float last_bias; //上次误差
	float lastlast_bias; //上上次误差

}PID_Process;

typedef struct PID_T PID_t;
typedef struct PID_T PID_T;
struct PID_T
{
	PID_Flag		flag;
	PID_Parameter	parameter;
	PID_Process 	process;
	PID_T* next_pid;//用于实现pid的并行或串级
	void(*variable)(PID_T* pid);
};


void PID_Init(int count, ...);
void PID_Disable(PID_T* pid);
void PID_Enable(PID_T* pid);
void PID_Hander(PID_T* start_pid, float cycle);
void _PID_Hander_Position(PID_T* pid, float cycle);
void _PID_Hander_Increment(PID_T* pid, float cycle);

#ifdef __cplusplus
}
#endif
#endif


