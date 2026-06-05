#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "mymath.h"
#include "pid_temp.h"
#include "arm_math.h"
#include "Motors.h"
#include "IMU_Task.h"
#include "controller.h"

#define BANDENG_LENGTH 0.16f

#define RADIUS_WHEEL   0.06f
#define MASS_WHEEL     0.572f         // +-   屁股0.117，加到轮上
#define MASS_BODY     12.0f          // +-0.1
#define L1_LENGTH     0.215f
#define L2_LENGTH     0.258f
#define L3_LENGTH     0.258f
#define L4_LENGTH     0.215f
#define L5_LENGTH     0.0f

#define POS 0
#define VEL 1
#define ACC 2

#define MAX_TORQUE_LEG_T 25.0f
#define MAX_TORQUE_LEG_W 8.0f
#define MAX_LEG_LENGTH 0.30f
#define MIN_LEG_LENGTH 0.15f

#define REDUCTION_RATIO 15.764705882352941176470588235294f   // 268/17
// #define TORQUE_RATE 0.06894876031507928693743672227522f      // 含减速箱0.92
// #define TORQUE_RATE 0.22983222247759135830843484256493f    // 0.3* 268/17 * 0.92 倒数
#define TORQUE_RATE 0.76334365325077430968421052631579f    // 1.0f/19.0f * 15.764705882352941176470588235294f * 0.92f 
// #define TORQUE_RATE 2824.5f     // 0.02nM -> 15.7


/* 电机参数 */
#define KT              0.0157f          // N·m/A
#define RATIO           (268.0f / 17.0f) // ≈ 15.7647
#define ETA             0.92f
#define TORQUE_TO_CMD_MOTOR(TORQUE_NM)   ((TORQUE_NM) * (819.2f / KT))
#define TORQUE_TO_CMD_LOAD(TORQUE_NM)    ((TORQUE_NM) * (819.2f / (KT * RATIO * ETA)))

typedef struct 
{
    float phi0[3];
    float L0[3];
    float phi1[3];
    float phi2[3];
    float phi3[3];
    float phi4[3];
    float JRM[2][2];            // 雅可比矩阵
    float JRM_inv[2][2];
    float alpha;                // 气弹簧解算对应角
    float ls;                   // 气弹簧解算对应长度
    float Fv;                   // 气弹簧等效作用支持力
    float s3;
    float theta3;
}vmc_Typedef;                   // vmc正解计算中间变量

typedef struct 
{
    struct 
    {
        float theta;
        float dtheta;
        float ddtheta;
        float s;
        float dot_s;
        float raw_dot_s;
        float phi;
        float dphi;
        float ddphi;
    }stateSpace;                          // 状态变量
    
    struct
    {
        float l0;                     // 目标腿长
        float roll;                   // roll 补偿
        float yaw;                    // 转向控制
        float d2theta;                // 目标劈叉角度

        float theta;
        float dtheta;
        float s;
        float dot_s;
        float phi;
        float dphi;
    }target;

    vmc_Typedef vmc_calc;

    struct 
    {
        Discreteness_TypeDef Phi_1;   // vmc正解phi1 求解速度
        Discreteness_TypeDef Phi_4;   // vmc正解phi4 求解速度
        Discreteness_TypeDef L;       // vmc正解L    求解拟合杆速度
        Discreteness_TypeDef D_L;     // vmc正解D_L  求解拟合杆加速度
        Discreteness_TypeDef Theta;   // vmc正解Theta 求解速度
        Discreteness_TypeDef dTheta;  // vmc正解dTheta 求解加速度
        Discreteness_TypeDef Phi;     // vmc正解Phi 
        Discreteness_TypeDef Theta_w; // 轮子差分      求解角速度
        Discreteness_TypeDef dS;      // 位移积分 
        Discreteness_TypeDef target_s;// 目标位置
        Discreteness_TypeDef target_theta; // 用于倒地自启
    }Discreteness;
    
    struct 
    {
        Luenberger_Typedef Theta;
        Luenberger_Typedef dTheta;
        Luenberger_Typedef Phi;
        Luenberger_Typedef Theta_w;
        Luenberger_Typedef dS;
    }Luenberger;

    struct 
    {
        pid_type_def F0_l_p;
        pid_type_def F0_l_s;
        PID_t F0_l_x;
        pid_type_def Yaw;
        pid_type_def Delta;             // 防劈叉
        pid_type_def Roll;              // roll 补偿
    }pid;

    struct 
    {
        uint8_t offGround;
        // uint8_t jump;
        // uint8_t stand;
        // uint16_t stand_count;
        // uint8_t step_flag;
        // uint8_t err[2];

    }status;

    struct 
    {
        float K[12];
        float T_p;
        float T_w;
        float delta;            // 劈叉角度
        float dF_0;             // 伸腿力
        float dF_roll;          // roll 补偿
        float dF_delta;         // 防劈叉
        float dF_yaw;           // 转向控制
        float F_0;              // 支持力
        float Fn;               // 轮子支持力，用于离地检测
        float torque_setT[2];
        float torque_setW;
        float torque_get_F_0;
        float torque_get_T_p;
    }LQR;
    
    struct 
    {
        float T_max;
        float W_max;
    }limit;

    struct 
    {
        float T1;
        float T2;
        float Tw;
    }torque_send;
    
    
}Leg_Typedef;

typedef enum
{
    ROBOT_DISABLE = 0,
    ROBOT_BALANCE,
    ROBOT_FALLEN,
    ROBOT_RISING,
    ROBOT_TRANSITION,
    ROBOT_STEP

} RobotMode_e;

typedef struct
{
    RobotMode_e mode;

    uint16_t fallen_count;
    uint16_t rising_count;
    uint16_t transition_count;
    uint16_t step_count;

} RobotManager_t;

extern RobotManager_t RobotManager;

extern Leg_Typedef Leg_l;
extern Leg_Typedef Leg_r;

void Vmc_Init(Leg_Typedef *object, float target_l0);
void Vmc_calcL(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
void Vmc_calcR(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
static void getPhi(vmc_Typedef *vmc, float phi1, float phi4, float l1, float l2, float l3, float l4, float l5);
static void getMatJRM(vmc_Typedef *vmc, float phi0, float phi1, float phi2, float phi3, float phi4, float L0, float l1, float l4);
static float Vmc_getFnL(Leg_Typedef *object, IMU_Data_t *imu);
static float Vmc_getFnR(Leg_Typedef *object, IMU_Data_t *imu);
// uint8_t ground_check(Leg_Typedef *leg, IMU_Data_t *imu, float *w, float b, float *mean, float *std);

float Vmc_get_Fs(float L0, vmc_Typedef *vmc);

#endif // !__VMC_H