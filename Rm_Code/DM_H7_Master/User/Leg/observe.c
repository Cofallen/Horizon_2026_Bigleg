#include "kalman_filter.h"
#include "observe.h"
#include "All_Init.h"
#include "chassisL.h"
#include "vmc.h"

KalmanFilter_t vaEstimateKF;

float KF_F[4] = {1.0f, 0.001f, 0.0f, 1.0f};            // dt应取实际值
float KF_P[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float KF_Q[4] = {0.1f, 0.0f, 0.0f, 1.0f};
// float KF_R[4] = {20000.0f, 0.0f, 0.0f, 200000.0f};      // 噪声
float KF_R[4] = {8000.0f, 0.0f, 0.0f, 2000000.0f};      // 噪声
float KF_K[4] = {1.0f, 0.0f, 0.0f, 1.0f};
float KF_H[4] = {1.0f, 0.0f, 0.0f, 1.0f};

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF, float dt)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);
    memcpy(EstimateKF->F_data, KF_F, sizeof(KF_F));
    memcpy(EstimateKF->P_data, KF_P, sizeof(KF_P));
    memcpy(EstimateKF->Q_data, KF_Q, sizeof(KF_Q));
    memcpy(EstimateKF->R_data, KF_R, sizeof(KF_R));
    memcpy(EstimateKF->H_data, KF_H, sizeof(KF_H));
}

// 反馈滤波后速度
float xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float vel, float acc)
{
    EstimateKF->MeasuredVector[0] = vel;
    EstimateKF->MeasuredVector[1] = acc;

    Kalman_Filter_Update(EstimateKF);
    return EstimateKF->FilteredValue[0];
}

float slip_Check(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r)
{
        // 新的打滑检测与恢复逻辑（去抖 + 确认 + 恢复）
    // 常量：短去抖、打滑确认（你想要的300）和恢复需要的连续稳定次数
    const int SLIP_DETECT_COUNT   = 0;    // 短去抖，避免开机瞬态立即进入打滑
    const int SLIP_CONFIRM_COUNT  = 200;  // 你需要的“计数300次”判定为持续打滑
    const int STABLE_CONFIRM_COUNT= 30;   // 打滑确认后需要连续稳定次数才能恢复

    float diff_l = fabsf(Leg_l->stateSpace.dot_s - Leg_l->stateSpace.raw_dot_s);
    float diff_r = fabsf(Leg_r->stateSpace.dot_s - Leg_r->stateSpace.raw_dot_s);

    static int kl_slip_count = 0, kr_slip_count = 0;
    static int kl_stable_count = 0, kr_stable_count = 0;

    // 左腿逻辑
    if (diff_l > 0.8f) {
        kl_stable_count = 0;              // 出现打滑则清稳定计数
        kl_slip_count++;                  // 增加打滑计数
        if (kl_slip_count >= SLIP_DETECT_COUNT) {
            kl = 0.3f;                    // 去抖后进入打滑降权
        }
        if (kl_slip_count > SLIP_CONFIRM_COUNT) {
            // 已达到持续打滑确认，保持降权直到检测到恢复连续稳定
        }
    } else {
        // 当前无打滑
        if (kl_slip_count > SLIP_CONFIRM_COUNT) {
            // 之前为持续打滑，开始计数稳定
            kl_stable_count++;
            if (kl_stable_count >= STABLE_CONFIRM_COUNT) {
                kl = 1.0f;                // 恢复为正常权重
                kl_slip_count = 0;
                kl_stable_count = 0;
            }
        } else {
            // 未达到持续打滑阈值，视为瞬态，直接复位
            kl_slip_count = 0;
            kl_stable_count = 0;
            kl = 1.0f;
        }
    }

    // 右腿逻辑（同上）
    if (diff_r > 0.8f) {
        kr_stable_count = 0;
        kr_slip_count++;
        if (kr_slip_count >= SLIP_DETECT_COUNT) {
            kr = 0.3f;
        }
    } else {
        if (kr_slip_count > SLIP_CONFIRM_COUNT) {
            kr_stable_count++;
            if (kr_stable_count >= STABLE_CONFIRM_COUNT) {
                kr = 1.0f;
                kr_slip_count = 0;
                kr_stable_count = 0;
            }
        } else {
            kr_slip_count = 0;
            kr_stable_count = 0;
            kr = 1.0f;
        }
    }
}