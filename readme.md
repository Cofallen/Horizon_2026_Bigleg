# 串联腿2026赛季总代码

> 因诸多原因，本代码最终未能上场，是我的一大遗憾。主题底盘采用LQR控制，针对异常情况采用状态机检测，实测多种异常情况均可实现，包括倒立起立，侧身起立等。但本框架在原框架基础上修改而成，建议提取关键算法重写。
> 

## 底盘篇

位置： `Rm_Code`

- `./lqr.py`：由我matlab文件转化而成，使用数值求解加速了求解速度。目前论坛港大开源和南航金城开源均考虑了质心偏移，但我认为质心偏移不是导致抖动的关键因素。

```python
 # 2. 物理参数 (直接定义为数值，不再用符号)
R_val = 0.06
L_val = leg_length / 2.0
L_M_val = leg_length / 2.0
Body_val = 0.574
l_val = 0.028
m_w_val = 0.572
m_p_val = 0.9810
M_val = 12.0 / 2.0
I_w_val = 0.5 * m_w_val * R_val**2
# I_p_val = m_p_val * ((leg_length)**2 + 0.12**2) / 12.0
# I_M_val = M_val * (0.24**2 + 0.22**2) / 12.0
I_p_val = m_p_val * leg_length**2 / 12.0 + m_p_val * 0.0084306**2
I_M_val = M_val * Body_val**2 / 12.0 + Body_val * l_val**2 
g_val = 9.81
```
leg_length： 实际腿长
R_val： 轮半径
L_val/L_M_val：摆杆重心到驱动轮轴距离 /摆杆重心到机体转轴距离
Body_val： 车简化长方体长度
l_val： 机体重心到其转轴距离
m_w_val： 轮子重量
m_p_val： 单边杆总重量
M_val： 车体重量

调节应调节下面，具体方法可见子文件夹/Webots/小腿心得
```python
# 8. LQR 求解
Q = np.diag([20000, 100, 500, 1, 80000, 500])
R_mat = np.diag([200, 1])  
```
其中参数位于 `User/Leg/get_K.c`。

**改进**： 针对不同工况，采取不同QR矩阵的效果。具体可见[南航金城开源]([[RM2026 偏置并联腿(整车)电控开源\] 南京航空航天大学金城学院-Born Of Fire战队-RoboMaster 社区](https://bbs.robomaster.com/article/1883510?source=1))。

拟合：目前使用的是三次多项式拟合，具体方法见相关代码。
```c
void Chassis_Fit_K(float coeffs[][4], float leg_length, float *LQR_K)
{
    // 三次多项式，后可考虑exp
    for (int i = 0; i < 12; i++)
    {
        LQR_K[i] =  coeffs[i][0] * leg_length * leg_length * leg_length +
                    coeffs[i][1] * leg_length * leg_length +
                    coeffs[i][2] * leg_length +
                    coeffs[i][3];
    }
}
```

- `./User/Leg/StateSelect.c` ： 状态机检测。

```c
typedef enum
{
    ROBOT_DISABLE = 0,
    ROBOT_BALANCE,
    ROBOT_FALLEN,
    ROBOT_RISING,
    ROBOT_TRANSITION,
    ROBOT_STEP

} RobotMode_e;
```
分别对应： 离线模式，平衡模式，倒地模式，转腿模式，偏倒模式，上台阶模式。 未写完：上台阶和离地模式。 检测方法很简单，观测 pitch/ roll/ 腿长即可。

下面还有每个状态的限幅（限幅很关键，为了好调节我把它单独分割出来，它会决定状态连续的润滑效果），以及最后极性。我的极性是假设所有关节电机逆时针为正，标定零点是水平。

离地检测：有必要单独说明。位于子文件下`vmc.c/ground_check`函数。虽然已经注释了，但是还要用。不加气弹簧的情况下，选取该12给参数没有问题，但是加了气弹簧会有一定误判，因此我认为应该加入气弹簧有关系数重新训练，效果应该会好。训练代码位于[这里](https://github.com/Cofallen/On-off-ground-check.git)。

**改进**：完善离地和上台阶。我的离地检测还是好用的。

- `./User/Leg/vmc.c`： vmc正逆解算以及支持力解算。

正逆解算我分成了左右两个函数，因为符号不同。但是受限于噪声，支持力解算在仿真里表现较好，但是实际上静态就是一坨，因此我离地检测方案未采用该方案，具体见上文。

气弹簧解算：采用简单的两个余弦定理和虚功原理即可实现。实际效果较好。注意验证补偿效果的时候应注释掉正常的力输出，仅考虑腿部重力和气弹簧补偿力即可。

- `./User/Leg/legRotate.c`：倒地自启旋转Pid使用。

该方法是一个预定轨迹，用于补偿pid输出过大，然后存在一定限位，保证不会倒立转动。实测效果较好，只要参数合理，能实现无抖动旋转。

`LegRotate_Control` 函数中包含了最后两个函数 `RollRecovery_Control` 和 `PitchRecovery_Control`，是因为这样写简单一点。建议应该分成状态机写，因为这两个是侧躺和倒立的状态。不过这样写也是可以的，不建议就是了。

- `./User/Leg/RecoveryControl.c`： 倒立和侧躺力矩补偿。

因为要使机体不是水平的情况下干到水平，需要一边或者两边提供一个大力矩。因此`RollRecovery_Control` 这个就是侧躺补偿，因为roll轴偏移，我就对对应的压着的腿给一个大力矩保证他能正回来。`PitchRecovery_Control` 这个就是倒立补偿，当你倒立到对应位置后，我就给个超大力矩，让你翻身180度再考虑正常控制。因为这两个都是小概率事件，因此我没有写成状态机，而是在翻腿的时候判断一下，当特殊状态处理了。写成状态机是好的。

**改进**： 逻辑重构一下，目前还是有一点重复。

- `./User/Leg/powerControl.c` ：功率控制。

简单采用缩放规则，受限于最后没有整车，我只是在单电机上测试了一下，效果较好。同样港科有功率开源，他是把功率超额后目标值缩放，目前来看我认为没什么用，这样足够了，你踩弹丸离地检测还直接削力矩呢，绕一圈有啥用。 这个需要测试。

**改进**：将约束写成mpc上位机归控，仍是没啥用。说白了除了装浪费资源我是想不到啥，简单不好吗。尤其存在二次项，更难办。

- `./User/Leg/observe.c`： 卡尔曼位移滤波，用于打滑检测。

实测纯靠这玩意还是不行，它能够起到50%作用吧，仍然看南航金城开源即可，他的方法与我类似，单独计打滑次数，达到了后削弱轮电机力矩输出。我好像原来重构代码删了。这个方法测试较好，小陀螺旋转200个弹丸没有问题。

- `./User/Leg/board2board.c`：板间通讯。
- `./User/Leg/KNN.c`：原来搞随机森林用于离地检测的，现在已经废弃，不如神经网络。

### 总结

以上是整体小代码。我把其中的功能都封装成了一个函数，你可以换一下，比如接收一个任务，解算发送一个任务。目前改代码最大的问题就是会**抖动**。这个我前前后后研究许久，尝试很多方案，频率提高到1khz仍然无效，但是仿真从来没有这个问题，在仿真里面尝试诸多补偿，例如MRAC也宣告失败（因为LQR自稳性，你添加补偿就是相当于争夺LQR力矩输出）。

希望你们能解决这个问题，然后写在这里。下附总控制调用。

```c
//云台
void StartGimbalTask(void const *argument)
{
    Power_control_init(&model);
    // BM_EnableDisable(&hfdcan1, 0x01);
    // BM_set_ID(&hfdcan1, 2, 1);
    osDelay(100);
    BM_EnableDisable(&hfdcan2, 0x02);
    // BM_save_flash(&hfdcan1);
    ChassisL_Init(&ALL_MOTOR, &Leg_l);
    ChassisR_Init(&ALL_MOTOR, &Leg_r);
    Vmc_Init(&Leg_l, MIN_LEG_LENGTH);
    Vmc_Init(&Leg_r, MIN_LEG_LENGTH);
    LegRotate_Init();
    Recovery_Init();
    while (IMU_Data.pitch == 0.0f)
    {
        osDelay(1);
    }
    xvEstimateKF_Init(&vaEstimateKF, 0.001f);
    for(;;)
    {
        RUI_V_CONTAL.DWT_TIME.Move_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.Move_DWT_Count);
        Vmc_calcL(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Vmc_calcR(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisL_UpdateState(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisR_UpdateState(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Chassis_UpdateStateS(&Leg_l, &Leg_r, &ALL_MOTOR, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        
        Robot_UpdateMode(&Leg_l,
                     &Leg_r,
                     &WHW_V_DBUS);

        Robot_Control(&ALL_MOTOR,
                    &Leg_l,
                    &Leg_r,
                    &WHW_V_DBUS,
                    RUI_V_CONTAL.DWT_TIME.Move_Dtime);

        Robot_LimitOutput(&Leg_l,
                        &Leg_r);

        Robot_SendTorque(&Leg_l,
                        &Leg_r);

        chassis_power_control_2wheel(&ALL_MOTOR, &Leg_l, &Leg_r, &model, 60.0f, model.rpm_to_rad);

        osDelay(1);
    }
}

```


## 云台篇

云台代码简单很多，但是当时懒得改文件名了，因此只是给出文件名即可。

- `Horizon_2026_Leg_gimbal/Horizon_Infantry/User/Leg/get_K.c`： 板件通信解算。
- `Horizon_2026_Leg_gimbal/Horizon_Infantry/User/App/Gimbal_Task.c`： 云台控制。

因为云台在腿部异常状态下应不动，所以应有一个异常后按照编码器停在对应位置。

- `Horizon_2026_Leg_gimbal/Horizon_Infantry/User/App/Heat_Task.c`：热量控制。