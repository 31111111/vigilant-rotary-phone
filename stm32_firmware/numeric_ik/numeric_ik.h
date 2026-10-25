/*
 * numeric_ik.h —— 6 自由度机械臂数值逆运动学（DLS 阻尼最小二乘）
 *
 * 用途：当解析逆解（半角法）的 4 个解全部因伪根 / 关节越限而被拒绝时
 *       （约 9% 工作空间点，见 patch1 的位置残差校验），用数值 IK 兜底，
 *       从一个或多个种子姿态出发迭代到满足“位置+姿态”的关节解。
 *
 * 运动学模型与固件 patch1 的 MDH 正运动学完全一致：
 *       A_i = Rx(alpha_i) * Rz(theta_i) * T(a_i, 0, d_i), theta_i = q_i
 *
 * 单位：内部全部使用弧度、毫米；输出由调用方换算成固件角度(0..360)。
 */
#ifndef __NUMERIC_IK_H__
#define __NUMERIC_IK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NIK_JOINT_NUM 6

/* 关节限位（度），与固件 g_joints_init 一致：
 * J1[0,360] J2[90,180] J3[-90,90] J4[-90,90] J5[0,90] J6[0,360] */
extern const float NIK_LIMIT_MIN_DEG[NIK_JOINT_NUM];
extern const float NIK_LIMIT_MAX_DEG[NIK_JOINT_NUM];

/* 与固件一致的 MDH 参数（a, alpha, d, theta_offset），单位 mm/rad */
extern const float NIK_DH[NIK_JOINT_NUM][4];

/* 正运动学：q_rad 为 6 个关节角（弧度），输出 4x4 齐次变换（行主序，mm） */
void nik_fk(const float q_rad[NIK_JOINT_NUM], float T[4][4]);

/* 数值 IK（单种子）。
 *   T_target : 目标 4x4 位姿
 *   q_seed   : 种子关节角（弧度）
 *   q_out    : 输出关节角（弧度），长度 6
 *   pos_err_mm: 输出末端位置误差（mm），可传 NULL
 *   rot_err_deg: 输出末端姿态误差（度），可传 NULL
 * 返回 0 = 收敛（位置<1mm 且 姿态<1 度），否则未收敛。
 */
int nik_solve(const float T_target[4][4],
              const float q_seed[NIK_JOINT_NUM],
              float q_out[NIK_JOINT_NUM],
              float *pos_err_mm, float *rot_err_deg);

/* 把弧度关节角映射为固件角度（0..360 的连续轴，或限位窗口），
 * 并检查是否落在关节限位内。返回 0 表示全部在限位内。 */
int nik_to_degrees_in_limits(const float q_rad[NIK_JOINT_NUM],
                             float q_deg[NIK_JOINT_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* __NUMERIC_IK_H__ */
