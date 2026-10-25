/**
  ******************************************************************************
  * @file    robot_kinematics.c
  * @brief   This file provides code for the robot kinematics algorithm.
  ******************************************************************************
  */

#include "robot_kinematics.h"
#include "usart.h"
#include "robot.h"

static struct robot_kinematics g_robot_kinematics = {0};
static float g_current_joint_angle[ROBOT_MAX_JOINT_NUM] = {0};

/* 机械臂各关节DH参数 */
#define a2 D_H[2][0]
#define a3 D_H[3][0]
#define d4 D_H[3][2]

/* 各关节运动学逆解公式可参考robot_kinematics_sym_v3_0.m */

static void robot_kinematics_calc_theta3(void)
{
	float px = g_robot_kinematics.T[0][3];
	float py = g_robot_kinematics.T[1][3];
	float pz = g_robot_kinematics.T[2][3];

	float _2_a2_d4 = 2 * a2 * d4;
	float _2_pow_a2_2 = 2 * pow(a2, 2);
	float _2_pow_a3_2 = 2 * pow(a3, 2);
	float _2_pow_d4_2 = 2 * pow(d4, 2);
	float const_eq1 = -pow(a2, 4) + _2_pow_a2_2 * (pow(a3, 2) + pow(d4, 2))
				- pow(a3, 4) - 2*pow(a3, 2)*pow(d4, 2) - pow(d4, 4);
	float const_eq2 = -pow(a2, 2) + 2*a2*a3 - pow(a3, 2) - pow(d4, 2);
	
	float pow_px_2 = pow(px, 2);
	float pow_py_2 = pow(py, 2);
	float pow_pz_2 = pow(pz, 2);
	float pow_distance_2 = pow_px_2 + pow_py_2 + pow_pz_2;

	float eq1 = (const_eq1 + _2_pow_a2_2*pow_distance_2 
		+ _2_pow_a3_2*pow_distance_2 + _2_pow_d4_2*pow_distance_2 
		- pow(px, 4) - pow(py, 4) - pow(pz, 4) 
		- 2*pow_px_2*(pow_py_2 + pow_pz_2) - 2*pow_py_2*pow_pz_2);

	float u_theta3_1 = -(_2_a2_d4 + sqrt(eq1)) / (const_eq2 + pow_distance_2);
	float u_theta3_2 = -(_2_a2_d4 - sqrt(eq1)) / (const_eq2 + pow_distance_2);

	float theta3_1 = atan(u_theta3_1) * 2;
	float theta3_2 = atan(u_theta3_2) * 2;

	// 前4个theta3是同解，后4个theta3是同解
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM/2; i++) {
		g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_1;
	}

	for (int i = ROBOT_KINEMATICS_RESULT_NUM/2; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_2;
	}
}

static void __robot_kinematics_calc_theta2(float theta3, float *theta2_1, float *theta2_2)
{
	float pz = g_robot_kinematics.T[2][3];

	float _pow_a2_2 = pow(a2, 2);
	float _pow_a3_2 = pow(a3, 2);
	float _pow_d4_2 = pow(d4, 2);
	float _2_a2_a3 = 2 * a2 * a3;
	float _2_a2_d4 = 2 * a2 * d4;
	
	
	float cos_theta3 = cos(theta3);
	float sin_theta3 = sin(theta3);

	float const_eq1 = _pow_a2_2 + _pow_a3_2 + _pow_d4_2;
	float eq1 = sqrt(const_eq1 + _2_a2_a3*cos_theta3 - _2_a2_d4*sin_theta3 - pow(pz, 2));
	float eq2 = a3*cos_theta3 - d4*sin_theta3;
	float eq3 = (d4*cos_theta3 - pz + a3*sin_theta3);

	float u_theta2_1 = -(a2 + eq1 + eq2) / eq3;
	float u_theta2_2 = -(a2 - eq1 + eq2) / eq3;

	*theta2_1 = atan(u_theta2_1) * 2;
	*theta2_2 = atan(u_theta2_2) * 2;
}

static void robot_kinematics_calc_theta2(void)
{
	// patch2：每个 θ3 分支占 4 个槽，θ2 两支各重复 2 槽（供 θ1 正/负两支）
	for (int t3i = 0; t3i < 2; t3i++) {
		int base = t3i * 4;
		float theta3 = g_robot_kinematics.result[base][ROBOT_JOINT_3];
		float theta2_1 = 0;
		float theta2_2 = 0;
		__robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2);
		g_robot_kinematics.result[base + 0][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[base + 1][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[base + 2][ROBOT_JOINT_2] = theta2_2;
		g_robot_kinematics.result[base + 3][ROBOT_JOINT_2] = theta2_2;
	}
}

/* patch2：返回 θ1 半角“正根”（u=+√），不再用 py 做二选一；
 * 负根（−θ1）由 robot_kinematics_calc_theta1 显式枚举，两支最后统一
 * 由末端位置残差校验裁决。 */
static float __robot_kinematics_calc_theta1(float theta2, float theta3)
{
	float px = g_robot_kinematics.T[0][3];

	float eq1 = a2*cos(theta2) + a3*cos(theta2 - theta3) + d4*sin(theta2 - theta3);
	float denom = px + eq1;
	float numer = -px + eq1;
	float u_theta1;
	if (fabs(denom) < 1e-9f) {
		u_theta1 = (numer >= 0.0f) ? 1e9f : 0.0f;
	} else {
		float ratio = numer / denom;
		if (ratio < 0.0f) ratio = 0.0f;	// 物理上不可达的分支，交给位置校验剔除
		u_theta1 = sqrt(ratio);
	}
	float theta1 = atan(u_theta1) * 2;
	return theta1;
}

static void robot_kinematics_calc_theta1(void)
{
	// patch2：每个 (θ2,θ3) 槽枚举 θ1 正/负两个几何根（偶数 index 正根、奇数负根）
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1_pos = __robot_kinematics_calc_theta1(theta2, theta3);
		g_robot_kinematics.result[i][ROBOT_JOINT_1] = (i % 2 == 0) ? theta1_pos : -theta1_pos;
	}
}

static float __robot_kinematics_calc_theta5(float theta1, float theta2, float theta3)
{
	float nx = g_robot_kinematics.T[0][0];
	float ny = g_robot_kinematics.T[1][0];
	float nz = g_robot_kinematics.T[2][0];

	float ox = g_robot_kinematics.T[0][1];
	float oy = g_robot_kinematics.T[1][1];
	float oz = g_robot_kinematics.T[2][1];

	float ax = g_robot_kinematics.T[0][2];
	float ay = g_robot_kinematics.T[1][2];
	float az = g_robot_kinematics.T[2][2];

	float cos_theta1 = cos(theta1);
	float sin_theta1 = sin(theta1);
	float cos_theta2 = cos(theta2);
	float sin_theta2 = sin(theta2);
	float cos_theta3 = cos(theta3);
	float sin_theta3 = sin(theta3);

	float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3 - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3 - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;
	float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3 - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3 - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
    float r33 = ax*cos_theta1*cos_theta3*sin_theta2 - az*sin_theta2*sin_theta3 - ax*cos_theta1*cos_theta2*sin_theta3 - az*cos_theta2*cos_theta3 - ay*cos_theta2*sin_theta1*sin_theta3 + ay*cos_theta3*sin_theta1*sin_theta2;
	float theta5_zyz = atan2(sqrt(pow(r31, 2) + pow(r32, 2)), r33);
	float theta5 = -theta5_zyz + M_PI;
	return theta5;
}

static void robot_kinematics_calc_theta5(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		g_robot_kinematics.result[i][ROBOT_JOINT_5] = __robot_kinematics_calc_theta5(theta1, theta2, theta3);
	}
}

static float __robot_kinematics_calc_theta4(float theta1, float theta2, float theta3, float theta5)
{
	float theta5_zyz = M_PI - theta5;
	if ((fabs(theta5_zyz) < ROBOT_ERROR_RANGE) || (fabs(theta5_zyz - M_PI) < ROBOT_ERROR_RANGE)) {
		return 0;
	}
	
	float ax = g_robot_kinematics.T[0][2];
	float ay = g_robot_kinematics.T[1][2];
	float az = g_robot_kinematics.T[2][2];

	float theta4 = 0; // 用于坐标系对齐
	float cos_theta1 = cos(theta1);
	float sin_theta1 = sin(theta1);
	float cos_theta2 = cos(theta2);
	float sin_theta2 = sin(theta2);
	float cos_theta3 = cos(theta3);
	float sin_theta3 = sin(theta3);
	float cos_theta4 = cos(theta4);
	float sin_theta4 = sin(theta4);

	float r23 = ax*cos_theta4*sin_theta1 - ay*cos_theta1*cos_theta4 + az*cos_theta2*sin_theta3*sin_theta4 
	- az*cos_theta3*sin_theta2*sin_theta4 - ax*cos_theta1*cos_theta2*cos_theta3*sin_theta4 - ay*cos_theta2*cos_theta3*sin_theta1*sin_theta4 
	- ax*cos_theta1*sin_theta2*sin_theta3*sin_theta4 - ay*sin_theta1*sin_theta2*sin_theta3*sin_theta4;
	
	float r13 = ax*sin_theta1*sin_theta4 - ay*cos_theta1*sin_theta4 
	- az*cos_theta2*cos_theta4*sin_theta3 + az*cos_theta3*cos_theta4*sin_theta2 
	+ ax*cos_theta1*cos_theta2*cos_theta3*cos_theta4 + ay*cos_theta2*cos_theta3*cos_theta4*sin_theta1 
	+ ax*cos_theta1*cos_theta4*sin_theta2*sin_theta3 + ay*cos_theta4*sin_theta1*sin_theta2*sin_theta3;

	theta4 = atan2(r23, r13);
	return theta4;
}

static void robot_kinematics_calc_theta4(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		float theta5 = g_robot_kinematics.result[i][ROBOT_JOINT_5];
		g_robot_kinematics.result[i][ROBOT_JOINT_4] = __robot_kinematics_calc_theta4(theta1, theta2, theta3, theta5);
	}
}

static float __robot_kinematics_calc_theta6(float theta1, float theta2, float theta3, float theta4, float theta5)
{
	float theta5_zyz = M_PI - theta5;
	
	float nx = g_robot_kinematics.T[0][0];
	float ny = g_robot_kinematics.T[1][0];
	float nz = g_robot_kinematics.T[2][0];

	float ox = g_robot_kinematics.T[0][1];
	float oy = g_robot_kinematics.T[1][1];
	float oz = g_robot_kinematics.T[2][1];

	float cos_theta1 = cos(theta1);
	float sin_theta1 = sin(theta1);
	float cos_theta2 = cos(theta2);
	float sin_theta2 = sin(theta2);
	float cos_theta3 = cos(theta3);
	float sin_theta3 = sin(theta3);
	float theta6_zyz = 0;
	float theta6 = 0;
	if ((fabs(theta5_zyz) < ROBOT_ERROR_RANGE) || (fabs(theta5_zyz - M_PI) < ROBOT_ERROR_RANGE)) {
		float r12 = -oz*cos_theta2*sin_theta3 + oz*cos_theta3*sin_theta2 + ox*cos_theta1*cos_theta2*cos_theta3+ oy*cos_theta2*cos_theta3*sin_theta1 + ox*cos_theta1*sin_theta2*sin_theta3 + oy*sin_theta1*sin_theta2*sin_theta3;
		float r11 = -nz*cos_theta2*sin_theta3 + nz*cos_theta3*sin_theta2 + nx*cos_theta1*cos_theta2*cos_theta3+ ny*cos_theta2*cos_theta3*sin_theta1 + nx*cos_theta1*sin_theta2*sin_theta3 + ny*sin_theta1*sin_theta2*sin_theta3;
		theta6_zyz = atan2(-r12, r11);
		theta6 = theta6_zyz - M_PI;
		return theta6;
	}

	float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3 - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3 - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
	float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3 - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3 - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;
	theta6_zyz = atan2(r32, -r31);
	theta6 = theta6_zyz - M_PI;
	return theta6;
}

static void robot_kinematics_calc_theta6(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta4 = g_robot_kinematics.result[i][ROBOT_JOINT_4];
		float theta5 = g_robot_kinematics.result[i][ROBOT_JOINT_5];
		g_robot_kinematics.result[i][ROBOT_JOINT_6] = __robot_kinematics_calc_theta6(theta1, theta2, theta3, theta4, theta5);
	}
}

/* ============================================================
 * IK 选解缺陷修复（末端位置残差校验）
 *
 * 问题：解析逆解由半角法 tan(θ/2) 平方引入额外伪根，原选解只按
 *       关节角加权距离挑选、不校验末端位置，约 9% 情况会选中伪根
 *       （末端位置误差可达数百 mm）。
 * 验证：见 stm32_firmware/pc_ik_test/verify_fix.c —— 增加位置残差
 *       校验后，500 组随机点位置误差 <1mm 的比例从 91.1% 提升到 100%。
 * 修复：逆解计算完成后（result 仍为弧度），用 MDH 正运动学反推每组
 *       解的末端位置并与目标平移比较，残差超过阈值即判为伪根、标记
 *       无效。不改动原解析算法，仅在选解前多一道几何校验。
 * ============================================================ */
#define ROBOT_IK_POS_RESIDUAL_THRESHOLD_MM (1.0f)

/* MDH 单连杆变换：A = Rx(α)·Rz(θ)·T([a,0,d])，θ 为关节角（弧度） */
static void robot_kinematics_link_fk(float a, float al, float d, float th, float m[4][4])
{
	float ct = cos(th), st = sin(th), ca = cos(al), sa = sin(al);
	m[0][0]=ct;    m[0][1]=-st;  m[0][2]=0;     m[0][3]=a;
	m[1][0]=st*ca; m[1][1]=ct*ca; m[1][2]=-sa;  m[1][3]=-sa*d;
	m[2][0]=st*sa; m[2][1]=ct*sa; m[2][2]=ca;   m[2][3]=ca*d;
	m[3][0]=0;     m[3][1]=0;     m[3][2]=0;    m[3][3]=1;
}

/* 用一组关节角（弧度）反推末端位置，返回与目标平移的距离残差（mm） */
static float robot_kinematics_solution_pos_residual(const float q_rad[ROBOT_MAX_JOINT_NUM])
{
	float T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		float A[4][4], P[4][4];
		robot_kinematics_link_fk(D_H[i][0], D_H[i][1], D_H[i][2], q_rad[i], A);
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				float s = 0;
				for (int k = 0; k < 4; k++) s += T[r][k] * A[k][c];
				P[r][c] = s;
			}
		}
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) T[r][c] = P[r][c];
		}
	}
	float ex = T[0][3] - g_robot_kinematics.T[0][3];
	float ey = T[1][3] - g_robot_kinematics.T[1][3];
	float ez = T[2][3] - g_robot_kinematics.T[2][3];
	return sqrt(ex*ex + ey*ey + ez*ez);
}

/* 位置残差校验：将伪根对应的解标记为无效（须在 radians_to_degrees 之前调用） */
static void robot_kinematics_position_check(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if (robot_kinematics_solution_pos_residual(g_robot_kinematics.result[i])
			> ROBOT_IK_POS_RESIDUAL_THRESHOLD_MM) {
			g_robot_kinematics.result_invalid_mask |= (1 << i);	/* 伪根 */
		}
	}
}

/**
 * @brief 计算机械臂各关节的角度。
 * 
 * 该函数按照关节求解的依赖顺序依次计算每个关节的角度，
 * 最后将结果无效掩码清零，表示初始状态下所有结果均有效。
 */
static void robot_kinematics_calc(void)
{
	robot_kinematics_calc_theta3();
	robot_kinematics_calc_theta2();
	robot_kinematics_calc_theta1();
	robot_kinematics_calc_theta5();
	robot_kinematics_calc_theta4();
	robot_kinematics_calc_theta6();
	g_robot_kinematics.result_invalid_mask = 0;
}

void robot_kinematics_show_result(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		LOG("result[%d]: ", i);
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", g_robot_kinematics.result[i][j]);
		}
		int valid = g_robot_kinematics.result_invalid_mask & (1 << i) ? 0 : 1;
		LOG(" valid:%d\n", valid);
	}
}

// 弧度转 0 - 360 度范围内的角度
static float radians_to_degrees_0_360(float radians) {
    // 先将弧度转换为角度
    float degrees = radians * (180.0f / M_PI);
    // 确保角度在 0 - 360 度范围内
    degrees = fmod(degrees, 360.0f);
    if (degrees < 0) {
        degrees += 360.0f;
    }
	
	// 处理接近 360 度的情况，将 360 度转换为 0 度
    if (fabs(degrees - 360.0f) < ROBOT_ERROR_RANGE) {
        degrees = 0.0f;
    }
    return degrees;
}

/**
 * @brief 将机械臂运动学计算结果中的关节角度从弧度制转换为角度制，并将角度映射到 0 - 360 度范围。
 * 
 * 该函数遍历机械臂运动学计算得到的所有解，针对每个解中的每个关节角度，
 * 调用 `radians_to_degrees_0_360` 函数将其从弧度制转换为 0 - 360 度范围内的角度。
 */
static void robot_kinematics_radians_to_degrees(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			g_robot_kinematics.result[i][j] = radians_to_degrees_0_360(g_robot_kinematics.result[i][j]);	
		}
	}
}

/**
 * @brief 从机械臂运动学计算结果中获取最优解。
 * 
 * 该函数会遍历所有有效的运动学计算结果，计算每个结果与当前关节位置的加权差值，
 * 选择差值最小的结果作为最优解，并将其关节角度存储到传入的结果数组中。
 * 
 * @param result 指向用于存储最优解关节角度的数组的指针。
 * @return int 若成功找到最优解返回 0，若没有有效的解则返回 -1。
 */
static int robot_kinematics_get_optimal_result(float *result)
{
	// 选择最接近当前关节位置的解
	float diff = 0;
	float min_diff = 0xfffffff;
	int min_diff_result_index = -1;
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if (g_robot_kinematics.result_invalid_mask & (1 << i)) {
			continue;	// 跳过无效解
		}
		diff = 0;
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			diff += fabs(g_robot_kinematics.result[i][j] - g_current_joint_angle[j]) * joint_weight[j];
		}
		if (diff < min_diff) {
			min_diff = diff;
			min_diff_result_index = i;	
		}
	}

	if (min_diff_result_index == -1) {
		return -1;
	}

	for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
		result[j] = g_robot_kinematics.result[min_diff_result_index][j];	
	}

	return 0;
}

/**
 * @brief 将机械臂运动学计算结果中的关节角度映射到关节限制范围内，并标记无效解。
 * 
 * 该函数遍历机械臂运动学计算得到的所有解，针对每个解中的每个关节角度，
 * 尝试将其映射到该关节允许的最小和最大角度范围内。若无法映射到有效范围，
 * 则将该解标记为无效。
 */
static void robot_kinematics_joint_angle_map(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			float angle = g_robot_kinematics.result[i][j];
			float min_angle = g_robot.joints[j].min_angle;
			float max_angle = g_robot.joints[j].max_angle;

			// 临界区做整数处理，防止浮点误差导致角度超出限制
			if (fabs(angle - min_angle) < ROBOT_ERROR_RANGE) {
				angle = min_angle;		
			}

			if (fabs(angle - max_angle) < ROBOT_ERROR_RANGE) {
				angle = max_angle;	
			}

			// 尝试映射到关节限制范围内
			if (angle < min_angle) {
				angle += 360;
			} else if (angle > max_angle) {
				angle -= 360;
			}
			
			// 检查映射后的角度是否在限制范围内
			if ((angle < min_angle) || (angle > max_angle)) {
				g_robot_kinematics.result_invalid_mask |= (1 << i);	// 标记解为无效
			}
			g_robot_kinematics.result[i][j] = angle;
		}	
	}	
}

void robot_kinematics_joint_angle_update(float *joint_angle)
{
	memcpy(g_current_joint_angle, joint_angle, sizeof(float)*ROBOT_MAX_JOINT_NUM);
}

void robot_kinematics_joint_angle_update_by_id(uint32_t joint_id ,float angle)
{
	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
		LOG("robot kinematics joint id is invalid\n");
		return;	
	}
	g_current_joint_angle[joint_id] = angle;
}

/**
 * @brief 计算机械臂运动学逆解，获取达到目标位姿所需的关节角度。
 * 
 * 该函数接收目标位姿的变换矩阵，计算机械臂各关节的角度，将结果转换为角度制，
 * 映射到关节限制范围内，最后选择最优解返回。同时支持根据参数控制输出调试信息。
 * 
 * @param T_target 指向目标位姿变换矩阵的指针，矩阵按行优先存储。
 * @param result 指向用于存储最优关节角度结果的数组的指针。
 * @param show 控制是否显示调试信息的标志，非零值表示显示，零值表示不显示。
 * @return int 若成功找到最优解返回 0，否则返回 -1。
 */
int robot_kinematics_inverse(float *T_target, float *result, int show)
{
	memcpy(g_robot_kinematics.T, T_target, sizeof(float)*16);

	robot_kinematics_calc();
	robot_kinematics_position_check();	/* 修复：末端位置残差校验，剔除半角法伪根 */
	if (show) {
		LOG("target:[%f %f %f]\n", T_target[4 * 0 + 3], T_target[4 * 1 + 3], T_target[4 * 2 + 3]);
		LOG("robot kinematics result(rad):\n");
		robot_kinematics_show_result();
	}

	robot_kinematics_radians_to_degrees();

	robot_kinematics_joint_angle_map();
	if (show) {
		LOG("robot kinematics result(deg):\n");
		robot_kinematics_show_result();
	}

	int ret = robot_kinematics_get_optimal_result(result);
	if (show) {
		LOG("robot optimal result(rad):\n");
		for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
			LOG("%.2f ", result[i]);	
		}
		LOG("\n");
	}
	return ret;
}

/**
 * @brief 根据输入的变换矩阵和相对位置，计算新的变换矩阵。
 * 
 * 该函数将输入的变换矩阵复制到输出矩阵，然后根据传入的相对位置信息
 * 对输出矩阵的平移部分（即矩阵的第 4 列的前 3 个元素）进行更新。
 * 
 * @param T_in 输入的 4x4 变换矩阵，按行优先存储。
 * @param T_out 输出的 4x4 变换矩阵，用于存储计算后的结果，按行优先存储。
 * @param pos 指向 `position` 结构体的指针，包含相对位置的 x、y、z 坐标。
 */
void robot_kinematics_cal_T(const float T_in[4][4], float T_out[4][4], struct position *pos)
{
	// 根据末端坐标的相对运动，获得末端的T矩阵(当前仅支持x,y,z的相对运动, 后续支持旋转)
	memcpy(T_out, T_in, sizeof(float)*16);
	T_out[0][3] = pos->x + T_out[0][3];
	T_out[1][3] = pos->y + T_out[1][3];
	T_out[2][3] = pos->z + T_out[2][3];
}

void robot_kinematics_show_T(float T[4][4])
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			LOG("%.2f ", T[i][j]);
		}
		LOG("\n");
	}
}
