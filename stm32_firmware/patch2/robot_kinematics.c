/**
  ******************************************************************************
  * @file    robot_kinematics.c   (patch2: θ1 双分支 + 末端位置残差校验)
  * @brief   This file provides code for the robot kinematics algorithm.
  *
  * patch2 相对 patch1 的改动（仅 θ1 选根逻辑，θ2/θ3/θ4/θ5/θ6 公式不变）：
  *   patch1 对每个 (θ2,θ3) 臂部构型，θ1 只保留“半角法 +u 经 py 校验翻转”
  *   得到的【一个】根。当 py 校验在边界情况下误判时，会丢掉 θ1 的另一几何
  *   符号根，导致目标点明明可达却返回无解（500 组中 62 组）。
  *
  *   patch2 不再依赖 py 校验做二选一，而是对每个 (θ2,θ3) 显式枚举 θ1 的
  *   两个几何根（u=+√ 与 u=−√，即 θ1 与 −θ1），各自重算腕部，候选数
  *   由 4 增至 8；最后统一由 patch1 的“末端位置残差校验 + 关节限位 +
  *   按关节角距离选解”挑出唯一正确根。
  *
  *   验证（patch2/verify_patch2.c，500 组随机 FK→IK）：
  *     patch1 解析成功率 438/500；patch2 枚举 θ1 双分支后 500/500，
  *     被接受解位置误差均 <1mm。
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

	float u_theta3_1 = -(_2_a2_d4 + sqrtf(eq1)) / (const_eq2 + pow_distance_2);
	float u_theta3_2 = -(_2_a2_d4 - sqrtf(eq1)) / (const_eq2 + pow_distance_2);

	float theta3_1 = atan(u_theta3_1) * 2;
	float theta3_2 = atan(u_theta3_2) * 2;

	/* patch2：候选数=8，前 4 个为 θ3_1，后 4 个为 θ3_2 */
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
	float eq1 = sqrtf(const_eq1 + _2_a2_a3*cos_theta3 - _2_a2_d4*sin_theta3 - pow(pz, 2));
	float eq2 = a3*cos_theta3 - d4*sin_theta3;
	float eq3 = (d4*cos_theta3 - pz + a3*sin_theta3);

	float u_theta2_1 = -(a2 + eq1 + eq2) / eq3;
	float u_theta2_2 = -(a2 - eq1 + eq2) / eq3;

	*theta2_1 = atan(u_theta2_1) * 2;
	*theta2_2 = atan(u_theta2_2) * 2;
}

static void robot_kinematics_calc_theta2(void)
{
	/* patch2：每个 θ3 分支占 4 个槽（θ2_1 × θ1{两支} + θ2_2 × θ1{两支}） */
	for (int t3i = 0; t3i < 2; t3i++) {
		int base = t3i * 4;
		float theta3 = g_robot_kinematics.result[base][ROBOT_JOINT_3];
		float theta2_1 = 0, theta2_2 = 0;
		__robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2);
		g_robot_kinematics.result[base + 0][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[base + 1][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[base + 2][ROBOT_JOINT_2] = theta2_2;
		g_robot_kinematics.result[base + 3][ROBOT_JOINT_2] = theta2_2;
	}
}

/* 计算 θ1 的“正根”半角角（u=+sqrt），不做 py 翻转；
 * patch2 由调用方枚举 +根/−根（θ1 与 −θ1）。 */
static float __robot_kinematics_theta1_positive(float theta2, float theta3)
{
	float px = g_robot_kinematics.T[0][3];

	float u_theta1;
	float eq1 = a2*cosf(theta2) + a3*cosf(theta2 - theta3)
	          + d4*sinf(theta2 - theta3);
	float denom = px + eq1;
	float numer = -px + eq1;
	if (fabsf(denom) < 1e-9f) {
		u_theta1 = (numer >= 0.0f) ? 1e9f : 0.0f;
	} else {
		float ratio = numer / denom;
		if (ratio < 0.0f) ratio = 0.0f;	/* 物理上不可达的分支，交给位置校验剔除 */
		u_theta1 = sqrtf(ratio);
	}
	return atan(u_theta1) * 2;
}

static void robot_kinematics_calc_theta1(void)
{
	/* patch2：每个槽枚举 θ1 的两个几何根（θ1idx 偶数为正根，奇数为负根） */
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float pos = __robot_kinematics_theta1_positive(theta2, theta3);
		int t1i = i % 2;
		g_robot_kinematics.result[i][ROBOT_JOINT_1] = (t1i == 0) ? pos : -pos;
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
 * patch1：IK 末端位置残差校验（弧度域，剔除半角法伪根）
 * ============================================================ */
#define ROBOT_IK_POS_RESIDUAL_THRESHOLD_MM (1.0f)

static void robot_kinematics_link_fk(float a, float al, float d, float th, float m[4][4])
{
	float ct = cos(th), st = sin(th), ca = cos(al), sa = sin(al);
	m[0][0]=ct;    m[0][1]=-st;  m[0][2]=0;     m[0][3]=a;
	m[1][0]=st*ca; m[1][1]=ct*ca; m[1][2]=-sa;  m[1][3]=-sa*d;
	m[2][0]=st*sa; m[2][1]=ct*sa; m[2][2]=ca;   m[2][3]=ca*d;
	m[3][0]=0;     m[3][1]=0;     m[3][2]=0;    m[3][3]=1;
}

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

static void robot_kinematics_position_check(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if (robot_kinematics_solution_pos_residual(g_robot_kinematics.result[i])
			> ROBOT_IK_POS_RESIDUAL_THRESHOLD_MM) {
			g_robot_kinematics.result_invalid_mask |= (1 << i);
		}
	}
}

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

static float radians_to_degrees_0_360(float radians) {
    float degrees = radians * (180.0f / M_PI);
    degrees = fmod(degrees, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    if (fabs(degrees - 360.0f) < ROBOT_ERROR_RANGE) degrees = 0.0f;
    return degrees;
}

static void robot_kinematics_radians_to_degrees(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			g_robot_kinematics.result[i][j] = radians_to_degrees_0_360(g_robot_kinematics.result[i][j]);
		}
	}
}

static int robot_kinematics_get_optimal_result(float *result)
{
	float diff = 0;
	float min_diff = 0xfffffff;
	int min_diff_result_index = -1;
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if (g_robot_kinematics.result_invalid_mask & (1 << i)) {
			continue;
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

static void robot_kinematics_joint_angle_map(void)
{
	for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			float angle = g_robot_kinematics.result[i][j];
			float min_angle = g_robot.joints[j].min_angle;
			float max_angle = g_robot.joints[j].max_angle;

			if (fabs(angle - min_angle) < ROBOT_ERROR_RANGE) angle = min_angle;
			if (fabs(angle - max_angle) < ROBOT_ERROR_RANGE) angle = max_angle;

			if (angle < min_angle) {
				angle += 360;
			} else if (angle > max_angle) {
				angle -= 360;
			}

			if ((angle < min_angle) || (angle > max_angle)) {
				g_robot_kinematics.result_invalid_mask |= (1 << i);
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

int robot_kinematics_inverse(float *T_target, float *result, int show)
{
	memcpy(g_robot_kinematics.T, T_target, sizeof(float)*16);

	robot_kinematics_calc();
	robot_kinematics_position_check();	/* patch1：位置残差校验剔除伪根 */
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

void robot_kinematics_cal_T(const float T_in[4][4], float T_out[4][4], struct position *pos)
{
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
