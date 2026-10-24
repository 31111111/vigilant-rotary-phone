/*
 * PC 端测试主程序：在普通 x86 上编译并运行固件的真实逆运动学算法。
 * 验证方式：
 *   1) 喂入固件作者标定的复位位姿 T_0_6_reset，反算关节角，应能还原复位关节角；
 *   2) 从复位位姿做相对平移，遍历多个可达目标点，验证均能求得有效解。
 * 编译：gcc -std=gnu11 -O2 main.c robot_kinematics.c -lm -o ik_test
 */
#include <stdio.h>
#include <math.h>
#include "robot.h"
#include "robot_kinematics.h"

/* 固件 robot.c 中的全局定义（数值原样搬运） */
struct robot g_robot;
const float D_H[6][4] = {
    {0,      0,       0,      M_PI/2},
    {0,      M_PI/2,  0,      M_PI/2},
    {200,    M_PI,    0,     -M_PI/2},
    {47.63f,-M_PI/2, -184.5f, 0},
    {0,      M_PI/2,  0,      M_PI/2},
    {0,      M_PI/2,  0,      0}
};
const float joint_weight[ROBOT_MAX_JOINT_NUM] = {5, 3, 3, 1, 1, 1};

/* 固件 T_0_6_reset：复位状态下末端 4x4 变换矩阵（位置单位 mm） */
static const float T_0_6_reset[4][4] = {
    {0, -1, 0, 0},
    {0,  0,-1, -47.63f},
    {1,  0, 0, 15.5f},
    {0,  0, 0, 1},
};

static void init_limits(void)
{
    /* 来自固件 g_joints_init 的各关节限位（角度） */
    float mn[ROBOT_MAX_JOINT_NUM] = {0, 90, -90, -90, 0, 0};
    float mx[ROBOT_MAX_JOINT_NUM] = {360, 180, 90, 90, 90, 360};
    for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
        g_robot.joints[j].min_angle = mn[j];
        g_robot.joints[j].max_angle = mx[j];
    }
}

static void show(const char *name, const float *a)
{
    printf("%s: ", name);
    for (int i = 0; i < 6; i++) printf("%8.2f", a[i]);
    printf("\n");
}

int main(void)
{
    init_limits();

    /* ---------- 测试 1：复位位姿反算 ---------- */
    float T[4][4];
    memcpy(T, T_0_6_reset, sizeof(T));
    float reset_angle[6] = {90, 90, -90, 0, 90, 0};
    robot_kinematics_joint_angle_update(reset_angle);

    printf("================ 测试 1：复位位姿 T_0_6_reset 反算 ================\n");
    float r[6];
    int ret = robot_kinematics_inverse((float *)T, r, 1);
    printf(">>> inverse 返回: %d（0=找到有效解）\n", ret);
    show("最优解(deg)  ", r);
    show("复位角(deg)  ", reset_angle);

    /* ---------- 测试 2：从复位位姿相对平移到多个目标点 ---------- */
    printf("\n================ 测试 2：相对平移后的可达目标点 ================\n");
    struct { float dx, dy, dz; } moves[] = {
        {0, 0, 0}, {0, 0, 30}, {0, 0, -10}, {0, 20, 0},
        {0, -20, 0}, {20, 0, 20}, {-20, 0, 10}, {0, 30, -15},
    };
    int ok = 0, total = (int)(sizeof(moves)/sizeof(moves[0]));
    for (int k = 0; k < total; k++) {
        float T2[4][4];
        struct position dp = { moves[k].dx, moves[k].dy, moves[k].dz };
        robot_kinematics_cal_T(T_0_6_reset, T2, &dp);   /* 固件自带的相对平移函数 */
        float res[6];
        int rc = robot_kinematics_inverse((float *)T2, res, 0);
        if (rc == 0) {
            ok++;
            printf("平移(%6.1f,%6.1f,%6.1f) mm -> 可解, 关节角:",
                   dp.x, dp.y, dp.z);
            for (int i = 0; i < 6; i++) printf("%7.1f", res[i]);
            printf("\n");
        } else {
            printf("平移(%6.1f,%6.1f,%6.1f) mm -> 无有效解（超出工作空间）\n",
                   dp.x, dp.y, dp.z);
        }
    }
    printf("\n可达点统计: %d/%d\n", ok, total);
    return 0;
}
