/*
 * dump_data.c —— 三版本（原版 / patch1 / patch2）共用的逐点数据导出器
 *
 * 直接 #include 对应版本的固件源码 robot_kinematics.c（不重新复刻算法），
 * 对同一组 srand(42) 的 500 个随机目标点，输出：
 *   1) 选定解误差：   selected_<ver>.csv  (i,ret,sel_err_mm)   ret:0=有解 -1=无解
 *   2) 全部候选分类： cand_<ver>.csv      (i,k,residual_mm,lim_ok,theta1_deg)
 *
 * 编译时用 -I 指定版本目录（pc_ik_test / patch / patch2），三个版本
 * calc 等内部函数同名、RESULT_NUM 宏自动取 4 或 8，故同一份驱动通用。
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "robot_kinematics.c"

struct robot g_robot;
const float D_H[6][4] = {
    {0, 0, 0, M_PI/2}, {0, M_PI/2, 0, M_PI/2}, {200, M_PI, 0, -M_PI/2},
    {47.63f, -M_PI/2, -184.5f, 0}, {0, M_PI/2, 0, M_PI/2}, {0, M_PI/2, 0, 0}};
const float joint_weight[6] = {5, 3, 3, 1, 1, 1};

static float mn[6] = {0, 90, -90, -90, 0, 0};
static float mx[6] = {360, 180, 90, 90, 90, 360};

static void mdh(float T[4][4], float a, float al, float d, float th) {
    float ct = cosf(th), st = sinf(th), ca = cosf(al), sa = sinf(al);
    float m[4][4] = {{ct, -st, 0, a}, {st*ca, ct*ca, -sa, -sa*d},
                     {st*sa, ct*sa, ca, ca*d}, {0, 0, 0, 1}};
    memcpy(T, m, sizeof(m));
}
static void mmul(float o[4][4], const float a[4][4], const float b[4][4]) {
    float t[4][4] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            double s = 0;
            for (int k = 0; k < 4; k++) s += a[i][k] * b[k][j];
            t[i][j] = (float)s;
        }
    memcpy(o, t, sizeof(t));
}
static void fk(const float q[6], float T[4][4]) {
    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    memcpy(T, I, sizeof(I));
    for (int i = 0; i < 6; i++) {
        float M[4][4], P[4][4];
        mdh(M, D_H[i][0], D_H[i][1], D_H[i][2], q[i]);
        mmul(P, T, M);
        memcpy(T, P, sizeof(P));
    }
}
static float frand(float lo, float hi) { return lo + (hi - lo) * (rand() / (float)RAND_MAX); }
static float pos_err(const float Tg[4][4], const float Tt[4][4]) {
    return sqrtf(powf(Tg[0][3] - Tt[0][3], 2) + powf(Tg[1][3] - Tt[1][3], 2)
               + powf(Tg[2][3] - Tt[2][3], 2));
}

/* 复刻固件 radians_to_degrees_0_360 + joint_angle_map 的限位判定 */
static int within_limits(const float q_rad[6]) {
    for (int j = 0; j < 6; j++) {
        float d = q_rad[j] * 180.0f / (float)M_PI;
        d = fmodf(d, 360.0f);
        if (d < 0) d += 360.0f;
        if (d < mn[j]) d += 360.0f;
        else if (d > mx[j]) d -= 360.0f;
        if (d < mn[j] || d > mx[j]) return 0;
    }
    return 1;
}

int main(int argc, char **argv) {
    const char *ver = (argc > 1) ? argv[1] : "x";
    char fn_s[128], fn_c[128];
    snprintf(fn_s, sizeof(fn_s), "selected_%s.csv", ver);
    snprintf(fn_c, sizeof(fn_c), "cand_%s.csv", ver);
    FILE *fs = fopen(fn_s, "w");
    FILE *fc = fopen(fn_c, "w");
    fprintf(fs, "i,ret,sel_err_mm\n");
    fprintf(fc, "i,k,residual_mm,lim_ok,theta1_deg\n");

    srand(42);
    for (int j = 0; j < 6; j++) {
        g_robot.joints[j].min_angle = mn[j];
        g_robot.joints[j].max_angle = mx[j];
    }
    float lo[6] = {5, 95, -85, -85, 5, 5};
    float hi[6] = {355, 175, 85, 85, 85, 355};
    int N = 500;

    for (int it = 0; it < N; it++) {
        float qd[6], qr[6];
        for (int j = 0; j < 6; j++) {
            qd[j] = frand(lo[j], hi[j]);
            qr[j] = qd[j] * (float)(M_PI / 180.0);
        }
        float Tt[4][4];
        fk(qr, Tt);

        /* 1) 真实 inverse 口径：最终选解（角度） */
        robot_kinematics_joint_angle_update(qd);
        float so[6];
        int ret = robot_kinematics_inverse((float *)Tt, so, 0);
        float sel = -1.0f;
        if (ret == 0) {
            float sr[6];
            for (int j = 0; j < 6; j++) sr[j] = so[j] * (float)(M_PI / 180.0);
            float To[4][4];
            fk(sr, To);
            sel = pos_err(To, Tt);
        }
        fprintf(fs, "%d,%d,%.5f\n", it, ret, sel);

        /* 2) 重新 calc 拿原始候选（弧度），逐组残差 + 限位 */
        memcpy(g_robot_kinematics.T, Tt, sizeof(Tt));
        robot_kinematics_calc();
        for (int k = 0; k < ROBOT_KINEMATICS_RESULT_NUM; k++) {
            float Tg[4][4];
            fk(g_robot_kinematics.result[k], Tg);
            float er = pos_err(Tg, Tt);
            int lim = within_limits(g_robot_kinematics.result[k]);
            float t1 = g_robot_kinematics.result[k][0] * 180.0f / (float)M_PI;
            t1 = fmodf(t1, 360.0f);
            if (t1 < 0) t1 += 360.0f;
            fprintf(fc, "%d,%d,%.5f,%d,%.3f\n", it, k, er, lim, t1);
        }
    }
    fclose(fs);
    fclose(fc);
    printf("dumped %s: %s, %s\n", ver, fn_s, fn_c);
    return 0;
}
