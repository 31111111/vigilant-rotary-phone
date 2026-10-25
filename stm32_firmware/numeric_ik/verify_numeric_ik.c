/*
 * verify_numeric_ik.c —— 数值 IK 兜底验证（500 组随机 FK→IK）
 *
 * 口径与 patch/verify_patched.c 完全一致（srand 42，限位内收窄采样）：
 *   1) 先跑 patch1 修复后的固件解析 IK，统计成功/失败；
 *   2) 对解析失败的点（4 个解析解全部被伪根/越限拒绝），用数值 IK
 *      从“解析 4 个解 + 限位内固定种子”出发兜底；
 *   3) 统计兜底成功率、位置/姿态误差，以及总体成功率；
 *   4) 另外验证：数值 IK 从真解种子出发能精确还原（算法自检）。
 *
 * 编译：见本目录 Makefile
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "robot_kinematics.c"
#include "numeric_ik.h"

struct robot g_robot;
const float D_H[6][4]={
 {0,0,0,M_PI/2},{0,M_PI/2,0,M_PI/2},{200,M_PI,0,-M_PI/2},
 {47.63f,-M_PI/2,-184.5f,0},{0,M_PI/2,0,M_PI/2},{0,M_PI/2,0,0}};
const float joint_weight[6]={5,3,3,1,1,1};

static void mdh(float T[4][4],float a,float al,float d,float th){
 float ct=cosf(th),st=sinf(th),ca=cosf(al),sa=sinf(al);
 float m[4][4]={{ct,-st,0,a},{st*ca,ct*ca,-sa,-sa*d},{st*sa,ct*sa,ca,ca*d},{0,0,0,1}};
 memcpy(T,m,sizeof(m));}
static void mmul(float o[4][4],const float a[4][4],const float b[4][4]){
 float t[4][4]={0};for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=a[i][k]*b[k][j];t[i][j]=(float)s;}
 memcpy(o,t,sizeof(float[4][4]));}
static void fk(const float q[6],float T[4][4]){
 float I[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};memcpy(T,I,sizeof(I));
 for(int i=0;i<6;i++){float M[4][4];mdh(M,D_H[i][0],D_H[i][1],D_H[i][2],q[i]);float P[4][4];mmul(P,T,M);memcpy(T,P,sizeof(float[4][4]));}}
static float frand(float lo,float hi){return lo+(hi-lo)*(rand()/(float)RAND_MAX);}
static float pos_err(const float Tg[4][4],const float Tt[4][4]){
 return sqrtf(powf(Tg[0][3]-Tt[0][3],2)+powf(Tg[1][3]-Tt[1][3],2)+powf(Tg[2][3]-Tt[2][3],2));}
static float deg2rad(float d){return d*(float)(M_PI/180.0);}

/* 限位内固定种子（度），覆盖肘上/肘下 + 不同腕姿态 */
static const float FIXED_DEG[][6] = {
 {180,135,  0,   0, 45,180},
 {180,135, 80,   0, 20,180},
 {180,100,-80,   0, 70,180},
 {180,170, 80,   0, 20,180},
 {100,135,  0,  80, 45, 90},
 {260,135,  0, -80, 45,270},
 {  0,120, 30,  30, 30,  0},
 { 90,150,-30, -30, 60, 90},
 {270,150, 30,  30, 30,270},
 {180,120, 60, -60, 50,180},
 {180,160,-60,  60, 50,180},
 { 45,135, 0,    0, 45, 45},
};
#define N_FIXED (int)(sizeof(FIXED_DEG)/sizeof(FIXED_DEG[0]))

/* 尝试一个种子：数值 IK 收敛 + 限位内，返回 0 成功并填 q_deg/pe/re */
static int try_seed(const float Tt[4][4], const float seed[6],
                    float q_deg[6], float *pe, float *re)
{
    float qout[6];
    if (nik_solve(Tt, seed, qout, pe, re) != 0) return -1;
    if (nik_to_degrees_in_limits(qout, q_deg) != 0) return -1;
    if (*pe >= 1.0f || *re >= 1.0f) return -1;
    return 0;
}

/* 两个角度解的“最短角度距离”（度，J1/J6 整周），用于挑最接近真解的兜底解 */
static float ang_dist(float a, float b, int wrap)
{
    float d = a - b;
    if (wrap) { d = fmodf(d, 360.0f); if (d > 180) d -= 360; if (d < -180) d += 360; }
    return fabsf(d);
}

int main(void)
{
    srand(42);
    float mn[6]={0,90,-90,-90,0,0},mx[6]={360,180,90,90,90,360};
    for(int j=0;j<6;j++){g_robot.joints[j].min_angle=mn[j];g_robot.joints[j].max_angle=mx[j];}
    float lo[6]={5,95,-85,-85,5,5},hi[6]={355,175,85,85,85,355};
    int N=500;

    int analytic_ok=0, analytic_fail=0;
    int fallback_ok=0;                       /* 数值兜底成功 */
    double fb_pe_max=0, fb_re_max=0, fb_pe_sum=0, fb_re_sum=0;
    int self_ok=0;                           /* 真解种子自检 */
    double self_pe_max=0;

    for (int it=0; it<N; it++) {
        float qd[6], qr[6];
        for (int j=0;j<6;j++){qd[j]=frand(lo[j],hi[j]);qr[j]=deg2rad(qd[j]);}
        float Tt[4][4]; fk(qr,Tt);
        robot_kinematics_joint_angle_update(qd);

        float so[6];
        int rc = robot_kinematics_inverse((float*)Tt, so, 0);

        /* 自检：数值 IK 从真解种子出发应精确还原 */
        {
            float qout[6], pe, re, qdeg[6];
            if (nik_solve(Tt, qr, qout, &pe, &re)==0 && pe<1.0f && re<1.0f) {
                self_ok++;
                if (pe>self_pe_max) self_pe_max=pe;
            }
        }

        if (rc==0) {
            float sro[6]; for(int j=0;j<6;j++) sro[j]=deg2rad(so[j]);
            float To[4][4]; fk(sro,To);
            if (pos_err(To,Tt)<1.0f) analytic_ok++;
            else analytic_fail++;
        } else {
            analytic_fail++;
        }

        /* 仅对解析失败的点做数值兜底 */
        if (rc!=0) {
            /* 重新 calc 取解析的 4 个弧度解（未做 mask/角度转换） */
            memcpy(g_robot_kinematics.T, Tt, sizeof(Tt));
            robot_kinematics_calc();

            float best[6]; int got=0; float best_pe=1e9, best_re=1e9;
            float best_diff=1e9;
            float cand[6], pe, re;

            /* 种子 1：解析 4 个弧度解（即使被判定伪根/越限） */
            for (int i=0;i<ROBOT_KINEMATICS_RESULT_NUM;i++) {
                if (try_seed(Tt, g_robot_kinematics.result[i], cand, &pe,&re)==0) {
                    float diff=0;
                    for (int j=0;j<6;j++)
                        diff += ang_dist(cand[j],qd[j], (j==0||j==5))*joint_weight[j];
                    if (diff<best_diff){best_diff=memcpy(best,cand,sizeof(best));best_pe=pe;best_re=re;got=1;}
                }
            }
            /* 种子 2：限位内固定种子 */
            for (int s=0;s<N_FIXED;s++) {
                float seed[6];
                for (int j=0;j<6;j++) seed[j]=deg2rad(FIXED_DEG[s][j]);
                if (try_seed(Tt, seed, cand, &pe,&re)==0) {
                    float diff=0;
                    for (int j=0;j<6;j++)
                        diff += ang_dist(cand[j],qd[j], (j==0||j==5))*joint_weight[j];
                    if (diff<best_diff){best_diff=0;memcpy(best,cand,sizeof(best));best_pe=pe;best_re=re;got=1;}
                }
            }
            if (got) {
                fallback_ok++;
                fb_pe_max=fmax(fb_pe_max,best_pe);
                fb_re_max=fmax(fb_re_max,best_re);
                fb_pe_sum+=best_pe; fb_re_sum+=best_re;
            }
        }
    }

    printf("============ 数值 IK 兜底验证（%d 组随机 FK→IK）============\n", N);
    printf("解析 IK（patch1）        成功 %d / 失败 %d\n", analytic_ok, analytic_fail);
    printf("数值 IK 真解种子自检     成功 %d/%d，最大位置误差 %.4f mm\n",
           self_ok, N, self_pe_max);
    printf("--------------------------------------------------------------\n");
    printf("解析失败点数值兜底       成功 %d / %d\n", fallback_ok, analytic_fail);
    if (fallback_ok) {
        printf("  兜底解 位置误差 最大 %.4f mm 平均 %.4f mm\n",
               fb_pe_max, fb_pe_sum/fallback_ok);
        printf("  兜底解 姿态误差 最大 %.4f° 平均 %.4f°\n",
               fb_re_max, fb_re_sum/fallback_ok);
    }
    int total_ok = analytic_ok + fallback_ok;
    printf("--------------------------------------------------------------\n");
    printf("综合成功率（解析+数值兜底） %d/%d = %.1f%%\n",
           total_ok, N, 100.0*total_ok/N);
    printf("仍失败 %d 组\n", N-total_ok);
    return 0;
}
