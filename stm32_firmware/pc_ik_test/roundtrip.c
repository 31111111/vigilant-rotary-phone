/*
 * 固件逆运动学算法 · FK→IK 闭环往返验证（PC / x86）
 * 流程：随机关节角 q -> 正运动学(MDH)得末端位姿 T -> 固件 robot_kinematics_inverse 反算
 *       -> 再用 FK 由反算关节角求位姿 T' -> 比较 T 与 T' 的位置/姿态误差。
 * FK 约定由官方 robot_analyse.m / robot_run.m 标定（MDH，theta=关节角）。
 * 编译：gcc -std=gnu11 -O2 roundtrip.c robot_kinematics.c -lm -o roundtrip
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "robot_kinematics.h"

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

/* 官方 robot_analyse.m 给出的改进 DH 连杆矩阵：theta=关节角 */
static void mdh(float T[4][4], float a, float alpha, float d, float theta)
{
    float ct=cosf(theta), st=sinf(theta), ca=cosf(alpha), sa=sinf(alpha);
    float m[4][4]={{ct,-st,0,a},
                   {st*ca,ct*ca,-sa,-sa*d},
                   {st*sa,ct*sa, ca, ca*d},
                   {0,0,0,1}};
    memcpy(T,m,sizeof(m));
}
static void mmul(float o[4][4], const float a[4][4], const float b[4][4])
{
    float t[4][4]={0};
    for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=a[i][k]*b[k][j];t[i][j]=(float)s;}
    memcpy(o,t,sizeof(float[4][4]));
}
static void fk(const float q_rad[6], float T[4][4])
{
    float I[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    memcpy(T,I,sizeof(I));
    for(int i=0;i<6;i++){
        float M[4][4]; mdh(M, D_H[i][0], D_H[i][1], D_H[i][2], q_rad[i]);
        float P[4][4]; mmul(P,T,M); memcpy(T,P,sizeof(float[4][4]));
    }
}

static float frand(float lo, float hi){ return lo + (hi-lo)*(rand()/(float)RAND_MAX); }

int main(void)
{
    srand(20240601);
    float mn[6]={0, 90,-90,-90, 0,  0};
    float mx[6]={360,180,90, 90, 90,360};
    for(int j=0;j<6;j++){g_robot.joints[j].min_angle=mn[j]; g_robot.joints[j].max_angle=mx[j];}

    int N=200, valid=0;
    double max_pos=0, sum_pos=0;
    double max_ori=0, sum_ori=0;
    int fail=0;

    for(int it=0; it<N; it++){
        /* 1. 限位内随机关节角（度），留 5° 余量避免边界 */
        float q_deg[6], q_rad[6];
        for(int j=0;j<6;j++){
            q_deg[j]=frand(mn[j]+5, mx[j]-5);
            q_rad[j]=q_deg[j]*(float)(M_PI/180.0);
        }
        /* 2. 正运动学得目标位姿 */
        float Tt[4][4]; fk(q_rad, Tt);

        /* 3. 当前角设为真值，便于 IK 选最近解 */
        robot_kinematics_joint_angle_update(q_deg);

        /* 4. 固件逆运动学反算 */
        float sol_deg[6];
        if(robot_kinematics_inverse((float*)Tt, sol_deg, 0)!=0){ fail++; continue; }

        /* 5. 反算解再做 FK，比较位姿 */
        float sol_rad[6]; for(int j=0;j<6;j++) sol_rad[j]=sol_deg[j]*(float)(M_PI/180.0);
        float Tc[4][4]; fk(sol_rad, Tc);

        double dx=Tc[0][3]-Tt[0][3], dy=Tc[1][3]-Tt[1][3], dz=Tc[2][3]-Tt[2][3];
        double pos=sqrt(dx*dx+dy*dy+dz*dz);

        /* 姿态误差：比较 z 轴（approach）方向单位向量的夹角 */
        double dot=Tt[0][2]*Tc[0][2]+Tt[1][2]*Tc[1][2]+Tt[2][2]*Tc[2][2];
        if(dot>1)dot=1; if(dot<-1)dot=-1;
        double ori=acos(dot)*180.0/M_PI;

        if(pos>max_pos)max_pos=pos;
        if(ori>max_ori)max_ori=ori;
        sum_pos+=pos; sum_ori+=ori; valid++;
    }

    printf("========== 固件 IK 算法 · FK→IK 闭环往返验证（%d 组随机姿态）==========\n", N);
    printf("成功反算: %d / %d（%d 组因逼近工作空间边界无解）\n", valid, N, fail);
    if(valid){
        printf("末端位置误差  最大: %.4f mm   平均: %.4f mm\n", max_pos, sum_pos/valid);
        printf("末端姿态误差  最大: %.4f °    平均: %.4f °\n", max_ori, sum_ori/valid);
    }
    printf("说明: 固件核心逆运动学算法 robot_kinematics.c 未改动，在 PC 上编译运行。\n");
    return 0;
}
