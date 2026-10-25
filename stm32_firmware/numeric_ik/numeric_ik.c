/*
 * numeric_ik.c —— 6 自由度机械臂数值逆运动学（DLS 阻尼最小二乘）
 */
#include "numeric_ik.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float NIK_DH[NIK_JOINT_NUM][4] = {
    {0.0f,    0.0f,       0.0f,    (float)(M_PI/2)},
    {0.0f,    (float)(M_PI/2),  0.0f,    (float)(M_PI/2)},
    {200.0f,  (float)M_PI,      0.0f,   (float)(-M_PI/2)},
    {47.63f, (float)(-M_PI/2), -184.5f, 0.0f},
    {0.0f,    (float)(M_PI/2),  0.0f,    (float)(M_PI/2)},
    {0.0f,    (float)(M_PI/2),  0.0f,    0.0f},
};

/* 关节限位（度），与固件一致 */
const float NIK_LIMIT_MIN_DEG[NIK_JOINT_NUM] = {0.0f,  90.0f, -90.0f, -90.0f, 0.0f, 0.0f};
const float NIK_LIMIT_MAX_DEG[NIK_JOINT_NUM] = {360.0f,180.0f, 90.0f,  90.0f, 90.0f,360.0f};

/* 数值求解时对有硬限位的关节（J2/J3/J4/J5）在弧度空间钳制；
 * J1/J6 为整周关节，内部使用连续值，输出时再映射到 0..360。
 * -1 表示该关节不钳制。 */
static const float CLAMP_LO[NIK_JOINT_NUM] = {
    -1e9f, (float)(M_PI/2), (float)(-M_PI/2), (float)(-M_PI/2), 0.0f, -1e9f};
static const float CLAMP_HI[NIK_JOINT_NUM] = {
     1e9f, (float)M_PI,     (float)( M_PI/2), (float)( M_PI/2), (float)(M_PI/2), 1e9f};

/* ---------- MDH 正运动学（与固件 patch1 完全一致） ---------- */
static void mdh(float M[4][4], float a, float al, float d, float th)
{
    float ct = cosf(th), st = sinf(th);
    float ca = cosf(al), sa = sinf(al);
    M[0][0]=ct;     M[0][1]=-st;    M[0][2]=0.0f;  M[0][3]=a;
    M[1][0]=st*ca;  M[1][1]=ct*ca;  M[1][2]=-sa;   M[1][3]=-sa*d;
    M[2][0]=st*sa;  M[2][1]=ct*sa;  M[2][2]=ca;    M[2][3]=ca*d;
    M[3][0]=0; M[3][1]=0; M[3][2]=0; M[3][3]=1;
}

void nik_fk(const float q[6], float T[4][4])
{
    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    memcpy(T, I, sizeof(I));
    for (int i = 0; i < 6; i++) {
        float M[4][4], P[4][4];
        mdh(M, NIK_DH[i][0], NIK_DH[i][1], NIK_DH[i][2], q[i]);
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++) {
                float s = 0;
                for (int k = 0; k < 4; k++) s += T[r][k]*M[k][c];
                P[r][c] = s;
            }
        memcpy(T, P, sizeof(P));
    }
}

/* 世界系姿态误差向量：0.5*sum(Rf_col_i x Rt_col_i)，重合时为 0 */
static void rot_error(const float Rf[4][4], const float Rt[4][4], float e[3])
{
    for (int i = 0; i < 3; i++) {
        float f[3] = {Rf[0][i], Rf[1][i], Rf[2][i]};
        float t[3] = {Rt[0][i], Rt[1][i], Rt[2][i]};
        e[0] += 0.5f*(f[1]*t[2]-f[2]*t[1]);
        e[1] += 0.5f*(f[2]*t[0]-f[0]*t[2]);
        e[2] += 0.5f*(f[0]*t[1]-f[1]*t[0]);
    }
}

/* 6 维误差（位置用米，姿态用 rad） */
static void pose_error(const float Tf[4][4], const float Tt[4][4], float e[6])
{
    e[0] = (Tt[0][3]-Tf[0][3])/1000.0f;
    e[1] = (Tt[1][3]-Tf[1][3])/1000.0f;
    e[2] = (Tt[2][3]-Tf[2][3])/1000.0f;
    float re[3] = {0,0,0};
    rot_error(Tf, Tt, re);
    e[3]=re[0]; e[4]=re[1]; e[5]=re[2];
}

/* 高斯消元解 6x6 线性系统 A x = b（带部分主元），成功返回 0 */
static int solve6(float A[6][6], float b[6], float x[6])
{
    for (int k = 0; k < 6; k++) {
        int piv = k;
        for (int r = k+1; r < 6; r++)
            if (fabsf(A[r][k]) > fabsf(A[piv][k])) piv = r;
        if (fabsf(A[piv][k]) < 1e-12f) return -1;
        if (piv != k) {
            for (int c = 0; c < 6; c++) { float t=A[k][c]; A[k][c]=A[piv][c]; A[piv][c]=t; }
            float t=b[k]; b[k]=b[piv]; b[piv]=t;
        }
        for (int r = k+1; r < 6; r++) {
            float f = A[r][k]/A[k][k];
            for (int c = k; c < 6; c++) A[r][c] -= f*A[k][c];
            b[r] -= f*b[k];
        }
    }
    for (int r = 5; r >= 0; r--) {
        float s = b[r];
        for (int c = r+1; c < 6; c++) s -= A[r][c]*x[c];
        x[r] = s/A[r][r];
    }
    return 0;
}

static void clamp_joints(float q[6])
{
    for (int i = 0; i < 6; i++) {
        if (q[i] < CLAMP_LO[i]) q[i] = CLAMP_LO[i];
        if (q[i] > CLAMP_HI[i]) q[i] = CLAMP_HI[i];
    }
}

int nik_solve(const float Tt[4][4], const float q_seed[6], float q_out[6],
              float *pos_err_mm, float *rot_err_deg)
{
    float q[6];
    memcpy(q, q_seed, sizeof(q));
    clamp_joints(q);

    const float lambda = 0.02f;        /* 阻尼系数 */
    const float dq = 1e-5f;            /* 数值差分步长 */
    const float step_max = 0.20f;      /* 单步最大关节增量（rad） */
    const int   max_iter = 500;

    for (int it = 0; it < max_iter; it++) {
        float Tf[4][4];
        nik_fk(q, Tf);
        float e[6];
        pose_error(Tf, Tt, e);

        float pe_mm = sqrtf(e[0]*e[0]+e[1]*e[1]+e[2]*e[2])*1000.0f;
        float re_rad = sqrtf(e[3]*e[3]+e[4]*e[4]+e[5]*e[5]);
        if (pe_mm < 1.0f && re_rad < 0.0174533f) {
            memcpy(q_out, q, sizeof(q));
            if (pos_err_mm) *pos_err_mm = pe_mm;
            if (rot_err_deg) *rot_err_deg = re_rad*57.29578f;
            return 0;
        }

        /* 空间雅可比 J = d(末端位姿)/dq（6x6）。
         * 误差 e 定义为“目标-当前”，其差分为 -J，故此处取负得到标准
         * 空间雅可比（位置 m/rad，姿态 rad/rad）。 */
        float J[6][6];
        memset(J, 0, sizeof(J));
        for (int i = 0; i < 6; i++) {
            float qp[6];
            memcpy(qp, q, sizeof(qp));
            float old = qp[i];
            float hi = old + dq, lo = old - dq;
            if (hi > CLAMP_HI[i]) hi = CLAMP_HI[i];
            if (lo < CLAMP_LO[i]) lo = CLAMP_LO[i];
            float step = hi - lo;
            if (step < 1e-9f) step = dq;
            float Tp[4][4], Tn[4][4], ep[6], en[6];
            qp[i] = hi; nik_fk(qp, Tp); pose_error(Tp, Tt, ep);
            qp[i] = lo; nik_fk(qp, Tn); pose_error(Tn, Tt, en);
            for (int r = 0; r < 6; r++)
                J[r][i] = -(ep[r]-en[r])/step;
        }

        /* A = J J^T + lambda^2 I；解 A x = e；dq = J^T x */
        float A[6][6] = {{0}}, x[6];
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                float s = 0;
                for (int k = 0; k < 6; k++) s += J[r][k]*J[c][k];
                A[r][c] = s;
            }
            A[r][r] += lambda*lambda;
            x[r] = e[r];
        }
        float dqvec[6] = {0};
        if (solve6(A, x, x) != 0) break;
        for (int i = 0; i < 6; i++) {
            float s = 0;
            for (int r = 0; r < 6; r++) s += J[r][i]*x[r];
            dqvec[i] = s;
            if (dqvec[i] >  step_max) dqvec[i] =  step_max;
            if (dqvec[i] < -step_max) dqvec[i] = -step_max;
            q[i] += dqvec[i];
        }
        clamp_joints(q);
    }

    /* 未收敛，仍输出最终姿态与误差 */
    float Tf[4][4];
    nik_fk(q, Tf);
    float e[6];
    pose_error(Tf, Tt, e);
    memcpy(q_out, q, sizeof(q));
    if (pos_err_mm) *pos_err_mm = sqrtf(e[0]*e[0]+e[1]*e[1]+e[2]*e[2])*1000.0f;
    if (rot_err_deg) *rot_err_deg = sqrtf(e[3]*e[3]+e[4]*e[4]+e[5]*e[5])*57.29578f;
    return -1;
}

int nik_to_degrees_in_limits(const float q_rad[6], float q_deg[6])
{
    int ok = 1;
    for (int i = 0; i < 6; i++) {
        float d = q_rad[i]*57.29577951f;
        if (i == 0 || i == 5) {                 /* 整周轴映射 0..360 */
            d = fmodf(d, 360.0f);
            if (d < 0) d += 360.0f;
        }
        if (d < NIK_LIMIT_MIN_DEG[i]-1e-3f || d > NIK_LIMIT_MAX_DEG[i]+1e-3f) ok = 0;
        q_deg[i] = d;
    }
    return ok ? 0 : -1;
}
