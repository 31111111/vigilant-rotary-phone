/* 验证改进：在选解中加入“位置残差校验”，对比固件原选择 vs 校验后选择 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "robot_kinematics.c"

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

/* 带位置残差校验的选解：只在 限位有效 且 FK 位置残差<阈值 的解中选离当前角最近的 */
static int select_with_pos_check(float *result, float tol_mm){
 int best=-1; float bestdiff=1e30f;
 for(int i=0;i<ROBOT_KINEMATICS_RESULT_NUM;i++){
   if(g_robot_kinematics.result_invalid_mask & (1<<i)) continue;   /* 限位无效 */
   float Tg[4][4]; fk(g_robot_kinematics.result[i],Tg);
   if(pos_err(Tg,g_robot_kinematics.T) > tol_mm) continue;         /* 位置残差校验 */
   float diff=0;
   for(int j=0;j<ROBOT_MAX_JOINT_NUM;j++)
     diff += fabsf(g_robot_kinematics.result[i][j]-g_current_joint_angle[j])*joint_weight[j];
   if(diff<bestdiff){bestdiff=diff;best=i;}
 }
 if(best<0) return -1;
 for(int j=0;j<ROBOT_MAX_JOINT_NUM;j++) result[j]=g_robot_kinematics.result[best][j];
 return 0;
}

int main(void){
 srand(42);
 float mn[6]={0,90,-90,-90,0,0},mx[6]={360,180,90,90,90,360};
 for(int j=0;j<6;j++){g_robot.joints[j].min_angle=mn[j];g_robot.joints[j].max_angle=mx[j];}
 float lo[6]={5,95,-85,-85,5,5},hi[6]={355,175,85,85,85,355};
 int N=500;
 int n=0, o1=0; double omax=0, c1=0,cfail=0; double cmax=0;
 for(int it=0;it<N;it++){
  float qd[6],qr[6];
  for(int j=0;j<6;j++){qd[j]=frand(lo[j],hi[j]);qr[j]=qd[j]*(float)(M_PI/180.0);}
  float Tt[4][4];fk(qr,Tt);
  robot_kinematics_joint_angle_update(qd);

  /* 固件原始选择 */
  float so[6];
  if(robot_kinematics_inverse((float*)Tt,so,0)!=0) continue;
  float sro[6];for(int j=0;j<6;j++)sro[j]=so[j]*(float)(M_PI/180.0);
  float To[4][4];fk(sro,To); float eo=pos_err(To,Tt);
  if(eo<1)o1++; if(eo>omax)omax=eo;

  /* 带位置校验：重新 calc，保留弧度，按 FK 位置残差<1mm 过滤后选最近 */
  memcpy(g_robot_kinematics.T,Tt,sizeof(Tt));
  robot_kinematics_calc();
  int best=-1; float bd=1e30f;
  for(int i=0;i<4;i++){
    float Tg[4][4];fk(g_robot_kinematics.result[i],Tg);
    if(pos_err(Tg,Tt)>1.0f) continue;
    float rd[6]; for(int j=0;j<6;j++){float d=g_robot_kinematics.result[i][j]*180.0f/M_PI;
        d=fmodf(d,360.0f); if(d<0)d+=360.0f; rd[j]=d;}
    float diff=0; for(int j=0;j<6;j++) diff+=fabsf(rd[j]-qd[j])*joint_weight[j];
    if(diff<bd){bd=diff;best=i;}
  }
  if(best<0){cfail++;continue;}
  float sc[6]; for(int j=0;j<6;j++){
    float d=g_robot_kinematics.result[best][j]*180.0f/M_PI;
    d=fmodf(d,360.0f); if(d<0)d+=360.0f;
    float a=mn[j],b=mx[j]; if(d<a)d+=360; else if(d>b)d-=360; sc[j]=d;}
  float src[6];for(int j=0;j<6;j++)src[j]=sc[j]*(float)(M_PI/180.0);
  float Tc[4][4];fk(src,Tc); float ec=pos_err(Tc,Tt);
  if(ec<1)c1++; if(ec>cmax)cmax=ec; n++;
 }
 printf("========== 固件 IK 选解逻辑：位置校验 前 vs 后（%d 组）==========\n",n);
 printf("【固件原始（仅按关节角距离选解）】  位置误差<1mm 占 %.1f%%，最大误差 %.1f mm\n",100.0*o1/n,omax);
 printf("【加入位置残差校验(<1mm)后】        位置误差<1mm 占 %.1f%%，最大 %.2f mm，无解 %.0f\n",
        100.0*c1/n,cmax,cfail);
 printf("\n结论：固件解析解数学正确（真解 FK 位置误差≈0），但 4 组中含半角法伪根；\n");
 printf("      原选解只比较关节角距离，不校验位置，约 %.0f%% 的情况会选中伪根导致大误差。\n",100.0*(1.0-(double)o1/n));
 printf("      增加一次末端位置残差校验即可把精度提升到接近 100%%。\n");
 return 0;
}
