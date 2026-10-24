/* 诊断：修复后 inverse 返回 -1（无解）的那些点，打印 4 组解的 position 残差与限位状态 */
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

int main(void){
 srand(42);
 float mn[6]={0,90,-90,-90,0,0},mx[6]={360,180,90,90,90,360};
 for(int j=0;j<6;j++){g_robot.joints[j].min_angle=mn[j];g_robot.joints[j].max_angle=mx[j];}
 float lo[6]={5,95,-85,-85,5,5},hi[6]={355,175,85,85,85,355};
 int N=500, shown=0;
 for(int it=0;it<N;it++){
  float qd[6],qr[6];
  for(int j=0;j<6;j++){qd[j]=frand(lo[j],hi[j]);qr[j]=qd[j]*(float)(M_PI/180.0);}
  float Tt[4][4];fk(qr,Tt);
  robot_kinematics_joint_angle_update(qd);
  float so[6];
  if(robot_kinematics_inverse((float*)Tt,so,0)==0) continue;
  if(shown>=5) continue;
  shown++;
  printf("=== 无解点 #%d 目标qd=[%.1f %.1f %.1f %.1f %.1f %.1f]° 目标平移=[%.1f %.1f %.1f]mm ===\n",
    it,qd[0],qd[1],qd[2],qd[3],qd[4],qd[5],Tt[0][3],Tt[1][3],Tt[2][3]);
  /* 重算弧度解，打印每组的残差 */
  memcpy(g_robot_kinematics.T,Tt,sizeof(Tt));
  robot_kinematics_calc();
  for(int i=0;i<4;i++){
    float Tg[4][4]; fk(g_robot_kinematics.result[i],Tg);
    float er=pos_err(Tg,Tt);
    int lim_ok=1; for(int j=0;j<6;j++){
      float d=g_robot_kinematics.result[i][j]*180.0f/M_PI;
      d=fmodf(d,360.0f); if(d<0)d+=360.0f;
      if(d<mn[j]||d>mx[j]) lim_ok=0;
    }
    printf("  组%d: 残差=%.3f mm 限位有效=%d  角=[%.0f %.0f %.0f %.0f %.0f %.0f]°\n",
      i,er,lim_ok,
      g_robot_kinematics.result[i][0]*180/M_PI,g_robot_kinematics.result[i][1]*180/M_PI,
      g_robot_kinematics.result[i][2]*180/M_PI,g_robot_kinematics.result[i][3]*180/M_PI,
      g_robot_kinematics.result[i][4]*180/M_PI,g_robot_kinematics.result[i][5]*180/M_PI);
  }
 }
 printf("(共诊断 %d 个无解点)\n",shown);
 return 0;
}
