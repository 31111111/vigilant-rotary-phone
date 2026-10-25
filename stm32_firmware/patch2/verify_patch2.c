/*
 * verify_patch2.c —— 验证 patch2（θ1 双分支）：500 组随机 FK→IK
 *
 * 与 patch1 的 verify_patched.c 口径一致（srand 42，限位内收窄采样）。
 * 关注：patch2 把候选解由 4 个扩到 8 个（枚举 θ1 两个几何根）后，
 *       解析成功率是否由 438/500 提升到 500/500，且被接受解的位置误差
 *       是否仍全部 <1mm（不能因多候选而误选）。
 *
 * 编译：见本目录 Makefile
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "robot_kinematics.c"   /* patch2 固件源码 */

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
 int N=500;
 int ok1=0,fail=0; double mxerr=0,sum=0;
 for(int it=0;it<N;it++){
  float qd[6],qr[6];
  for(int j=0;j<6;j++){qd[j]=frand(lo[j],hi[j]);qr[j]=qd[j]*(float)(M_PI/180.0);}
  float Tt[4][4];fk(qr,Tt);
  robot_kinematics_joint_angle_update(qd);
  float so[6];
  if(robot_kinematics_inverse((float*)Tt,so,0)!=0){fail++;continue;}
  float sro[6];for(int j=0;j<6;j++)sro[j]=so[j]*(float)(M_PI/180.0);
  float To[4][4];fk(sro,To); float eo=pos_err(To,Tt);
  if(eo<1)ok1++; if(eo>mxerr)mxerr=eo; sum+=eo;
 }
 printf("========== patch2（θ1 双分支）robot_kinematics_inverse（%d 组）==========\n",N);
 printf("位置误差<1mm 占 %.1f%%，最大 %.4f mm，平均 %.5f mm，无解 %d\n",
        100.0*ok1/N, mxerr, sum/N, fail);
 printf("\n对照：patch1（4 候选）438/500 成功、62 无解；patch2（8 候选，枚举 θ1 两支）为 %d/%d。\n",
        N-fail, N);
 return 0;
}
