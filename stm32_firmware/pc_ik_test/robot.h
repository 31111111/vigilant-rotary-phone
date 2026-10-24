/*
 * PC 端精简版 robot.h —— 仅为在 PC 上编译固件逆运动学算法而提供，
 * 去除 FreeRTOS / GPIO / CMSIS 等硬件依赖，保留 IK 所需的数据结构与常量。
 * 数据结构与固件 Core/Inc/robot.h 保持一致（字段、数值完全相同）。
 */
#ifndef __ROBOT_H_PC__
#define __ROBOT_H_PC__
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ROBOT_MAX_JOINT_NUM 6
#define ROBOT_ERROR_RANGE   (1e-4f)

enum motor_dir { MOTOR_DIR_CW = 0, MOTOR_DIR_CCW = 1 };
enum dir       { DIR_POSITIVE = 1, DIR_NEGATIVE = -1 };

/* 与固件 struct joint 中 IK 用到的字段一致（限位部分） */
struct joint {
    float current_angle;
    enum motor_dir postive_direction;
    float reduction_ratio;
    float min_angle;
    float max_angle;
    enum dir reset_dir;
    uint32_t status;
    float velocity;
    float acceleration;
};

struct position { float x, y, z; };
struct rotate   { float x, y, z; };

struct robot {
    float T[4][4];
    struct joint joints[ROBOT_MAX_JOINT_NUM];
    struct position cur_pos;
    struct rotate cur_rot;
};

enum {
    ROBOT_JOINT_1 = 0, ROBOT_JOINT_2, ROBOT_JOINT_3,
    ROBOT_JOINT_4, ROBOT_JOINT_5, ROBOT_JOINT_6,
};

extern struct robot g_robot;
extern const float D_H[6][4];
extern const float joint_weight[ROBOT_MAX_JOINT_NUM];

#endif
