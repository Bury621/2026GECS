#include "robot.h"

/* 轮子编号：1 左前，2 右前，3 左后，4 右后 */
#define WHEEL_LF  1
#define WHEEL_RF  2
#define WHEEL_LR  3
#define WHEEL_RR  4

/* 加速度，按实际需求设置，0~255 */
#define MOVE_ACCEL  0

/* 最大轮速 */
#define WHEEL_MAX_SPEED  3000

/* 方向修正：1 保持，-1 取反，按实际调试修改 */
static const int8_t dir_fix[4] = { 1, -1, 1, -1 };

/**
 * @brief 设置小车的运动状态（X型麦克纳姆轮）
 *
 * 轮子布局:
 *     1  2
 *     3  4
 *
 * @param vx    前后速度，向前为正
 * @param vy    左右速度，向右为正
 * @param omega 旋转速度，逆时针为正
 */
void Robot_set_move(int16_t vx, int16_t vy, int16_t omega)
{
    /* X型麦克纳姆轮速度分解 */
    int32_t w1 = (int32_t)vx + vy - omega;   /* 左前 */
    int32_t w2 = (int32_t)vx - vy + omega;   /* 右前 */
    int32_t w3 = (int32_t)vx - vy - omega;   /* 左后 */
    int32_t w4 = (int32_t)vx + vy + omega;   /* 右后 */

    /* 四个轮子的原始值 */
    int32_t raw[4] = { w1, w2, w3, w4 };

    /* 找最大绝对值，用于等比缩放 */
    int32_t max_abs = 0;
    for (int i = 0; i < 4; i++) {
        int32_t a = raw[i] > 0 ? raw[i] : -raw[i];
        if (a > max_abs) max_abs = a;
    }

    /* 超过 3000 则等比缩放，保持运动方向比例 */
    if (max_abs > WHEEL_MAX_SPEED) {
        for (int i = 0; i < 4; i++) {
            raw[i] = (int32_t)((int64_t)raw[i] * WHEEL_MAX_SPEED / max_abs);
        }
    }

    /* 应用方向修正 */
    for (int i = 0; i < 4; i++) {
        raw[i] *= dir_fix[i];
    }

    /* 依次发送给四个电机，addr 对应轮子编号 */
    Emm_Speed_Mode(WHEEL_LF, raw[0], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_RF, raw[1], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_LR, raw[2], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_RR, raw[3], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Sync_Trig();
}