#include "robot.h"
#include <math.h>
#include <stdio.h>
#include "Maxi_Connect.h"
#include "Ray.h"
#include "Servo.h"

/* 轮子编号：1 左前，2 右前，3 左后，4 右后 */
#define WHEEL_LF  1
#define WHEEL_RF  2
#define WHEEL_LR  3
#define WHEEL_RR  4

/* 加速度，按实际需求设置，0~255 */
#define MOVE_ACCEL  0

/* 最大轮速 */
#define WHEEL_MAX_SPEED  3000

#define ROBOT_REMOTE_CONTROL_SPEED 80
#define ROBOT_QUICK_TURN_REDIUS 130.0f
/* 方向修正：1 保持，-1 取反，按实际调试修改 */
static const int8_t dir_fix[4] = { 1, -1, 1, -1 };

/* 角度环输出方向：轮子方向整体取反时同步改为 -1.0f */
#define ROBOT_ANGLE_OUTPUT_SIGN  1.0f

/* 缓存三个运动分量，单轴更新时保留其他分量 */
static int16_t robot_vx = 0;
static int16_t robot_vy = 0;
static int16_t robot_omega = 0;

/* 角度环内部状态 */
static float robot_angle_continuous = 0.0f;
static float robot_angle_last_yaw = 0.0f;
static float robot_angle_last_omega = 0.0f;
static uint8_t robot_angle_has_yaw = 0;

/* 差速转弯内部状态 */
static volatile Robot_differential_turn_state robot_diff_turn_state =
    ROBOT_DIFFERENTIAL_TURN_IDLE;
static float robot_diff_turn_start_angle = 0.0f;
static float robot_diff_turn_target_angle = 0.0f;
static float robot_diff_turn_absolute_target = 0.0f;
static float robot_diff_turn_linear_speed = 0.0f;
static int16_t robot_diff_turn_resume_vx = 0;
static float robot_diff_turn_omega = 0.0f;
static int8_t robot_diff_turn_direction = 1;
static uint32_t robot_diff_turn_start_tick = 0;

/* 计时器内部状态 */
static uint32_t robot_time=0; //用来计时的变量
static uint32_t robot_last_tick=0;
static uint32_t robot_task1_ball_stable_start_tick=0;
static uint8_t robot_task1_ball_stable_timing=0;
static uint32_t robot_task2_target_stable_start_tick=0;
static uint8_t robot_task2_target_stable_timing=0;
static uint32_t robot_task2_laser_on_start_tick=0;
static uint8_t robot_task2_laser_on_timing=0;
static uint32_t robot_task3_hostage_stable_start_tick=0;
static uint8_t robot_task3_hostage_stable_timing=0;
static uint32_t robot_task1_back_time=0;
static uint8_t robot_task1_back_started=0;
static uint8_t robot_task1_dmg_started=0;

/* 保存小车当前状态 */
static Robot_stage robot_now_stage=ROBOT_STAGE_TASK1_DONE; //直接从任务二结束开始调试

Robot_remote_sign remote_sign=ROBOT_REMOTE_STOP; //遥控状态标志位

extern volatile uint16_t robot_angle; //App.c里面的，虽然耦合度很高很屎山但是不管了，懒得改了妈的
extern hwt101_csx hwt101_t; //App.c里面的陀螺仪句柄
extern DMG6220 dmg6220_t;

extern volatile Maxi_posion_package Maxi_posion_data; //maxicam的数据
extern PIDController robot_task1_ball_pid_t; //任务一闭环pid
extern PIDController robot_task2_target_pid_t; //任务二闭环pid
extern PIDController robot_task3_hostage_pid_t; //任务三闭环pid

extern volatile uint8_t robot_y_pid_is_open;

//任务是否完成状态，0表示没走到，1表示走到了还没完成，2表示完成了
uint8_t robot_task_1_state=0;
uint8_t robot_task_2_state=0;
uint8_t robot_task_3_state=0;

static void Robot_update_move(void)
{
    /* X型麦克纳姆轮速度分解 */
    int32_t w1 = (int32_t)robot_vx + robot_vy - robot_omega;   /* 左前 */
    int32_t w2 = (int32_t)robot_vx - robot_vy + robot_omega;   /* 右前 */
    int32_t w3 = (int32_t)robot_vx - robot_vy - robot_omega;   /* 左后 */
    int32_t w4 = (int32_t)robot_vx + robot_vy + robot_omega;   /* 右后 */

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
    robot_vx = vx;
    robot_vy = vy;
    robot_omega = omega;
    Robot_update_move();
}

void Robot_set_vx(int16_t vx)
{
    robot_vx = vx;
    Robot_update_move();
}

void Robot_set_vy(int16_t vy)
{
    robot_vy = vy;
    Robot_update_move();
}

void Robot_set_omega(int16_t omega)
{
    robot_omega = omega;
    Robot_update_move();
}

static uint16_t Robot_normalize_angle(float angle)
{
    int32_t normalized = (int32_t)angle % 360;
    if (normalized < 0)
    {
        normalized += 360;
    }
    return (uint16_t)normalized;
}

static void Robot_reset_angle_pid_state(PIDController *pid)
{
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->out = 0.0f;
    pid->prevMeasurement = robot_angle_continuous;
}

static void Robot_update_differential_turn(PIDController *pid)
{
    if (robot_diff_turn_state != ROBOT_DIFFERENTIAL_TURN_RUNNING)
    {
        return;
    }

    if ((HAL_GetTick() - robot_diff_turn_start_tick) >= ROBOT_DIFF_TURN_TIMEOUT_MS)
    {
        robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_DONE;
        Robot_set_move(0, 0, 0);
        robot_angle = Robot_normalize_angle(robot_angle_continuous);
        robot_angle_last_omega = 0.0f;
        Robot_reset_angle_pid_state(pid);
        return;
    }

    float turned_angle = robot_angle_continuous - robot_diff_turn_start_angle;
    float remaining_angle = robot_diff_turn_target_angle - turned_angle;
    uint8_t reached = 0;

    if (robot_diff_turn_direction > 0)
    {
        reached = remaining_angle <= ROBOT_DIFF_TURN_ANGLE_DEADBAND;
    }
    else
    {
        reached = remaining_angle >= -ROBOT_DIFF_TURN_ANGLE_DEADBAND;
    }

    if (reached)
    {
        robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_DONE;
        //Robot_set_move(0, 0, 0); 差速转弯之后不要马上停车
        Robot_set_move(robot_diff_turn_resume_vx, 0, 0);
        robot_angle = Robot_normalize_angle(robot_diff_turn_absolute_target);
        robot_angle_last_omega = 0.0f;

        /* 差速转弯结束后清掉角度环历史，避免恢复正常角度环时回抽。 */
        Robot_reset_angle_pid_state(pid);
        return;
    }

    Robot_set_move((int16_t)robot_diff_turn_linear_speed,
                   0,
                   (int16_t)(robot_diff_turn_direction * robot_diff_turn_omega));
}

uint8_t Robot_start_differential_turn_ex(float radius_cm,
                                          float target_angle_deg,
                                          float linear_speed)
{
    if (!isfinite(radius_cm) || !isfinite(target_angle_deg) ||
        !isfinite(linear_speed) || radius_cm < 0.0f || linear_speed <= 0.0f)
    {
        return 2;
    }

    float yaw;
    if (hwt101_read_yaw(&hwt101_t, &yaw) != 0)
    {
        return 1;
    }

    if (robot_angle_has_yaw == 0)
    {
        robot_angle_continuous = yaw;
        robot_angle_last_yaw = yaw;
        robot_angle_has_yaw = 1;
    }
    else
    {
        float delta_angle = yaw - robot_angle_last_yaw;
        if (delta_angle > 180.0f)
        {
            delta_angle -= 360.0f;
        }
        else if (delta_angle < -180.0f)
        {
            delta_angle += 360.0f;
        }
        robot_angle_continuous += delta_angle;
        robot_angle_last_yaw = yaw;
    }

    float abs_radius = fabsf(radius_cm);
    float abs_omega;
    float bounded_target = (float)Robot_normalize_angle(target_angle_deg);
    float target_error = bounded_target - robot_angle_continuous;

    while (target_error > 180.0f)
    {
        target_error -= 360.0f;
    }
    while (target_error < -180.0f)
    {
        target_error += 360.0f;
    }

    if (fabsf(target_error) <= ROBOT_DIFF_TURN_ANGLE_DEADBAND)
    {
        robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_IDLE;
        robot_angle = Robot_normalize_angle(bounded_target);
        return 0;
    }

    robot_diff_turn_start_angle = robot_angle_continuous;
    robot_diff_turn_target_angle = target_error;
    robot_diff_turn_absolute_target = bounded_target;
    robot_diff_turn_direction = (target_error > 0.0f) ? 1 : -1;
    robot_diff_turn_resume_vx = robot_vx;
    robot_diff_turn_linear_speed = linear_speed;

    if (robot_diff_turn_linear_speed > WHEEL_MAX_SPEED)
    {
        robot_diff_turn_linear_speed = WHEEL_MAX_SPEED;
    }

    if (abs_radius < ROBOT_DIFF_TURN_MIN_RADIUS_CM)
    {
        robot_diff_turn_linear_speed = 0.0f;
        robot_diff_turn_omega = ROBOT_DIFF_TURN_MAX_OMEGA;
    }
    else
    {
        abs_omega = robot_diff_turn_linear_speed * ROBOT_DIFF_TURN_OMEGA_GAIN / abs_radius;
        if (abs_omega > ROBOT_DIFF_TURN_MAX_OMEGA)
        {
            abs_omega = ROBOT_DIFF_TURN_MAX_OMEGA;
            robot_diff_turn_linear_speed =
                abs_omega * abs_radius / ROBOT_DIFF_TURN_OMEGA_GAIN;
        }
        robot_diff_turn_omega = abs_omega;
    }

    robot_diff_turn_start_tick = HAL_GetTick();
    robot_angle_last_omega = 0.0f;
    robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_RUNNING;
    Robot_set_move((int16_t)robot_diff_turn_linear_speed,
                   0,
                   (int16_t)(robot_diff_turn_direction * robot_diff_turn_omega));
    return 0;
}

uint8_t Robot_start_differential_turn(float radius_cm, float target_angle_deg)
{
    return Robot_start_differential_turn_ex(radius_cm,
                                             target_angle_deg,
                                             ROBOT_DIFF_TURN_DEFAULT_LINEAR_SPEED);
}

void Robot_stop_differential_turn(void)
{
    robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_IDLE;
    robot_angle = Robot_normalize_angle(robot_angle_continuous);
    robot_angle_last_omega = 0.0f;
    Robot_set_move(0, 0, 0);
}

uint8_t Robot_is_differential_turning(void)
{
    return (robot_diff_turn_state == ROBOT_DIFFERENTIAL_TURN_RUNNING) ? 1 : 0;
}

void Robot_Angle_Update(hwt101_csx *hwt101, PIDController *pid, uint16_t target_angle)
{
    float angle_sensor_data;
    if(hwt101_read_yaw(hwt101,&angle_sensor_data)!=0)
    {
        if (robot_diff_turn_state == ROBOT_DIFFERENTIAL_TURN_RUNNING &&
            (HAL_GetTick() - robot_diff_turn_start_tick) >= ROBOT_DIFF_TURN_TIMEOUT_MS)
        {
            Robot_stop_differential_turn();
        }
        return;
    }

    if(robot_angle_has_yaw==0)
    {
        robot_angle_continuous=angle_sensor_data;
        robot_angle_last_yaw=angle_sensor_data;
        robot_angle_has_yaw=1;
        pid->prevMeasurement=robot_angle_continuous;
    }
    else
    {
        float delta_angle=angle_sensor_data-robot_angle_last_yaw;
        if(delta_angle>180.0f)
        {
            delta_angle-=360.0f;
        }
        else if(delta_angle<-180.0f)
        {
            delta_angle+=360.0f;
        }
        robot_angle_continuous+=delta_angle;
        robot_angle_last_yaw=angle_sensor_data;
    }

    if (Robot_is_differential_turning())
    {
        Robot_update_differential_turn(pid);
        return;
    }

    int16_t bounded_target=(int16_t)((int32_t)target_angle%360);
    if(bounded_target>180)
    {
        bounded_target-=360;
    }

    float target_error=(float)bounded_target-robot_angle_continuous;
    while(target_error>180.0f)
    {
        target_error-=360.0f;
    }
    while(target_error<-180.0f)
    {
        target_error+=360.0f;
    }

    float target_setpoint=robot_angle_continuous+target_error;
    float angle_pid_output=PIDController_Update(pid,target_setpoint,robot_angle_continuous);

    /* HWT101 的正向与本车 omega 的正向相反 */
    float omega_output=ROBOT_ANGLE_OUTPUT_SIGN*angle_pid_output;
    float omega_delta=omega_output-robot_angle_last_omega;
    if(omega_delta>10.0f)
    {
        omega_output=robot_angle_last_omega+10.0f;
    }
    else if(omega_delta<-10.0f)
    {
        omega_output=robot_angle_last_omega-10.0f;
    }
    robot_angle_last_omega=omega_output;
    Robot_set_omega((int16_t)omega_output);
}

void Robot_stop_move(void)
{
    if (robot_diff_turn_state == ROBOT_DIFFERENTIAL_TURN_RUNNING)
    {
        robot_angle = Robot_normalize_angle(robot_angle_continuous);
        robot_angle_last_omega = 0.0f;
    }
    robot_diff_turn_state = ROBOT_DIFFERENTIAL_TURN_IDLE;
    Robot_set_move(0,0,0);
}
void Robot_remote_control(void)
{
    if(remote_sign==ROBOT_REMOTE_FORWARD)
    {
        Robot_set_vx(ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_BACKWARD)
    {
        Robot_set_vx(-ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_LEFT)
    {
        Robot_set_vy(-ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_RIGHT)
    {
        Robot_set_vy(ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    else if(remote_sign==ROBOT_REMOTE_TO_STOP)
    {
        Robot_set_move(0,0,0);
        remote_sign=0;
    }
}

/**
 * @brief 获取车是否旋转到位
 * 
 * @param hwt101 陀螺仪句柄
 * @param target_angle 目标角度（0~360°）
 * @return uint8_t 已到位返回1，还在转返回0，错误返回2
 */
uint8_t Robot_is_turning(hwt101_csx *hwt101, float target_angle)
{
    float angle_sensor_data;
    if (hwt101_read_yaw(hwt101, &angle_sensor_data) != 0)
    {
        return 2;
    }

    /* 直接求带符号差值，再归一到 (-180, 180] */
    float diff = angle_sensor_data - target_angle;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    /* 取绝对值得到最小夹角 */
    float angle_diff = (diff < 0.0f) ? -diff : diff;

    return (angle_diff < ROBOT_TURNING_JUJGUE_ANGLE_DEADBAND) ? 1 : 0;
}
void Robot_Tower_down(void)
{
    Emm_position_Mode(0x06,1000,0,135000,EMM_MODE_ABSOLUTE,EMM_SYNC_DISABLE);
}

void Robot_Tower_up(void)
{
    Emm_position_Mode(0x06,1000,0,0,EMM_MODE_ABSOLUTE,EMM_SYNC_DISABLE);
}
/**
 * @brief 机器人的时钟，在主循环里面10ms调用一次
 * 
 */
void Robot_tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = 0;

    /* 计算距离上次进入Robot_tick经过了多少个10ms周期 */
    if (robot_last_tick != 0)
    {
        elapsed = (now - robot_last_tick) / 10U;
    }
    robot_last_tick = now;

    /* 差速转弯由Robot_Angle_Update控制，转弯期间暂停状态机计时 */
    if (Robot_is_differential_turning())
    {
        robot_time = 0;
        return;
    }

    /* 没有在差速转弯时，累加状态机运行时间 */
    robot_time += elapsed;



    /* 初始状态：开始前进，然后进入发车后的直行计时 */
    if(robot_now_stage==ROBOT_STAGE_WAIT_FOR_START)
    {
        Robot_set_move(100,0,0);
        
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_START;
    }
    /* 发车后直行ROBOT_ROAD_TIME1，随后差速转到90度 */
    else if(robot_now_stage==ROBOT_STAGE_START && robot_time >= ROBOT_ROAD_TIME1)
    {
        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 90.0f,25);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TURN1;
    }
    /* 第一次差速转弯完成：开始第一段直行 */
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_TURN1)
    {
        Robot_set_move(100,0,0);
        
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_STRAGE1;
    }
    /* 第一段直行结束：差速转到0度，准备穿越障碍物 */
    else if(robot_now_stage==ROBOT_STAGE_STRAGE1 && robot_time >= ROBOT_ROAD_TIME2)
    {
        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 0.0f,25);
        //robot_angle=0; //旋转

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_THROUGH_OBSTACLE; //切换到穿过障碍物状态
    }
    /* 穿越障碍物前的差速转弯完成：先停车，进入等待状态 */
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_THROUGH_OBSTACLE)
    {
        Robot_set_move(0,0,0);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_THROUGH_OBSTACLE_WAIT;
    }
    /* 穿越障碍物前等待时间未到：持续保持停车 */
    // else if(robot_now_stage==ROBOT_STAGE_THROUGH_OBSTACLE_WAIT && robot_time < ROBOT_OBSTACLE_TURN_WAIT_TIME)
    // {
    //     Robot_set_move(0,0,0);
    // }
    /* 等待时间到：前进穿越障碍物，随后进入下一段直行计时 */
    else if(robot_now_stage==ROBOT_STAGE_THROUGH_OBSTACLE_WAIT && robot_time >= ROBOT_OBSTACLE_TURN_WAIT_TIME)
    {
        Robot_set_move(50,0,0);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TURN2;
    }
    /* 穿越障碍物后的直行结束：差速转到90度，进入第二条直道 */
    else if(robot_now_stage==ROBOT_STAGE_TURN2 && robot_time >= ROBOT_ROAD_TIME3)
    {
        //Robot_set_move(0,0,0);
        //robot_angle=90; //旋转
        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 90.0f,25);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_STRAGE2; //切换到第二条直道
    }

    //这个地方加扫二维码的
    /* 第二条直道前的差速转弯完成：后退，准备扫描二维码 */
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_STRAGE2)
    {
        Robot_set_move(-100,0,0);
        
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_SCAN_QR;
    }
    /* 扫描二维码后退时间结束：停车等待二维码处理 */
    else if(robot_now_stage==ROBOT_STAGE_SCAN_QR && robot_time >= ROBOT_QR_BACK_TIME)
    {
        Robot_set_move(0,0,0);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_QR_WAIT;
    }
    /* 二维码等待时间结束：开始向前行驶 */
    else if(robot_now_stage==ROBOT_STAGE_QR_WAIT && robot_time >= ROBOT_QR_WAIT_TIME)
    {
        Robot_set_move(100,0,0);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_QR_FORWARD;
    }
    /* 二维码后前进时间结束：继续前进，并进入下一次差速转弯计时 */
    else if(robot_now_stage==ROBOT_STAGE_QR_FORWARD && robot_time >= ROBOT_QR_FORWARD_TIME)
    {
        Robot_set_move(100,0,0);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TURN3;
    }
    /* 二维码后的直行结束之后再走ROBOT_ROAD_TIME4然后就差速转弯 */
    else if(robot_now_stage==ROBOT_STAGE_TURN3 && robot_time >= ROBOT_ROAD_TIME4)
    {
        // Robot_set_move(0,0,0);
        // robot_angle=0; //旋转
        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 0.0f,25);
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_STRAGE3;
    }
    /* 差速转弯完成：开始下一段直行 */
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_STRAGE3)
    {
        Robot_set_move(100,0,0);
        
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TURN4;
    }
    /* 下一段直行结束：差速转到270度 */
    else if(robot_now_stage==ROBOT_STAGE_TURN4 && robot_time >= ROBOT_ROAD_TIME5) //if里面写的是什么，这里干的就是什么事情
    {
        //Robot_set_move(0,0,0);
        //robot_angle=270; //旋转

        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 270.0f,25);

        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_STRAGE4;
    }
    /* 270度差速转弯完成：进入任务一 */
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_STRAGE4)
    {
        Robot_set_move(50,0,0);
        
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1;

        DMG6220_MoveAngle(&dmg6220_t, 190, 120.0f);
        Maxi_Send_cmd(0x01);
        //HAL_Delay(4);
        //Emm_position_Mode(0x06,-100,0,5000,EMM_MODE_RELATIVE_TO_PREVIOUS,EMM_SYNC_DISABLE);
    }
    /* 任务一：闭环目标小球的位置 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1)
    {
        //Robot_set_move(0,0,0);

        robot_time=0; //清零
        if(robot_task_1_state == 0)
        {
            DMG6220_MoveAngle(&dmg6220_t, 145, 120.0f);
            robot_task1_ball_stable_timing=0;
        }
        robot_task_1_state=1; //置标志位，走到任务一了

        if(Maxi_posion_data.state == 0) //如果有有效数据的话
        {
            float ball_error=(float)ROBOT_TASK1_BALL_TARGET-(float)Maxi_posion_data.x;

            Robot_set_vx(-PIDController_Update(&robot_task1_ball_pid_t,ROBOT_TASK1_BALL_TARGET,Maxi_posion_data.x));

            if(fabsf(ball_error) < ROBOT_TASK1_BALL_ERROR_TOLERANCE)
            {
                if(robot_task1_ball_stable_timing == 0)
                {
                    robot_task1_ball_stable_timing=1;
                    robot_task1_ball_stable_start_tick=now;
                }
                else if((now-robot_task1_ball_stable_start_tick) >= ROBOT_TASK1_BALL_STABLE_TIME_MS)
                {
                    //在目标值停留了一段时间
                    
                    Robot_set_vx(0);
                    //robot_task_1_state=2;
                    robot_time=0;
                    robot_now_stage=ROBOT_STAGE_TASK1_EXTEND_GRIPPER;
                    
                    DMG6220_MoveAngle(&dmg6220_t, 139, 120.0f);
                    HAL_Delay(10);
                    Robot_Claw_open();
                    HAL_Delay(10);
                    Robot_Tower_down();
                    HAL_Delay(10);
                    Robot_Claw_out1();
                   
                    

                    //重置maxi的数据，防止数据干扰
                    Maxi_posion_data.state = 1;
                }
            }
            else
            {
                robot_task1_ball_stable_timing=0;
            }
        }
        else
        {
            robot_task1_ball_stable_timing=0;
        }
    }
    
    /* 夹爪伸出后等待100个节拍，然后进入夹爪闭合状态 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_EXTEND_GRIPPER && robot_time >= ROBOT_TASK1_EXTEND_GRIPPER_WAIT_TIME)
    {
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_CLOSE_GRIPPER;
    }
    /* 执行夹爪闭合，然后等待1000ms再启动塔吊上升 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_CLOSE_GRIPPER)
    {
        Robot_Claw_close();
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_CLOSE_GRIPPER_WAIT;
    }
    /* 夹爪闭合1000ms后，启动塔吊上升 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_CLOSE_GRIPPER_WAIT && robot_time >= ROBOT_TASK1_CLOSE_GRIPPER_WAIT_TIME)
    {
        Robot_Tower_up();
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_TOWER_UP;
    }
    /* 塔吊上升1000ms后，执行Claw_out2并进入后退状态 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_TOWER_UP && robot_time >= ROBOT_TASK1_TOWER_UP_WAIT_TIME)
    {
        Robot_Claw_out2();
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_BACK_BY_POSITION;
    }
    /* 根据position选择后退时间，速度只设置一次，时间到停车 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_BACK_BY_POSITION)
    {
        if(robot_task1_back_started == 0)
        {
            if(Maxi_posion_data.position == 1)
            {
                robot_task1_back_time=ROBOT_TASK1_BACK_TIME1;
            }
            else if(Maxi_posion_data.position == 2)
            {
                robot_task1_back_time=ROBOT_TASK1_BACK_TIME2;
            }
            else if(Maxi_posion_data.position == 3)
            {
                robot_task1_back_time=ROBOT_TASK1_BACK_TIME3;
            }

            Robot_set_move(-30,0,0);
            robot_time=0; //清零
            robot_task1_back_started=1;
        }
        else if(robot_task1_back_started == 1 && robot_time >= robot_task1_back_time)
        {
            Robot_set_move(0,0,0);
            robot_time=0; //清零
            robot_task1_back_started=2;
            robot_now_stage=ROBOT_STAGE_TASK1_DMG_TO_BUCKET;
        }
    }
    /* 后退停车后，旋转DMG6220到排爆桶上方 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_DMG_TO_BUCKET && robot_task1_dmg_started == 0)
    {
        DMG6220_MoveAngle(&dmg6220_t, ROBOT_TASK1_BUCKET_DMG_ANGLE, 120.0f);
        robot_task1_dmg_started=1;
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_DMG_TO_BUCKET_WAIT;
    }
    /* DMG转到280后等待3000ms，再打开夹爪 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_DMG_TO_BUCKET_WAIT && robot_time >= ROBOT_TASK1_BUCKET_DMG_WAIT_TIME)
    {
        Robot_Claw_open();
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_OPEN_GRIPPER_WAIT;
    }
    /* 打开夹爪1000ms后，关闭夹爪并转动DMG6220到190 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_OPEN_GRIPPER_WAIT && robot_time >= ROBOT_TASK1_OPEN_GRIPPER_WAIT_TIME)
    {
        Robot_Claw_close();
        DMG6220_MoveAngle(&dmg6220_t, ROBOT_TASK1_DMG_190_ANGLE, 120.0f);
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_DMG_190_WAIT;
    }
    /* DMG转到190后等待500ms，再转动DMG6220到100 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_DMG_190_WAIT && robot_time >= ROBOT_TASK1_DMG_190_WAIT_TIME)
    {
        DMG6220_MoveAngle(&dmg6220_t, ROBOT_TASK1_DMG_100_ANGLE, 120.0f);
        Robot_Claw_in();
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_DMG_100_WAIT;
    }
    /* DMG转到100并收爪1000ms后，再开始前往下一个地方 */
    else if(robot_now_stage==ROBOT_STAGE_TASK1_DMG_100_WAIT && robot_time >= ROBOT_TASK1_DMG_100_WAIT_TIME)
    {
        Robot_set_move(80,0,0); //前往下一个地方
        robot_y_pid_is_open=1;
        robot_time=0; //清零
        robot_now_stage=ROBOT_STAGE_TASK1_DONE;
    }

    /* 任务一完成后的预留分支，现在为了方便调试任务二就直行短时间然后切换到真正的任务二*/
    else if(robot_now_stage==ROBOT_STAGE_TASK1_DONE && robot_time >=ROBOT_ROAD_TIME7)
    {
        Robot_set_move(50,0,0); //开始运动
        robot_y_pid_is_open=0;
        robot_now_stage=ROBOT_STAGE_TASK2; //切到任务二
        Maxi_Send_cmd(0x02); //让摄像机拍靶子
        
        Maxi_posion_data.state=1;
        robot_task_2_state=1;
        robot_task2_target_stable_timing=0;
        robot_task2_laser_on_timing=0;
        robot_time=0;
    }

    else if(robot_now_stage==ROBOT_STAGE_TASK2) //开始任务二
    {
        if(Maxi_posion_data.state==0)
        {
            float target_error=(float)ROBOT_TASK2_TARGET_TARGET-(float)Maxi_posion_data.x;

            Robot_set_vx(-PIDController_Update(&robot_task2_target_pid_t,ROBOT_TASK2_TARGET_TARGET,Maxi_posion_data.x));

            if(fabsf(target_error) < ROBOT_TASK2_TARGET_ERROR_TOLERANCE)
            {
                if(robot_task2_target_stable_timing == 0)
                {
                    robot_task2_target_stable_timing=1;
                    robot_task2_target_stable_start_tick=now;
                }
                else if((now-robot_task2_target_stable_start_tick) >= ROBOT_TASK2_TARGET_STABLE_TIME_MS)
                {
                    Robot_set_vx(0);
                    robot_task_2_state=2;
                    robot_now_stage=ROBOT_STAGE_TASK2_DONE;
                    robot_time=0;

                    //重置maxi的数据，防止数据干扰
                    Maxi_posion_data.state=1;
                }
            }
            else
            {
                robot_task2_target_stable_timing=0;
            }
        }
        else
        {
            robot_task2_target_stable_timing=0;
        }
    }

    else if(robot_now_stage==ROBOT_STAGE_TASK2_DONE)
    {
        if(robot_task2_laser_on_timing == 0)
        {
            Ray_on();
            robot_task2_laser_on_timing=1;
            robot_task2_laser_on_start_tick=now;
        }
        else if((now-robot_task2_laser_on_start_tick) >= ROBOT_TASK2_LASER_ON_TIME_MS)
        {
            Ray_off();
            robot_task2_laser_on_timing=0;
            robot_now_stage=ROBOT_STAGE_TO_TASK3_STRAIGHT1;
            robot_time=0;
        }
    }
    else if(robot_now_stage==ROBOT_STAGE_TO_TASK3_STRAIGHT1 && robot_time >= ROBOT_ROAD_TIME8)
    {
        Robot_start_differential_turn_ex(ROBOT_QUICK_TURN_REDIUS, 180.0f,25);

        robot_time=0;
        robot_now_stage=ROBOT_STAGE_TO_TASK3_STRAIGHT2; //切换状态
    }
    //转弯完成了
    else if(!Robot_is_differential_turning() && robot_now_stage==ROBOT_STAGE_TO_TASK3_STRAIGHT2)
    {
        Robot_set_move(50,0,0); //慢速直行
        DMG6220_MoveAngle(&dmg6220_t, 100, 120.0f);

        robot_time=0;
        robot_now_stage=ROBOT_STAGE_TASK3; //切换状态
        Maxi_Send_cmd(0x03); //切换目标
        Maxi_posion_data.state=1; //清零状态
        robot_task_3_state=1;
        robot_task3_hostage_stable_timing=0;
    }
    else if(robot_now_stage==ROBOT_STAGE_TASK3)
    {
        if(Maxi_posion_data.state==0)
        {
            float hostage_error=(float)ROBOT_TASK3_HOSTAGE_TARGET-(float)Maxi_posion_data.x;

            Robot_set_vx(-PIDController_Update(&robot_task3_hostage_pid_t,ROBOT_TASK3_HOSTAGE_TARGET,Maxi_posion_data.x));

            if(fabsf(hostage_error) < ROBOT_TASK3_HOSTAGE_ERROR_TOLERANCE)
            {
                if(robot_task3_hostage_stable_timing == 0)
                {
                    robot_task3_hostage_stable_timing=1;
                    robot_task3_hostage_stable_start_tick=now;
                }
                else if((now-robot_task3_hostage_stable_start_tick) >= ROBOT_TASK3_HOSTAGE_STABLE_TIME_MS)
                {
                    Robot_set_move(80,0,0); //设置指定的速度，前往终点
                    robot_task_3_state=2;
                    robot_time=0; //清零时间
                    robot_now_stage=ROBOT_STAGE_TASK3_DONE;

                    //重置maxi的数据，防止数据干扰
                    Maxi_posion_data.state=1;
                }
            }
            else
            {
                robot_task3_hostage_stable_timing=0;
            }
        }
        else
        {
            robot_task3_hostage_stable_timing=0;
        }
    }
    //前往终点
    else if(robot_now_stage==ROBOT_STAGE_TASK3_DONE && robot_time >= ROBOT_ROAD_TIME9)
    {
        Robot_set_move(0,0,0); //停在终点
        robot_now_stage=ROBOT_STAGE_END; //切换到结束状态
    }
}

void Robot_Claw_open(void)
{
    Servo_Set(135);
}

void Robot_Claw_close(void)
{
    Servo_Set(42);
}
void Robot_Claw_out1(void)
{
    Emm_position_Mode(0x05,80,0,1510,EMM_MODE_ABSOLUTE,EMM_SYNC_DISABLE);
}
void Robot_Claw_out2(void)
{
    Emm_position_Mode(0x05,80,0,2850,EMM_MODE_ABSOLUTE,EMM_SYNC_DISABLE);
}
void Robot_Claw_in(void)
{
    Emm_position_Mode(0x05,80,0,0,EMM_MODE_ABSOLUTE,EMM_SYNC_DISABLE);
}
