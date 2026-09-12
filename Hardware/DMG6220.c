#include "DMG6220.h"

#include <string.h>

#define DMG6220_BITRATE 1000000U

/* This board runs CAN1 from an 8 MHz APB1 clock. */
#ifndef DMG6220_CAN_CLOCK_HZ
#define DMG6220_CAN_CLOCK_HZ 8000000U
#endif

#define DMG6220_POSITION_VELOCITY_ID_OFFSET 0x100U
#define DMG6220_VELOCITY_ID_OFFSET 0x200U

static float DMG6220_Clamp(float value, float minimum, float maximum)
{
    if (!(value > minimum))
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static uint16_t DMG6220_FloatToUint(float value, float minimum,
                                    float maximum, uint8_t bits)
{
    float span;
    uint32_t max_raw;

    value = DMG6220_Clamp(value, minimum, maximum);
    span = maximum - minimum;
    max_raw = (1UL << bits) - 1UL;

    return (uint16_t)((value - minimum) * (float)max_raw / span);
}

static float DMG6220_UintToFloat(uint16_t value, float minimum,
                                 float maximum, uint8_t bits)
{
    float span = maximum - minimum;
    uint32_t max_raw = (1UL << bits) - 1UL;

    return (float)value * span / (float)max_raw + minimum;
}

static uint32_t DMG6220_TimeSeg1(uint32_t time_quanta)
{
    switch (time_quanta)
    {
        case 1U: return CAN_BS1_1TQ;
        case 2U: return CAN_BS1_2TQ;
        case 3U: return CAN_BS1_3TQ;
        case 4U: return CAN_BS1_4TQ;
        case 5U: return CAN_BS1_5TQ;
        case 6U: return CAN_BS1_6TQ;
        case 7U: return CAN_BS1_7TQ;
        case 8U: return CAN_BS1_8TQ;
        case 9U: return CAN_BS1_9TQ;
        case 10U: return CAN_BS1_10TQ;
        case 11U: return CAN_BS1_11TQ;
        case 12U: return CAN_BS1_12TQ;
        case 13U: return CAN_BS1_13TQ;
        case 14U: return CAN_BS1_14TQ;
        case 15U: return CAN_BS1_15TQ;
        case 16U: return CAN_BS1_16TQ;
        default: return 0U;
    }
}

static uint32_t DMG6220_TimeSeg2(uint32_t time_quanta)
{
    switch (time_quanta)
    {
        case 1U: return CAN_BS2_1TQ;
        case 2U: return CAN_BS2_2TQ;
        case 3U: return CAN_BS2_3TQ;
        case 4U: return CAN_BS2_4TQ;
        case 5U: return CAN_BS2_5TQ;
        case 6U: return CAN_BS2_6TQ;
        case 7U: return CAN_BS2_7TQ;
        case 8U: return CAN_BS2_8TQ;
        default: return 0U;
    }
}

static HAL_StatusTypeDef DMG6220_ConfigureCan(CAN_HandleTypeDef *hcan,
                                              uint8_t master_id)
{
    CAN_FilterTypeDef filter = {0};
    uint32_t pclk1 = DMG6220_CAN_CLOCK_HZ;
    uint32_t prescaler = 0U;
    uint32_t time_seg1 = 0U;
    uint32_t time_seg2 = 0U;
    uint32_t candidate;

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    for (candidate = 1U; candidate <= 1024U; candidate++)
    {
        uint32_t divisor = DMG6220_BITRATE * candidate;
        uint32_t candidate_tq;
        uint32_t candidate_seg2;
        uint32_t candidate_seg1;

        if ((pclk1 % divisor) != 0U)
        {
            continue;
        }

        candidate_tq = pclk1 / divisor;
        if ((candidate_tq < 4U) || (candidate_tq > 24U))
        {
            continue;
        }

        candidate_seg2 = candidate_tq / 4U;
        if (candidate_seg2 < 1U)
        {
            candidate_seg2 = 1U;
        }
        if (candidate_seg2 > 8U)
        {
            candidate_seg2 = 8U;
        }

        candidate_seg1 = candidate_tq - 1U - candidate_seg2;
        if ((candidate_seg1 < 1U) || (candidate_seg1 > 16U))
        {
            continue;
        }

        prescaler = candidate;
        time_seg1 = DMG6220_TimeSeg1(candidate_seg1);
        time_seg2 = DMG6220_TimeSeg2(candidate_seg2);
        break;
    }

    if ((prescaler == 0U) || (time_seg1 == 0U) || (time_seg2 == 0U))
    {
        return HAL_ERROR;
    }

    hcan->Instance = CAN1;
    if (HAL_CAN_DeInit(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hcan->Init.Prescaler = prescaler;
    hcan->Init.Mode = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1 = time_seg1;
    hcan->Init.TimeSeg2 = time_seg2;
    hcan->Init.TimeTriggeredMode = DISABLE;
    hcan->Init.AutoBusOff = ENABLE;
    hcan->Init.AutoWakeUp = DISABLE;
    hcan->Init.AutoRetransmission = ENABLE;
    hcan->Init.ReceiveFifoLocked = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    filter.FilterIdHigh = (uint16_t)(((uint32_t)master_id << 5U) & 0xFFFFU);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint16_t)((0x7FFUL << 5U) & 0xFFFFU);
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef DMG6220_TransmitFrame(DMG6220 *motor,
                                               uint32_t standard_id,
                                               const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;

    if ((motor == NULL) || (motor->hcan == NULL) || (data == NULL) ||
        (standard_id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    tx_header.StdId = standard_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(motor->hcan, &tx_header, (uint8_t *)data,
                               &mailbox);
}

static void DMG6220_WriteFloatLittleEndian(uint8_t output[4], float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    output[0] = (uint8_t)bits;
    output[1] = (uint8_t)(bits >> 8U);
    output[2] = (uint8_t)(bits >> 16U);
    output[3] = (uint8_t)(bits >> 24U);
}

HAL_StatusTypeDef DMG6220_Init(DMG6220 *motor, CAN_HandleTypeDef *hcan,
                              uint8_t can_id, uint8_t master_id)
{
    if ((motor == NULL) || (hcan == NULL) || (can_id > 0x0FU))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->can_id = can_id;
    motor->master_id = master_id;
    motor->position_max = DMG6220_DEFAULT_P_MAX;
    motor->velocity_max = DMG6220_DEFAULT_V_MAX;
    motor->torque_max = DMG6220_DEFAULT_T_MAX;

    return DMG6220_ConfigureCan(hcan, master_id);
}

HAL_StatusTypeDef DMG6220_SendCommand(DMG6220 *motor,
                                     DMG6220_Command command)
{
    uint8_t data[8];
    uint8_t tail;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    switch (command)
    {
        case DMG6220_CMD_ENABLE:
            tail = 0xFCU;
            break;
        case DMG6220_CMD_DISABLE:
            tail = 0xFDU;
            break;
        case DMG6220_CMD_SAVE_ZERO:
            tail = 0xFEU;
            break;
        case DMG6220_CMD_CLEAR_ERROR:
            tail = 0xFBU;
            break;
        default:
            return HAL_ERROR;
    }

    memset(data, 0xFF, sizeof(data));
    data[7] = tail;

    return DMG6220_TransmitFrame(motor, motor->can_id, data);
}

HAL_StatusTypeDef DMG6220_SendMit(DMG6220 *motor, float position,
                                 float velocity, float kp, float kd,
                                 float torque)
{
    uint8_t data[8];
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque_raw;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    position_raw = DMG6220_FloatToUint(position, -motor->position_max,
                                       motor->position_max, 16U);
    velocity_raw = DMG6220_FloatToUint(velocity, -motor->velocity_max,
                                       motor->velocity_max, 12U);
    kp_raw = DMG6220_FloatToUint(kp, 0.0f, 500.0f, 12U);
    kd_raw = DMG6220_FloatToUint(kd, 0.0f, 5.0f, 12U);
    torque_raw = DMG6220_FloatToUint(torque, -motor->torque_max,
                                     motor->torque_max, 12U);

    data[0] = (uint8_t)(position_raw >> 8U);
    data[1] = (uint8_t)position_raw;
    data[2] = (uint8_t)(velocity_raw >> 4U);
    data[3] = (uint8_t)(((velocity_raw & 0x0FU) << 4U) | (kp_raw >> 8U));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4U);
    data[6] = (uint8_t)(((kd_raw & 0x0FU) << 4U) | (torque_raw >> 8U));
    data[7] = (uint8_t)torque_raw;

    return DMG6220_TransmitFrame(motor, motor->can_id, data);
}

HAL_StatusTypeDef DMG6220_SendPositionVelocity(DMG6220 *motor,
                                              float position,
                                              float velocity)
{
    uint8_t data[8];

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    DMG6220_WriteFloatLittleEndian(&data[0], position);
    DMG6220_WriteFloatLittleEndian(&data[4], velocity);

    return DMG6220_TransmitFrame(
        motor, (uint32_t)motor->can_id + DMG6220_POSITION_VELOCITY_ID_OFFSET,
        data);
}

HAL_StatusTypeDef DMG6220_SendVelocity(DMG6220 *motor, float velocity)
{
    uint8_t data[8] = {0};

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    DMG6220_WriteFloatLittleEndian(&data[0], velocity);

    return DMG6220_TransmitFrame(
        motor, (uint32_t)motor->can_id + DMG6220_VELOCITY_ID_OFFSET, data);
}

static void DMG6220_UpdateFeedback(DMG6220 *motor, const uint8_t data[8])
{
    DMG6220_Feedback *feedback = &motor->feedback;

    feedback->motor_id = data[0] & 0x0FU;
    feedback->state = data[0] >> 4U;
    feedback->position_raw = (uint16_t)(((uint16_t)data[1] << 8U) | data[2]);
    feedback->velocity_raw = (uint16_t)(((uint16_t)data[3] << 4U) |
                                       (data[4] >> 4U));
    feedback->torque_raw = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8U) |
                                     data[5]);
    feedback->position = DMG6220_UintToFloat(feedback->position_raw,
                                             -motor->position_max,
                                             motor->position_max, 16U);
    feedback->velocity = DMG6220_UintToFloat(feedback->velocity_raw,
                                             -motor->velocity_max,
                                             motor->velocity_max, 12U);
    feedback->torque = DMG6220_UintToFloat(feedback->torque_raw,
                                           -motor->torque_max,
                                           motor->torque_max, 12U);
    feedback->temperature_mos = data[6];
    feedback->temperature_rotor = data[7];
    feedback->last_rx_tick = HAL_GetTick();
    feedback->rx_count++;
}

uint32_t DMG6220_Poll(DMG6220 *motor)
{
    uint32_t received = 0U;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return 0U;
    }

    while (HAL_CAN_GetRxFifoFillLevel(motor->hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef rx_header = {0};
        uint8_t data[8];

        if (HAL_CAN_GetRxMessage(motor->hcan, CAN_RX_FIFO0, &rx_header,
                                 data) != HAL_OK)
        {
            motor->rx_error_count++;
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA) ||
            (rx_header.DLC != 8U) || (rx_header.StdId != motor->master_id) ||
            ((data[0] & 0x0FU) != motor->can_id))
        {
            continue;
        }

        DMG6220_UpdateFeedback(motor, data);
        received++;
    }

    return received;
}

HAL_StatusTypeDef DMG6220_WaitFeedback(DMG6220 *motor, uint32_t timeout_ms)
{
    uint32_t start_tick;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    do
    {
        if (DMG6220_Poll(motor) > 0U)
        {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return HAL_TIMEOUT;
}

uint8_t DMG6220_IsOnline(const DMG6220 *motor, uint32_t timeout_ms)
{
    if ((motor == NULL) || (motor->feedback.rx_count == 0U))
    {
        return 0U;
    }

    return ((HAL_GetTick() - motor->feedback.last_rx_tick) <= timeout_ms) ?
           1U : 0U;
}
