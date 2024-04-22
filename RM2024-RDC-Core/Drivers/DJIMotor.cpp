#include "DJIMotor.hpp"
// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif
namespace DJIMotor
{
// Initialize motor's controller instance
#define DJI_MOTOR_COUNT 2
#define maxCurrent 10000
DJIMotor motorFeedback[DJI_MOTOR_COUNT];
uint32_t mailbox;
uint8_t rxData[8];
uint8_t txData[8];
float temp;
HAL_StatusTypeDef status;
// CAN_FilterTypeDef filter[DJI_MOTOR_COUNT];
CAN_FilterTypeDef filterList = {0x201 << 5,
                                0x202 << 5,  // only two motors are used
                                0,
                                0,
                                CAN_FILTER_FIFO0,
                                0,
                                CAN_FILTERMODE_IDLIST,
                                CAN_FILTERSCALE_16BIT,
                                CAN_FILTER_ENABLE,
                                0};
CAN_TxHeaderTypeDef txHeader = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 4, DISABLE};
CAN_RxHeaderTypeDef rxHeader;
/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/

/**
 * @todo initializing can module
 */
void init()
{
    HAL_CAN_ConfigFilter(&hcan, &filterList);
    HAL_CAN_Start(&hcan);
}

// get feedback from motors
uint32_t fifoLevel;
uint32_t curFifoLevel;
void getRxMessage()
{
    // HAL_CAN_Stop(&hcan);
    // HAL_CAN_Start(&hcan);

    fifoLevel = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    for (int i = 0; i < fifoLevel; i++)
    {
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        curFifoLevel   = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
        uint16_t canID = rxHeader.StdId - 0x200; // determining the data from which motor ID

        DJIMotor *motor = &motorFeedback[canID - 1];
        motor->canID    = canID;

        motor->lastRotorAngle = motor->rotorAngle; // put rotorAngle received last time in lastRotorAngle

        motor->rotorAngle = rxData[0] << 8 | rxData[1]; // decode current rotorAngle

        motor->rpm = rxData[2] << 8 | rxData[3]; // decode current rpm

        motor->torqueCurrent = rxData[4] << 8 | rxData[5]; // decode current torque current
        /**
         * normailization of rotor angle
         * to get the total actual angular displacements of wheels
         * unit: degree
        */
        if (motor->rotorAngle - motor->lastRotorAngle > 4096)
        {
            motor->roundCount--;
            motor->angleChange =
                motor->rotorAngle - motor->lastRotorAngle - 8192;
        }
        else if (motor->rotorAngle - motor->lastRotorAngle < -4096)
        {
            motor->roundCount++;
            motor->angleChange =
                motor->rotorAngle - motor->lastRotorAngle + 8192;
        }
        else
        {
            motor->angleChange = motor->rotorAngle - motor->lastRotorAngle;
        }
        // motor->total_orientation += motor->angleChange;
        motor->total_orientation += motor->angleChange;
        motor->actual_angle = motor->total_orientation / (36 * 8192 / 360); // unit changing
    }
}

/**
 * @todo get the raw encoder from corresponding motor
 */
float getEncoder(uint16_t canID) {}

/**
 * @todo get the current rpm of corresponding motor
 */
float getRPM(uint16_t canID) { return motorFeedback[canID - 1].rpm; }

/**
 * @todo get the current total actual angular displacements of corresponding motor/wheel
 */
float getActualAngle(uint16_t canID)
{
    return motorFeedback[canID - 1].actual_angle;
}

/**
 * @todo initialize the total angular displacements of corresponding motor/wheel
*/
void initOrientation(uint16_t canID)
{
    motorFeedback[canID - 1].total_orientation = 0;
    motorFeedback[canID - 1].roundCount        = 0;
    motorFeedback[canID - 1].actual_angle      = 0;
    motorFeedback[canID - 1].initAngle         = 0;
}

/**
 * @todo set the initial angle (i.e. the 0 point) of corresponding motor/wheel
*/
void setInitAngle(uint16_t canID)
{
    motorFeedback[canID - 1].initAngle = motorFeedback[0].rotorAngle;
}

/**
 * @todo set the output current to be transmitted of corresponding motor
*/
void setOutput(float output, uint16_t canID)
{
    // clamping the output
    if (output > maxCurrent)
    {
        output = maxCurrent;
    }
    else if (output < -maxCurrent)
    {
        output = -maxCurrent;
    }
    temp = output;
    // uint8_t mask                = 0xff;
    txData[(canID - 1) * 2 + 1] = (static_cast<int>(output));
    txData[(canID - 1) * 2]     = (static_cast<int>(output) >> 8);
}

/**
 * @todo transmit the data to motors
 */
void transmit() { HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &mailbox); }
}  // namespace DJIMotor
#endif