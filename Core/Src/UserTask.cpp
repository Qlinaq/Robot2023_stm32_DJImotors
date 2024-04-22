/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2024 (EthenJ@outlook.sg)
 * @author GUO, Zilin
 * @brief RDC user task files
 * @version 0.3
 * @date 2022-10-20
 *
 * @copyright Copyright (c) 2024
 */
#include "AppConfig.h"   // Include our customized configuration
#include "DJIMotor.hpp"  // Include DR16
#include "DR16.hpp"
#include "FreeRTOS.h"  // Include FreeRTOS.h
#include "PID.hpp"     // Include PID
#include "Servo.hpp"
#include "main.h"
#include "task.h"  // Include task
#include "usart.h"

#define LEFT_Motor_ID 1
#define RIGHT_Motor_ID 2

/*Allocate the stack for our PID task*/
StackType_t uxPIDTaskStack[512];
/*Declare the PCB for our PID task*/
StaticTask_t xPIDTaskTCB;

/*Allocate the stack for our Receiving task*/
StackType_t uxRxTaskStack[512];
/*Declare the PCB for our Receiving task*/
StaticTask_t xRxTaskTCB;

// W1NT3R

// define sensors data variables
static volatile int leftSensor;
static volatile int rightSensor;

// initialize PID parameters, 0 stands for displacement ring, 1 stands for
// velocity ring
static volatile float kp0 = 10.0f;
static volatile float ki0 = 1.0f;
static volatile float kd0 = 0.0f;
static volatile float kp1 = 10.0f;
static volatile float ki1 = 1.0f;
static volatile float kd1 = 0.0f;

static volatile uint16_t canID = 0;
Control::PID pid[2]{{kp0, ki0, kd0}, {kp1, ki1, kd1}};

static volatile float target = 2000.0;  // rpm of the wheels during line
                                        // tracking
static volatile float targetCurrent[2];  // output current to be transmitted
static int progress = 5;                 // indicating the current progress
/*
1 -> line tracking
2 -> go forward with a distance "distance = 375.0" unit: mm
3 -> turning and dropball with angle "timebase = 450" and pattern
"bucketsPattern[4] from bluetooth" 4 -> wait for data of buckets pattern from
bluetooth 5 -> wait for start of the ar Sequence: 5 -> 2 -> 1 -> 2 -> 4 -> 3
*/
static uint8_t buffer[4];  // buffer for UART data (bluetooth)

// timeing for delay after pressing PB6 button, otherwise your finger go with ar
uint32_t curTick;
uint32_t startTick;

static volatile uint8_t
    bucketsPattern[4];  // storing the buckets pattern, eg 1100 stands for
                        // blue/red blue/red green green
static volatile int count    = 0;      // counting variable
static volatile bool next    = false;  // determining if first ball is dropped
static volatile int timebase = 450;    // time for rotating angle
static volatile float distance =
    375.0;                          // required forward distance in progress 2
static volatile bool start = true;  // determining if progress 2 is first
                                    // entered
static volatile bool run = false;  // temporary variable for testing switching
                                   // progress, i.e. can be deleted

/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
void ball1() // drop ball 1
{
    Servo::intermediate();
    vTaskDelay(1000);
}
void ball2() // drop ball 2
{
    Servo::drop_second_ball();
    vTaskDelay(1000);
}
void dropBall() // dropping function if current bucket is detected as blue/red
{
    if (!next)
    {
        ball1();
        next = true;
        count++;
    }
    else
    {
        ball2();
        Servo::drop_first_ball();
        count++;
    }
}

void userTask(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/
    HAL_UART_Receive_IT(&huart2, buffer, 4); // start receiving UART data (bluetooth) with ISR
    DJIMotor::getRxMessage(); // receive data from motors in first turn
    // the following idk why should be implemented but the ar has problem if not do so
    progress = 5;
    start    = true; 
    count    = 0;
    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        switch (progress)
        {
        case 1:  // lineTracking
            // the following part should implement the logic control based on IR
            // sensor
            // feedback int readIRSensor(); //should return two int variable,
            // left and right int left, right; //should be either 1 or 0
            // sensor:0 -> white  sensor:1 -> black
            /*
            logic:
            left = 0, right = 0: motor1 = 1, motor2 = 1; i.e. move forward
            left = 1, right = 0: motor1 = 0, motor2 = 1; i.e. turn left
            left = 0, right = 1: motor1 = 1, motor2 = 0; i.e. turn right
            left = 1, right = 1: motor1 = 0, motor2 = 0; i.e. stop
            */

            //read IR line tracking sensor data, 0 -> white 1 -> black
            leftSensor  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
            rightSensor = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

            //line tracking logic
            if (!leftSensor && !rightSensor) // go forward
            {
                targetCurrent[0] =
                    pid[1].update(target, DJIMotor::getRPM(LEFT_Motor_ID));
                targetCurrent[1] =
                    pid[1].update(-target, DJIMotor::getRPM(RIGHT_Motor_ID));
            }
            else if (leftSensor && !rightSensor) // go left
            {
                targetCurrent[0] =
                    pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                targetCurrent[1] =
                    pid[1].update(-target, DJIMotor::getRPM(RIGHT_Motor_ID));
            }
            else if (rightSensor && !leftSensor) // go right
            {
                targetCurrent[0] =
                    pid[1].update(target, DJIMotor::getRPM(LEFT_Motor_ID));
                targetCurrent[1] =
                    pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
            }
            else // detected black line, stop and switch progress
            {
                targetCurrent[0] =
                    pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                targetCurrent[1] =
                    pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 && // switch progress after stopping
                    DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                {
                    targetCurrent[0] = 0;
                    targetCurrent[1] = 0;
                    progress         = 2;
                    count            = 0;
                    DJIMotor::initOrientation(LEFT_Motor_ID); // initialize motorFeedback data
                    DJIMotor::initOrientation(RIGHT_Motor_ID);
                }
            }
            // set corresponding output current for motors and transmit
            for (canID = 1; canID <= 2; canID++)
            {
                DJIMotor::setOutput(targetCurrent[canID - 1], canID);
            }
            DJIMotor::transmit();
            break;
        case 2:  // forward to buckets
            targetCurrent[0] = pid[1].update(
                pid[0].update(distance / (67 * 3.1415926) * 360,
                              DJIMotor::getActualAngle(LEFT_Motor_ID)),
                DJIMotor::getRPM(LEFT_Motor_ID));
            targetCurrent[1] = pid[1].update(
                pid[0].update(-distance / (67 * 3.1415926) * 360,
                              DJIMotor::getActualAngle(RIGHT_Motor_ID)),
                DJIMotor::getRPM(RIGHT_Motor_ID));
            if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 && // switch progress after stopping
                DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
            {
                count++; // delay to prevent accidental progress switch
                if (count >= 500)
                {
                    run              = 0;
                    targetCurrent[0] = 0;
                    targetCurrent[1] = 0;
                    count            = 0;
                    if (start) // determining next progress
                    {
                        start    = false;
                        progress = 1;
                    }
                    else
                    {
                        progress = 4;
                    }
                }
            }
            // set corresponding output current for motors and transmit
            for (canID = 1; canID <= 2; canID++)
            {
                DJIMotor::setOutput(targetCurrent[canID - 1], canID);
            }
            DJIMotor::transmit();

            break;
        case 3:  // ball
            // rotates to the leftmost first i.e. <= (3 * timebase)
            // then return to the middle i.e. <= (6 * timebase)
            // and last rotates to the rightmost i.e. <= (9 * timebase)
            // detection when rotates to the middle of corresponding buckets
            if (count <= (3 * timebase))
            {
                if (count == (3 * timebase) && bucketsPattern[0] == 1)
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 &&
                        DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                    {
                        targetCurrent[0] = 0;
                        targetCurrent[1] = 0;
                        dropBall();
                    }
                }
                else
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(-1000, DJIMotor::getRPM(RIGHT_Motor_ID));
                    count++;
                }
            }
            else if (count <= (6 * timebase))
            {
                if (count == (5 * timebase) && bucketsPattern[1] == 1)
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 &&
                        DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                    {
                        targetCurrent[0] = 0;
                        targetCurrent[1] = 0;
                        dropBall();
                    }
                }
                else if (count == (6 * timebase))
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 &&
                        DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                    {
                        targetCurrent[0] = 0;
                        targetCurrent[1] = 0;
                        count++;
                    }
                }
                else
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(1000, DJIMotor::getRPM(RIGHT_Motor_ID));
                    count++;
                }
            }
            else if (count <= (9 * timebase))
            {
                if (count == (7 * timebase) && bucketsPattern[2] == 1)
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 &&
                        DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                    {
                        targetCurrent[0] = 0;
                        targetCurrent[1] = 0;
                        dropBall();
                    }
                }
                else if (count == (9 * timebase) && bucketsPattern[3] == 1)
                {
                    targetCurrent[0] =
                        pid[1].update(0, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    if (DJIMotor::getRPM(LEFT_Motor_ID) == 0 &&
                        DJIMotor::getRPM(RIGHT_Motor_ID) == 0)
                    {
                        targetCurrent[0] = 0;
                        targetCurrent[1] = 0;
                        dropBall();
                    }
                }
                else
                {
                    targetCurrent[0] =
                        pid[1].update(1000, DJIMotor::getRPM(LEFT_Motor_ID));
                    targetCurrent[1] =
                        pid[1].update(0, DJIMotor::getRPM(RIGHT_Motor_ID));
                    count++;
                }
            }
            else // finish all progress
            {
                targetCurrent[0] = 0;
                targetCurrent[1] = 0;
            }
            // set corresponding output current for motors and transmit
            for (canID = 1; canID <= 2; canID++)
            {
                DJIMotor::setOutput(targetCurrent[canID - 1], canID);
            }
            DJIMotor::transmit();
            // progress = 4;
            break;
        case 4:  // wait for bluetooth data, switch only when data received is reasonable with 2 elements in bucketsPattern[] are 1
            if (bucketsPattern[0] + bucketsPattern[1] + bucketsPattern[2] +
                    bucketsPattern[3] ==
                2)
            {
                progress = 3;
            }
            break;
        case 5:  // wait for starting (button)
            // if (bucketsPattern[0] + bucketsPattern[1] + bucketsPattern[2] +
            //         bucketsPattern[3] ==
            // 4)
            curTick = HAL_GetTick(); // start timing
            if (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == 1) // detect button pressing
            {
                run       = true;
                startTick = HAL_GetTick();
            }
            if (run && (curTick - startTick > 1000)) // delay for 1 second after pressing button
            {
                // vTaskDelay(1000);
                progress = 2;
                count    = 0;
                DJIMotor::initOrientation(LEFT_Motor_ID);
                DJIMotor::initOrientation(RIGHT_Motor_ID);
                // targetCurrent[0] = 0;
                // targetCurrent[1] = 0;
            }
            break;
        default:
            break;
        }  // switch close
        vTaskDelay(1);
        /* Your user layer codes in loop end here*/
        /*=================================================*/
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 * receive motor feedback data continuously
 */
void RxDataReceive(void *)
{
    while (true)
    {
        DJIMotor::getRxMessage();
        vTaskDelay(1);
    }
}

/**
 * @todo decoding after receiving UART data from bluetooth
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    bucketsPattern[0] = buffer[0] - 48;
    bucketsPattern[1] = buffer[1] - 48;
    bucketsPattern[2] = buffer[2] - 48;
    bucketsPattern[3] = buffer[3] - 48;
    //HAL_UART_Receive_IT(&huart2, buffer, 4);
}

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
 */
void startUserTasks()
{
    DJIMotor::init();  // Initalize the DJIMotor driver
    DR16::init();      // Intialize the DR16 driver
    Servo::ServoInit();
    DJIMotor::initOrientation(LEFT_Motor_ID);
    DJIMotor::initOrientation(RIGHT_Motor_ID);

    // HAL_UART_RegisterRxEventCallback(&huart2, rxEventCallback);
    xTaskCreateStatic(userTask,
                      "user_default ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      10,
                      uxPIDTaskStack,
                      &xPIDTaskTCB);

    xTaskCreateStatic(RxDataReceive,
                      "user_default ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      15,
                      uxRxTaskStack,
                      &xRxTaskTCB);  // Add the main task into the scheduler
}