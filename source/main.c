/*
 * example_mpu_task.c
 *
 *  Created on: Nov 7, 2024
 *      Author: manto
 */

/** @example example_freeRTOSRestrictedTask.c
*  This is an example which descibes the steps to create an example application which
*  toggles the High End Timer (HET) pin 17 (LED in USB & HDK) based on the FreeRTOS timer tick of one second.
*
*  @b Step @b 1:
*
*  Create a new project.
*
*  Navigate: -> File -> New -> Project
*
*  @image html example_createProject.jpg "Figure: Create a new Project"
*
*  @b Step @b 2:
*
*  Configure driver code generation:
*  - Enable GIO driver
*  - Disable others
*
*  Navigate: -> TMS570LSxx /RM4 -> Driver Enable
*
*  @image html example_freeRTOSBlinky_enableDrivers_TMS570LS3x-RMx.jpg "Figure: Driver Configuration"
*
*  @b Step @b 3:
*
*  Configure Interrupt handling:
*  - Enable SVC
*  - Enter FreeRTOS SVC handler name 'vPortSWI'
*
*  Navigate: -> TMS570LSxx /RM4 -> Interrupts
*
*  @image html example_freeRTOSBlinky_interrupts.jpg "Figure: Interrupt Configuration"
*
*  @b Step @b 4:
*
*  Configure VIM RAM:
*  - Enter FreeRTOS Timer Tick handler name 'vPortPreemptiveTick' at offset 0x0000000C
*  - Enter SSI handler name 'vPortYeildWithinAPI ' at offset 0x00000058
*
*  Navigate: -> TMS570LSxx /RM4 -> VIM RAM
*
*  @image html example_freeRTOSBlinky_VimRam.jpg "Figure: VIM RAM Configuration"
*
*  @b Step @b 5:
*
*  Configure Vectored Interrupt Module Channels:
*  - Enable VIM Channel 2
*  - Map VIM Channel 2 to IRQ
*  - Enable VIM Channel 21
*  - Map VIM Channel 21 to IRQ
*
*  Navigate: -> TMS570LSxx /RM4 -> VIM Channel 0-31
*
*  @image html example_freeRTOSBlinky_vimChannelView.jpg "Figure: VIM Channel Configuration"
*
*  @b Step @b 6:
*
*  Configure OS timer tick to 1 ms:
*  - Enter Tick Rate of 1000
*
*  Navigate: -> OS -> General
*
*  @image html example_freeRTOSBlinky_osGeneral.jpg "Figure: OS General Configuration"
*
*  @b Step @b 7:
*
*  Generate code
*
*  Navigate: -> File -> Generate Code
*
*  @image html example_freeRTOS_generateCode.jpg "Figure: Generate Code"
*
*  @b Step @b 8:
*
*  Copy source code below into your application.
*
*  The example file example_freeRTOSBlinky.c can also be found in the examples folder: ../HALCoGen/examples
*
*  @note HALCoGen generates an enpty main function in sys_main.c,
*        please make sure that you link in the right main function or copy the source into the user code sections of this file.
*
*  @note Enable GCC extension in the CCS project (Project properties -> Build -> ARM Compiler -> Advanced options -> Language options -> Enable support for GCC extensions)
*
*/

/*
* Copyright (C) 2009-2015 Texas Instruments Incorporated - www.ti.com
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"
#include "system.h"

/* USER CODE BEGIN (1) */
/* Include FreeRTOS scheduler files */
#include "FreeRTOS.h"
#include "os_task.h"

/* Include HET header file - types, definitions and function declarations for system driver */
#include "het.h"
#include "esm.h"
#include "gio.h"
#include "rti.h"
#include <math.h>
#include "source/core_main.c"

uint8   emacAddress[6U] =   {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
uint32  emacPhyAddress  =   0U;

void CoreMarkTask(void *pvParameters) {
    for(;;) {
        main();
        break;

    }

    vTaskDelete(NULL);
}

static TaskHandle_t xFlightCtrl_Handle = NULL;
static TaskHandle_t xSensor1_Handle = NULL;
static TaskHandle_t xRCctrl_Handle = NULL;
static TaskHandle_t xComms_Handle = NULL;
static TaskHandle_t xCmark_Handle = NULL;

void FlightCtrl_Task(void *pvParameters) {
    static float prev_error = 0.0f;
    static float integral = 0.0f;
    const float dt = 0.01f;  // 10ms loop
    const float Kp = 1.2f, Ki = 0.1f, Kd = 0.5f;

    static float prev_angle = 0.0f;
    const float alpha = 0.98f;

    for (;;) {
        // Simulate sensor input
        float error = 0.5f;  // Example error
        float gyro_rate = 0.02f;  // Example gyro data (rad/s)
        float accel_angle = 1.5f;  // Example accelerometer angle (degrees)

        // PID Control
        float derivative = (error - prev_error) / dt;
        integral += error * dt;
        float output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;

        // Angle Estimation with Sensor Fusion
        float gyro_angle = prev_angle + gyro_rate * dt;
        prev_angle = alpha * gyro_angle + (1 - alpha) * accel_angle;

        // Enforce 10ms loop period using a hardware timer or RTOS delay
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void Sensor_Task(void *pvParameters) {
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 9.81f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.02f;
    //TC_WDOG_INTERFER();
    //TC_EMAC_INTERFER();

    for (;;) {
        // Simulate accelerometer & gyroscope readings
        accel_x = sinf(gyro_x) * 9.81f;
        accel_y = cosf(gyro_y) * 9.81f;
        accel_z = 9.81f + 0.01f * gyro_z;

        gyro_x += 0.001f;
        gyro_y += 0.002f;
        gyro_z += 0.0015f;

        // Enforce 300ms sampling interval
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void RCctrl_Task(void *pvParameters) {
    float throttle = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    for (;;) {
        // Simulate receiving commands from the remote controller
        throttle = (throttle >= 100.0f) ? 0.0f : throttle + 0.5f;
        roll = sinf(throttle) * 30.0f;
        pitch = cosf(throttle) * 30.0f;
        yaw = atan2f(roll, pitch);

        // Enforce 40ms update rate
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void Comms_Task(void *pvParameters) {
    int packet_counter = 0;
    float signal_strength = -50.0f;  // RSSI in dBm

    for (;;) {
        // Simulate sending telemetry data
        packet_counter++;
        signal_strength += (packet_counter % 10 == 0) ? -0.1f : 0.05f;

        // Simulate receiving a control packet
        if (packet_counter % 20 == 0) {
            float received_value = 1.0f / (float)packet_counter;
        }

        // Enforce 100ms communication interval
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#pragma DATA_ALIGN(FlightCtrl_Stack, 256 * sizeof(portSTACK_TYPE))
#pragma DATA_ALIGN(Sensor1_Stack, 256 * sizeof(portSTACK_TYPE))
#pragma DATA_ALIGN(RCctrl_Stack, 256 * sizeof(portSTACK_TYPE))
#pragma DATA_ALIGN(Comms_Stack, 256 * sizeof(portSTACK_TYPE))

static portSTACK_TYPE FlightCtrl_Stack[256] __attribute__ ((aligned (256 * sizeof(portSTACK_TYPE))));
static portSTACK_TYPE Sensor1_Stack[256] __attribute__ ((aligned (256 * sizeof(portSTACK_TYPE))));
static portSTACK_TYPE RCctrl_Stack[256] __attribute__ ((aligned (256 * sizeof(portSTACK_TYPE))));
static portSTACK_TYPE Comms_Stack[256] __attribute__ ((aligned (256 * sizeof(portSTACK_TYPE))));
char cReadyOnlyArray[ 32 ] __attribute__((aligned(32)));

static portSTACK_TYPE Cmark_Stack[256*3] __attribute__ ((aligned ((128) * sizeof(portSTACK_TYPE))));

static const xTaskParameters xFlightCtrl_Params = {
    FlightCtrl_Task, "FlightCtrl", 128, NULL, (6 | portPRIVILEGE_BIT), FlightCtrl_Stack,
    {{cReadyOnlyArray,32,portMPU_REGION_READ_ONLY}, {0,0,0}, {0,0,0}}
};

static const xTaskParameters xSensor1_Params = {
    Sensor_Task, "Sensor1", 128, NULL, 5, Sensor1_Stack,
    {{cReadyOnlyArray,32,portMPU_REGION_READ_ONLY}, {0,0,0}, {0,0,0}}
};

static const xTaskParameters xRCctrl_Params = {
    RCctrl_Task, "RCctrl", 128, NULL, 5, RCctrl_Stack,
    {{cReadyOnlyArray,32,portMPU_REGION_READ_ONLY}, {0,0,0}, {0,0,0}}
};

static const xTaskParameters xComms_Params = {
    Comms_Task, "Comms", 128, NULL, 3, Comms_Stack,
    {{cReadyOnlyArray,32,portMPU_REGION_READ_ONLY}, {0,0,0}, {0,0,0}}
};

static const xTaskParameters xCmark_Params = {
    CoreMarkTask, "Cmark", 256*3, NULL, (2 | portPRIVILEGE_BIT), Cmark_Stack,
    {{cReadyOnlyArray,32,portMPU_REGION_READ_ONLY}, {0,0,0}, {0,0,0}}
};

void os_main(void)
{
    _enable_IRQ();

    if (xTaskCreateRestricted(&xFlightCtrl_Params, &xFlightCtrl_Handle) != pdTRUE) while(1);
    if (xTaskCreateRestricted(&xSensor1_Params, &xSensor1_Handle) != pdTRUE) while(1);
    if (xTaskCreateRestricted(&xRCctrl_Params, &xRCctrl_Handle) != pdTRUE) while(1);
    if (xTaskCreateRestricted(&xComms_Params, &xComms_Handle) != pdTRUE) while(1);
    if (xTaskCreateRestricted(&xCmark_Params, &xCmark_Handle) != pdTRUE) while (1);

    /* Start Scheduler */
    vTaskStartScheduler();

    /* Run forever */
    while(1);
/* USER CODE END */
}


void rtiNotification(uint32 not) {
    int z = 0;
    z += 1;
}

void EMACCore0RxIsr() {
    return;
}

void EMACCore0TxIsr() {
    return;
}

void custom_dabort() {
    return;
}
