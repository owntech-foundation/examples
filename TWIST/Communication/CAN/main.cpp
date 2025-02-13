/*
 * Copyright (c) 2021-present OwnTech Technologies
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This example deploy a simple CAN based communication using
 *         Thingset library.
 *
 * @author Jean Alinei <jean.alinei@owntech.io>
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "user_data_objects.h"
#include <thingset/can.h>
#include "CommunicationAPI.h"


/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */
float32_t control_reference;
float32_t received_value;
bool      received_start_stop;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your hardware and tasks.
 *
 * In this default main, we only spawn two tasks
 *  - A background task.
 *  - A critical task is defined but not started.
 *
 * NOTE: It is important to follow the steps and initialize the hardware first
 * and the tasks second.
 */
void setup_routine()
{
    /* STEP 1 - SETUP THE HARDWARE */

    /* Buck voltage mode */
    shield.power.initBuck(ALL);
    shield.sensors.enableDefaultTwistSensors();

    /* Use functions below to enable or disable
       - control over CAN
       - measure broadcasting over CAN
     */
    communication.can.setCtrlEnable(false);
    communication.can.setBroadcastEnable(false);

    /* STEP 2 - SETUP THE TASKS */
    /* Control frames are not sent by default.*/
    uint32_t background_task_number =
                            task.createBackground(loop_background_task);

    /* Uncomment the following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    /* STEP 3 - LAUNCH THE TASKS */
    task.startBackground(background_task_number);

    /* Uncomment the following line if you use the critical task */
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * You can use it to execute slow code such as state-machines.
 * The pause define its pseudo-periodicity.
 */
void loop_background_task()
{
    spin.led.toggle();

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
    if (meas_data != NO_VALUE) temp_1_value = meas_data;

    shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);

    meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
    if (meas_data != NO_VALUE) temp_2_value = meas_data;

    printk("%8.3f:", (double)I1_low_value);
    printk("%8.3f:", (double)V1_low_value);
    printk("%8.3f:", (double)I2_low_value);
    printk("%8.3f:", (double)V2_low_value);
    printk("%8.3f:", (double)I_high_value);
    printk("%8.3f:", (double)V_high_value);
    printk("%8.3f:", (double)temp_1_value);
    printk("%8.3f:", (double)received_value);
    printk("\n");

    /* This pauses the task for 1000 milli seconds */
    task.suspendBackgroundMs(1000);
    control_reference += 0.001;

    /* Use the following function to send a control reference over CAN */
    communication.can.setCtrlReference(control_reference);
    /* Use the following functions to send a start or stop order over CAN */
    communication.can.startSlaveDevice();
    communication.can.stopSlaveDevice();


    /* The following functions retrieve any reference sent over CAN */
    received_value = communication.can.getCtrlReference();
    /* The following functions retrieve any start stop order sent over CAN */
    received_start_stop = communication.can.getStartStopState();
}

/**
 * This is the code loop of the critical task
 * It is executed every 100 micro-seconds defined in the setup_routine function.
 * You can use it to execute an ultra-fast code with the highest priority which
 * cannot be interrupted.
 */
void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high_value = meas_data;

}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}