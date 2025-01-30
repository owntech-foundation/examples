/*
 * Copyright (c) 2021-present LAAS-CNRS
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
 * @brief  This example demonstrates how to deploy a Buck converter with
 *         peak current mode control on the Twist Power shield.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"

/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/*--------------USER VARIABLES DECLARATIONS------------------- */

uint8_t received_serial_char;
/* [us] period of the control task */
static uint32_t control_task_period = 100;

/* Power enable variable*/
bool pwm_enable = false;

/* Measure variables */
float32_t meas_data;
static float32_t V1_low_value;
static float32_t V2_low_value;

static float32_t Ts = control_task_period * 1e-6F;
static float32_t Kp = 0.1;
/* (Kp/Ki = Ti) */
static float32_t Ti = 8.0e-4;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 10.0;
static float32_t lower_bound = -10.0;
static Pid pid;
static PidParams pid_params(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);


/* Reference voltage/current */
static float32_t Vref = 15.0;
static float32_t Iref;
static float32_t PeakRef;


/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/**
 * This is the setup routine.
 * Here the setup task :
 *  - Initializes the power shield in Buck, current mode
 *  - Initializes the power shield sensors
 *  - Sets the slope compensation
 *  - Spawns three tasks
 */
void setup_routine()
{
    /* Buck voltage mode */
    shield.power.initBuck(ALL,CURRENT_MODE);

    shield.sensors.enableDefaultTwistSensors();

    /* Initial setting slope compensation*/
    shield.power.setSlopeCompensation(ALL,1.4, 1.0);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();


    pid.init(pid_params);
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This task implements a minimalistic interface through the USB serial port
 * in order to control the Buck converter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ---- MENU buck current mode ----   |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press u : voltage reference UP     |\n"
               "|     press d : voltage reference DOWN   |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        break;
    case 'u':
        Vref += 0.5;
        break;
    case 'd':
        Vref -= 0.5;
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * This task mainly logs back measurement on the USB serial interface.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();

        printk("%.3f:", (double)Vref);
        printk("%.3f:", (double)V1_low_value);
        printk("%.3f:", (double)V2_low_value);
        printk("%.3f:", (double)PeakRef);
        printk("\n");

    }
    task.suspendBackgroundMs(200);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensor values
 *  - It computes the peak current reference using a PID controller
 *  - It updates the comparator with new peak current reference.
 */
void loop_critical_task()
{

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        Iref = pid.calculateWithReturn(Vref, V1_low_value);

        /* Convert the current in voltage for slope compensation */
        PeakRef = 0.1 * Iref + 1.024;

        /* Set slope compensation*/
        shield.power.setSlopeCompensation(ALL, PeakRef, PeakRef - 0.5);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
        }
    }

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
