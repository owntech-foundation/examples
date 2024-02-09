/*
 * Copyright (c) 2021-2024 LAAS-CNRS
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
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"

#include "zephyr/console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

uint8_t received_serial_char;
static uint32_t control_task_period = 100; //[us] period of the control task

/* power enable variable*/
bool pwm_enable = false;

/* Measure variables */
float32_t meas_data;
static float32_t V1_low_value;
static float32_t V2_low_value;

// PID parameters
static float32_t p = 0.1;            // proportional coefficient
static float32_t i = 125;            // integral coefficient
static float32_t integrator_mem = 0; // integral memory
static float Kb = 1e4;               // back-tracking coefficient
static float32_t pid_period = control_task_period / 1000000.0f;

// reference voltage/current
static float32_t Vref = 15.0;
static float32_t Iref;
static float32_t PeakRef;


//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//--------------SETUP FUNCTIONS-------------------------------


float32_t PID_CM(float reference, float measurement)
{
    /////
    // Compute error

    float32_t error = reference - measurement;

    /////
    // Compute derivative term

    float32_t sum = (p * error) + integrator_mem;

    ////
    // Current reference
    float32_t Iref = 0;

    if (sum > 10)
        Iref = 10;
    else if (sum < -10)
        Iref = -10;
    else
        Iref = sum;

    /////
    // Compute integral term with anti-windup

    integrator_mem += ((Iref - sum) * Kb + i * error) * pid_period;

    return Iref;
}


/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    // Setup the hardware first
    spin.version.setBoardVersion(TWIST_v_1_1_2);
    twist.setVersion(shield_TWIST_V1_2);

    /* buck voltage mode */
    twist.initAllBuck(CURRENT_MODE);

    data.enableTwistDefaultChannels();

    /* initial setting slope compensation*/
    twist.setAllSlopeCompensation(1.4, 1.0);

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

void loop_communication_task()
{
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press s : serial mode              |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
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
}

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    while (1)
    {

        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
        }
        else if (mode == POWERMODE)
        {
            spin.led.turnOn();

            printk("%f:", V1_low_value);
            printk("%f:", V2_low_value);
            printk("%f\n", PeakRef);
        }
        k_msleep(100);
    }

}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != -10000)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != -10000)
        V2_low_value = meas_data;


    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            twist.stopAll();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        Iref = PID_CM(Vref, V1_low_value); // Calculate Iref

        PeakRef = 0.1 * Iref + 1.024; // Convert the current in voltage for slope compensation

        // /*set slope compensation*/
        twist.setAllSlopeCompensation(PeakRef, PeakRef - 0.5);

        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startAll();
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
