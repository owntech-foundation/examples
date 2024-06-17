/*
 * Copyright (c) 2021-2024 OwnTech Foundation
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
 * @brief  This file is an application example for 
 *         BLDC motor control using OwnVerter.
 *
 * @author Jean Alinei <jean.alinei@owntech.org>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"

#include "zephyr/console/console.h"

#define HALL1 PA7
#define HALL2 PC6
#define HALL3 PD2

#define PHASE_A PWMA
#define PHASE_B PWMC
#define PHASE_C PWME

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;
uint16_t hall_state;
uint8_t hall1_value;
uint8_t hall2_value;
uint8_t hall3_value;

static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.5;

/* PID coefficient for a 8.6ms step response*/

// static float32_t kp = 0.000215;
// static float32_t ki = 2.86;
// static float32_t kd = 0.0;
void chopper(hrtim_tu_number_t high_phase, hrtim_tu_number_t low_phase) {
	spin.pwm.setDutyCycle(low_phase, 1.0 - duty_cycle);
	spin.pwm.startDualOutput(low_phase);
	spin.pwm.setDutyCycle(high_phase, duty_cycle);
	spin.pwm.startDualOutput(high_phase);
}
//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* --------------SETUP FUNCTIONS-------------------------------*/

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    /* Setup the hardware first */ 
    twist.setVersion(shield_ownverter);

    /* Set the high switch convention for all legs */
    twist.initAllBuck();
    spin.pwm.setModulation(PHASE_C, UpDwn);
    spin.pwm.setSwitchConvention(PHASE_C, PWMx1);
    spin.pwm.initUnit(PHASE_C);

    /* Setup all the measurments */
    data.enableTwistDefaultChannels();

    /* Declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); 

    /* Start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); 

    spin.gpio.configurePin(HALL1, INPUT);
    spin.gpio.configurePin(HALL2, INPUT);
    spin.gpio.configurePin(HALL3, INPUT);
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
            duty_cycle = 0.5;
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'u':
            duty_cycle += 0.01;
            break;
        case 'd':
            duty_cycle -= 0.01;
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
    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
        printk("%f:", V1_low_value);
        printk("%f:", I2_low_value);
        printk("%f:", V2_low_value);
        printk("%f:", I_high);
        printk("%f", V_high);
        printk("%5d\n", hall_state);

    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();
        printk("%5.5f:", duty_cycle);
        printk("%f:", V1_low_value);
        printk("%f:", I2_low_value);
        printk("%f:", V2_low_value);
        printk("%f:", I_high);
        printk("%f", V_high);
        printk("%5d\n", hall_state);
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
    /* Retrieve the rotor position from hall sensors*/
    hall1_value = spin.gpio.readPin(HALL1);
    hall2_value = spin.gpio.readPin(HALL2);
    hall3_value = spin.gpio.readPin(HALL3);

    /* Compute the sector from hall values */
    hall_state = hall1_value + 2*hall2_value + 4*hall3_value;

    /* Retrieve sensor values */
    meas_data = data.getLatest(I1_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I1_low_value = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != -10000)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != -10000)
        V2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I2_low_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data < 10000 && meas_data > -10000)
        I_high = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != -10000)
        V_high = meas_data;

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            spin.pwm.stopDualOutput(PHASE_A);
            spin.pwm.stopDualOutput(PHASE_B);
            spin.pwm.stopDualOutput(PHASE_C);
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        switch (hall_state)
        {
            /* This switch case implements classic BLDC logic */
            case 0b001:
				spin.pwm.stopDualOutput(PHASE_B);
				chopper(PHASE_A, PHASE_C);
                break;
            case 0b010:
				spin.pwm.stopDualOutput(PHASE_C);
				chopper(PHASE_B, PHASE_A);
                break;
            case 0b011:
				spin.pwm.stopDualOutput(PHASE_A);
				chopper(PHASE_B, PHASE_C);
                break;
            case 0b100:
				spin.pwm.stopDualOutput(PHASE_A);
				chopper(PHASE_C, PHASE_B);
                break;
            case 0b101:
				spin.pwm.stopDualOutput(PHASE_C);
				chopper(PHASE_A, PHASE_B);
                break;
            case 0b110:
				spin.pwm.stopDualOutput(PHASE_B);
				chopper(PHASE_C, PHASE_A);
                break;
        }
        
        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            spin.pwm.startDualOutput(PHASE_A);
            spin.pwm.startDualOutput(PHASE_B);
            spin.pwm.startDualOutput(PHASE_C);
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
