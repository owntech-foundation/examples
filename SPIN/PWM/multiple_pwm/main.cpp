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
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
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

float32_t duty_cycle = 0.3;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    // Setup the hardware first
    spin.version.setBoardVersion(SPIN_v_1_0);

    /* PWM A initialization */
    spin.pwm.setModulation(PWMA, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMA, EdgeTrigger_up);
    spin.pwm.setMode(PWMA, VOLTAGE_MODE);

    spin.pwm.initUnit(PWMA); // timer initialization

    spin.pwm.startDualOutput(PWMA); // Start PWM

    /* PWM C initialization */
    spin.pwm.setModulation(PWMC, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMC, EdgeTrigger_up);
    spin.pwm.setMode(PWMC, VOLTAGE_MODE);

    spin.pwm.initUnit(PWMC); // timer initialization

    spin.pwm.setPhaseShift(PWMC, 72); // Phase shift of 72°
    spin.pwm.startDualOutput(PWMC); // Start PWM

    /* PWM D initialization */
    spin.pwm.setModulation(PWMD, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMD, EdgeTrigger_up);
    spin.pwm.setMode(PWMD, VOLTAGE_MODE);

    spin.pwm.initUnit(PWMD); // timer initialization

    spin.pwm.setPhaseShift(PWMD, 144); // Phase shift of 144°
    spin.pwm.startDualOutput(PWMD); // Start PWM

    /* PWM E initialization */
    spin.pwm.setModulation(PWME, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWME, EdgeTrigger_up);
    spin.pwm.setMode(PWME, VOLTAGE_MODE);

    spin.pwm.initUnit(PWME); // timer initialization

    spin.pwm.setPhaseShift(PWME, 216); // Phase shift of 216°
    spin.pwm.startDualOutput(PWME); // Start PWM

    /* PWM F initialization */
    spin.pwm.setModulation(PWMF, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMF, EdgeTrigger_up);
    spin.pwm.setMode(PWMF, VOLTAGE_MODE);

    spin.pwm.initUnit(PWMF); // timer initialization

    spin.pwm.setPhaseShift(PWMF, 288); // Phase shift of 288°
    spin.pwm.startDualOutput(PWMF); // Start PWM

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
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'u':
            duty_cycle += 0.05;
            break;
        case 'd':
            duty_cycle -= 0.05;
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
    // Task content
    printk("%f\n", duty_cycle);

    // Pause between two runs of the task
    task.suspendBackgroundMs(1000);

}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
        spin.pwm.setDutyCycle(PWMA, duty_cycle);
        spin.pwm.setDutyCycle(PWMC, duty_cycle);
        spin.pwm.setDutyCycle(PWMD, duty_cycle);
        spin.pwm.setDutyCycle(PWME, duty_cycle);
        spin.pwm.setDutyCycle(PWMF, duty_cycle);
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
