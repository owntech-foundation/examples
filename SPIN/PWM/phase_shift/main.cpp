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
 * @brief  This example shows how to do phase shift control using SpinAPI.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/* --------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */
uint8_t received_serial_char;

float32_t duty_cycle = 0.3;
float32_t phase_shift = 180;

/*------------------------------------------------------------- */
/* List of possible modes for OwnTech Board */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here we define two PWM signals running at 200kHz.
 * We initialize them with a phase shift of 180°.
 * Then we spawn three tasks to control the phase shift.
 */
void setup_routine()
{
    /* Set frequency of pwm */
    spin.pwm.initFixedFrequency(200000);
    /* PWM A initialization */
    spin.pwm.initUnit(PWMA);
    /* Start PWM A */
    spin.pwm.startDualOutput(PWMA);

    /* PWM C initialization */
    spin.pwm.initUnit(PWMC);
    /* Phase shift of 180° */
    spin.pwm.setPhaseShift(PWMC, 180);
    /* Start PWM C */
    spin.pwm.startDualOutput(PWMC);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * Here we define a minimalistic interface over USB serial to control
 * Duty cycle using U and D keys respectively Increasing or Decreasing duty.
 * Phase shift using R and T keys to respectively Increase or Decrease phase
 * shift of PWMC.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /* ----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ------- MENU ---------             |\n"
               "|     press u : duty cycle UP            |\n"
               "|     press d : duty cycle DOWN          |\n"
               "|     press r : phase shift UP           |\n"
               "|     press t : phase shift DOWN         |\n"
               "|________________________________________|\n\n");
        /* ------------------------------------------------------ */
        break;
    case 'u':
        duty_cycle += 0.05;
        break;
    case 'd':
        duty_cycle -= 0.05;
        break;
    case 'r':
        phase_shift += 5;
        break;
    case 't':
        phase_shift -= 5;
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * Here we log back the duty cycle and phase shift reference sent over
 * USB serial.
 */
void loop_application_task()
{
    /* Task content */
    printk("%f,", (double)duty_cycle);
    printk("%f\n", (double)phase_shift);

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(1000);

}

/**
 * This is the code loop of the critical task
 * The critical task runs periodically at 10kHz. Here we simply update the
 * duty cycle and phase shift references of the PWMs.
 */
void loop_critical_task()
{
    spin.pwm.setDutyCycle(PWMA, duty_cycle);
    spin.pwm.setDutyCycle(PWMC, duty_cycle);
    spin.pwm.setPhaseShift(PWMC, phase_shift);
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
