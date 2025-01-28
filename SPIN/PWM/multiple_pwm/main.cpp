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
 * @brief  This example demonstrate how to spawn multiple fast and precise PWM
 *         signals using SpinAPI.
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

/* ------------------------------------------------------------ */
/* List of possible modes for the OwnTech Board */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here we initialize all five PWM from the Spin Board, effectively providing
 * ten fast an precise signals ready to use.
 * Each PWM pair is phase shifted at the initialization.
 *
 * We also spawn three tasks to control the duty cycle through USB serial.
 */
void setup_routine()
{
    /* PWM A initialization */
    spin.pwm.setModulation(PWMA, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMA, EdgeTrigger_up);
    spin.pwm.setMode(PWMA, VOLTAGE_MODE);
    /* Timer initialization */
    spin.pwm.initUnit(PWMA);
    /* Start PWM */
    spin.pwm.startDualOutput(PWMA);

    /* PWM C initialization */
    spin.pwm.setModulation(PWMC, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMC, EdgeTrigger_up);
    spin.pwm.setMode(PWMC, VOLTAGE_MODE);
    /* Timer initialization */
    spin.pwm.initUnit(PWMC);
    /* Phase shift of 72° */
    spin.pwm.setPhaseShift(PWMC, 72);
    /* Start PWM */
    spin.pwm.startDualOutput(PWMC);

    /* PWM D initialization */
    spin.pwm.setModulation(PWMD, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMD, EdgeTrigger_up);
    spin.pwm.setMode(PWMD, VOLTAGE_MODE);
    /* Timer initialization */
    spin.pwm.initUnit(PWMD);
    /* Phase shift of 144° */
    spin.pwm.setPhaseShift(PWMD, 144);
    /* Start PWM */
    spin.pwm.startDualOutput(PWMD);

    /* PWM E initialization */
    spin.pwm.setModulation(PWME, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWME, EdgeTrigger_up);
    spin.pwm.setMode(PWME, VOLTAGE_MODE);
    /* Timer initialization */
    spin.pwm.initUnit(PWME);
    /* Phase shift of 216° */
    spin.pwm.setPhaseShift(PWME, 216);
    /* Start PWM */
    spin.pwm.startDualOutput(PWME);

    /* PWM F initialization */
    spin.pwm.setModulation(PWMF, UpDwn);
    spin.pwm.setAdcEdgeTrigger(PWMF, EdgeTrigger_up);
    spin.pwm.setMode(PWMF, VOLTAGE_MODE);
    /* Timer initialization */
    spin.pwm.initUnit(PWMF);
    /* Phase shift of 288° */
    spin.pwm.setPhaseShift(PWMF, 288);
    /* Start PWM */
    spin.pwm.startDualOutput(PWMF);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, we start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * Minimalistic task to implement USB serial communication.
 * Here U and D keys respectively control PWMs duty cycle Increase and Decrease.
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
               "|________________________________________|\n\n");
        /* ------------------------------------------------------ */
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

/**
 * This is the code loop of the background task
 * Here we simply log back the duty cycle reference and print is on USB serial.
 */
void loop_application_task()
{
    /* Task content */
    printk("%f\n", (double)duty_cycle);

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(1000);

}

/**
 * This is the code loop of the critical task
 * Here the critical task runs periodically at 10kHz, we simply update the
 * duty cycle references, of all running PWMs.
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
