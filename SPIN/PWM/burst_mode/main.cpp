/*
 * Copyright (c) 2024-present OwnTech Technologies
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
 * @brief  This example shows how burst mode can be used.
 *         It activates four pairs of PWM, that create two phase shifted
 *         H-bridges. At start-up burst mode is initialized
 *         with a duty of 8 PWM events and a period of 10 PWM events.
 *         Please refer to README.md for more details.
 *
 * @author Jean Alinei <jean.alinei@owntech.io>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "CommunicationAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

#include "zephyr/console/console.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */
void setup_routine(); // Setups the hardware and software of the system

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the slow communication task */
void loop_communication_task();
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */
uint8_t received_serial_char;

float32_t duty_cycle = 0.5;
float32_t phase_shift = 0;
uint8_t burst_duty = 8;
uint8_t burst_period = 10;
/* [bool] state of the PWM (control task) */
static bool pwm_enable = false;

/* ------------------------------------------------------------ */
/* List of possible power mode for the OwnTech board. */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here we defined multiple PWMs using SpinAPI.
 * PWM A and PWM C are meant to be one H bridge
 * PWM E and PWM F are meant to be the second H bridge
 *
 * Phase shift is initialized at 0°. PWM A phase shift is not set as it is
 * the phase reference per definition.
 */
void setup_routine()
{

    /* Setup the hardware first */

    /* Set frequency of pwm */
    spin.pwm.setFrequency(200000);

    /* Set switch convention to have proper H bridges */
    spin.pwm.setSwitchConvention(PWMC, PWMx2);
    spin.pwm.setSwitchConvention(PWMF, PWMx2);

    /* Timer initialization */
    spin.pwm.initUnit(PWMA);
    spin.pwm.initUnit(PWMC);
    spin.pwm.initUnit(PWME);
    spin.pwm.initUnit(PWMF);

    /* Set PWM to Left Aligned */
    spin.pwm.setModulation(PWMA, Lft_aligned);
    spin.pwm.setModulation(PWMC, Lft_aligned);
    spin.pwm.setModulation(PWME, Lft_aligned);
    spin.pwm.setModulation(PWMF, Lft_aligned);

    /* Set initial phase shifts */
    spin.pwm.setPhaseShift(PWMC, 0);
    spin.pwm.setPhaseShift(PWME, 0);
    spin.pwm.setPhaseShift(PWMF, 0);

    /* Set duty cycles */
    spin.pwm.setDutyCycle(PWMA, duty_cycle);
    spin.pwm.setDutyCycle(PWMC, duty_cycle);
    spin.pwm.setDutyCycle(PWME, duty_cycle);
    spin.pwm.setDutyCycle(PWMF, duty_cycle);

    /* Initialize Burst Mode */
    spin.pwm.initBurstMode();
    spin.pwm.setBurstMode(burst_duty, burst_period);
    spin.pwm.startBurstMode();
    communication.sync.initMaster();

    /* Now Hardware is initialized we declare tasks */
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
 * Communication loop permits to implement simple control interface through
 * USB serial. In this example we defined :
 * Power ON / OFF using respectively P and I keys.
 * Phase shift control using U and D keys to respectively Increase and Decrease
 * phase shift.
 * Burst period control using T and Y keys to respectively Increase and Decrease
 * burst mode period.
 * Burst mode duty cycle using E and R keys to respectively Increase and Decrease
 * burst mode duty.
 */
void loop_communication_task()
{
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            /* ----------SERIAL INTERFACE MENU----------------------- */
            printk(" ________________________________________ \n"
                   "|     ------- MENU ---------             |\n"
                   "|     press p : power mode               |\n"
                   "|     press i : idle mode                |\n"
                   "|     press e : burst mode duty UP       |\n"
                   "|     press r : burst mode duty DOWN     |\n"
                   "|     press t : burst mode period UP     |\n"
                   "|     press y : burst mode period DOWN   |\n"
                   "|     press u : phase shift UP           |\n"
                   "|     press d : phase shift DOWN         |\n"
                   "|________________________________________|\n\n");
            /* ------------------------------------------------------ */
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'e':
            burst_duty += 1;
            break;
        case 'r':
            burst_duty -= 1;
            break;
        case 't':
            burst_period += 1;
            break;
        case 'y':
            burst_period -= 1;
            break;
        case 'u':
            phase_shift += 0.5;
            break;
        case 'd':
            phase_shift -= 0.5;
            break;
        default:
            break;
        }
    }
}

/**
 * This is the code loop of the background task
 * Here we only log inputs for demonstration purposes.
 */
void loop_application_task()
{
    /* Task content */
    printk("phase shift: %f\n", phase_shift);
    printk("burst mode duty: %d\n", burst_duty);
    printk("burst mode period: %d\n", burst_period);
    /* Pause between two runs of the task */
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * A minimalistic state machine makes sure that power is off in Idle Mode
 * And that we enable power when requested through USB serial.
 * Here we set the phase shift to PWM E and PWM F effectively phase shifting
 * the second H bridge from the first one.
 * We also update the burst mode parameters.
 */
void loop_critical_task()
{
    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            spin.pwm.stopDualOutput(PWMA);
            spin.pwm.stopDualOutput(PWMC);
            spin.pwm.stopDualOutput(PWME);
            spin.pwm.stopDualOutput(PWMF);
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        spin.pwm.setPhaseShift(PWME, phase_shift);
        spin.pwm.setPhaseShift(PWMF, phase_shift);
        spin.pwm.setBurstMode(burst_duty, burst_period);
    /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            spin.pwm.startDualOutput(PWMA);
            spin.pwm.startDualOutput(PWMC);
            spin.pwm.startDualOutput(PWME);
            spin.pwm.startDualOutput(PWMF);
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
