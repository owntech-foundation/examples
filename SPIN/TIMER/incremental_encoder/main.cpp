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
 * @brief  This example shows how to read values from an incremental encoder.
 *         using SpinAPI.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"

/* --------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

static uint32_t incremental_value_timer_3;
static uint32_t incremental_value_timer_4;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here we setup the timer that takes care of counting the pulses of the
 * incremental encoder.
 * We spawn one task to send the measurement over USB serial.
 */
void setup_routine()
{
    /* Init timer */
    spin.timer.startLogIncrementalEncoder(TIMER3);
    spin.timer.startLogIncrementalEncoder(TIMER4);

    /* Then declare tasks */
    uint32_t background_task_number =
                        task.createBackground(loop_background_task);

    /* Uncomment following line if you want to use the critical task */
    /* task.createCritical(loop_critical_task, 500); */

    /* Finally, start task */
    task.startBackground(background_task_number);
    /* Uncomment following line if you want to use the critical task */
    /* task.startCritical(); */
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * Here we retrieve value from the encoder, and then print it over USB serial.
 */
void loop_background_task()
{
    /* Task content */
    incremental_value_timer_3 = spin.timer.getIncrementalEncoderValue(TIMER3);
    incremental_value_timer_4 = spin.timer.getIncrementalEncoderValue(TIMER4);
    printk("TIM3: %u , TIM4: %d\n", incremental_value_timer_3, incremental_value_timer_4);

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * Not used in this example - Can be enabled in setup routine
 */
void loop_critical_task()
{

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
