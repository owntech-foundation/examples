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
 * @brief  This example demonstrates how to do a software triggered acquisition
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
static float32_t adc_value;
uint8_t err;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * We initialize the acquisition on pin 35.
 * We declare and start two tasks for the demonstration.
 */
void setup_routine()
{
    /* Acquisition on pin 35 */
    spin.data.enableAcquisition(35, ADC_2);

    /* Then declare tasks */
    uint32_t background_task_number =
                            task.createBackground(loop_background_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(background_task_number);
    task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * Here we simply send acquired through USB Serial.
 */
void loop_background_task()
{
    /* Task content */
    if (err == DATA_IS_OK)
    {
        printk("%f\n", (double)adc_value);
    }
    else
    {
        printk("No new value\n");
    }

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(1000);
}

/**
 * This is the code loop of the critical task
 * It is executed every 100 micro-seconds defined in the setup_routine function.
 */
void loop_critical_task()
{
    spin.data.triggerAcquisition(ADC_2);
    /* Get latest value acquired on pin 35 */
    adc_value = spin.data.getLatestValue(35, &err);
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
