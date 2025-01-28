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
 * @brief  This examples demonstrates how to use the Digital to Analog converter
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
static uint32_t dac_value;

/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * It initializes the DAC. And spawn one task for demonstration purpose.
 */
void setup_routine()
{
    /* DAC initialization, here DAC2 is used */
    spin.dac.initConstValue(2);

    /* DAC value ranges from 0 to 4096 (12 bit resolution)
     * On Spin, the default Analog reference is 2048mV
     * Output voltage range from 0mV to 2048mV */
    spin.dac.setConstValue(2, 1, 0);

    uint32_t background_task_number =
                            task.createBackground(loop_background_task);

    /* task.createCritical(loop_critical_task, 100); */


    /* Finally, start tasks */
    task.startBackground(background_task_number);
    /* task.startCritical(); */
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * Here it changes the DAC value.
 */
void loop_background_task()
{
    /* Task content */

    /* Here we increment the dac value by 100 quanta
     * The modulo is here to make sure that dac_value
     * ranges from 0 to 4096 */
    dac_value = (dac_value + 100) % 4096;
    /* DAC reference is updated. We expect to see a
     * sawtooth signal from 0mV to 2000mV on Spin pin 34 */
    spin.dac.setConstValue(2, 1, dac_value);

    /* Pause between two runs of the task */
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * For now it is not initialized in the setup_routine but you can
 * easily uncomment it to implement your own fast DAC control logic.
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
