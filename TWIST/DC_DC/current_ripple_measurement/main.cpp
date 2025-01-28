/*
 * Copyright (c) 2023-present LAAS-CNRS
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
 * @brief  This example demonstrates how to measure the current ripple using
 *         onboard sensors of the Twist power shield.
 *
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*-------------- Libraries------------------------------------ */
#include "pid.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>

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

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

/* Temporary storage for measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;
static bool enable_acq;
static float32_t trig_ratio;
static float32_t begin_trig_ratio = 0.05;
static float32_t end_trig_ratio = 0.95;
static uint32_t num_trig_ratio_point = 1024;
/* static float32_t voltage_reference = 5.0; */

/* PID coefficient for a 8.6ms step response*/

static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

static const uint16_t NB_DATAS = 2048;
static const float32_t minimal_step = 1.0F / (float32_t) NB_DATAS;
static uint16_t number_of_cycle = 2;
static ScopeMimicry scope(NB_DATAS, 7);
static bool is_downloading;

/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* Trigger function for scope manager */
bool a_trigger() {
    return enable_acq;
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data) */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
    printk("begin record\n");
    printk("#");
    for (uint16_t k=0;k < scope.get_nb_channel(); k++) {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here the setup :
 *  - Initialize the power shield in Buck mode
 *  - Initialize the power shield sensors
 *  - Initialize the PID controller
 *  - Initialize the scope to retrieve live data using ScopeMimicry
 *  - Spawn three tasks.
 */
void setup_routine()
{

    /* Buck voltage mode */
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();

    pid.init(pid_params);

    scope.connectChannel(I1_low_value, "I1_low");
    scope.connectChannel(V1_low_value, "V1_low");
    scope.connectChannel(I2_low_value, "I2_low");
    scope.connectChannel(V2_low_value, "V2_low");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(trig_ratio, "trig_ratio");
    scope.set_trigger(&a_trigger);
    scope.set_delay(0.0F);
    scope.start();

    trig_ratio = 0.05;

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This tasks implements a minimalistic USB serial interface to control
 * the buck converter, to trigger data acquisition of the current ripple,
 * and to download the samples.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
        case 'h':
            /*----------SERIAL INTERFACE MENU----------------------- */
            printk(" ________________________________________ \n"
                   "|     ------- MENU ---------             |\n"
                   "|     press i : idle mode                |\n"
                   "|     press p : power mode               |\n"
                   "|     press u : duty cycle UP            |\n"
                   "|     press d : duty cycle DOWN          |\n"
                   "|     press r : download datas           |\n"
                   "|     press a : toggle enable_acq var    |\n"
                   "|     press c : increment trig ratio     |\n"
                   "|________________________________________|\n\n");
            /*------------------------------------------------------ */
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            /* Reset the scope trigger */
            scope.start();
            mode = POWERMODE;
            break;
        case 'u':
            if (duty_cycle < 0.9) {
                duty_cycle += 0.1;
            }
            break;
        case 'd':
            if (duty_cycle > 0.1) {
                duty_cycle -=0.1;
            }
            break;
        case 'r':
            is_downloading = true;
            break;
        case 'a':
            enable_acq = !(enable_acq);
            break;
        case 'c':
            num_trig_ratio_point = (num_trig_ratio_point + 1) % NB_DATAS;
        default:
            break;
    }
}

/**
 * This is the code loop of the background task
 * This task mostly logs back measurements to the USB serial interface.
 * A special logic is added to handle samples downloading.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        if (is_downloading) {
            dump_scope_datas(scope);
            is_downloading = false;
        } else {
        printk("% 7d:", scope.has_trigged());
        printk("% 7.2f:", (double)duty_cycle);
        printk("% 7d:", num_trig_ratio_point);
        printk("% 7.2f:", (double)V_high);
        printk("% 7.2f\n", (double)V1_low_value);
        }
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();
        printk("% 7d:", scope.has_trigged());
        printk("% 7.2f:", (double)duty_cycle);
        printk("% 7d:", num_trig_ratio_point);
        printk("% 7.2f:", (double)V_high);
        printk("% 7.2f\n", (double)V1_low_value);
    }
    task.suspendBackgroundMs(250);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - A special section does the ADC trigger instant calculation and setup
 *  - Scope is updated
 *  - It update the PWM signals
 */
void loop_critical_task()
{

    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

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
        /* Duty_cycle =
         *      pid.calculateWithReturn(voltage_reference, V1_low_value); */
        shield.power.setDutyCycle(ALL,duty_cycle);

        if (enable_acq) {
            trig_ratio += (end_trig_ratio - begin_trig_ratio) /
                          (float32_t)num_trig_ratio_point;

            /* Make a cycle */
            if (trig_ratio > end_trig_ratio) {
                trig_ratio = begin_trig_ratio;
            }
            shield.power.setTriggerValue(LEG1, trig_ratio);
        }
        scope.acquire();
    }
    /* Set POWER ON */
    if (!pwm_enable)
    {
        pwm_enable = true;
        shield.power.start(ALL);
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
