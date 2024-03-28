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
#include "arm_math_types.h"
#include "pid.h"
#include "zephyr/console/console.h"
#include <ScopeMimicry.h>

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.3;
static bool enable_acq;
static float32_t trig_ratio;
static float32_t begin_trig_ratio = 0.05;
static float32_t end_trig_ratio = 0.95;
static uint32_t num_trig_ratio_point = 10;
//static float32_t voltage_reference = 5.0; //voltage reference

/* PID coefficient for a 8.6ms step response*/

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

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

// trigger function for scope manager
bool a_trigger() {
    return enable_acq;
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    uint16_t buffer_size = scope.get_buffer_size() >> 2; // we divide by 4 (4 bytes per float data) 
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
    twist.setVersion(shield_TWIST_V1_3);
    spin.gpio.configurePin(PC6, OUTPUT);
    spin.gpio.configurePin(PB7, OUTPUT);
    spin.gpio.resetPin(PC6); // use the capacitor
    spin.gpio.resetPin(PB7); // use the capacitor
    /* buck voltage mode */
    twist.initAllBuck();

    data.enableTwistDefaultChannels();

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
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : duty cycle UP            |\n");
            printk("|     press d : duty cycle DOWN          |\n");
            printk("|     press r : download datas           |\n");
            printk("|     press a : toggle enable_acq var    |\n");
            printk("|     press c : increment trig ratio     |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            scope.start(); // reset the scope trigger
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
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
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
        printk("% 7.2f:", duty_cycle);
        printk("% 7d:", num_trig_ratio_point);
        printk("% 7.2f:", V_high);
        printk("% 7.2f\n", V1_low_value);
        }
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();
        printk("% 7d:", scope.has_trigged());
        printk("% 7.2f:", duty_cycle);
        printk("% 7d:", num_trig_ratio_point);
        printk("% 7.2f:", V_high);
        printk("% 7.2f\n", V1_low_value);
    }
    task.suspendBackgroundMs(250);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{


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
            twist.stopAll();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        //duty_cycle = pid.calculateWithReturn(voltage_reference, V1_low_value);
        twist.setAllDutyCycle(duty_cycle);
        if (enable_acq) {
            trig_ratio += (end_trig_ratio - begin_trig_ratio) / (float32_t)num_trig_ratio_point;
            if (trig_ratio > end_trig_ratio) { // make a cycle
                trig_ratio = begin_trig_ratio;
            }
            twist.setLegTriggerValue(LEG1, trig_ratio);
        }
        scope.acquire();
    }
    /* Set POWER ON */
    if (!pwm_enable)
    {
        pwm_enable = true;
        twist.startAll();
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
