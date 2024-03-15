/*
 *
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

// from control library
#include "pr.h"
#include "trigo.h"
#include "ScopeMimicry.h"

#include "zephyr/console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------
static const uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t V_high;

// offset calculation
static float32_t I1_offset = 0.0F;
static float32_t I2_offset = 0.0F;
static float32_t I1_offset_tmp = 0.0F;
static float32_t I2_offset_tmp = 0.0F;
// we use 500*100e-6 : 0.5 ms to compute offset
static const float32_t nb_offset_meas = 500.0F; 
static float meas_data; // temp storage meas value (ctrl task)

/* duty_cycle*/
static float32_t duty_cycle;

static float32_t Udc = 40.0F;
/* Sinewave settings */
static float32_t Vgrid; 
static float32_t Vgrid_amplitude = 16.0; 
static float32_t w0 = 2.0 * PI * 50.0;   // pulsation
float angle = 0; // [rad]
//------------- PR RESONANT -------------------------------------
static Pr prop_res;
static float32_t pr_value;
static float32_t Kp = 0.02F;
static float32_t Kr = 4000.0F;
static float32_t Ts = control_task_period * 1.0e-6F;

static uint32_t critical_task_counter; 

typedef struct Record
{
    float32_t I1_low;
    float32_t I2_low;
    float32_t V1_low;
    float32_t V2_low;
    float32_t Vhigh_value;
    float32_t duty_cycle;
    float32_t Vgrid;
    float32_t angle;
    float32_t pr_value;
} record_t;

static ScopeMimicry scope(1024, 9);
static bool is_downloading;
bool a_trigger() {
    return true;
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
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

static uint8_t mode = IDLEMODE;

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

    data.enableTwistDefaultChannels();

    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(V1_low_value, "V1_low_value");
    scope.connectChannel(V2_low_value, "V2_low_value");
    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.connectChannel(Vgrid, "Vgrid");
    scope.connectChannel(angle, "angle");
    scope.connectChannel(pr_value, "pr_value");
    scope.set_delay(0.0);
    scope.set_trigger(a_trigger);
    scope.start();

    /* buck voltage mode */
    twist.initLegBuck(LEG1);
    twist.initLegBoost(LEG2);

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, control_task_period); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task

    // PR initialisation.
    PrParams params = PrParams(Ts, Kp, Kr, w0, 0.0F, -Udc, Udc);
    prop_res.init(params);
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
            printk("|     ------- grid forming ------        |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : vgrid up                 |\n");
            printk("|     press p : vgrid down               |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            scope.start();
            break;
        case 'p':
                if (!is_downloading){
                    printk("power mode\n");
                    mode = POWERMODE;
                }
            break;
        case 'u': 
            if (Vgrid_amplitude < 18.0)
                    Vgrid_amplitude += 2.0;
            break;
        case 'd': 
            if (Vgrid_amplitude > 2.0)
                    Vgrid_amplitude -= 2.0;
            break;
        case 'r':
            is_downloading = true;
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
        if (!is_downloading) {
            printk("I1_offset = %f:", I1_offset);
            printk("I2_offset = %f\n", I2_offset);
        } else {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else if (mode == POWERMODE)
    {
        printk("%f:", duty_cycle);
        printk("%f:", Vgrid);
        printk("%f:", I2_low_value);
        printk("%f:", I1_low_value);
        printk("%f:\n", V1_low_value);
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 100 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{

    meas_data = data.getLatest(I1_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I1_low_value = meas_data / 1000.0 - I1_offset;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        V2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I2_low_value = meas_data / 1000.0 - I2_offset;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != -10000)
        V_high = meas_data;


    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            twist.stopAll();
        }
        // OFFSET MANAGEMENT
        if (critical_task_counter < nb_offset_meas)
        {
            I1_offset_tmp += I1_low_value;
            I2_offset_tmp += I2_low_value;
            spin.led.turnOn();
        } 
        if (critical_task_counter == nb_offset_meas)
        {
            I1_offset = I1_offset_tmp / nb_offset_meas;
            I2_offset = I2_offset_tmp / nb_offset_meas;
        }
        if (critical_task_counter > nb_offset_meas) {
            spin.led.turnOff();
         }
        // END OF OFFFSET MANAGEMENT
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        /* Set POWER ON */
        angle += w0 * Ts; 
        angle = ot_modulo_2pi(angle);
        Vgrid = Vgrid_amplitude * ot_sin(angle);
        pr_value = prop_res.calculateWithReturn(Vgrid, V1_low_value - V2_low_value);
        duty_cycle = pr_value / (2.0 * Udc) + 0.5; 
        twist.setAllDutyCycle(duty_cycle);
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startAll();
        }

        scope.acquire();
    }
    critical_task_counter++;
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
