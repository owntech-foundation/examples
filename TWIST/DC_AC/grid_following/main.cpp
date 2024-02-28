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
#include "pr.h"
#include "trigo.h"
#include "filters.h"

#include "zephyr/console/console.h"

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
static float32_t V_high;
static float32_t I_high;

static float32_t I1_offset = 0.0F;
static float32_t I2_offset = 0.0F;
static float32_t I1_offset_tmp = 0.F;
static float32_t I2_offset_tmp = 0.F;

static float32_t Iref; 
static float32_t Vgrid;
static float32_t Vgrid_amplitude = 16.0F; // amplitude of the voltage.
static float meas_data; // temp storage meas value (ctrl task)
static float32_t Iref_amplitude = 0.5;
/* duty_cycle*/
static float32_t duty_cycle;
static const float32_t Udc = 40.0; // Vhigh assumed to be around 40V
/* Sinewave settings */
static const float f0 = 50.0F;
static const float w0 = 2.F * PI * f0; 

//------------- PR RESONANT -------------------------------------
static Pr prop_res;
static PllSinus pll;
static PllDatas pll_datas;
static uint32_t pll_counter = 0;
static bool pll_is_locked = false;
static float32_t pr_value;
static float32_t Ts = control_task_period * 1.0e-6F;
static float32_t Kp = 0.01;
static float32_t Kr = 1200.0;

uint32_t control_loop_counter;
typedef struct Record
{
    int16_t I1_low;
    int16_t I2_low;
    int16_t V1_low;
    int16_t V2_low;
    int16_t Ihigh_value;
    int16_t Iref;
    int16_t duty_cycle;
    int16_t Vgrid;
    int16_t pll_angle;
    int16_t pll_error;
    int16_t pll_w;

} record_t;

record_t record_array[2048];
uint32_t record_counter = 0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;
uint8_t mode_asked = IDLEMODE;

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
    data.setParameters(I1_LOW, 5., -10000.00);
    data.setParameters(I2_LOW, 5., -10000.00);

    /* buck voltage mode */
    twist.initLegBuck(LEG1);
    twist.initLegBoost(LEG2);

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task

    // Proportional resonant initialisation.
    PrParams params(Ts, Kp, Kr, w0, 0.0, -Udc, Udc);
    prop_res.init(params);
    float32_t rise_time = 50e-3;
    pll.init(Ts, Vgrid_amplitude, f0, rise_time);
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
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode_asked = IDLEMODE;
            record_counter = 0;
            Iref_amplitude = 0.4;
            break;
        case 'p':
            printk("power mode\n");
            mode_asked = POWERMODE;
            record_counter = 0;
            break;
        case 'u':
            if (Iref_amplitude < 0.5)
                Iref_amplitude += 0.1;
            break;
        case 'd':
            if (Iref_amplitude > 0.2)
                Iref_amplitude -= 0.1;
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
        printk("I1_offset = %f:", I1_offset);
        printk("I2_offset = %f\n", I2_offset);
    }
    else if (mode == POWERMODE)
    {

        printk("%f:", Iref_amplitude);
        printk("%f:", duty_cycle);
        printk("%f:", V1_low_value);
        printk("\n");
    }
    task.suspendBackgroundMs(100);
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

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != -10000)
        I_high = meas_data;

    if (mode_asked == POWERMODE)
    { // we must launch the PLL and wait its locking.
        pll_datas = pll.calculateWithReturn(V1_low_value - V2_low_value);
        Iref = Iref_amplitude * ot_sin(pll_datas.angle);
        if (control_loop_counter % 3 == 0)
        {
            record_array[record_counter].I1_low = (int16_t) (256.0 * I1_low_value);
            record_array[record_counter].I2_low = (int16_t) (256.0 * I2_low_value);
            record_array[record_counter].V1_low = (int16_t) (256.0 * V1_low_value);
            record_array[record_counter].V2_low = (int16_t) (256.0 * V2_low_value);
            record_array[record_counter].Ihigh_value = (int16_t) (256.0 * I_high);
            record_array[record_counter].duty_cycle = (int16_t) (256.0 * duty_cycle);
            record_array[record_counter].Iref = (int16_t) (256.0 * Iref); 
            record_array[record_counter].Vgrid = (int16_t) (256.0 * Vgrid);
            record_array[record_counter].pll_angle = (int16_t) (256.0 * pll_datas.angle);
            record_array[record_counter].pll_error = (int16_t) (256.0 * pll_datas.error);
            record_array[record_counter].pll_w = (int16_t) (256.0 * pll_datas.w / (2.0 * PI));
            if (record_counter < 2047) {
                record_counter++;
                spin.led.turnOff();
            } else {
                spin.led.turnOn();
            }
        }
    }
    else
    {
        pll_datas.error = 100;
        pll_counter = 0;
        pll.reset(f0);
        pll_is_locked = false;
    }
    // criteria of PLL locked
    if ((pll_is_locked == false) && (pll_datas.error < 2.5) && (pll_datas.error > -2.5) && (pll_datas.w < 333.0) && (pll_datas.w > 295.0) ) {
        pll_counter++;
    }
    if (pll_counter > 400) {
        pll_is_locked = true;
    }
    if (pll_is_locked)
    {
        mode = POWERMODE;
        pr_value = prop_res.calculateWithReturn(Iref, I1_low_value);
        Vgrid = V1_low_value - V2_low_value;
        duty_cycle = (Vgrid + pr_value) / (2 * 40.0) + 0.5;
        twist.setAllDutyCycle(duty_cycle);
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startAll();
        }
        spin.led.turnOn();

    }
    else
    {
        mode = IDLEMODE;
        if (pwm_enable == true)
        {
            twist.stopAll();
            duty_cycle = 0;
            spin.led.turnOff();
            pwm_enable = false;
        }
        if (control_loop_counter < 100)
        {
            I1_offset_tmp += 0.01 * I1_low_value;
            I2_offset_tmp += 0.01 * I2_low_value;
            spin.led.turnOn();
        } 
        if (control_loop_counter == 100)
        {
            I1_offset = I1_offset_tmp;
            I2_offset = I2_offset_tmp;
            spin.led.turnOff();
        } 
        if (control_loop_counter > 100) {
            spin.led.turnOff();
        }
    }

    control_loop_counter++;
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
