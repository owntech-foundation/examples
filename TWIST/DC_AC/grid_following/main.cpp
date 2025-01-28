/*
 * Copyright (c) 2024-present LAAS-CNRS
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
 * @brief  This example shows how to create a Grid following inverter using the
 *         Twist power shield.
 *         This example uses a Proportional Resonant controller from the
 *         OwnTech Control Library.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "pr.h"
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"

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
/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */
static float32_t V1_low_value; /*[V] */
static float32_t V2_low_value; /*[V] */
static float32_t I1_low_value; /*[A] */
static float32_t I2_low_value; /*[A] */
static float32_t V_high; /*[V] */
static float32_t I_high; /*[A] */

static float32_t I1_offset = 0.0F;
static float32_t I2_offset = 0.0F;
static float32_t I1_offset_tmp = 0.F;
static float32_t I2_offset_tmp = 0.F;
static const uint32_t NB_OFFSET = 100;
static const float32_t INV_NB_OFFSET = 1.0F/((float32_t) NB_OFFSET);

static float32_t Iref; /* [A]  */
static float32_t Vgrid; /*[V] */
static float32_t Vgrid_amplitude = 16.0F; /* Amplitude of the voltage in [V] */
static float meas_data; /* Temp storage meas value (ctrl task) */
static float32_t Iref_amplitude = 0.5F; /* [A] */
/* duty_cycle*/
static float32_t duty_cycle;
static const float32_t Udc = 40.0F; /* Vhigh assumed to be around 40V */
/* Sinewave settings */
static const float f0 = 50.0F;
static const float w0 = 2.F * PI * f0;

/* ------------- PR RESONANT ------------------------------------- */
static Pr prop_res;
static PllSinus pll;
static PllDatas pll_datas;
static float32_t pll_w;
static float32_t pll_angle;
static uint32_t pll_counter = 0;
static bool pll_is_locked = false;
static float32_t pr_value;
static float32_t Ts = control_task_period * 1.0e-6F;
static float32_t Kp = 0.2F;
static float32_t Kr = 3000.0F;

uint32_t control_loop_counter;

/*--------------------------------------------------------------- */
static ScopeMimicry scope(1024, 9);
bool is_downloading = false;

bool a_trigger() {
    return true;
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data)  */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
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

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;
uint8_t mode_asked = IDLEMODE;

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here are defined :
 * - Two opposed legs to operate as H bridge.
 * - Scope functionality to retrieve data using ScopeMimicry.
 * - PR resonant controller is initalized
 * - Three tasks are spawned.
 */
void setup_routine()
{
    shield.sensors.enableDefaultTwistSensors();

    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I2_low_value, "I2_low_value");
    scope.connectChannel(V1_low_value, "V1_low_value");
    scope.connectChannel(V2_low_value, "V2_low_value");
    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.connectChannel(Vgrid, "Vgrid");
    scope.connectChannel(pll_angle, "pll_angle");
    scope.connectChannel(pll_w, "pll_w");

    scope.set_delay(0.0F);
    scope.set_trigger(a_trigger);
    scope.start();

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();

    /* Proportional resonant initialisation. */
    PrParams params(Ts, Kp, Kr, w0, 0.0F, -Udc, Udc);
    prop_res.init(params);
    float32_t rise_time = 50e-3;
    pll.init(Ts, Vgrid_amplitude, f0, rise_time);
}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * Implements a minimalistic interface to control the inverter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     --- grid following example -----   |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press u : Iref up                  |\n"
               "|     press d : Iref down                |\n"
               "|     press r : retrieve data recorded   |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode_asked = IDLEMODE;
        scope.start();
        Iref_amplitude = 0.4F;
        break;
    case 'p':
        if (!is_downloading)
        {
            printk("power mode\n");
            mode_asked = POWERMODE;
        }
        break;
    case 'u':
        if (Iref_amplitude < 0.5F){
            Iref_amplitude += 0.1F;
        }
        break;
    case 'd':
        if (Iref_amplitude > 0.2F){
            Iref_amplitude -= 0.1F;
        }
        break;
    case 'r':
        is_downloading = true;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * It logs measurements in the USB serial port.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        if (!is_downloading)
        {
            printk("I1_offset = %f:", I1_offset);
            printk("I2_offset = %f\n", I2_offset);
        }
        else
        {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else if (mode == POWERMODE)
    {

        printk("%.3f:", Iref_amplitude);
        printk("%.3f:", duty_cycle);
        printk("%.3f:", V1_low_value);
        printk("\n");
    }
    task.suspendBackgroundMs(100);
}

/**
 * The critical_task runs at 10kHz.
 */
void loop_critical_task()
{
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data - I1_offset;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data - I2_offset;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    if (mode_asked == POWERMODE)
    {
        /* We must launch the PLL and wait its locking. */
        pll_datas = pll.calculateWithReturn(V1_low_value - V2_low_value);
        Iref = Iref_amplitude * ot_sin(pll_datas.angle);
        pll_w = pll_datas.w;
        pll_angle = pll_datas.angle;
        scope.acquire();
    }
    else
    {
        pll_datas.error = 100.0F;
        pll_counter = 0;
        pll.reset(f0);
        pll_is_locked = false;
    }

    /* Criteria of PLL locked */
    if ((pll_is_locked == false) &&
        (pll_datas.error < 2.5F) &&
        (pll_datas.error > -2.5F) &&
        (pll_datas.w < 333.0F) &&
        (pll_datas.w > 295.0F))
    {
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
        duty_cycle = (Vgrid + pr_value) / (2.0 * Udc) + 0.5F;
        shield.power.setDutyCycle(ALL,duty_cycle);

        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
        }
        spin.led.turnOn();
    }

    else
    {
        mode = IDLEMODE;
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
            duty_cycle = 0;
            spin.led.turnOff();
            pwm_enable = false;
        }
        if (control_loop_counter < NB_OFFSET)
        {
            I1_offset_tmp += INV_NB_OFFSET * I1_low_value;
            I2_offset_tmp += INV_NB_OFFSET * I2_low_value;
            spin.led.turnOn();
        }
        if (control_loop_counter == NB_OFFSET)
        {
            I1_offset = I1_offset_tmp;
            I2_offset = I2_offset_tmp;
            spin.led.turnOff();
        }
        if (control_loop_counter > NB_OFFSET) {
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
