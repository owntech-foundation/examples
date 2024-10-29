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
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 */

//--------------Zephyr----------------------------------------
#include <zephyr/console/console.h>

//--------------OWNTECH APIs----------------------------------
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"
#include "CommunicationAPI.h"

//-------------- Libraries------------------------------------
#include "trigo.h"
#include "pid.h"
#include "pr.h"
#include "ScopeMimicry.h"

#define GENERATOR      // Role : GENERATOR or CONSUMER

#ifdef GENERATOR
#define ROLE_TXT "GENERATOR"
#else
#define ROLE_TXT "CONSUMER"
#endif

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

/* duty_cycle*/
static float32_t duty_cycle;

/* Sinewave settings */
static const float f0 = 50.F;
static const float w0 = 2 * PI * f0;
#ifdef GENERATOR
static float angle = 0;
static float Vac_ref;
#endif
static const float Udc = 25.0;
static const float32_t Ts = control_task_period * 1e-6F;
/* PEER 2 PEER variables */
static float32_t I_ac_ref;
#ifdef CONSUMER
static float32_t Vac_meas;
static float32_t gain_current = -0.20;

static const float32_t Kp = 0.01;
static const float32_t Ti = 0.1;  // (Kp/Ki = Ti)
static const float32_t Td = 0.0;
static const float32_t N = 0.0;
static const float32_t upper_bound = 2.0;
static const float32_t lower_bound = -2.0;
static Pid pid_current_control;
static const PidParams pid_params(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
#endif
static float32_t P_ref = 20; // 20W power reference from generator to consumer
static const float Rdc = 10;
static float32_t v_dc_ref = sqrt(P_ref*Rdc); // must be superior to at least 20V


static ScopeMimicry scope(1024, 9);
static bool is_downloading = false;
static bool trigger = false;
//------------- PR RESONANT -------------------------------------
static const float32_t Kp_pr = 0.2;
static const float32_t Kr = 3000.0;
static float32_t pr_upper_bound = 50.0; // assume Udc ~ 50V
static float32_t pr_lower_bound = -50.0;
static Pr pr;
static const PrParams pr_params(Ts, Kp_pr, Kr, w0, 0.0, pr_lower_bound, pr_upper_bound);



struct consigne_struct
{
    float32_t P_ref_fromGENERATOR;
    uint8_t id_and_status; // Contains status
};

struct consigne_struct tx_consigne;
struct consigne_struct rx_consigne;
uint8_t* buffer_tx = (uint8_t*)&tx_consigne;
uint8_t* buffer_rx =(uint8_t*)&rx_consigne;

extern float frequency;

uint8_t status;
uint32_t critical_task_counter;
uint32_t record_counter;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

// trigger function for scope manager
bool a_trigger() {
    return trigger;
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




//--------------SETUP FUNCTIONS-------------------------------

void reception_function(void)
{

#ifdef CONSUMER
    if (rx_consigne.id_and_status >> 6 == 1)
    {
        status = rx_consigne.id_and_status;

        if ((rx_consigne.id_and_status & 2) == 1)
            scope.start();

        P_ref = rx_consigne.P_ref_fromGENERATOR;
    }
#endif

}



/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    shield.sensors.enableDefaultTwistSensors();

    /* buck voltage mode */
    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(consigne_struct), reception_function, SPEED_20M); // custom configuration for RS485

#ifdef GENERATOR
    communication.sync.initMaster(); // start the synchronisation
#endif

#ifndef GENERATOR
    communication.sync.initSlave();
#endif

    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task

#ifndef GENERATOR
    pid_current_control.init(pid_params);
    pid_current_control.reset(-gain_current); // output initialisation of the pid.
#endif
    pr.init(pr_params);
    scope.connectChannel(I1_low_value, "I1_low");
    scope.connectChannel(I2_low_value, "I2_low");
    scope.connectChannel(V1_low_value, "V1_low");
    scope.connectChannel(V2_low_value, "V2_low");
    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(I_high, "I_high");
    scope.connectChannel(I_ac_ref, "Iac_ref");
    scope.connectChannel(v_dc_ref, "Vdc_ref");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.set_trigger(a_trigger);
    scope.set_delay(0.0F);
    scope.start();
    I_ac_ref = 0.0;
    v_dc_ref = 0.0;

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
        printk("      ------ MENU :%s             \n", ROLE_TXT);
        printk("|     press i : idle mode                |\n");
        printk("|     press p : power mode               |\n");
        printk("|________________________________________|\n\n");
        //------------------------------------------------------
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        record_counter = 0;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        break;
    case 't':
        trigger=true;
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
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
        spin.led.turnOff();
        if (is_downloading) {
            dump_scope_datas(scope);
        }
        is_downloading = false;


    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();

#ifndef GENERATOR
        printk("%i:", status);
        printk("%.3f:", v_dc_ref);
        printk("%.3f:", V_high);
        printk("%.3f:", duty_cycle);
        printk("%.3f:", I2_low_value);
        printk("%.3f:", I1_low_value);
        printk("%f:\n", V1_low_value);
#endif



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
    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;
    if (V_high < 10.0) V_high = 10.0; // to prevent div by 0.

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

#ifdef GENERATOR

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
            tx_consigne.id_and_status = (1 << 6) + 0;
            communication.rs485.startTransmission();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
        /* Set POWER ON */

        angle += w0 * Ts;
        angle = ot_modulo_2pi(angle);
        Vac_ref = 10.0F;
        duty_cycle = 0.5 + Vac_ref * ot_sin(angle) / (2.0 * Udc);
        shield.power.setDutyCycle(ALL,duty_cycle);

        if (record_counter == 0)
            tx_consigne.id_and_status = (1 << 6) + 2;
        else
            tx_consigne.id_and_status = (1 << 6) + 1;

        tx_consigne.P_ref_fromGENERATOR = P_ref;

        communication.rs485.startTransmission();

        if (critical_task_counter % 1 == 0)
        {
            scope.acquire();
            record_counter++;
        }

        if (!pwm_enable)
        {
            pwm_enable = true;
            shield.power.start(ALL);
        }


        critical_task_counter++;
    }

#endif

    ////
#ifndef GENERATOR ////// CONSUMER

    if ((((status & 0x3) == 1 || (status & 0x3) == 2)))
    {
        mode = POWERMODE;
        v_dc_ref = sqrt(P_ref*Rdc); // V_dc²/R = P, where R = 115Ω the output ressitor
        Vac_meas = V1_low_value - V2_low_value;
        gain_current = pid_current_control.calculateWithReturn(v_dc_ref, V_high);
        I_ac_ref = -gain_current * Vac_meas;
        duty_cycle = (Vac_meas + pr.calculateWithReturn(I_ac_ref, I1_low_value)) / (2.0F * Udc) + 0.5F;
        shield.power.setDutyCycle(ALL,duty_cycle);

        if (!pwm_enable)
        {
            pwm_enable = true;
            spin.led.turnOn();
            shield.power.start(ALL);
        }

        if (critical_task_counter % 4 == 0)
        {
            scope.acquire();
        }
        critical_task_counter++;
    }
    else
    {
        mode = IDLEMODE;
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
            spin.led.turnOff();
            pwm_enable = false;
        }
    }

#endif
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
