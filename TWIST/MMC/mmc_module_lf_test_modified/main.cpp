/*
 * Copyright (c) 2025-present LAAS-CNRS
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
 * @brief  This example deploys the open-loop control of a MMC arm integrating a Capacitor Voltage Balancing algorithm. 
 *         This research was funded in whole by the French National Research Agency (ANR) under the project CARROTS "ANR-24-CE05-0920-01".
 *
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @author Ana Luiza Haas Bezerra <ana-luiza.haas-bezerra@centralesupelec.fr>
 * @author Zaid Jabbar <zaid.jabbar@grenoble-inp.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Jean Alinei <jean.alinei@owntech.org>
 * @author Noemi Lanciotti <noemi.lanciotti@centralesupelec.fr>
 * @author Loïc Quéval <loic.queval@centralesupelec.fr>
 */

/*--------------Zephyr---------------------------------------- */
#include <zephyr/console/console.h>

/*--------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "ShieldAPI.h"
#include "TaskAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
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
static uint32_t control_task_period = 100; // 100 µs
static const float32_t Ts = control_task_period * 1e-6F;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value = 0;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;

/* PID coefficients for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/* Scope variables */

static const uint16_t NB_DATAS = 2048; //Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t) NB_DATAS;
static ScopeMimicry scope(NB_DATAS, 5);
static bool is_downloading;
static bool trigger = false;
static uint32_t scope_timer = 0;
static uint32_t scope_period = 10; // scope acquire data every t = scope_period (10) * critical_task_period (100 µs) = 1 ms;

/* SM test variables */

static uint8_t g = 2;
static float32_t g_float;
static float seq_timer = 0;
static uint32_t critical_task_timer = 0;
static const float32_t decalage_source = 0;
static bool Vsource_turnoff_indicator = false;
static bool Vsource_ON_once_indicator = false;
static float32_t udc = 20.0; // VDC value on the module test

/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    FIRSTSEQUENCEMODE = 1,
    SECONDSEQUENCEMODE = 2,
};

uint8_t mode = IDLEMODE;

/*--------------SCOPE FUNCTIONS------------------------------- */

/* Trigger function for scope manager */
bool a_trigger() {
    return trigger;
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
 *  - Initializes the power shield in Buck mode
 *  - Initializes the power shield sensors
 *  - Initializes the PID controller
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Buck voltage mode */
    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    shield.sensors.enableDefaultTwistSensors();

    shield.power.connectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);

    /* Enable switch control with max and min duty cycle*/
    shield.power.setDutyCycleMax(ALL,1.0);
    shield.power.setDutyCycleMin(ALL,0.0);

    /* Configure scope channels, what measurelents do you want to acquire? */
    scope.connectChannel(I1_low_value, "I_SM");
    scope.connectChannel(V1_low_value, "V_SM");
    scope.connectChannel(g_float, "mode");
    scope.connectChannel(seq_timer, "time"); // to verify if there is nothing
    scope.connectChannel(V_high, "V_high"); // to verify capacitor voltage
    scope.set_trigger(&a_trigger);
    scope.set_delay(0.0F);
    scope.start();
    //Vc_BTS indicates the bootstrap capacitor charge level, we have to measure it externally

    pid.init(pid_params);

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
 * the buck converter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ---- MENU buck voltage mode ----   |\n"
               "|     press i : idle mode                |\n"
               "|     press d : discharge capacitor mode |\n"
               "|     press r : download datas           |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'f':
        printk("first sequence part\n");
        mode = FIRSTSEQUENCEMODE;
        trigger = true;
        seq_timer = 0;
        break;
    case 's':
        printk("second sequence part\n");
        mode = SECONDSEQUENCEMODE;
        seq_timer = 0;
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
 * This task mostly logs back measurements to the USB serial interface.
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
    else if (mode == SECONDSEQUENCEMODE)
    {
        spin.led.turnOn();
    }
    else if (mode == FIRSTSEQUENCEMODE)
    {
        spin.led.toggle();
    }


        printk("%.3f:", (double)I1_low_value);
        printk("%.3f:", (double)V1_low_value);
        printk("%.3f:", (double)g);
        printk("%.3f:", (double)Vsource_ON_once_indicator);
        printk("%.3f:", (double)seq_timer);
        printk("%.3f:", (double)critical_task_timer);
        printk("%.3f:", (double)scope_timer);
        printk("%i:", mode);
        printk("\n");
    task.suspendBackgroundMs(1000);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 *  - It retrieves sensors values
 *  - It runs the PID controller
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
    /*
    //For testing logic
    if(critical_task_timer == 100000)
        {
            V1_low_value=20;
        }
    */

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
        }
        pwm_enable = false;

        if (V1_low_value<2) // If VDC is OFF, sequence can be restarted without uploading again
        {
            Vsource_ON_once_indicator = false;
        }

        if (V1_low_value>=2 && Vsource_ON_once_indicator == false) // If VDC is ON, starts sequence with small delay
        {
            mode = FIRSTSEQUENCEMODE;
            trigger = true;
            Vsource_ON_once_indicator = true;
            seq_timer = 0;
        }
    }

    else if (mode == FIRSTSEQUENCEMODE)
    {
        
        if(seq_timer >= 0 && seq_timer < 0.1) // BLOCK
        {
            g=2;
            
        }
        if(seq_timer >= 0.1 && seq_timer < 0.2) // ON
        {
            g=1;
        }
        if(seq_timer >= 0.2) // OFF
        {
            g=0;
            if (V1_low_value < udc/2) // If VDC is TURNED OFF, pass to second part of the sequence
            {
                mode = SECONDSEQUENCEMODE;
                seq_timer = 0.45;
            }
        }

        g_float = (float)g;
        
        /* Scope data acquisition */
        if (scope_timer == scope_period)
        {
            scope.acquire();
            scope_timer = 0;
        }
        scope_timer++;
        
        if(g == 0) // SM is off
        {
            shield.power.setDutyCycle(LEG1,0.0);
            if (!pwm_enable)
            {
                pwm_enable = true;
                shield.power.start(LEG1);
            }
        }
        if(g == 1) // SM is on
        {
            shield.power.setDutyCycle(LEG1,1.0);
            if (!pwm_enable)
            {
                pwm_enable = true;
                shield.power.start(LEG1);
            }
        }            
        if(g == 2) // SM is blocked
        {
            if (pwm_enable == true)
            {
                shield.power.stop(ALL);
            }
            pwm_enable = false;
        }            
        
        seq_timer += Ts;
    }
    else if (mode == SECONDSEQUENCEMODE)
    {

        if(seq_timer >= 0.2 && seq_timer < 0.7) // OFF
        {
            g=0;
        }
        if(seq_timer >= 0.7 && seq_timer < 0.8) // BLOCK
        {
            g=2;
        }
        if(seq_timer >= 0.8 && seq_timer < 1) // ON
        {
            g=1;
        }
        if(seq_timer >= 1) // IDLE
        {
            mode = IDLEMODE;
        }
        g_float = (float)g;
        
        /* Scope data acquisition */
        if (scope_timer == scope_period)
        {
            scope.acquire();
            scope_timer = 0;
        }
        scope_timer++;
        
        if(g == 0) // SM is off
        {
            shield.power.setDutyCycle(LEG1,0.0);
            if (!pwm_enable)
            {
                pwm_enable = true;
                shield.power.start(LEG1);
            }
        }
        if(g == 1) // SM is on
        {
            shield.power.setDutyCycle(LEG1,1.0);
            if (!pwm_enable)
            {
                pwm_enable = true;
                shield.power.start(LEG1);
            }
        }            
        if(g == 2) // SM is blocked
        {
            if (pwm_enable == true)
            {
                shield.power.stop(ALL);
            }
            pwm_enable = false;
        }            
        
        seq_timer += Ts;
    }
    critical_task_timer++;

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