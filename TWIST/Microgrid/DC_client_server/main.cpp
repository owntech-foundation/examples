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
 * @author Antoine Boche <antoine.boche@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "CommunicationAPI.h"
#include "pid.h"

#include "zephyr/console/console.h"

#define SLAVE // MASTER, SLAVE
#define VREF 2.048


#ifdef MASTER
#define ROLE_TXT "MASTER"
#else
#define ROLE_TXT "SLAVE"
#endif


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();      // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

uint8_t received_serial_char;
static uint32_t control_task_period = 100; //[us] period of the control task

/* power enable variable*/
bool pwr_enable = false;
bool slave_listen = false;
int8_t communication_count = 0;
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

/* Measure variables */
float32_t meas_data;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;

// reference voltage/current
float32_t duty_cycle = 0.5;
#ifdef MASTER
static float32_t Vref = 12.0;
#endif
static float32_t Iref;
static float32_t PeakRef_Raw;
int count = 0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

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
    
    /* buck voltage mode */
    twist.initAllBuck();

    data.enableTwistDefaultChannels();

    communication.analog.init();
#ifdef MASTER
    communication.sync.initMaster(); // start the synchronisation
    communication.analog.setAnalogCommValue(0);
#endif

#ifdef SLAVE
    communication.sync.initSlave(TWIST_v_1_1_4); // wait for synchronisation
#endif

    pid.init(pid_params);

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
    while (1)
    {
        received_serial_char = console_getchar();
        switch (received_serial_char)
        {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("      %s\n", ROLE_TXT);
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : Iref UP                  |\n");
            printk("|     press d : Iref DOWN                |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            printk("idle mode\n");
            mode = IDLEMODE;
            break;
        case 'p':
            printk("power mode\n");
            mode = POWERMODE;
            break;
        case 'u':
            Iref += 0.10;
            break;
        case 'd':
            Iref -= 0.10;
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
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();

        printk("%f:", I1_low_value);
        printk("%f:", V1_low_value);
        printk("%f:", I2_low_value);
        printk("%f:", V2_low_value);
    }
    printk("%f:", PeakRef_Raw);
    printk("\n");
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
#ifdef SLAVE
    meas_data = communication.analog.getAnalogCommValue() + 20.0f;
    if (meas_data != -10000)
        PeakRef_Raw = meas_data;
    if (PeakRef_Raw < 1800)
        mode = IDLEMODE;
    else
        mode = POWERMODE;
#endif

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

    if (mode == IDLEMODE)
    {
        if (pwr_enable == true)
        {
            pwr_enable = false;
            twist.stopAll();
#ifdef MASTER
            communication.analog.setAnalogCommValue(0);
#endif
        }
    }
    else if (mode == POWERMODE)
    {

        if (pwr_enable == false)
        {
            pwr_enable = true;
            twist.startAll();
            count = 0;
#ifdef MASTER
            Iref = 0.6F; // initial current reference
#endif
        }

#ifdef MASTER
        count++;
        if (count == 40000)
        {
            Iref = 1.0F; // update reference value after 4s
        }
        PeakRef_Raw = (0.100F * Iref + 1.024F) * (4096.0F / 2.048F);

        duty_cycle = pid.calculateWithReturn(Vref, (V1_low_value));

        /* sending value to slave board*/
        communication.analog.setAnalogCommValue(PeakRef_Raw);
#endif

#ifdef SLAVE
        Iref = ((2.048F * PeakRef_Raw / 4096.0F) - 1.024) / 0.100;

        duty_cycle = pid.calculateWithReturn(Iref, (I1_low_value + I2_low_value));
#endif

        twist.setAllDutyCycle(duty_cycle);
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
