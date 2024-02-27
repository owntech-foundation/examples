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
#include "opalib_control_pid.h"

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
static float32_t I_high;
static float32_t V_high;
int count = 0;
static float meas_data; // temp storage meas value (ctrl task)

//Define for selecting the card to be flashed 
#define DROOP1

// Droop coeficient configuration 
#ifdef DROOP
static float32_t coef_droop = 1.2;
#endif
#ifdef DROOP1
static float32_t coef_droop1 = 1.1;
#endif
#ifdef DROOP2
static float32_t coef_droop2 = 1.6;
#endif

#if defined(DROOP) || defined(DROOP1) || defined(DROOP2)
    float32_t duty_cycle = 0.1;
    static float32_t reference = 12; //current reference 
    static float32_t serial_step = 0.1; //current reference 
#endif

/* PID coefficient for a 8.6ms step response*/

static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//--------------SETUP FUNCTIONS-------------------------------

/**
 * Convert I_low RAW value in mA with proper calibration
 */
float32_t Convert_Ilow_in_Quantum_To_mA(uint16_t IlowInQuantum, float32_t Ilow_Offset_mA, float32_t IlowMeasSlope)
{
    float32_t I_low_value_mV_ADC_mV;
    float32_t I_low_value_shifted_mV;
    float32_t I_low_value_shifted_mA;
    float32_t I_low_value_shifted_mA_offset_corrected;
    float32_t I_low_value_shifted_mA_offset_corrected_slope_corrected;

    uint16_t IlowVoltageShifting_mV = 1024;                // Shifting for Ilow (mV)
    static uint16_t ACS730_mV_To_mA_Conversion_Ratio = 10; // ACS730 sensitivity: 1mV = 10mA, <=> 100mV/A

    I_low_value_mV_ADC_mV = (IlowInQuantum * 0.5);
    I_low_value_shifted_mV = (I_low_value_mV_ADC_mV - IlowVoltageShifting_mV);
    I_low_value_shifted_mA = (I_low_value_shifted_mV * ACS730_mV_To_mA_Conversion_Ratio);
    I_low_value_shifted_mA_offset_corrected = (I_low_value_shifted_mA - Ilow_Offset_mA);
    I_low_value_shifted_mA_offset_corrected_slope_corrected = (I_low_value_shifted_mA_offset_corrected / IlowMeasSlope);

    return I_low_value_shifted_mA_offset_corrected_slope_corrected;
}


/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    // Setup the hardware first
    spin.version.setBoardVersion(TWIST_v_1_1_2);
    twist.setVersion(shield_TWIST_V1_2);

    /* buck voltage mode */
    twist.initAllBuck();

    data.enableTwistDefaultChannels();

    opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);
    opalib_control_init_leg1_pid(kp, ki, kd, control_task_period);

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
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press s : serial mode              |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : vref UP                  |\n");
            printk("|     press d : Vref DOWN                |\n");
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
            
            reference += serial_step;
            break;
        case 'd':
            reference -= serial_step;
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
        while (1)
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
        }
        else if (mode == POWERMODE)
        {
            spin.led.turnOn();
        }

        printk("%.2f:", duty_cycle);
        printk("%.2f:", V_high);
        printk("%.2f:", I_high);
        printk("%.2f:", V1_low_value);
        printk("%.2f:", I1_low_value);
        printk("%.2f:", V2_low_value);
        printk("%.2f:", I2_low_value);
        printk("%.2f:", reference);
        printk("%.2f\n", I1_low_value+I2_low_value);

        task.suspendBackgroundMs(100);
    }
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
        pwm_enable = false;
        twist.stopAll();
    }
    else if (mode == POWERMODE)
    {   
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startAll();
        }

        #ifdef DROOP
            float Vref_droop = reference - (I1_low_value+I2_low_value)*coef_droop;

            duty_cycle = opalib_control_leg1_pid_calculation(Vref_droop, V1_low_value); 
        #endif

        #ifdef DROOP1
            float Vref_droop = reference - (I1_low_value+I2_low_value)*coef_droop1;
            duty_cycle = opalib_control_leg1_pid_calculation(Vref_droop, V1_low_value);
        #endif

        #ifdef DROOP2
            float Vref_droop = reference - (I1_low_value+I2_low_value)*coef_droop2;
            duty_cycle = opalib_control_leg1_pid_calculation(Vref_droop, V1_low_value);  
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
