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
#include "filters.h"
#include "ScopeMimicry.h"

#include "zephyr/console/console.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F
#define UDC_STARTUP 15.0F
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
static float32_t V1_low_value; // [V]
static float32_t V2_low_value; // [V]
static float32_t I1_low_value; // [A]
static float32_t I2_low_value; // [A]
static float32_t V_high; // [V]
static float32_t I_high; // [A]
static float32_t V_high_filt; // [V]



static float meas_data; // temp storage meas value (ctrl task)

/* duty_cycle*/
static float32_t duty_cycle;// [No unit]

static float32_t Udc = 40.0F; // dc voltage supply assumed [V]
static const float f0 = 50.0F; // fundamental frequency [Hz]
static const float32_t w0 = 2.0F * PI * f0;   // pulsation [rad/s]
/* Sinewave settings */
static float32_t Vgrid_ref; //[V]
static float32_t Vgrid_amplitude_ref = 0.0F; // [V] 
static float32_t Vgrid_amplitude = 0.0F; // [V]
static float angle = 0.F; // [rad]
//------------- PR RESONANT -------------------------------------
static Pr prop_res; // proportional resonant regulator instance
static float32_t pr_value; // value returned by the calculation of the prop_res
static float32_t Kp = 0.02F; // prop_res parameter
static float32_t Kr = 4000.0F; // prop_res parameter
static float32_t Ts = control_task_period * 1.0e-6F;

// comes from "filters.h"
LowPassFirstOrderFilter vHighFilter(Ts, 0.1F);
static uint32_t critical_task_counter; 

// the scope help us to record datas during the critical task
// its a library which must be included in platformio.ini
static ScopeMimicry scope(1024, 9); 
static bool is_downloading;
//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE=1,
    ERRORMODE=3,
    STARTUPMODE=4
};

static uint8_t mode = IDLEMODE;
static uint8_t mode_asked = IDLEMODE;
static float32_t spying_mode = 0; 
static const float32_t MAX_CURRENT = 8.0F;

bool a_trigger() {
    return (mode == POWERMODE);
}

/**
 * @brief print recorded data of the ScopeMimicry instance to console
 * we use this function in coordination with a miniterm python filter on the host side.
 * `filter_recorded_data.py` to save the data in a file and format them in float.
 *
 * @param scope 
 */
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

// UTILS FUNCTIONS FOR CONTROL
float32_t saturate(const float32_t x, float32_t min, float32_t max) {
    if (x > max) { 
        return max;
    }
    if (x < min) {
        return min;
    }
    return x;
}

float32_t sign(float32_t x, float32_t tol=1e-3) {
    if (x > tol) {
        return 1.0F;
    }
    if (x < -tol) {
        return -1.0F;
    }
    return 0.0F; 
}

float32_t rate_limiter(const float32_t ref, float32_t value, const float32_t rate) {
    value += Ts * rate * sign(ref - value);
    return value;
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

    data.enableTwistDefaultChannels();
    // DISABLE DC LOW CAPACITORS
    spin.gpio.configurePin(PC6, OUTPUT);
    spin.gpio.configurePin(PB7, OUTPUT);
    spin.gpio.resetPin(PC6);
    spin.gpio.resetPin(PB7);

    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I_high, "iHigh");
    scope.connectChannel(V1_low_value, "V1_low_value");
    scope.connectChannel(V2_low_value, "V2_low_value");
    scope.connectChannel(V_high_filt, "V_high_filt");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.connectChannel(Vgrid_ref, "Vgrid_ref");
    scope.connectChannel(Vgrid_amplitude, "Vgrid_amplitude");
    scope.connectChannel(spying_mode, "mode");
    scope.set_delay(0.0F);
    scope.set_trigger(a_trigger);
    scope.start();
    
    // PR initialisation.
    PrParams params = PrParams(Ts, Kp, Kr, w0, 0.0F, -Udc, Udc);
    prop_res.init(params);

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
            mode_asked = IDLEMODE;
            break;
        case 'p':
                if (!is_downloading){
                    printk("power mode\n");
                    scope.start();
                    mode_asked = POWERMODE;
                }
            break;
        case 'u': 
            if (Vgrid_amplitude_ref < 50.0F)
                    Vgrid_amplitude_ref += .5F;
            break;
        case 'd': 
            if (Vgrid_amplitude_ref > 0.5F)
                    Vgrid_amplitude_ref -= .5F;
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
/* --- STATE MACHINE --------------------------------------------------------*/
// mode is the STATE variable
// in each state we compute the transitions
switch (mode) {
        case IDLEMODE:
            if (mode_asked == POWERMODE && V_high_filt >= UDC_STARTUP) {
                mode = STARTUPMODE;
            }
        break;
        case STARTUPMODE:
            if (duty_cycle > 0.49F ) mode = POWERMODE; 
        break;
        case POWERMODE:
            if (mode_asked == IDLEMODE) {
                mode = IDLEMODE;
            }
        break;
        case ERRORMODE:
        break;
    }
    if (mode_asked == IDLEMODE) mode = IDLEMODE; // global return to idle possible
/* --- END OF STATE MACHINE -------------------------------------------------*/

    if (mode == IDLEMODE)
    {
        if (!is_downloading) {
            printk("%d:", mode);
            printk("% 7.3f:", Vgrid_amplitude_ref);
            printk("% 7.3f:", I1_low_value);
            printk("% 7.3f:", I2_low_value);
            printk("% 7.3f:", V1_low_value);
            printk("\n");
        } else {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else 
    {
        printk("%d:", mode);
        printk("% 6.2f:", Vgrid_amplitude_ref);
        printk("% 6.2f:", Vgrid_amplitude);
        printk("% 6.2f:\n", V1_low_value);
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
    // RETRIEVE MEASUREMENTS 
    meas_data = data.getLatest(I1_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I1_low_value = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        V2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data < 10000 && meas_data > -10000)
        I2_low_value = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != -10000)
        V_high = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != -10000)
        I_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);

    // MANAGE OVERCURRENT
    if (I1_low_value > MAX_CURRENT 
        || I1_low_value < -MAX_CURRENT 
        || I2_low_value > MAX_CURRENT 
        || I2_low_value < -MAX_CURRENT)
    {
        mode = ERRORMODE;
    }


    if (mode == IDLEMODE || mode == ERRORMODE)
    {
        // FIRST WE STOP THE PWM
        if (pwm_enable == true)
        {
            twist.stopAll();
            spin.led.turnOff();
            pwm_enable = false;
        }
        Vgrid_amplitude = 0.F;
        duty_cycle = DUTY_MIN;
        prop_res.reset();
    }

    if (mode == STARTUPMODE) { // ramp up the common voltage to Udc/2
        duty_cycle = rate_limiter(0.5F, duty_cycle, 50.0F); // ramp of 50/s
        if (duty_cycle > 0.5F) {
            duty_cycle = 0.5F;
        }
        twist.setLegDutyCycle(LEG2, 1-duty_cycle);
        twist.setLegDutyCycle(LEG1, duty_cycle);
        // WE START THE PWM
        if (!pwm_enable)
        {
            twist.startAll();
            pwm_enable = true;
        }
    }
    if (mode == POWERMODE)
    {
        angle = ot_modulo_2pi(angle + w0 * Ts); 
        Vgrid_amplitude = rate_limiter(Vgrid_amplitude_ref, Vgrid_amplitude, 10.F); 
        Vgrid_ref = Vgrid_amplitude * ot_sin(angle);
        pr_value = prop_res.calculateWithReturn(Vgrid_ref, V1_low_value - V2_low_value);
        duty_cycle = pr_value / (2.0F * V_high_filt) + 0.5F; 
        twist.setAllDutyCycle(duty_cycle);

    }
    if (critical_task_counter%3 == 0) {
        spying_mode = (float32_t) mode;
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
