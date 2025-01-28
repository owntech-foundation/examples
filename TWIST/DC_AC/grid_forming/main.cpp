/*
 *
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
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"

/* From control library */
#include "pr.h"
#include "trigo.h"
#include "filters.h"
#include "ScopeMimicry.h"

#include "zephyr/console/console.h"

#define DUTY_MIN 0.1F
#define DUTY_MAX 0.9F
#define UDC_STARTUP 15.0F
#define NUM_SCOPE_CHANNELS 11
#define SIZE_SCOPE_BUFFER 1024

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
static const uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;
/* [bool] state to trigger the PWM (ctrl task) */
static bool trigger = false;

uint8_t received_serial_char;

/* Measure variables */
static float32_t V1_low_value;  /* [V] */
static float32_t V2_low_value;  /* [V] */
static float32_t I1_low_value;  /* [A] */
static float32_t I2_low_value;  /* [A] */
static float32_t V_high;        /* [V] */
static float32_t I_high;        /* [A] */
static float32_t V_high_filt;   /* [V] */
static float32_t V_AC;          /* [V] */

/* Temporary storage for measured values (ctrl task) */
static float meas_data;

/* Duty_cycle */
static float32_t duty_cycle; /* [No unit] */

static float32_t Udc = 25.0F;   /* DC voltage supply assumed [V] */
static const float f0 = 50.0F;  /* Fundamental frequency [Hz] */
static const float32_t w0 = 2.0F * PI * f0;   /* pulsation [rad/s] */

/* Sinewave settings */
static float32_t Vgrid_ref;                     /* [V] */
static float32_t Vgrid_amplitude_ref = 0.0F;    /* [V] */
static float32_t Vgrid_amplitude = 0.0F;        /* [V] */
static float angle = 0.F;                       /* [rad] */

/*------------- PR RESONANT ------------------------------------- */

static Pr prop_res;         /* Proportional resonant regulator instance */
static float32_t pr_value;  /* Value returned by the calculation of the prop_res */
static float32_t Kp = 0.02F;    /* prop_res parameter */
static float32_t Kr = 4000.0F;  /* prop_res parameter */
static float32_t Ts = control_task_period * 1.0e-6F;

/* Comes from "filters.h" */
LowPassFirstOrderFilter vHighFilter(Ts, 0.1F);
static uint32_t critical_task_counter;

/* The scope help us to record datas during the critical task
 * Its a library which must be included in platformio.ini */
static ScopeMimicry scope(SIZE_SCOPE_BUFFER, NUM_SCOPE_CHANNELS);
static bool is_downloading;
/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
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
    return trigger;
}


/**
 * @brief Dumps the data from the scope buffer for recording or analysis.
 *
 * This function retrieves and prints the data from a `ScopeMimicry` object's
 * buffer. The buffer size is adjusted to account for the fact that each data
 * point is a 4-byte floating-point value. The function outputs the recorded
 * data, channel names, and then suspends the background task for a short period
 * after printing each data point to avoid overwhelming the output stream.
 *
 * @param scope  A reference to a `ScopeMimicry` object containing the
 *               buffer and channel information.
 *
 * @details
 * - The function begins by printing "begin record" to signify the start
 *   of the data dump.
 * - It then prints the names of all channels followed by a comma.
 * - For each data point in the buffer, it prints the hexadecimal representation
 *   of the data and suspends the background task for 100 microseconds.
 * - Finally, it prints "end record" to indicate the completion of the data dump.
 *
 * @note This function is used in coordination with a miniterm python filter on
 *       the host side. `filter_recorded_data.py` to save the data in a file and
 *       format them in float.
 *
 */
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

/**
 * @brief Constrains a value within a specified range.
 *
 * This function limits the input value `x` to the specified range
 * defined by `min` and `max`. If `x` exceeds `max`, the function
 * returns `max`. If `x` is less than `min`, the function returns `min`.
 * Otherwise, it returns `x`.
 *
 * @param x    The input value to be constrained.
 * @param min  The minimum allowed value.
 * @param max  The maximum allowed value.
 *
 * @returns The constrained value of `x`, ensuring it falls within the
 *          range [min, max].
 */
float32_t saturate(const float32_t x, float32_t min, float32_t max) {
    if (x > max) {
        return max;
    }
    if (x < min) {
        return min;
    }
    return x;
}

/**
 * @brief Give a sign linked to a tolerance value
 *
 * @param x    Input signal
 * @param tol  tolerance value
 *
 * @returns -1 if under tol, or 1 if above tol
 */
float32_t sign(float32_t x, float32_t tol=1e-3) {
    if (x > tol) {
        return 1.0F;
    }
    if (x < -tol) {
        return -1.0F;
    }
    return 0.0F;
}

/**
 * @brief Ramps up a signal at a given rate
 *
 * @param ref    signal final amplitude
 * @param value  initial value
 * @param rate   rate to reach reference
 *
 * @returns current value ramping up.
 */
float32_t rate_limiter(float32_t ref, float32_t value, float32_t rate) {
    value += Ts * rate * sign(ref - value);
    return value;
}

/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * Here we :
 *  - Initialize default sensors.
 *  - Set two legs to operate as an H bridge.
 *  - Disable electrolytic capacitors of the TWIST board to operate in AC mode.
 *  - Initialize the scope to retrieve live data using ScopeMimicry.
 *  - Initialize the Proportional resonant controller
 *  - We spawn three tasks
 */
void setup_routine()
{

    shield.sensors.enableDefaultTwistSensors();

    scope.connectChannel(I1_low_value, "I1_low_value");
    scope.connectChannel(I_high, "I_High");
    scope.connectChannel(V1_low_value, "V1_low_value");
    scope.connectChannel(V2_low_value, "V2_low_value");
    scope.connectChannel(V_AC, "V_AC");
    scope.connectChannel(V_high, "V_High");
    scope.connectChannel(duty_cycle, "duty_cycle");
    scope.connectChannel(Vgrid_ref, "Vgrid_ref");
    scope.connectChannel(Vgrid_amplitude, "Vgrid_amplitude");
    scope.connectChannel(Vgrid_amplitude_ref, "Vgrid_amp_ref");
    scope.connectChannel(spying_mode, "mode");
    scope.set_delay(0.0F);
    scope.set_trigger(a_trigger);
    scope.start();

    /* PR initialisation. */
    PrParams params = PrParams(Ts, Kp, Kr, w0, 0.0F, -Udc, Udc);
    prop_res.init(params);

    /* Create a H bridge using LEG1 and LEG2 */
    shield.power.initBuck(LEG1);
    shield.power.initBoost(LEG2);

    shield.power.disconnectCapacitor(ALL);

    /* Then declare tasks */
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, control_task_period);

    /* Finally, start tasks */
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical();


}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * Implements a minimalistic menu to control the Grid forming inverter.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ------- grid forming ------        |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press u : vgrid up                 |\n"
               "|     press p : vgrid down               |\n"
               "|     press t : trigger scope acquisition|\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode_asked = IDLEMODE;
        break;
    case 'p':
        if (!is_downloading){
            printk("power mode\n");
            mode_asked = POWERMODE;
        }
        break;
    case 'u':
        if (Vgrid_amplitude_ref < 50.0F){
            Vgrid_amplitude_ref += .5F;
        }
        break;
    case 'd':
        if (Vgrid_amplitude_ref > 0.5F){
            Vgrid_amplitude_ref -= .5F;
        }
        break;
    case 'r':
        is_downloading = true;
        trigger = false;
        break;
    case 't':
        trigger = true;
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * This task implements a basic state machine
 * It also logs data to the USB serial.
 */
void loop_application_task()
{
/* --- STATE MACHINE --------------------------------------------------------*/

/* Mode is the STATE variable in each state we compute the transitions */
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
    if (mode_asked == IDLEMODE){
        /* Global return to idle possible */
        mode = IDLEMODE;
    }

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
        }
        else {
            dump_scope_datas(scope);
            is_downloading = false;
        }
    }
    else
    {
        printk("%d:", mode);
        printk("% 6.2f:", Vgrid_amplitude_ref);
        printk("% 6.2f:", Vgrid_amplitude);
        printk("% 7.3f:", I1_low_value);
        printk("% 7.3f:", I2_low_value);
        printk("% 6.2f:\n", V1_low_value);
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 */
void loop_critical_task()
{
    /* RETRIEVE MEASUREMENTS  */
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

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    V_high_filt = vHighFilter.calculateWithReturn(V_high);

    V_AC = V1_low_value-V2_low_value;

    /* MANAGE OVERCURRENT */
    if (I1_low_value > MAX_CURRENT ||
        I1_low_value < -MAX_CURRENT ||
        I2_low_value > MAX_CURRENT ||
        I2_low_value < -MAX_CURRENT)
    {
        mode = ERRORMODE;
    }


    if (mode == IDLEMODE || mode == ERRORMODE)
    {
        /* FIRST WE STOP THE PWM */
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
            spin.led.turnOff();
            pwm_enable = false;
        }
        Vgrid_amplitude = 0.F;
        duty_cycle = DUTY_MIN;
        prop_res.reset();
    }

    /* Ramp up the common voltage to Udc/2 */
    if (mode == STARTUPMODE) {
        /* Ramp of 50/s */
        duty_cycle = rate_limiter(0.5F, duty_cycle, 50.0F);
        if (duty_cycle > 0.5F) {
            duty_cycle = 0.5F;
        }
        shield.power.setDutyCycle(LEG2, 1 - duty_cycle);
        shield.power.setDutyCycle(LEG1, duty_cycle);
        /* WE START THE PWM */
        if (!pwm_enable)
        {
            shield.power.start(ALL);
            pwm_enable = true;
        }
    }

    if (mode == POWERMODE)
    {
        angle = ot_modulo_2pi(angle + w0 * Ts);
        Vgrid_amplitude = rate_limiter(Vgrid_amplitude_ref, Vgrid_amplitude, 10.F);
        Vgrid_ref = Vgrid_amplitude_ref * ot_sin(angle);
        pr_value = prop_res.calculateWithReturn(Vgrid_ref, V_AC);
        duty_cycle = pr_value / (2.0F * V_high_filt) + 0.5F;
        shield.power.setDutyCycle(ALL,duty_cycle);

    }
    if (critical_task_counter%1 == 3) {
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
