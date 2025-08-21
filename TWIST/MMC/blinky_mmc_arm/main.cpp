/*
 * Copyright (c) 2021-present LAAS-CNRS
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
 * @brief  This example shows how to blink the onboard LED of the Spin board.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "pid.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>

/*-- Zephyr includes --*/
#include "zephyr/console/console.h"

#define MMC_LEAD 0
#define MMC_SM1 1
#define MMC_SM2 2
#define MMC_SM3 3
#define MMC_SM4 4
#define MMC_SM5 5
#define MMC_SM6 6

/**
 * @brief This function is considering a byte called 'cmd'
 *        which can turn on or off signals.
 *        The signals are identified by their 'id'.
 *        The value 'val' is used to set the signal:
 *        - if val is true, the signal is set to 1
 *        - if val is false, the signal is set to 0
 */
#define SET_SIGNAL(cmd, id, val)   \
    do                             \
    {                              \
        if (val)                   \
            (cmd) |= (1 << (id));  \
        else                       \
            (cmd) &= ~(1 << (id)); \
    } while (0)

/**
 * @brief This function is to get the turn on/off state of a signal
 *        identified by its 'id' from a byte called 'cmd'.
 */
#define GET_SIGNAL(cmd, id) (((cmd) >> (id)) & 0x01)

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

/* TODO : Define module_ID depending on the ID of the board */
uint8_t module_ID = MMC_SM1; // The ID of the module, can be set to MMC_LEAD or any other SMx

static uint8_t module_comand; // The command the followers needs to apply
static uint8_t module_command_past;
static bool change_state_command = false; // Flag to change the state of the command
static bool send_idle = false;            // Flag to send idle command from master to followers

/**
 * This is a structure that defines the frame
 * that will be sent and received through the RS485 communication.
 * command is a byte that contains the state of the signals
 * Capacitor_Voltage is the voltage of the capacitor
 * ID is the ID of the module
 */
struct MMC_frame
{
    uint8_t command;
    float32_t Capacitor_Voltage;
    uint8_t status;
    uint8_t ID;
} __packed;

typedef MMC_frame MMC_frame_t;
static MMC_frame_t dataTX_mmc;
static MMC_frame_t dataRX_mmc;

float32_t MMC_capacitor_voltage[6];

uint8_t buffer_tx[7];
uint8_t buffer_rx[7];

float32_t MMC_voltage = 0.0f;

uint32_t counter_timer = 0;
uint32_t counter_receive = 0;

uint8_t received_serial_char; // Variable to store the received character from the serial interface
int8_t CommTask_num;

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

serial_interface_menu_mode mode = IDLEMODE;

void loop_communication_task(); // Code to be executed in the communication task

/* --------------- Firmware CVB variables ------------------*/

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage fore measured value (ctrl task) */
static float meas_data;

float32_t duty_cycle = 0.3;

/* Voltage reference */
static float32_t voltage_reference = 15;

/* PID coefficients for a 8.6ms step response*/
static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;

/* Scope variables */

static bool enable_acq; // trigger variable
static float32_t trig_ratio;
static float32_t begin_trig_ratio = 0.05;
static float32_t end_trig_ratio = 0.95;
static uint32_t num_trig_ratio_point = 1024;
static const uint16_t NB_DATAS = 2048; // Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t)NB_DATAS;
static uint16_t number_of_cycle = 2;
static ScopeMimicry scope(NB_DATAS, 5);
static bool is_downloading;

/* SM switching variables */

static uint32_t critical_period = 100; // 100 µs;
static uint8_t N_u;
static uint8_t N_l;
static float32_t scope_1;
static float32_t number_of_connected_submodules_upper_arm;
static float32_t number_of_connected_submodules_lower_arm;

static float32_t scope_2;
static bool master = true;
static uint8_t seq_u[6] = {1, 2, 3, 2, 1, 0};
static uint8_t seq_l[6] = {2, 1, 0, 1, 2, 3};
static uint8_t counter_seq = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
static uint32_t f_sw = 2; // 2 Hz = 0.5 s to transition;
// static uint32_t sw_period = 1/(f_sw*critical_period)*1000000; // 2 Hz = 0.5 s to transition;
static uint32_t sw_period = 1000; // 2 Hz = 0.5 s to transition;
static uint32_t scope_period = 1; // acquire every 1000 * 100 µs;

/* CVB variables */
static float32_t values[3] = {3.0, 5.0, 4.0}; // Example values to be sorted
static uint8_t indexes[3] = {1, 2, 3};        // Example indexes to be sorted
static uint8_t counter = 0;
static uint8_t N_modules = 3;
static float32_t index_1;
static float32_t index_2;
static float32_t index_3;

/* Gate logic */
static uint8_t gate_index;
uint8_t g[3] = {0, 0, 0}; // Example gate signals to send
static uint8_t g_SM;
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;

/* --------------SETUP FUNCTIONS------------------------------- */

void config_led_LL()
{
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnON_LL()
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnOFF_LL()
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

/* Trigger function for scope manager */
bool a_trigger()
{
    return enable_acq;
}

void dump_scope_datas(ScopeMimicry &scope)
{
    uint8_t *buffer = scope.get_buffer();
    /* We divide by 4 (4 bytes per float data) */
    uint16_t buffer_size = scope.get_buffer_size() >> 2;
    printk("begin record\n");
    printk("#");
    for (uint16_t k = 0; k < scope.get_nb_channel(); k++)
    {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k = 0; k < buffer_size; k++)
    {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

void reception_function(void)
{
    dataRX_mmc = *(MMC_frame_t *)buffer_rx;

    if (module_ID == MMC_LEAD)
    {
        MMC_capacitor_voltage[dataRX_mmc.ID - 1] = dataRX_mmc.Capacitor_Voltage;
    }

    else
    {
        if (dataRX_mmc.ID == MMC_LEAD)
        {
            /* retrievig command from lead message*/
            module_comand = GET_SIGNAL(dataRX_mmc.command, module_ID);
            /* retrieving status */
            if (dataRX_mmc.status == 1)
            {
                mode = POWERMODE;
            }
            else
            {
                mode = IDLEMODE;
            }
        }

        /* The board following the ID of the one who sent will start sending
            the next message */
        if ((dataRX_mmc.ID == module_ID - 1))
        {
            dataTX_mmc = dataRX_mmc; // Copy the received data to the transmission data
            dataTX_mmc.ID = module_ID;
            dataTX_mmc.Capacitor_Voltage = MMC_voltage; /* TODO :uncomment when we get the voltage */
            if (mode == POWERMODE)
            {
                memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
                communication.rs485.startTransmission();
            }
        }
    }
    counter_receive++;
}

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, power shields
 * and tasks.
 *
 * In this example, we spawn a background task.
 * An optional critical task can be initialized by uncommenting the two
 * commented lines.
 */
void setup_routine()
{

    config_led_LL(); // Configure the LED pin in Low Level

    shield.power.initBuck(ALL);
    /* Declare task */
    uint32_t background_task_number =
        task.createBackground(loop_background_task);

    /* Uncomment following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    shield.sensors.enableDefaultTwistSensors();

    /* Finally, start tasks */
    task.startBackground(background_task_number);
    /* Uncomment following line if you use the critical task */
    task.startCritical();

    CommTask_num = task.createBackground(loop_communication_task);
    task.startBackground(CommTask_num);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M); // custom configuration for RS485
                                              /* Configure scope channels, what measurements do you want to acquire? */
    if (master == true)
    {
        scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
        scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
        // scope.connectChannel(values[0], "value 1");
        // scope.connectChannel(values[1], "value 2");
        // scope.connectChannel(values[2], "value 3");
        // scope.connectChannel(index_1, "index 1");
        // scope.connectChannel(index_2, "index 2");
        // scope.connectChannel(index_3, "index 3");
        scope.connectChannel(g_u_1, "g_u_1");
        scope.connectChannel(g_u_2, "g_u_2");
        scope.connectChannel(g_u_3, "g_u_3");
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.0F);
        scope.start();
    }
}

/* --------------LOOP FUNCTIONS-------------------------------- */

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
               "|     press p : power mode               |\n"
               "|     press r : record data              |\n"
               "|     press a : toggle enable_acq var    |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        send_idle = false; 
        break;
    case 'r':
        is_downloading = true;
        break;
    case 'a':
        enable_acq = !(enable_acq);
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * It runs perpetually. Here a `suspendBackgroundMs` is used to pause during
 * 1000ms between each LED toggles.
 * Hence we expect the LED to blink each second.
 */
void loop_background_task()
{
    if (module_ID == MMC_LEAD)
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
            if (is_downloading)
            {
                dump_scope_datas(scope);
                is_downloading = false;
            }
        }
        if (mode == POWERMODE)
        {
            spin.led.toggle();
            printk("%1.f:", number_of_connected_submodules_upper_arm);
            printk("%1.f:", number_of_connected_submodules_lower_arm);
            printk("%u:", counter_seq);
            printk("%u:", sw_timer);
            printk("%1.f:", index_1);
            printk("%1.f:", index_2);
            printk("%1.f:", index_3);
            printk("%u:", g_u_1);
            printk("%u:", g_u_2);
            printk("%u:", g_u_3);
            printk("\n");
        }
    }
    else
    {
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
        }
    }

    task.suspendBackgroundMs(2000);
}

/**
 * Uncomment lines in setup_routine() to use critical task.
 *
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software
 * function. You can use it to execute an ultra-fast code with
 * the highest priority which cannot be interrupted by the background tasks.
 *
 * In the critical task, you can implement your control algorithm that will
 * run in Real Time and control your power flow.
 */
void loop_critical_task()
{
    if (mode == POWERMODE)
    {
        /* The lead sends commands to the followers */
        if (module_ID == MMC_LEAD)
        {
            if (sw_timer == sw_period)
            {
                if (counter_seq >= 6)
                {
                    counter_seq = 0;
                }
                number_of_connected_submodules_upper_arm = (float)seq_u[counter_seq]; // recuperate
                number_of_connected_submodules_lower_arm = (float)seq_l[counter_seq]; // recuperate
                counter_seq++;
                sw_timer = 0;
            }

            if (number_of_connected_submodules_upper_arm == 0)
            {
                g[0] = 0;
                g[1] = 0;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 1)
            {
                g[0] = 1;
                g[1] = 0;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 2)
            {
                g[0] = 1;
                g[1] = 1;
                g[2] = 0;
            }
            if (number_of_connected_submodules_upper_arm == 3)
            {
                g[0] = 1;
                g[1] = 1;
                g[2] = 1;
            }

            index_1 = (float)indexes[0]; // recuperate
            index_2 = (float)indexes[1]; // recuperate
            index_3 = (float)indexes[2]; // recuperate

            g_u_1 = (float)g[0]; // recuperate
            g_u_2 = (float)g[1]; // recuperate
            g_u_3 = (float)g[2]; // recuperate

            if (scope_timer == scope_period)
            {
                scope.acquire();
                scope_timer = 0;
            }
            sw_timer++;
            scope_timer++;

            SET_SIGNAL(dataTX_mmc.command, MMC_SM1, g[0]);
            SET_SIGNAL(dataTX_mmc.command, MMC_SM2, g[1]);
            SET_SIGNAL(dataTX_mmc.command, MMC_SM3, g[2]);

            dataTX_mmc.ID = module_ID;
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            dataTX_mmc.status = 1;
            communication.rs485.startTransmission();
        }
        else
        {
            if (module_comand != module_command_past)
            {
                change_state_command = true; // Set the flag to change the state
            }

            if (module_comand)
            {
                if (change_state_command)
                {
                    Led_turnON_LL();
                    change_state_command = false; // Reset the flag
                }
            }
            else
            {
                if (change_state_command)
                {
                    Led_turnOFF_LL();
                    change_state_command = false; // Reset the flag
                }
            }
        }
        module_command_past = module_comand; // Update the past command
    }
    else if (mode == IDLEMODE)
    {
        /* Made to send IDLE flag only once */
        if (!send_idle)
        {
            dataTX_mmc.ID = module_ID;
            dataTX_mmc.status = 0;
            memcpy(buffer_tx, &dataTX_mmc, sizeof(dataTX_mmc));
            communication.rs485.startTransmission();
            send_idle = true; // Set the flag to send idle command
        }
    }
    counter_timer++;
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