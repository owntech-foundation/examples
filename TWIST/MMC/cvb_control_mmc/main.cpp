/*
 * Copyright (c) 2023-present LAAS-CNRS
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

void sorting();

/*--------------USER VARIABLES DECLARATIONS------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

uint8_t received_serial_char;

float32_t duty_cycle = 0.3;

/* Scope variables */

static bool enable_acq; //trigger variable
static const uint16_t NB_DATAS = 1024; //Number of data acquired
static const float32_t minimal_step = 1.0F / (float32_t) NB_DATAS;
static ScopeMimicry scope(NB_DATAS, 8);
static bool is_downloading;

/* SM switching variables */

static float32_t number_of_connected_submodules_upper_arm;
static float32_t number_of_connected_submodules_lower_arm;

static bool master = true;
static uint8_t seq_u[6] = {1, 2, 3, 2, 1, 0};
static uint8_t seq_l[6] = {2, 1, 0, 1, 2, 3};
static uint8_t counter_seq = 0;
static uint32_t sw_timer = 0;
static uint32_t scope_timer = 0;
static uint32_t sw_period = 10000; // 1 Hz = 1 s to transition;
static uint32_t scope_period = 100; // acquire every 100 * 100 µs;

/* CVB variables */
static float32_t modules_capacitor_voltages_upper_arm[3] = {3.0,5.0,4.0}; // Example values to be sorted
static uint8_t modules_indexes_upper_arm[3] = {0,1,2}; // Example indexes to be sorted
static float32_t modules_capacitor_voltages_lower_arm[3] = {3.0,5.0,4.0}; // Example values to be sorted
static uint8_t modules_indexes_lower_arm[3] = {0,1,2}; // Example indexes to be sorted
static uint8_t total_number_of_modules_arm= 3;

/* Gate logic */
uint8_t g_u[3] = {0,0,0}; // Example gate signals to send
uint8_t g_l[3] = {0,0,0}; // Example gate signals to send
static int8_t i_upper_arm= 1;
static int8_t i_lower_arm= -1;
static float32_t g_u_1;
static float32_t g_u_2;
static float32_t g_u_3;
static float32_t g_l_1;
static float32_t g_l_2;
static float32_t g_l_3;


/*--------------------------------------------------------------- */

/* LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER */
enum serial_interface_menu_mode
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;

/* Trigger function for scope manager */
bool a_trigger() {
    return enable_acq;
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
 *  - Spawns three tasks.
 */
void setup_routine()
{
    /* Buck voltage mode */
    shield.power.initBuck(ALL);

    shield.sensors.enableDefaultTwistSensors();

    /* Configure scope channels, what measurements do you want to acquire? */
    scope.connectChannel(number_of_connected_submodules_upper_arm, "N_u");
    scope.connectChannel(number_of_connected_submodules_lower_arm, "N_l");
    scope.connectChannel(g_u_1, "g_u_1");
    scope.connectChannel(g_u_2, "g_u_2");
    scope.connectChannel(g_u_3, "g_u_3");
    scope.connectChannel(g_l_1, "g_l_1");
    scope.connectChannel(g_l_2, "g_l_2");
    scope.connectChannel(g_l_3, "g_l_3");
    scope.set_trigger(&a_trigger);
    scope.set_delay(0.0F);
    scope.start();

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
 * This task mostly logs back measurements to the USB serial interface.
 */
void loop_application_task()
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
        printk("%u:", g_u_1);
        printk("%u:", g_u_2);
        printk("%u:", g_u_3);
        printk("%u:", g_l_1);
        printk("%u:", g_l_2);
        printk("%u:", g_l_3);
        printk("\n");
    }
    task.suspendBackgroundMs(1000);

}


void sorting()
{
    uint8_t counter_loops_sorting = 0;
    while(counter_loops_sorting < 10){ // Sorts modules indexes according to capacitor voltage
            for(uint8_t counter = 0; counter < total_number_of_modules_arm-1; counter++)
            {
                if(modules_capacitor_voltages_upper_arm[counter] > modules_capacitor_voltages_upper_arm[counter + 1])
                {
                    float32_t temp = modules_capacitor_voltages_upper_arm[counter];
                    modules_capacitor_voltages_upper_arm[counter] = modules_capacitor_voltages_upper_arm[counter + 1];
                    modules_capacitor_voltages_upper_arm[counter + 1] = temp;
                    float32_t temp2 = modules_indexes_upper_arm[counter];
                    modules_indexes_upper_arm[counter] = modules_indexes_upper_arm[counter + 1];
                    modules_indexes_upper_arm[counter + 1] = temp2;
                }

                if(modules_capacitor_voltages_lower_arm[counter] > modules_capacitor_voltages_lower_arm[counter + 1])
                {
                    float32_t temp = modules_capacitor_voltages_lower_arm[counter];
                    modules_capacitor_voltages_lower_arm[counter] = modules_capacitor_voltages_lower_arm[counter + 1];
                    modules_capacitor_voltages_lower_arm[counter + 1] = temp;
                    float32_t temp2 = modules_indexes_lower_arm[counter];
                    modules_indexes_lower_arm[counter] = modules_indexes_lower_arm[counter + 1];
                    modules_indexes_lower_arm[counter + 1] = temp2;
                }
            }

            counter_loops_sorting++;
        }
    g_u[0] = 0;
    g_u[1] = 0;
    g_u[2] = 0;
    g_l[0] = 0;
    g_l[1] = 0;
    g_l[2] = 0;
    
    for(uint8_t counter = 0; counter < total_number_of_modules_arm; counter++) // Choses the modules to connect according to sorted indexes
        {
            if(counter < number_of_connected_submodules_upper_arm)
                {
                    if(i_upper_arm>=0)
                    {
                        uint8_t index_smallest_voltage_capacitor_upper_arm = modules_indexes_upper_arm[counter];
                        g_u[index_smallest_voltage_capacitor_upper_arm] = 1;
                    }
                    else{
                        uint8_t higher_index = total_number_of_modules_arm-1-counter;
                        uint8_t index_highest_voltage_capacitor_upper_arm = modules_indexes_upper_arm[higher_index];
                        g_u[index_highest_voltage_capacitor_upper_arm] = 1;
                    }

                }
            if(counter < number_of_connected_submodules_lower_arm)
                {
                    if(i_lower_arm>=0)
                    {
                        uint8_t index_smallest_voltage_capacitor_lower_arm = modules_indexes_lower_arm[counter];
                        g_l[index_smallest_voltage_capacitor_lower_arm] = 1;
                    }
                    else{
                        uint8_t higher_index = total_number_of_modules_arm-1-counter;
                        uint8_t index_highest_voltage_capacitor_lower_arm = modules_indexes_lower_arm[higher_index];
                        g_l[index_highest_voltage_capacitor_lower_arm] = 1;
                    }
                }
        }

}


/**
 * This is the code loop of the critical task
 * This task runs at 10kHz.
 * - It update main N_u and N_l
 * - It update sets follower logic -> SM on or off
 * - It does the voltage sorting
 * - It creates the gate signals to the SM
 */
void loop_critical_task()
{   
    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            shield.power.stop(ALL);
        }
        pwm_enable = false;
    }

    if (mode == POWERMODE)
    {
        if (sw_timer == sw_period)
        {
            if (counter_seq >= 6) {
                counter_seq = 0;
            }
            number_of_connected_submodules_upper_arm = (float)seq_u[counter_seq];  // recuperate
            number_of_connected_submodules_lower_arm = (float)seq_l[counter_seq];  // recuperate
            counter_seq++;
            sw_timer = 0;
        }

        sorting();

        g_u_1 = (float)g_u[0];  // recuperate for scope acquisition
        g_u_2 = (float)g_u[1];  // recuperate for scope acquisition
        g_u_3 = (float)g_u[2];  // recuperate for scope acquisition

        g_l_1 = (float)g_l[0];  // recuperate for scope acquisition
        g_l_2 = (float)g_l[1];  // recuperate for scope acquisition
        g_l_3 = (float)g_l[2];  // recuperate for scope acquisition
        
        if (scope_timer == scope_period)
        {
            scope.acquire();
            scope_timer = 0;
        }
        sw_timer++;
        scope_timer++;
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