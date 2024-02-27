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
#include "Rs485Communication.h"
#include "SyncCommunication.h"
#include "opalib_control_pid.h"

#include "zephyr/console/console.h"


#define DMA_BUFFER_SIZE 8 // size of transfered data
#define MASTER

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

uint8_t received_serial_char;
static uint32_t control_task_period = 150; //[us] period of the control task

/* power enable variable*/
bool pwr_enable = false;
/* PID coefficient for a 8.6ms step response*/

static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

// reference voltage/current
float32_t duty_cycle = 0.3;
static float32_t Vref = 12.0;
#ifdef SLAVE2
static float32_t Iref = 0;
#endif
static float32_t Imax = 1;
static float32_t Imin = 0.2;
static float32_t Icom = 0;
int Isend;

bool flag = false;

/* Measure variables */
float32_t meas_data;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;

static uint8_t delay = 0;
static uint8_t tx_usart_val[DMA_BUFFER_SIZE];
static uint8_t rx_usart_val[DMA_BUFFER_SIZE];

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    LISTENMODE,
    POWERMODE
};

uint8_t mode = IDLEMODE;

//--------------SETUP FUNCTIONS-------------------------------

void reception_function()
{
        if (delay < 1)
        {  
            tx_usart_val[0] = rx_usart_val[0];
            tx_usart_val[1] = rx_usart_val[1];
            tx_usart_val[2] = rx_usart_val[2];
            tx_usart_val[3] = rx_usart_val[3];
            tx_usart_val[4] = rx_usart_val[4];
            tx_usart_val[5] = rx_usart_val[5];
            tx_usart_val[6] = rx_usart_val[6];
            tx_usart_val[7] = rx_usart_val[7];

            delay++;

        }


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

    #ifdef MASTER
        syncCommunication.initMaster(); // start the synchronisation
            /* Initializing TX buffer */
            tx_usart_val[0] = 0;
            tx_usart_val[1] = 0;
            tx_usart_val[2] = 0;
            tx_usart_val[3] = 0;
            tx_usart_val[4] = 0;
            tx_usart_val[5] = 0;
            tx_usart_val[6] = 0;
            tx_usart_val[7] = 0;

    #endif
    #ifdef SLAVE
        syncCommunication.initSlave(); // start the synchronisation
    #endif
    #ifdef SLAVE2
        syncCommunication.initSlave(); // start the synchronisation
    #endif

    rs485Communication.configure(tx_usart_val, rx_usart_val, DMA_BUFFER_SIZE, reception_function, 10625000, true); // custom configuration for RS485

    opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);

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
        case 'l':
            printk("listen mode\n");
            mode = LISTENMODE;
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
    }

    printk("%.2f:", V1_low_value);
    printk("%.2f:", V2_low_value);
    printk("%.2f:", I1_low_value);
    printk("%.2f:", I2_low_value);
    printk("%.2f:", I1_low_value+I2_low_value);
    #ifdef MASTER
    printk("%d:", tx_usart_val[0]);
    printk("%d:", tx_usart_val[1]);
    printk("%d:", tx_usart_val[2]);
    #endif
    #ifdef SLAVE2
    printk("%d:", rx_usart_val[0]);
    printk("%d:", rx_usart_val[1]);
    printk("%d:", rx_usart_val[2]);
    #endif
    printk("%d:\n", flag);

    #ifdef MASTER
        if (flag == true)
        {
            Icom = (I1_low_value + I2_low_value + Icom) * 0.4;
            if (Icom < 0.5)
            {
                Icom = 0.5;
            }
            Isend = (int)(Icom*255/2);
            

            tx_usart_val[0] = 2;
            tx_usart_val[1] = 1;
            tx_usart_val[2] = Isend;
            tx_usart_val[3] = 0;
            tx_usart_val[4] = 0;
            tx_usart_val[5] = 0;
            tx_usart_val[6] = 0;
            tx_usart_val[7] = 0;
        }
        else if (flag == false)
        {
            Icom = 0;
            Isend = (int)((0.45)*255/2);

            tx_usart_val[0] = 2;
            tx_usart_val[1] = 0;
            tx_usart_val[2] = Isend;
            tx_usart_val[3] = 0;
            tx_usart_val[4] = 0;
            tx_usart_val[5] = 0;
            tx_usart_val[6] = 0;
            tx_usart_val[7] = 0;
        }
    #endif

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
delay = 0;

    #ifdef SLAVE2 
        if((rx_usart_val[0]==2) && (rx_usart_val[1]==1)) mode = POWERMODE;
        else mode = LISTENMODE;
    #endif

    if (mode == IDLEMODE)
    {   
        if (pwr_enable == true)
        {
            pwr_enable = false;
            twist.stopAll();
        }

    }
    if (mode == LISTENMODE)
    {   
        #ifdef MASTER
            rs485Communication.startTransmission();
        #endif
        if (pwr_enable == true)
        {
            pwr_enable = false;
            twist.stopAll();
        }

    }
    if (mode == POWERMODE)
    {   
        if (pwr_enable == false)
        {
            pwr_enable = true;
            twist.startAll();
        }
        float32_t I12 = I1_low_value + I2_low_value;

        #ifdef MASTER

            rs485Communication.startTransmission();

            duty_cycle = opalib_control_interleaved_pid_calculation(Vref, (V1_low_value));

            if ((I12) > Imax)
            {
                flag = true;
            }

            if (((I12) < Imin))
            {
                flag = false;
            }
        #endif

        #ifdef SLAVE
            duty_cycle = opalib_control_interleaved_pid_calculation(Vref, (V1_low_value));
        #endif

        #ifdef SLAVE2
            flag =true ;
            if (rx_usart_val[0] == 2)
            {
                Iref = ((float)(rx_usart_val[2]))/255*2;
                if (Iref > 1)
                {
                    Iref = 1;
                }
                else if (Iref < 0)
                {
                    Iref = 0;
                }
            }
            duty_cycle = opalib_control_interleaved_pid_calculation(Iref, I12); 
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
