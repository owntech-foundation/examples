/*
 * Copyright (c) 2021-present OwnTech Technologies
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
 * @brief  This example deploy a simple CAN based communication using
 *         Thingset library.
 *
 * @author Jean Alinei <jean.alinei@owntech.io>
 */

/*--------------OWNTECH APIs---------------------------------- */
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SpinAPI.h"
#include "user_data_objects.h"
#include <thingset/can.h>
#include "CommunicationAPI.h"


/*--------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/*--------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
/* Code to be executed in real time in the critical task */
void loop_critical_task();

void master_reception_function();
void slave_reception_function();

/*--------------USER VARIABLES DECLARATIONS------------------- */
float32_t CAN_control_reference=520;
float32_t CAN_received_value;
bool      CAN_received_start_stop;
bool master = false;

/**
 * @brief Structure representing various measurements and statuses.
 *
 * This structure holds variables for testing RS485, Sync, analog measurements, and status information.
 */
typedef struct {
    uint8_t test_RS485;             /**< Variable for testing RS485 */
    uint8_t test_Sync;              /**< Variable for testing Sync */
    uint16_t test_CAN;             /**< Variable for testing the CAN Bus */
    bool test_bool_CAN;             /**< Boolean variable for testing the CAN Bus */
    uint16_t analog_value_measure;  /**< Analog measurement */
    uint8_t id_and_status;          /**< Status information */
} ConsigneStruct_t;

ConsigneStruct_t tx_consigne;
ConsigneStruct_t rx_consigne;
uint8_t* buffer_tx = (uint8_t*)&tx_consigne;
uint8_t* buffer_rx =(uint8_t*)&rx_consigne;

uint8_t ctrl_slave_counter;
uint8_t sync_master_counter;
uint32_t counter_time;
bool test_start;
uint8_t rs485_send = 125;
uint8_t slave_rs485_step = 25;
uint32_t analog_value_ref = 2050;

uint8_t rs485_receive;
float32_t CAN_Bus_receive;
uint8_t sync_receive;
uint16_t analog_value_receive;

bool RS485_success;
bool Analog_success;
bool Can_success;
bool Sync_success;


/*--------------SETUP FUNCTIONS------------------------------- */

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your hardware and tasks.
 *
 * In this default main, we only spawn two tasks
 *  - A background task.
 *  - A critical task is defined but not started.
 *
 * NOTE: It is important to follow the steps and initialize the hardware first
 * and the tasks second.
 */
void setup_routine()
{
    /* STEP 1 - SETUP THE HARDWARE */

    /* Buck voltage mode */
    shield.power.initBuck(ALL);
    shield.sensors.enableDefaultTwistSensors();

    /* Initializes the communicaiton */
    if(master){
        communication.sync.initMaster(); /* start the synchronisation as master */
        communication.analog.init(); /* Initializes the analog communication */
        communication.analog.setAnalogCommValue(analog_value_ref); /* Sends an analog value by default from the master at initialization */    
        communication.rs485.configure(buffer_tx, 
                                    buffer_rx, 
                                    sizeof(ConsigneStruct_t), 
                                    master_reception_function, 
                                    SPEED_20M); // custom configuration for RS485
    }else{
        communication.sync.initSlave(); /* sets the sync as slave */
        communication.analog.init(); /* Initializes the analog communication */
        communication.rs485.configure(buffer_tx, 
                                    buffer_rx, 
                                    sizeof(ConsigneStruct_t), 
                                    slave_reception_function, 
                                    SPEED_20M); // custom configuration for RS485

    }

    communication.rs485.turnOnCommunication();


    /* STEP 2 - SETUP THE TASKS */
    /* Control frames are not sent by default.*/
    uint32_t background_task_number =
                            task.createBackground(loop_background_task);

    /* Uncomment the following line if you use the critical task */
    task.createCritical(loop_critical_task, 100);

    /* STEP 3 - LAUNCH THE TASKS */
    task.startBackground(background_task_number);

    /* Uncomment the following line if you use the critical task */
    task.startCritical();


}

/*--------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the code loop of the background task
 * You can use it to execute slow code such as state-machines.
 * The pause define its pseudo-periodicity.
 */
void loop_background_task()
{
    communication.can.setCtrlEnable(true);
    communication.can.setBroadcastEnable(true);

    printk("RS485 -");
    printk("%d:", rs485_send);
    printk("%d:", rs485_receive);
    printk("Analog -");
    printk("%d:", analog_value_ref);
    printk("%d:", analog_value_receive);
    printk("CAN -");
    printk("%8.3f:", (double)CAN_control_reference);
    printk("%8.3f:", (double)CAN_received_value);
    printk("%u:", CAN_received_start_stop);
    printk("Sync -");
    printk("%d:", sync_master_counter);
    printk("Success -");
    printk("%u:",RS485_success);
    printk("%u:",Analog_success);
    printk("%u:",Can_success);
    printk("%u:",Sync_success);
    printk("\n");



    /* This pauses the task for 1000 milli seconds */
    task.suspendBackgroundMs(1000);

    if(master){
        /* Use the following function receives a control reference over CAN */
        CAN_received_value = communication.can.getCtrlReference();
        /* Use the following functions to send a start or stop order over CAN */
        communication.can.startSlaveDevice();
        // communication.can.stopSlaveDevice();
        spin.led.toggle();

    } else{
        /* The following functions retrieve any start stop order sent over CAN */
        CAN_received_start_stop = communication.can.getStartStopState();
        if(CAN_received_start_stop){
            spin.led.turnOn();
            /* The following functions sends reference over CAN */
            communication.can.setCtrlReference(CAN_control_reference);;
        }
        ctrl_slave_counter++;

    }
}

/**
 * This is the code loop of the critical task
 * It is executed every 100 micro-seconds defined in the setup_routine function.
 * You can use it to execute an ultra-fast code with the highest priority which
 * cannot be interrupted.
 */
void loop_critical_task()
{

    if(master){
            /* writting rs485 value */
            tx_consigne.test_RS485 = rs485_send;

            counter_time++; /* Counts time to not start immediately */
            if (counter_time > 50){ 
                test_start =  true; /* Starts the test after 50 periods */
            }
            communication.rs485.startTransmission();
    } else {
        meas_data = shield.sensors.getLatestValue(ANALOG_COMM);
        if (meas_data != NO_VALUE)
            analog_value_receive = (uint16_t)meas_data;
    }
}

void slave_reception_function(void)
{
    tx_consigne = rx_consigne; /* Matches the buffers to send back information */
    tx_consigne.test_bool_CAN  = CAN_received_start_stop; /* Sends the CAN start-stop state */
    tx_consigne.test_RS485 = rx_consigne.test_RS485 + slave_rs485_step; /* Sends the RS485 reference plus a certain value */
    tx_consigne.test_Sync = ctrl_slave_counter;           /* Sends the counter of the critical function*/
    tx_consigne.analog_value_measure = analog_value_receive; /* Sends the analog value measured */

    communication.rs485.startTransmission();
}

void master_reception_function(void)
{
    /* Receives the data from the slave converter */
    analog_value_receive = rx_consigne.analog_value_measure; /* receives the analog value from slave */
    rs485_receive = rx_consigne.test_RS485;          /* receives the rs485 reference value from slave */
    sync_receive = rx_consigne.test_Sync;           /* Receives the counter of the critical task from the slave*/

    if(test_start){
        if (rs485_receive == rs485_send + slave_rs485_step){  /* The RS485 is good is the master receives its reference value plus the step of the slave */
            RS485_success = true;
        }
        if (analog_value_receive - analog_value_ref > 50 || analog_value_receive - analog_value_ref > -50){ /* The analog is good if the value received by the master is within 50 quantum of the value sent by the master */
            Analog_success = true;
        }
        if(CAN_received_value == CAN_control_reference){ /* The CAN is good if the master has received the expected reference value from the slave via the control */
            Can_success = true;
        }
        if(sync_master_counter < 5 && rx_consigne.test_Sync > 10){ /* The sync is good if the master has received a counter of the slave critical function more than 5 times */
            sync_master_counter++;
            if(sync_master_counter == 5){
                Sync_success = true;
            }
        }
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