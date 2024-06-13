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
 */
//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "ScopeMimicry.h"
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h"
#include "zephyr/console/console.h"
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_background_task(); // Code to be executed in the background task
void loop_critical_task();   // Code to be executed in real time in the critical
void application_task();
//--------------USER VARIABLES DECLARATIONS-------------------
#define HALL1 PA7
#define HALL2 PC6
#define HALL3 PD2

static const float32_t AC_CURRENT_LIMIT = 3.0;
static const float32_t DC_CURRENT_LIMIT = 2.0;
static const float32_t MIN_DC_VOLTAGE = 30.0F; // for control
static const float32_t V_HIGH_MIN = 5.0;       // to start the POWER
static const float32_t Ts = 100.e-6F;
static const uint32_t control_task_period = (uint32_t)(Ts * 1.e6F);

/* Hall effect sensors */
static uint8_t HALL1_value;
static uint8_t HALL2_value;
static uint8_t HALL3_value;

static uint8_t angle_index;
static float32_t hall_angle;
static PllAngle pllangle = controlLibFactory.pllAngle(Ts, 10.0F, 0.04F);
static PllDatas pllDatas;
static float32_t angle_filtered;
static float32_t w_meas;
/*
 * one sector for one index value
 * index = H1*2^0 + H2*2^1 + H3*2^2
 */
static int16_t sector[] = {-1, 5, 1, 0, 3, 4, 2};
static float32_t k_angle_offset = 0.0F;

/* LEG meas */
static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;
static const float32_t NB_OFFSET = 2000.0;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V12_value;

/* DC meas */
static float32_t I_high;
static float32_t V_high;

/* Three phase system and dqo */
static three_phase_t Vabc;
static three_phase_t duty_abc;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;
static float32_t angle_4_control;

/* variables used to get static value for scopemimicry */
static three_phase_t Iabc_ref;
static float32_t duty_a, duty_b;
static float32_t Ia_ref;
static float32_t Ib_ref;
static float32_t Va;
static float32_t Iq_meas;
static float32_t Iq_ref;
static float32_t Iq_max;
static float32_t Vd, Vq;
static float32_t HALL1_value_f;
static float32_t HALL2_value_f;
static float32_t HALL3_value_f;
static float32_t angle_index_f;

// We make only torque control.
static float32_t manual_Iq_ref;

static LowPassFirstOrderFilter vHigh_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter w_mes_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static float32_t V_high_filtered;
static float32_t inverse_Vhigh;

/*--- PID ---*/
static float32_t Kp = 30 * 0.035;
static float32_t Ti = 0.002029;
static float32_t Td = 0.0F;
static float32_t N = 1.0;
/* coef 0.4 comes from Va_max =  (α_max - 0.5) * Udc */
static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.4;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.4;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
const static uint32_t decimation = 10;
static uint32_t counter_time;
float32_t counter_time_f;
static float32_t w_estimate;
uint8_t received_serial_char;
enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH
				// CONVERTER
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

enum control_state_mode {
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

enum control_state_mode control_state;
static float32_t control_state_f;

static uint16_t error_counter;
static bool pwm_enable;
uint8_t asked_mode = IDLEMODE;

const uint16_t SCOPE_SIZE = 512;
uint16_t k_app_idx;
ScopeMimicry scope(SCOPE_SIZE, 10);
static bool is_downloading;
static bool memory_print;

bool mytrigger()
{
	return (control_state == POWER_ST);
}

void dump_scope_datas(ScopeMimicry &scope) {
	printk("begin record\n");
	scope.reset_dump();
	while (scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

void init_filt_and_reg(void)
{
	pllangle.reset(0.F);
	vHigh_filter.reset(V_HIGH_MIN);
	pi_d.reset();
	pi_q.reset();
	error_counter = 0;
}

/**
 * @brief a periodmeter function which estimate pulsation
 * for the sector variable (one integer value correspond to π/3).
 *
 * @param sector assume sector is integer in [0, 5]
 * @param time in [s]
 * @return pulsation (float)
 */
float32_t pulsation_estimator(int16_t sector, float32_t time)
{
	static float32_t w_estimate_intern = 0.0F;
	static int16_t prev_sector;
	static float32_t prev_time = 0.0F;
	int16_t delta_sector;
	float32_t sixty_degre_step_time;
	delta_sector = sector - prev_sector;
	prev_sector = sector;
	if (delta_sector == 1 || delta_sector == -5) {
		// positive speed
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	if (delta_sector == -1 || delta_sector == 5) {
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (-PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	return w_estimate_intern;
}

inline void retrieve_analog_datas()
{
	meas_data = data.getLatest(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data + I1_offset;
	}

	meas_data = data.getLatest(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data + I2_offset;
	}

	if (control_state == OFFSET_ST && counter_time < NB_OFFSET) {
		tmpI1_offset += I1_low_value;
		tmpI2_offset += I2_low_value;
	}

	meas_data = data.getLatest(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}

	meas_data = data.getLatest(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_high = -meas_data; // normal when we see the kicad
	}

	meas_data = data.getLatest(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}

	meas_data = data.getLatest(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	// Mesure de Vhigh
	V_high_filtered = vHigh_filter.calculateWithReturn(V_high);

	V12_value = V1_low_value - V2_low_value;
}

inline void get_position_and_speed()
{
	// we build angle index using HALL values.
	HALL1_value = spin.gpio.readPin(HALL1);
	HALL2_value = spin.gpio.readPin(HALL2);
	HALL3_value = spin.gpio.readPin(HALL3);
	angle_index = HALL3_value * 4 + HALL2_value * 2 + HALL1_value * 1;
	hall_angle = ot_modulo_2pi(PI / 3.0 * sector[angle_index] + PI * k_angle_offset / 24.0);
	w_estimate = pulsation_estimator(sector[angle_index], counter_time * Ts);
	pllDatas = pllangle.calculateWithReturn(hall_angle);

	angle_filtered = pllDatas.angle;
	w_meas = w_mes_filter.calculateWithReturn(pllDatas.w);
}

inline void overcurrent_mngt()
{
	if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT ||
	    I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT ||
	    I_high > DC_CURRENT_LIMIT) {
		error_counter++;
	}
	if (error_counter > 2) {
		control_state = ERROR_ST;
	}
}

inline void stop_pwm_and_reset_states_ifnot()
{
	if (pwm_enable == true) {
		twist.stopAll();
		// reset filters and pid
		init_filt_and_reg();
		pwm_enable = false;
	}
}

inline void control_torque()
{
	angle_4_control = angle_filtered;
	Idq_ref.q = manual_Iq_ref;
	if (Idq_ref.q > Iq_max) {
		Idq_ref.q = Iq_max;
	}
	if (Idq_ref.q < -Iq_max) {
		Idq_ref.q = -Iq_max;
	}
	Idq_ref.d = 0.0F;
	Iabc.a = I1_low_value;
	Iabc.b = I2_low_value;
	Iabc.c = -(Iabc.a + Iabc.b);
	Idq = Transform::to_dqo(Iabc, angle_4_control);
	Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
	Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
	Vdq.o = 0.0F;
	Vabc = Transform::to_threephase(Vdq, angle_4_control);
}

inline void compute_duties()
{
	inverse_Vhigh = 1.0 / MIN_DC_VOLTAGE;
	duty_abc.a = (Vabc.a * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.a, comp_dt);
	duty_abc.b = (Vabc.b * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.b, comp_dt);
	duty_abc.c = (Vabc.c * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.c, comp_dt);
}

inline void apply_duties()
{
	twist.setLegDutyCycle(LEG1, duty_abc.a);
	twist.setLegDutyCycle(LEG2, duty_abc.b);
	twist.setLegDutyCycle(LEG3, duty_abc.c);
}

void start_pwms_ifnot()
{
	if (!pwm_enable) {
		pwm_enable = true;
		twist.startAll();
	}
}

void init_variables()
{
	/* to count the time */
	counter_time = 0;
	/* variable for measurements */
	I1_low_value = 0.0F;
	I2_low_value = 0.0F;
	I_high = 0.0F;
	V_high = 0.0F;
	/* offset variables */
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	/* state view of the pwm */
	pwm_enable = false;
	/* idle or power mode*/
	asked_mode = IDLEMODE;
	/* we begin to measure the current offset before all */
	control_state = OFFSET_ST;
	/* */
	Iq_max = 2.0;
	manual_Iq_ref = 0.0F;
}
//--------------SETUP FUNCTIONS-------------------------------
/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data
 * and/or tasks. In this example, we setup the version of the spin board and a
 * background task. The critical task is defined but not started.
 */
void setup_routine()
{
	// Setup the hardware first
	twist.setVersion(shield_ownverter);
	twist.initAllBuck();
	data.enableTwistDefaultChannels();

	spin.gpio.configurePin(HALL1, INPUT);
	spin.gpio.configurePin(HALL2, INPUT);
	spin.gpio.configurePin(HALL3, INPUT);

	// scope configuration
	scope.connectChannel(V12_value, "V12_value");           /* 0 */
	scope.connectChannel(Vq, "Vq");                         /* 1 */
	scope.connectChannel(Vd, "Vd");                         /* 2 */
	scope.connectChannel(I1_low_value, "I1_low_value");     /* 3 */
	scope.connectChannel(Iq_meas, "Iq_meas");               /* 4 */
	scope.connectChannel(angle_filtered, "angle_filtered"); /* 5 */
	scope.connectChannel(Ib_ref, "Ib_ref");                 /* 6 */
	scope.connectChannel(hall_angle, "hall_angle");         /* 7 */
	scope.connectChannel(Ia_ref, "Ia_ref");                 /* 8 */
	scope.connectChannel(control_state_f, "control_state"); /* 9 */
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();
	/* Initialize values */
	init_filt_and_reg();
	init_variables();
	spin.led.turnOn();
	// Declare tasks
	uint32_t background_task_number = task.createBackground(loop_background_task);
	uint32_t app_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, control_task_period);
	// Finally, start tasks
	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();
}
//--------------LOOP FUNCTIONS--------------------------------
void loop_background_task()
{
	// Task content
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'h':
		printk("help");
		break;
	case 'p':
		printk("power asked");
		asked_mode = POWERMODE;
		scope.start();
		break;
	case 'i':
		printk("idle asked");
		asked_mode = IDLEMODE;
		break;
	case 'r':
		is_downloading = true;
	case 'u':
		manual_Iq_ref += 0.1;
		break;
	case 'd':
		manual_Iq_ref -= 0.1F;
		break;
	case 'm':
		/* to print scope datas in ownplot as possible */
		memory_print = !memory_print;
		break;
	case 'q':
		/* relaunch scope acquisition */
		scope.start();
		break;
	}
}

void application_task()
{
	if (!memory_print) {
		printk("%7.2f", V_high);
		printk("%7.2f:", k_angle_offset);
		printk("%7.2f:", Iq_max);
		printk("%7.2f:", manual_Iq_ref);
		printk("%7.2f:", I1_offset);
		printk("%7d\n", control_state);
	} else {
		/* If memory_print is true then we plot scope datas in an infinite loop
		 * This can be used with ownplot if you have not python script installed to
		 * plot downloaded data using dump_scope_datas().
		* */
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.2f:", scope.get_channel_value(k_app_idx, 0));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 1));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 2));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 3));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 4));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 5));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 6));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 7));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 8));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 9));
		printk("\n");
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}
	switch (control_state) {
	case OFFSET_ST:
		if (counter_time > (uint32_t)NB_OFFSET) {
			spin.led.turnOff();
			I1_offset = -tmpI1_offset / NB_OFFSET;
			I2_offset = -tmpI2_offset / NB_OFFSET;
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) && (V_high_filtered > V_HIGH_MIN)) {
			control_state = POWER_ST;
		}
		break;

	case POWER_ST:
		if (asked_mode == IDLEMODE) {
			control_state = IDLE_ST;
		}
		break;

	case ERROR_ST:
		if (asked_mode == IDLEMODE) {
			error_counter = 0;
			control_state = IDLE_ST;
		}
		break;
	}

	task.suspendBackgroundMs(250);
}

void loop_critical_task()
{
	counter_time++;

	retrieve_analog_datas();

	get_position_and_speed();

	overcurrent_mngt();

	switch (control_state) {
	case OFFSET_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case IDLE_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case ERROR_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case POWER_ST:
		control_torque();
		compute_duties();
		apply_duties();
		start_pwms_ifnot();
		break;
	}

	if (counter_time % decimation == 0) {
		angle_index_f = angle_index;
		Va = Vabc.a;
		duty_a = duty_abc.a;
		duty_b = duty_abc.b;
		Iq_ref = Idq_ref.q;
		Iq_meas = Idq.q;
		Vd = Vdq.d;
		Vq = Vdq.q;
		Iabc_ref = Transform::to_threephase(Idq_ref, angle_4_control);
		Ia_ref = Iabc_ref.a;
		Ib_ref = Iabc_ref.b;
		counter_time_f = (float32_t)counter_time;
		HALL1_value_f = HALL1_value;
		HALL2_value_f = HALL2_value;
		HALL3_value_f = HALL3_value;
		control_state_f = control_state;
		scope.acquire();
	}
}

int main(void)
{
	setup_routine();

	return 0;
}
