#include "sin_tab.h"

typedef struct {
	float32_t a;
	float32_t b;
	float32_t c;
} three_phase_t;

typedef struct {
	float32_t d;
	float32_t q;
	float32_t o;
} park_t;

typedef struct {
	float32_t alpha;
	float32_t beta;
	float32_t o;
} clark_t;

typedef struct
{
	float32_t Kp;
	float32_t Ki;
	float32_t Kw;
	float32_t control_period_us;
	float32_t integral;
	float32_t upper_bound;
	float32_t lower_bound;
	float32_t _Ki_Te;
	float32_t _Kw_Te;
} pi_instance;

static __inline__  float32_t ot_sin(float32_t theta)
{
	uint32_t theta_k;
	theta_k = ((int32_t)(theta * RAD_TO_INTEGER)) & (SINUS_ARRAY_SIZE-1);
	return sin_array[theta_k];
}

static __inline__  float32_t ot_cos(float32_t theta)
{
	return ot_sin(theta + PI_DIV_2);
}

static __inline__  clark_t clark(three_phase_t Xabc)
{
	clark_t Xab;
	Xab.alpha = 2.0/3.0 * (Xabc.a - 0.5 * (Xabc.b + Xabc.c));
	Xab.beta = SQRT3_INVERSE * (Xabc.b - Xabc.c);
	Xab.o = 2.0/3.0 * 0.5 * (Xabc.a + Xabc.b + Xabc.c);
	return Xab;
}

static __inline__  three_phase_t clark_inverse(clark_t Xab)
{
	three_phase_t Xabc;

	Xabc.a = Xab.alpha + Xab.o;
	Xabc.b = -0.5  * Xab.alpha  + SQRT3_DIV_2 * Xab.beta + Xab.o;
	Xabc.c = -0.5  * Xab.alpha  - SQRT3_DIV_2 * Xab.beta + Xab.o;

	return Xabc;
}

static __inline__  park_t rotation_to_park(clark_t Xab, float32_t theta)
{
	park_t Xdq;
	float32_t cos_theta = ot_cos(theta);
	float32_t sin_theta = ot_sin(theta);
	Xdq.d = Xab.alpha * cos_theta + Xab.beta * sin_theta;
	Xdq.q = - Xab.alpha * sin_theta + Xab.beta * cos_theta;
	Xdq.o = Xab.o;

	return Xdq;
}

static __inline__  clark_t rotation_to_clark(park_t Xdq, float32_t theta)
{
	// FIXME: change the way to have rotation_to_clark and rotation_to_clark equals
	clark_t Xab;
	float32_t cos_theta = ot_cos(theta);
	float32_t sin_theta = ot_sin(theta);
	Xab.alpha = Xdq.d * cos_theta - Xdq.q * sin_theta;
	Xab.beta = + Xdq.d * sin_theta + Xdq.q * cos_theta;
	Xab.o = Xdq.o;

	return Xab;

}

static __inline__  park_t park(three_phase_t Xabc, float32_t theta)
{
	return rotation_to_park(clark(Xabc), theta);
};

static __inline__  three_phase_t park_inverse(park_t Xdq, float32_t theta)
{
	return clark_inverse(rotation_to_clark(Xdq, theta));
};

static __inline__  void pi_init(pi_instance* p)
{
	p->integral = 0.0;
	p->_Ki_Te = p->Ki * p->control_period_us * 1.e-6F;
	p->_Kw_Te = p->Kw * p->control_period_us * 1.e-6F;
}

static __inline__  float32_t pi_calc(float32_t reference, float32_t measurement, pi_instance* p)
{
	float32_t out_sat;
	float32_t out;
	float32_t error;
	float32_t Kp_error;

	error = reference - measurement;

	Kp_error = p->Kp * error;
	p->integral = p->integral + p->_Ki_Te * error;
	out = Kp_error + p->integral;
	if (out > p->upper_bound) {
		out_sat = p->upper_bound;
		} else if (out < p->lower_bound) {
		out_sat = p->lower_bound;
	} else {
		out_sat = out;
	}
	// digital integral anti-windup
	p->integral = p->integral + p->_Kw_Te * (out_sat - out);
	return out_sat;
}

void park_current_control(three_phase_t *currents, float theta, float welec, float dcVoltage, int status, three_phase_t *voltages)
{
	park_t Idq;
	park_t Vdq;
	park_t Idq_ref;
	three_phase_t Vabc;
	static pi_instance pi_d;
	static pi_instance pi_q;
	if (status == 0) { // initialisation
		// pi for d axis
		pi_d.Ki = 35.0;
		pi_d.Kp = 0.0766;
		pi_d.Kw = 35.0*0.01;
		pi_d.control_period_us = 100.0;
		pi_d.lower_bound = -60.0;
		pi_d.upper_bound = 60.0;
		// pi for q axis
		pi_q.Ki = 35.0;
		pi_q.Kp = 0.0766;
		pi_q.Kw = 35.0*0.01;
		pi_q.control_period_us = 100.0;
		pi_q.lower_bound = -60.0;
		pi_q.upper_bound = 60.0;

		pi_init(&pi_d);
		pi_init(&pi_q);

		Vabc.a = 0.0;
		Vabc.b = 0.0;
		Vabc.c = 0.0;
		Vdq.d = 0.0;
		Vdq.q = 0.0;
		voltages->a = 0.;
		voltages->b = 0.;
		voltages->c = 0.;

	} else {

		Idq_ref.q = 10.0;
		Idq_ref.d = 0.0;
		Idq = park(*currents, theta);

		Vdq.d = pi_calc(Idq_ref.d, Idq.d, &pi_d);
		Vdq.q = pi_calc(Idq_ref.q, Idq.q, &pi_q);

		Vabc = park_inverse(Vdq, theta);
		voltages->a = Vabc.a;
		voltages->b = Vabc.b;
		voltages->c = Vabc.c;
	}
}

static __inline__ float32_t dead_time_comp(float32_t I, float32_t comp_value)
{
	float32_t dt;
	dt = comp_value * I;
	if (dt > comp_value)
		{
			dt = comp_value;
		}
	else if (I < -comp_value)
	{
	dt = -comp_value;
}
	return dt;
}

//--- Proportional Resonant -----------------------------------------------------------
typedef struct {
	float32_t num[2];
	float32_t den[3];
	float32_t Ts;
	float32_t Ki;
	float32_t Kp;
} pr_params_t;

float32_t pr_resonant(float32_t reference, float32_t mesure, float32_t w0, float32_t phi_prime, pr_params_t *pr_params)
{
	// Proportional resonant using discretization of R1h with: impulse invariant with delay compensation
	// (see page p. 55 of Ph D Digital Resonant Current Controller For Voltage Source Inverter)
	pr_params->num[0] = reference - mesure;
	pr_params->den[0] = pr_params->Ts * (pr_params->num[0] * ot_cos(phi_prime) - pr_params->num[1] * ot_cos(phi_prime - w0 * pr_params->Ts))
		+ pr_params->den[1] * 2 * ot_cos(w0*pr_params->Ts) - pr_params->den[2];
	pr_params->num[1] = pr_params->num[0];

	pr_params->den[2] = pr_params->den[1];
	pr_params->den[1] = pr_params->den[0];

	return (pr_params->Kp * pr_params->num[0] + pr_params->Ki * pr_params->den[0]);
}

