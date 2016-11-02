#include "kalmanfilter.h"

void kalman_init(kalman_state* state, double q, double r, double p,
		double intial_value)
{
	state->q = q;
	state->r = r;
	state->p = p;
	state->x = intial_value;
}

void kalman_update(kalman_state* state, double measurement)
{
	if (state->is_initialized)
	{
		/* prediction update */
		state->p = state->p + state->q;

		/* measurement update */
		state->k = state->p / (state->p + state->r);
		state->x = state->x + state->k * (measurement - state->x);
		state->p = (1 - state->k) * state->p;
	}
	else
	{
		state->x = measurement;
	}
}
