/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */

// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT)
{

	// Pre calculate and store RC
	if (fabsf(filter->RC) < 1e-6f) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

/* sets up a biquad Filter */
void BiQuadNewLpf(float filterCutFreq, biquad_t *newState, uint32_t refreshRate)
{
    float sampleRate;

    sampleRate = 1 / ((float)refreshRate * 0.000001f);

    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;

    /* setup variables */
    omega = 2 * M_PI_FLOAT * filterCutFreq / sampleRate;
    sn = sinf(omega);
    cs = cosf(omega);
    alpha = sn * sinf(M_LN2_FLOAT /2 * BIQUAD_BANDWIDTH * omega /sn);

    b0 = (1 - cs) /2;
    b1 = 1 - cs;
    b2 = (1 - cs) /2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    /* precompute the coefficients */
    newState->b0 = b0 /a0;
    newState->b1 = b1 /a0;
    newState->b2 = b2 /a0;
    newState->a1 = a1 /a0;
    newState->a2 = a2 /a0;

    /* zero initial samples */
    newState->x1 = newState->x2 = 0;
    newState->y1 = newState->y2 = 0;
}

/* Computes a biquad_t filter on a sample */
float applyBiQuadFilter(float sample, biquad_t *state)
{
    float result;

    /* compute result */
    result = state->b0 * sample + state->b1 * state->x1 + state->b2 * state->x2 -
        state->a1 * state->y1 - state->a2 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = sample;

    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}

int32_t filterApplyAverage(int32_t input, uint8_t count, int32_t averageState[])
{
    int32_t sum = 0;
    for (int ii = count - 1; ii > 0; --ii) {
        averageState[ii] = averageState[ii-1];
        sum += averageState[ii];
    }
    averageState[0] = input;
    sum += input;
    return sum / count;
}

float filterApplyAveragef(float input, uint8_t count, float averageState[])
{
    float sum = 0;
    for (int ii = count - 1; ii > 0; --ii) {
        averageState[ii] = averageState[ii-1];
        sum += averageState[ii];
    }
    averageState[0] = input;
    sum += input;
    return sum / count;
}

