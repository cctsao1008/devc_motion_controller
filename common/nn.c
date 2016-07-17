/**
 * @file nn.c
 *
 * neural network
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include "system.h"
#include "../platform/platform.h"

#define DEBUG false

float sigmoid(float x)
{
    float exp_value;
    float return_value;

    /* Exponential calculation */
    exp_value = exp((double) -x);

    /* Final sigmoid value */
    return_value = 1 / (1 + exp_value);

    return return_value;
}

bool neural_network_init(system_data* sd)
{

}

bool neural_network_update(system_data* sd)
{

}

