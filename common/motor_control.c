/**
 * @file motor_control.c
 *
 * motor control
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#include "system.h"
#include "..\platform\platform.h"

void AGR2RPM(motor_value* mv)
{
    mv->rpm1 = mv->w1 / M_PI;
    mv->rpm2 = mv->w2 / M_PI;
    mv->rpm3 = mv->w3 / M_PI;
    mv->rpm4 = mv->w4 / M_PI;
}

void RPM2PWM(motor_value* mv)
{
    
}
