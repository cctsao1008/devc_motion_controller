/**
 * @file spec.h
 *
 * platform settings for OMR 10 4WD mecanum wheel platform
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#ifdef PLATFORM_OMR

/* Specification */
#define DEFAULT_DIFF_DRV                4                     // 4WD
#define DEFAULT_L1                      ((450.0f/2)/1000.0f)  // m, width/2
#define DEFAULT_L2                      ((480.0f/2)/1000.0f)  // m, high/2
#define DEFAULT_R                       ((101.6f/2)/1000.0f)  // m, Wheel radius
#define DEFAULT_WET                     0.6f                  // Kg, weight
#define DEFAULT_MAX_VX                  0.6f                  // m/s, velocity
#define DEFAULT_MAX_VY                  0.6f                  // m/s, velocity
#define DEFAULT_MAX_W0                  0.6f                  // rad/s, rotation rate
#define DEFAULT_MAX_LOAD                20.0f                 // Kg

/* Motor Driver */
#define DEFAULT_MAX_PWM                 100.0f
#define DEFAULT_MIN_PWM                 10.0f
#define DEFAULT_MAN_RPM                 283.0f
#define DEFAULT_MIN_RPM                 12.0f
#define DEFAULT_MOTOR_POWER             30.0f                 // Watt (W)

/*
    Statistics Calculator: Linear Regression
    http://www.alcula.com/calculators/statistics/linear-regression/

    --------
    RPM, PWM
    --------
    283, 100
    256,  90
    196,  80
    145,  70
    101,  60
     80,  55
     61,  50
     42,  45
     26,  40
     12,  35
     -------

   Regression line equation: y = 35.2831183017210 + 0.22642996421197x
 */
#define DEFAULT_RPM2PWM_A               35.2831183017210f // Intercept (a)
#define DEFAULT_RPM2PWM_B               0.22642996421197f // Slope (b)

/* Guide Sensor */
#define DEFAULT_GUIDE_SENSOR_OFFSET     (450.0f/2)

#endif
