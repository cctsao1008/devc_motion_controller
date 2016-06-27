/**
 * @file nexus10011_spec.h
 *
 * platform settings for NEXUS 10011 4WD mecanum wheel platform
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#define PLATFORM_NEXUS

/* Specification */
#define DEFAULT_DIFF_DRV              4               // 4WD
#define DEFAULT_L1                    (450.0f/2)      // mm, width/2
#define DEFAULT_L2                    (480.0f/2)      // mm, high/2
#define DEFAULT_WET                   0.6f            // Kg, weight
#define DEFAULT_MAX_V                 0.6f            // m/s, velocity
#define DEFAULT_MAX_W                 0.6f            // rad/s, rotation rate
#define DEFAULT_MAX_LOAD              20.0f           // Kg

/* Guide Sensors */
#define DEFAULT_GUIDE_SENSOR_OFFSET   450.0f
