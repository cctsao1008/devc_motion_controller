/**
 * @file platform.h
 *
 * common configuration file.
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#define PLATFORM_OMR 

#ifdef PLATFORM_OMR
    #include "omr10/spec.h"
    #include "omr10/helper.c"
    #include "omr10/motor_driver.c"
#endif
