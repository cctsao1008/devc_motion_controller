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
    #include "omr10/omr10_spec.h"
    #include "omr10/omr10_helper.c"
#endif
