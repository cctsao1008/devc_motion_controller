/**
 * @file common_cfg.h
 *
 * common configuration file for fake guide sensor
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#pragma once

#define MM2M    1000    // convert mm to m

#ifdef USE_GUIDE_SENSOR_D16
    #include "digital_16/digital_16.h"
#endif
