/**
 * @file system.c
 *
 * system
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "system.h"

static system_data data;

static bool initialized = false;

system_data* system_init(void)
{
    printf("%-45s", "[INFO] system_init... ");

    if(initialized == false)
    {
        initialized = true;
        printf("PASSED \n");
    }

    return &data;
}

/* parameters */

