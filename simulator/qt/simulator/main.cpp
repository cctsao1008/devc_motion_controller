#include "simulator.h"
#include <QApplication>
//#include <QPushButton>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <time.h>
//#include <unistd.h>
#include "../../../common/system.h"
#include "../../../platform/platform.h"

#define DEFAULT_SERIAL_PORT         "\\\\.\\COM6"
#define DEFAULT_LOOP_TIME           50

#define BILLION      1000000000L

#define EN_INFO_F    false
#define EN_INFO_T    true
#define EN_INFO_C    true

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    simulator sim;

    /* time, clock... */
    //struct timespec start, end;
    time_t t = time(NULL);
    struct tm tm= *localtime(&t);
    double  t_elap = 0.0f;
    uint64_t t_diff = 0;
    clock_t ticks = 0;

    /* file... */
    char log[128] = {0};
    char log_name[64];
    FILE *pLog = NULL;

    /* system... */
    system_data *sd = NULL;

    /* serial port */
    char port[64] = DEFAULT_SERIAL_PORT;

    //QPushButton button("Hello world");
    //button.show();
    sim.show();

    return a.exec();
}
