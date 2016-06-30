/**
 * @file motor_driver.c
 *
 * motor driver
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

#include <windows.h>

#include "..\..\common\system.h"
#include "..\platform\platform.h"

#define DEBUG true

/* Add "\\\\.\\" for COM > 10 */
static char* com_port = (char*)"\\\\.\\COM6";

uint8_t bcc(char *data, uint16_t length)
{
    int i = 0, bcc = 0xFF;
    
    for(i = 0 ; i < length ; i++)
    {
        bcc ^= data[i];
    }
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] CRC-32 = 0X%x \n", bcc);
    #endif
    
    return bcc;
}

/* variables used with the com port */
BOOL            bPortReady;
DCB             dcb;
COMMTIMEOUTS    CommTimeouts;

bool setup_port(HANDLE hcom, uint16_t baud, uint8_t data_bits, uint8_t stop_bits, bool parity, bool hardware_control)
{
    if(hcom == NULL)
    {
        return false;
    }

    /* struct termios options */
    if(!SetupComm(hcom, 1, 128))
    {
        MSG(sd->log, "[ERROR] setup_port(SetupComm), failed! \n");
        return false;
    }

    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);


    if(!GetCommState(hcom, &dcb))
    {
        MSG(sd->log, "[ERROR] setup_port(GetCommState), failed! \n");
        return false;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = data_bits;
    dcb.Parity = NOPARITY;
    dcb.StopBits = data_bits;
    dcb.fAbortOnError = TRUE;
    dcb.fBinary = TRUE;

    /*  set XON/XOFF */
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;

    bPortReady = SetCommState(hcom, &dcb);

    /* Communication timeouts are optional */
    CommTimeouts.ReadIntervalTimeout = 0;
    CommTimeouts.ReadTotalTimeoutConstant = 0;
    CommTimeouts.ReadTotalTimeoutMultiplier = 500;
    CommTimeouts.WriteTotalTimeoutConstant = 0;
    CommTimeouts.WriteTotalTimeoutMultiplier = 500;

    if(!SetCommTimeouts (hcom, &CommTimeouts))
    {
        MSG(sd->log, "[ERROR] setup_port(SetCommTimeouts), failed! \n");
        return false;
    }
        
    if(!GetCommState(hcom, &dcb))
    {
        MSG(sd->log, "[ERROR] setup_port(GetCommState), failed! \n");
        return false;
    }

    return true;
}

HANDLE open_port(const char* com_port)
{
    HANDLE hcom = NULL;

    hcom = CreateFile( com_port, 
                       GENERIC_READ | GENERIC_WRITE,
                       0, // exclusive access
                       0, // no security
                       OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL, // no overlapped I/O
                       0); // null template

    if(hcom == NULL)
    {
        return false;
    }

    return hcom;
}

bool close_port(HANDLE hcom)
{
    if(hcom != NULL)
        CloseHandle(hcom);
    else
    {
        return false;
    }

    return true;
}

bool motor_driver_init(system_data* sd)
{
    HANDLE hcom = NULL;

    fflush(stdout);

    hcom = open_port(com_port);

    if(hcom == NULL)
    {
        MSG(sd->log, "[ERROR] motor_driver_init, failed! \n");
        return false;
    }
        
    if(!setup_port(hcom, 9600, 8, 0, false, false))
    {
        MSG(sd->log, "[ERROR] motor_driver_init, failed! \n");
        return false;
    }

    return true;
}
 
bool motor_driver_update(system_data* sd)
{
    return true;
}

