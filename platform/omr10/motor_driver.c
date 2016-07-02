/**
 * @file motor_driver.c
 *
 * motor driver
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "..\..\common\system.h"
#include "..\platform\platform.h"

#define DEBUG false

/* protocol */
#define STC1 0xEB // start code 1
#define STC2 0x90 // start code 2

static bool initialized = false;

/* Add "\\\\.\\" for COM > 10 */
static char* com_port = (char*)"\\\\.\\COM6";

uint8_t bcc(char *data, uint16_t len)
{
    int i = 0, bcc = 0xFF;
    
    for(i = 0 ; i < len ; i++)
    {
        bcc ^= data[i];
    }
    
    bcc = bcc & 0xFF;
    
    #if DEBUG
    MSG(sd->log, "[DEBUG] CRC-32 = 0x%X \n", bcc);
    #endif
    
    return bcc;
}

HANDLE open_port(const char* com_port)
{
    HANDLE hComm = NULL;

    hComm = CreateFile( com_port, 
                        GENERIC_READ | GENERIC_WRITE,
                        0, // exclusive access
                        0, // no security
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL, // no overlapped I/O
                        0); // null template

    if(hComm == INVALID_HANDLE_VALUE)
    {
        MSG(sd->log, "[ERROR] motor_driver_init, open_port failed! \n");
        return false;
    }

    return hComm;
}

bool close_port(HANDLE hComm)
{
    if(hComm != INVALID_HANDLE_VALUE)
        CloseHandle(hComm);
    else
    {
        return false;
    }

    return true;
}

/* variables used with the com port */
BOOL            bPortReady;
DCB             dcb;
COMMTIMEOUTS    CommTimeouts;

bool setup_port(HANDLE hComm, uint16_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits)
{
    if(hComm == NULL)
    {
        return false;
    }

    /* struct termios options */
    if(!SetupComm(hComm, 128, 128))
    {
        MSG(sd->log, "[ERROR] setup_port(SetupComm), failed! \n");
        return false;
    }

    SecureZeroMemory(&dcb, sizeof(DCB));
    dcb.DCBlength = sizeof(DCB);


    if(!GetCommState(hComm, &dcb))
    {
        MSG(sd->log, "[ERROR] setup_port(GetCommState), failed! \n");
        return false;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = data_bits;
    dcb.Parity = parity;
    dcb.StopBits = stop_bits;
    dcb.fAbortOnError = TRUE;
    dcb.fBinary = TRUE;

    /*  set XON/XOFF */
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;

    bPortReady = SetCommState(hComm, &dcb);

    /* Communication timeouts are optional */
    CommTimeouts.ReadIntervalTimeout = 100;
    CommTimeouts.ReadTotalTimeoutConstant = 0;
    CommTimeouts.ReadTotalTimeoutMultiplier = 500;
    CommTimeouts.WriteTotalTimeoutConstant = 0;
    CommTimeouts.WriteTotalTimeoutMultiplier = 500;

    if(!SetCommTimeouts (hComm, &CommTimeouts))
    {
        MSG(sd->log, "[ERROR] setup_port(SetCommTimeouts), failed! \n");
        return false;
    }
        
    if(!GetCommState(hComm, &dcb))
    {
        MSG(sd->log, "[ERROR] setup_port(GetCommState), failed! \n");
        return false;
    }

    MSG(sd->log, "[INFO] setup_port, passed. \n");
    return true;
}

/**
    Reading:
    To tell windows that data is going to be send over the serial port, the handle, the data and the amount of data are needed.
    The following transmission is a simple fileIO operation.
 */

DWORD uart_rx(HANDLE hComm, uint8_t * buffer, int buffersize)
{
    DWORD dwBytesRead = 0;
    if(!ReadFile(hComm, buffer, buffersize, &dwBytesRead, NULL)){
        //handle error
    }
    return dwBytesRead;
}

/**
    Writing
    The same information is needed when writing to the port.
 */

DWORD uart_tx(HANDLE hComm, uint8_t * data, int length)
{

    DWORD dwBytesRead = 0;
    if(!WriteFile(hComm, data, length, &dwBytesRead, NULL)){
        //handle error
    }
    return dwBytesRead;

}

bool motor_driver_init(system_data* sd)
{    
    MSG(sd->log, "%s", "[INFO] motor_driver_init... \n");
    
    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] motor_driver_init, failed! \n");
        return false;
    }

    if(initialized == true)
        return true;

    fflush(stdout);

    sd->hComm = open_port(com_port);
    
    if(sd->hComm ==INVALID_HANDLE_VALUE)
        return false;
    
    if(!setup_port(sd->hComm, CBR_9600, 8, NOPARITY, ONESTOPBIT))
        return false;

    initialized = true;

    return true;
}
 
bool motor_driver_update(system_data* sd)
{
    uint8_t pwm1 = (uint8_t) sd->mot.out.pwm1;
    uint8_t pwm2 = (uint8_t) sd->mot.out.pwm2;
    uint8_t pwm3 = (uint8_t) sd->mot.out.pwm3;
    uint8_t pwm4 = (uint8_t) sd->mot.out.pwm4;
    
    uint8_t fr1 = (uint8_t) sd->mot.fr1;
    uint8_t fr2 = (uint8_t) sd->mot.fr2;
    uint8_t fr3 = (uint8_t) sd->mot.fr3;
    uint8_t fr4 = (uint8_t) sd->mot.fr4;

    uint8_t wb[128] = {STC1, STC2, 0xA3, 0x08,
                        fr1,  fr2,  fr3,  fr4,
                        pwm1, pwm2, pwm3, pwm4};
    
    uint8_t rb[128] = {0}, rc = 0, i;

    if((sd == NULL) || (initialized != true))
    {
        MSG(sd->log, "[ERROR] motor_driver_update, failed! \n");
        return false;
    }

    wb[127] = bcc(wb, 12);
    uart_tx(sd->hComm, wb, 12);
    uart_tx(sd->hComm, &wb[127], 1);

    #if DEBUG
    rc = uart_rx(sd->hComm, rb,13);
    
    if(rc > 0)
    {
        for(i = 0 ; i < rc ; i++)
            printf("0x%X \n", rb[i]);
    }
    #endif

    return true;
}

