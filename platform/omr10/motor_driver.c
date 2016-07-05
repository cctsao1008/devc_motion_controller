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
//#include <termios.h> /* POSIX terminal control definitions */

#include "..\..\common\system.h"
#include "..\platform\platform.h"

#define DEBUG false

/* protocol */
#define STC1 0xEB // start code 1
#define STC2 0x90 // start code 2

static bool initialized = false;

/* Add "\\\\.\\" for COM > 10 */
static char* com_port = (char*)"\\\\.\\COM6";

uint8_t pwm_limiter(uint8_t pwm)
{
    if(pwm < DEFAULT_MIN_PWM)
        return DEFAULT_MIN_PWM;
    else if(pwm > DEFAULT_MAX_PWM)
        return DEFAULT_MAX_PWM;

    return pwm;
}

uint8_t RPM2PWM(float rpm)
{
    float pwm = 0;

    if(rpm < DEFAULT_MIN_RPM)
        pwm = DEFAULT_MIN_PWM;
    else
        pwm = (DEFAULT_RPM2PWM_B * rpm) + DEFAULT_RPM2PWM_A;

    //printf("rpm, pwm = %f,%f \n", rpm, pwm);

    return pwm_limiter((uint8_t)pwm);
}

uint8_t bcc(char *data, uint16_t len)
{
    int i = 0, bcc = 0xFF;

    for(i = 0 ; i < len ; i++)
    {
        bcc ^= data[i];
    }

    bcc = bcc & 0xFF;

    #if DEBUG
    //MSG(sd->log, "[DEBUG] CRC-32 = 0x%X \n", bcc);
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

    if(!ReadFile(hComm, buffer, buffersize, &dwBytesRead, NULL))
    {
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

    if(!WriteFile(hComm, data, length, &dwBytesRead, NULL))
    {
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
    static bool fr1_last, fr2_last, fr3_last, fr4_last;
    static uint8_t pwm1, pwm2, pwm3, pwm4;

    bool fr1, fr2, fr3, fr4;

    char dir_forward[] = {0xEB, 0x90, 0xA1, 0x01, 0x04, 0x20}; // forward
    char motor_ctrl[] = {0xEB, 0x90, 0xA1, 0x01, 0x0F, 0x2B};
    char motor_stop[] = {0xEB, 0x90, 0xA1, 0x01, 0x00, 0x24};

    fr1 = !sd->mot.out.fr1;
    fr2 =  sd->mot.out.fr2;
    fr3 = !sd->mot.out.fr3;
    fr4 =  sd->mot.out.fr4;

    pwm1 = RPM2PWM(sd->mot.out.rpm1);
    pwm2 = RPM2PWM(sd->mot.out.rpm2);
    pwm3 = RPM2PWM(sd->mot.out.rpm3);
    pwm4 = RPM2PWM(sd->mot.out.rpm4);

    #if 1
    uint8_t wb[128] = {0xEB, 0x90, 0xA3, 0x08,
                        fr2,  fr1,  fr3,  fr4,
                        pwm2, pwm1, pwm3, pwm4};
    #else /* For regression analysis*/
    uint8_t wb[128] = {0xEB, 0x90, 0xA3, 0x08,
                        1,  0,  0,  1,
                        65, 65, 65, 65};
    #endif

    uint8_t rb[128] = {0}, rc = 0, i;

    if(sd == NULL)
    {
        MSG(sd->log, "[ERROR] motor_driver_update, failed! \n");
        return false;
    }

    #if DEBUG
    MSG(sd->log, "[INFO] motor_driver_update! \n");
    MSG(sd->log, "fr1, fr2, fr3, fr4 (1 forward, 0 reverse) = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", fr1, fr2, fr3, fr4);
    MSG(sd->log, "pwm1, pwm2, pwm3, pwm4 (MAX : 100) = \n");
    MSG(sd->log, "%9d %9d %9d %9d \n\n", pwm1, pwm2, pwm3, pwm4);
    #endif

    {
        if((fr1 != fr1_last) || (fr2 != fr2_last) || (fr3 != fr3_last) || (fr4 != fr4_last))
        {
            MSG(sd->log, "[INFO] motor_driver_update, fr changes \n");
            uart_tx(sd->hComm, motor_stop, 6);
            Sleep(600);
        }

        fr1_last = fr1;
        fr2_last = fr2;
        fr3_last = fr3;
        fr4_last = fr4;

        //uart_tx(sd->hComm, forward_test, 6);
        uart_tx(sd->hComm, motor_ctrl, 6);

        wb[127] = bcc(wb, 12);
        uart_tx(sd->hComm, wb, 12);
        uart_tx(sd->hComm, &wb[127], 1);
    }


    #if DEBUG
    //rc = uart_rx(sd->hComm, rb, 2);

    if(rc > 0)
    {
        MSG(sd->log, "[INFO] motor_driver_update, uart_rx! \n");
        for(i = 0 ; i < rc ; i++)
            printf("0x%X \n", rb[i]);
    }
    #endif

    return true;
}

