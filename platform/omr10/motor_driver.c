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

bool setup_port(HANDLE hComm, uint16_t baud, uint8_t data_bits, uint8_t stop_bits, bool parity, bool hardware_control)
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

    bPortReady = SetCommState(hComm, &dcb);

    /* Communication timeouts are optional */
    CommTimeouts.ReadIntervalTimeout = 0;
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

DWORD readFromSerialPort(HANDLE hComm, uint8_t * buffer, int buffersize)
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

DWORD writeToSerialPort(HANDLE hComm, uint8_t * data, int length)
{

    DWORD dwBytesRead = 0;
    if(!WriteFile(hComm, data, length, &dwBytesRead, NULL)){
        //handle error
    }
    return dwBytesRead;

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

#if 0
BOOL WriteABuffer(HANDLE hComm, unsigned char * lpBuf, DWORD dwToWrite)
{
    OVERLAPPED osWrite = {0};
    DWORD dwWritten;
    DWORD dwRes;
    BOOL fRes;

    // Create this write operation's OVERLAPPED structure's hEvent.
    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    if (osWrite.hEvent == NULL)
        // error creating overlapped event handle
        return FALSE;

    // Issue write.
    if (!WriteFile(hComm, lpBuf, dwToWrite, &dwWritten, &osWrite))
    {
        if (GetLastError() != ERROR_IO_PENDING) { 
        // WriteFile failed, but isn't delayed. Report error and abort.
        fRes = FALSE;
        }
        else
            // Write is pending.
            dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);

        switch(dwRes)
        {
            // OVERLAPPED structure's event has been signaled. 
            case WAIT_OBJECT_0 :

                if (!GetOverlappedResult(hComm, &osWrite, &dwWritten, FALSE))
                    fRes = FALSE;
                else
                    // Write operation completed successfully.
                    fRes = TRUE;
                break;
            
            default :
                // An error has occurred in WaitForSingleObject.
                // This usually indicates a problem with the
                // OVERLAPPED structure's event handle.
                fRes = FALSE;
                break;
        }
   }
   else
        // WriteFile completed immediately.
        fRes = TRUE;

   CloseHandle(osWrite.hEvent);
   return fRes;
}
#endif

bool motor_driver_init(system_data* sd)
{
    uint8_t rc;

    fflush(stdout);

    sd->hComm = open_port(com_port);
    
    if(sd->hComm ==INVALID_HANDLE_VALUE)
        return false;
        
    rc = setup_port(sd->hComm, CBR_9600, 8, 0, false, false);
    
    if(!rc)
        return false;

    return true;
}
 
bool motor_driver_update(system_data* sd)
{
    uint8_t  data = 0x5A;

    writeToSerialPort(sd->hComm, &data, 1);
    return true;
}

