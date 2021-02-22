#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_DriverInterface.h"

#include "Arduino.h"
#include "FreematicsPlus.h"

#include "buzz_device.h"

extern FreematicsESP32 sys;

Peripheral_Descriptor_t Buzz_Open( const int8_t * pcPath,
                                       const uint32_t ulFlags );

BaseType_t Buzz_Ioctl( Peripheral_Descriptor_t const xPeripheral,
                           uint32_t ulRequest,
                           void * pvValue );

Peripheral_device_t gBuzzDevice = 
{
    "/dev/buzz",
    Buzz_Open,
    NULL,           /* Empty Write function. */
    NULL,           /* Empty Read function. */
    Buzz_Ioctl,
    NULL
};

Peripheral_Descriptor_t Buzz_Open( const int8_t * pcPath,
                                  const uint32_t ulFlags )
{
    Peripheral_Descriptor_t retDesc = NULL;

    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    sys.buzzer(0);
    retDesc = ( Peripheral_Descriptor_t )&gBuzzDevice;

    return retDesc;
}

BaseType_t Buzz_Ioctl( Peripheral_Descriptor_t const pxPeripheral,
                      uint32_t ulRequest,
                      void * pvValue )
{
    Peripheral_device_t *pDevice = ( Peripheral_device_t* )pxPeripheral;
    BaseType_t retValue = pdPASS;
    /* TODO : remove later. */
    static uint32_t buzzFrequency = 1000;
    
    // configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    if( pDevice == NULL )
    {
        configPRINTF(("Buzz_Ioctl bad param pxPeripheral\r\n"));
        retValue = pdFAIL;
    }
    else
    {
        switch( ulRequest ) 
        {
            case ioctlBUZZ_ON:
                sys.buzzer(buzzFrequency);
                break;
            case ioctlBUZZ_OFF:
                sys.buzzer(0);
                break;
            case ioctlBUZZ_SET_FREQUENCY:
                /* TODO : add frequency ioctl. */
                break;
            default:
                break;
        }
    }
    return retValue;
}
