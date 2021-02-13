#include <string.h>

#include "FreeRTOS_IO.h"
#include "FreeRTOS_DriverInterface.h"

#include "obd_device.h"

static Peripheral_device_t *pPeripheralDevices[] =
{
    &gObdDevice
};

Peripheral_Descriptor_t FreeRTOS_open( const int8_t * pcPath,
                                       const uint32_t ulFlags )
{
    uint32_t i = 0;
    uint32_t deviceListSize = sizeof(pPeripheralDevices) / sizeof(Peripheral_device_t*);
    Peripheral_device_t *pPeripheralDevice = NULL;
    
    configPRINTF(("Device number %u\r\n", deviceListSize ));
    
    for( i = 0; i < deviceListSize; i++ )
    {
        if( strcmp( pPeripheralDevices[i]->pDevicePath, ( const char* )pcPath ) == 0 )
        {
            pPeripheralDevice = pPeripheralDevices[i];
            break;
        }
        else
        {
            configPRINTF(("%s != %s\r\n", pcPath, pPeripheralDevices[i]->pDevicePath ));
        }
    }

    /* Call the open function. */
    if( ( pPeripheralDevice != NULL ) && ( pPeripheralDevice->pOpen != NULL ) )
    {
        pPeripheralDevice = ( Peripheral_device_t* )pPeripheralDevice->pOpen( pcPath, ulFlags );
    }

    return ( Peripheral_Descriptor_t )pPeripheralDevice;
}

size_t FreeRTOS_read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes )
{
    Peripheral_device_t *pPeripheralDevice = NULL;
    size_t retRead = 0;

    if( pxPeripheral != NULL )
    {
        pPeripheralDevice = ( Peripheral_device_t* ) pxPeripheral;
        if( pPeripheralDevice->pRead != NULL )
        {
            retRead = pPeripheralDevice->pRead( pPeripheralDevice, pvBuffer, xBytes );
        }
    }
    
    return retRead;
}

size_t FreeRTOS_write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes )
{
    Peripheral_device_t *pPeripheralDevice = NULL;
    size_t retWrite = 0;

    if( pxPeripheral != NULL )
    {
        pPeripheralDevice = ( Peripheral_device_t* ) pxPeripheral;
        if( pPeripheralDevice->pWrite != NULL )
        {
            retWrite = pPeripheralDevice->pWrite( pPeripheralDevice, pvBuffer, xBytes );
        }
    }
    
    return retWrite;
}

BaseType_t FreeRTOS_ioctl( Peripheral_Descriptor_t const pxPeripheral,
                           uint32_t ulRequest,
                           void * pvValue )
{
    Peripheral_device_t *pPeripheralDevice = NULL;
    size_t retIoctl = 0;

    if( pxPeripheral != NULL )
    {
        pPeripheralDevice = ( Peripheral_device_t* ) pxPeripheral;
        if( pPeripheralDevice->pIoctl != NULL )
        {
            retIoctl = pPeripheralDevice->pIoctl( pPeripheralDevice, ulRequest, pvValue );
        }
    }
    
    return retIoctl;
}
