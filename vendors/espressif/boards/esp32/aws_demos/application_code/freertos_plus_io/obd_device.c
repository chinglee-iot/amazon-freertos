#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_DriverInterface.h"

Peripheral_Descriptor_t Obd_Open( const int8_t * pcPath,
                                       const uint32_t ulFlags );

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes );

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes );

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const xPeripheral,
                           uint32_t ulRequest,
                           void * pvValue );

Peripheral_device_t gObdDevice = 
{
    .pDevicePath = "/dev/obd",
    .pOpen = Obd_Open,
    .pRead = Obd_Read,
    .pWrite = Obd_Write,
    .pIoctl = Obd_Ioctl,
    .pDeviceData = NULL
};

Peripheral_Descriptor_t Obd_Open( const int8_t * pcPath,
                                       const uint32_t ulFlags )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return &gObdDevice;
}

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return 0;
}

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return 0;
}

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const xPeripheral,
                           uint32_t ulRequest,
                           void * pvValue )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return 0;
}
