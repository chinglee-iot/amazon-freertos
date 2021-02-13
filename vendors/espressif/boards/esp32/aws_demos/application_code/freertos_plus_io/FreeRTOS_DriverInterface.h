#ifndef __FREERTOS_DRIVERINTERFACE_H__
#define __FREERTOS_DRIVERINTERFACE_H__

#define DEVICE_PATH_MAX     ( 128 )

typedef Peripheral_Descriptor_t ( *Peripheral_open_Function_t )
    ( const int8_t * pcPath, const uint32_t ulFlags );

typedef size_t ( *Peripheral_write_Function_t )
    ( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );

typedef size_t ( *Peripheral_read_Function_t )
    ( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );

typedef BaseType_t ( *Peripheral_ioctl_Function_t )
    ( Peripheral_Descriptor_t const pxPeripheral, uint32_t ulRequest, void *pvValue );

typedef struct Peripheral_device
{
    char pDevicePath[DEVICE_PATH_MAX];
    Peripheral_open_Function_t pOpen;
    Peripheral_write_Function_t pWrite;
    Peripheral_read_Function_t pRead;
    Peripheral_ioctl_Function_t pIoctl;
    void *pDeviceData;
} Peripheral_device_t;

#endif
