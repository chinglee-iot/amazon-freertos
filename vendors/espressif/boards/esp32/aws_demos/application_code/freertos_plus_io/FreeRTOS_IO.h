#ifndef __FREERTOS_IO_H__
#define __FREERTOS_IO_H__

#include "FreeRTOS.h"

#define FREERTOS_IO_OKAY               ( 0 )
#define FREERTOS_IO_ERROR_BAD_PARAM    ( -1 )

typedef const void * Peripheral_Descriptor_t;

Peripheral_Descriptor_t FreeRTOS_open( const int8_t * pcPath,
                                       const uint32_t ulFlags );

size_t FreeRTOS_read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes );

size_t FreeRTOS_write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes );

BaseType_t FreeRTOS_ioctl( Peripheral_Descriptor_t const pxPeripheral,
                           uint32_t ulRequest,
                           void * pvValue );

#endif /*  __FREERTOS_IO_H__. */
