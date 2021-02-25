
#include "FreeRTOS.h"

#include "FreeRTOS_IO.h"
#include "buzz_device.h"

#define BUZZ_BEEP_FREQUENCY     ( 2000 )

#define BUZZ_BEEP_INTERVAL_MS   ( 50 )

void buzz_playtone( Peripheral_Descriptor_t buzzDevice, uint16_t freq, uint32_t durationMs )
{
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_SET_FREQUENCY, &freq );
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_ON, NULL );
    vTaskDelay( pdMS_TO_TICKS( durationMs ) );       
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_OFF, NULL );
}

void buzz_beep( Peripheral_Descriptor_t buzzDevice, uint32_t beepDurationMs, uint32_t times )
{
    uint32_t i = 0;
    uint16_t freq = BUZZ_BEEP_FREQUENCY;
    
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_SET_FREQUENCY, &freq );
    
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_ON, NULL );
    vTaskDelay( pdMS_TO_TICKS( beepDurationMs ) );
    FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_OFF, NULL );
    for( i = 0; i < ( times - 1 ); i++ )
    {
        vTaskDelay( pdMS_TO_TICKS( BUZZ_BEEP_INTERVAL_MS ) );
        FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_ON, NULL );
        vTaskDelay( pdMS_TO_TICKS( beepDurationMs ) );
        FreeRTOS_ioctl( buzzDevice, ioctlBUZZ_OFF, NULL );
    }
}

void buzz_init( Peripheral_Descriptor_t buzzDevice )
{
    
}