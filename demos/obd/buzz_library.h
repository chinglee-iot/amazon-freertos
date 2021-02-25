#ifndef __BUZZ_LIBRARY_H__
#define __BUZZ_LIBRARY_H__

void buzz_init( Peripheral_Descriptor_t buzzDevice );
void buzz_beep( Peripheral_Descriptor_t buzzDevice, uint32_t beepDurationMs, uint32_t times );
void buzz_playtone( Peripheral_Descriptor_t buzzDevice, uint16_t freq, uint32_t durationMs );

#endif
