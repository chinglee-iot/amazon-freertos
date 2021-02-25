#ifndef __OBD_LIBRARY_H__
#define __OBD_LIBRARY_H__

#define MAX_DTC_CODES           ( 6U )

size_t OBDLib_SendCommand( Peripheral_Descriptor_t obdDevice, const char* pCmd, 
    char *pBuf, uint32_t bufSize, uint32_t readTimeout );

bool OBDLib_ReadPID( Peripheral_Descriptor_t obdDevice, uint8_t pid, int *pResult);

int OBDLib_Init( Peripheral_Descriptor_t obdDevice );

int OBDLib_ReadDTC( Peripheral_Descriptor_t obdDevice, uint16_t codes[], uint8_t maxCodes );

void OBDLib_ClearDTC( Peripheral_Descriptor_t obdDevice );

bool ODBLib_GetVIN( Peripheral_Descriptor_t obdDevice, char* buffer, uint8_t bufsize );


#endif
