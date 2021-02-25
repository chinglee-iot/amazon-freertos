#ifndef __GPS_LIBRARY_H__
#define __GPS_LIBRARY_H__

/* This should be the same GPS_Data. */
typedef struct ObdGpsData
{
	uint32_t ts;
	uint32_t date;
	uint32_t time;
	float lat;
	float lng;
	float alt; /* meter */
	float speed; /* knot */
	uint16_t heading; /* degree */
	uint8_t hdop;
	uint8_t sat;
	uint16_t sentences;
	uint16_t errors;
} ObdGpsData_t;

int GPSLib_GetNMEA( Peripheral_Descriptor_t obdDevice, char* buffer, int bufsize );
bool GPSLib_Begin( Peripheral_Descriptor_t obdDevice );
bool GPSLib_GetData( Peripheral_Descriptor_t obdDevice, ObdGpsData_t* gpsData );
bool GPSLib_End( Peripheral_Descriptor_t obdDevice );

#endif
