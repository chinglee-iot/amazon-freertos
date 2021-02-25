#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS_IO.h"

#include "obd_library.h"
#include "gps_library.h"

#define TICKS_TO_MS( ticks )    ( ticks * 1000 / configTICK_RATE_HZ )
#define GET_UPTIME_MS()         TICKS_TO_MS( xTaskGetTickCount() )

#define GPS_DEVICE_READY_TIME_MS       ( 1000 )
#define GPS_COMMAND_TIMEOUT_MS         ( 100 )
#define GPS_NMEA_COMMAND_TIMEOUT_MS    ( 200 )

int GPSLib_GetNMEA( Peripheral_Descriptor_t obdDevice,
                    char * buffer,
                    int bufsize )
{
    return OBDLib_SendCommand( obdDevice, "ATGRR\r", buffer, bufsize, GPS_NMEA_COMMAND_TIMEOUT_MS );
}

bool GPSLib_Begin( Peripheral_Descriptor_t obdDevice )
{
    char buf[ 64 ] = { 0 };
    uint32_t startTime = GET_UPTIME_MS();
    bool retGPSBegin = false;

    OBDLib_SendCommand( obdDevice, "ATGPSON\r", buf, sizeof( buf ), GPS_COMMAND_TIMEOUT_MS );

    do
    {
        /* Wait until get NMEA is okay. */
        if( ( GPSLib_GetNMEA( obdDevice, buf, sizeof( buf ) ) > 0 ) && ( strstr( buf, ( "$G" ) ) != NULL ) )
        {
            retGPSBegin = true;
            break;
        }
    } while ( ( GET_UPTIME_MS() - startTime ) < GPS_DEVICE_READY_TIME_MS );

    return retGPSBegin;
}

bool GPSLib_GetData( Peripheral_Descriptor_t obdDevice,
                     ObdGpsData_t * gpsData )
{
    char buf[ 160 ];

    if( OBDLib_SendCommand( obdDevice, "ATGPS\r", buf, sizeof( buf ), GPS_COMMAND_TIMEOUT_MS ) == 0 )
    {
        return false;
    }

    char * s = strstr( buf, "$GNIFO," );

    if( !s )
    {
        return false;
    }

    s += 7;
    float lat = 0;
    float lng = 0;
    float alt = 0;
    bool good = false;

    do
    {
        uint32_t date = atoi( s );

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        uint32_t time = atoi( ++s );

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        if( !date )
        {
            break;
        }

        gpsData->date = date;
        gpsData->time = time;
        lat = ( float ) atoi( ++s ) / 1000000;

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        lng = ( float ) atoi( ++s ) / 1000000;

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        alt = ( float ) atoi( ++s ) / 100;
        good = true;

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        gpsData->speed = ( float ) atoi( ++s ) / 100;

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        gpsData->heading = atoi( ++s ) / 100;

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        gpsData->sat = atoi( ++s );

        if( !( s = strchr( s, ',' ) ) )
        {
            break;
        }

        gpsData->hdop = atoi( ++s );
    } while( 0 );

    if( good && ( gpsData->lat || gpsData->lng || gpsData->alt ) )
    {
        /* filter out invalid coordinates */
        good = ( abs( lat * 1000000 - gpsData->lat * 1000000 ) < 100000 && abs( lng * 1000000 - gpsData->lng * 1000000 ) < 100000 );
    }

    if( !good )
    {
        return false;
    }

    gpsData->lat = lat;
    gpsData->lng = lng;
    gpsData->alt = alt;
    return true;
}

bool GPSLib_End( Peripheral_Descriptor_t obdDevice )
{
    char buf[ 16 ] = { 0 };
    bool retEnd = false;

    if( obdDevice == NULL )
    {
        retEnd = false;
    }
    else
    {
        if( OBDLib_SendCommand( obdDevice, "ATGPSOFF", buf, sizeof( buf ), 0 ) > 0 )
        {
            retEnd = true;
        }
    }

    return retEnd;
}
