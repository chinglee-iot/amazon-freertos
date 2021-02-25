#include <string.h>
#include <sys/random.h>

#include "FreeRTOSConfig.h"
#include "iot_demo_logging.h"
#include "task.h"
#include "semphr.h"
#include "platform/iot_network.h"

#include "core_mqtt.h"

#include "obd_data.h"
#include "FreeRTOS_IO.h"

#include "obd_pid.h"
#include "obd_device.h"
#include "buzz_device.h"

#include "obd_library.h"
#include "gps_library.h"

/* The simulated car info. */
#define CAR_GAS_TANK_SIZE                      ( 50.0 )      /* Litres. */
#define CAR_HIGH_SPEED_THRESHOLD               ( 120.0 )     /* KM/hr. */
#define CAR_IDLE_SPEED_THRESHOLD               ( 0.0 )       /* KM/hr. */
#define CAR_IGINITION_IDLE_OFF_MS              ( 30 * 1000 ) /* Idle interval time to trigger iginition off. */
#define CAR_ACCELARATOR_PADEL_RPM_THRESHOLD    ( 6000.0 )

/* The OBD data collect interval time. */
#define OBD_DATA_COLLECT_INTERVAL_MS           ( 2000 )

#define OBD_AGGREGATED_DATA_INTERVAL_MS        ( 20000 )
#define OBD_TELEMETRY_DATA_INTERVAL_MS         ( 2000 )
#define OBD_LOCATION_DATA_INTERVAL_MS          ( 4000 )

#define OBD_MESSAGE_BUF_SIZE                   ( 1024 )
#define OBD_TOPIC_BUF_SIZE                     ( 64 )

#define OBD_AGGREGATED_DATA_INTERVAL_STEPS     ( OBD_AGGREGATED_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_TELEMETRY_DATA_INTERVAL_STEPS      ( OBD_TELEMETRY_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_LOCATION_DATA_INTERVAL_STEPS       ( OBD_LOCATION_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )

#define OBD_ISO_TIME_MAX                       ( 32 )
#define OBD_VIN_MAX                            ( 32 )
#define OBD_IGNITION_MAX                       ( 4 )
#define OBD_TRANSMISSION_GEAR_POSITION_MAX     ( 10 )
#define OBD_TRIP_ID_MAX                        ( 16 )

#define TIME_SELECTION_NONE                    ( 0 )
#define TIME_SELECTION_GPS                     ( 1 )
#define TIME_SELECTION_UPTIME                  ( 2 )

#define OBD_DEFAULT_VIN                        "78D4H8DTVH6B25TQY\0"

/* MQTT Handle is opaque pointer. */
struct MQTTConnection;
typedef struct MQTTConnection * MQTTHandle_t;

typedef struct obdContext
{
    MQTTHandle_t pMqttContext;
    ObdAggregatedData_t obdAggregatedData;
    ObdTelemetryData_t obdTelemetryData;
    char tripId[ 16 ];
    char vin[ OBD_VIN_MAX ];
    char ignition_status[ OBD_IGNITION_MAX ];
    char transmission_gear_position[ OBD_TRANSMISSION_GEAR_POSITION_MAX ];
    double latitude;
    double longitude;
    double odometer;
    bool brake_pedal_status;
    double fuel_level; /* 0 to 100 in %. */
    double start_fuel_level;
    double fuel_consumed_since_restart;
    uint64_t highSpeedDurationMs;
    uint64_t idleSpeedDurationMs;
    uint64_t idleSpeedDurationIntervalMs;
    ObdTelemetryDataType_t telemetryIndex;
    char topicBuf[ OBD_TOPIC_BUF_SIZE ];
    char messageBuf[ OBD_MESSAGE_BUF_SIZE ];
    Peripheral_Descriptor_t obdDevice;
    Peripheral_Descriptor_t buzzDevice;
    char isoTime[ OBD_ISO_TIME_MAX ];
    uint8_t timeSelection;
    uint64_t startTicks;
    uint64_t lastUpdateTicks; /* Uptime ticks. */
    uint32_t updateCount;
} obdContext_t;

static obdContext_t gObdContext =
{
    .pMqttContext                = NULL,
    .obdAggregatedData           = { 0 },
    .obdTelemetryData            = { 0 },
    .tripId                      = "123",
    .vin                         = "WASM_test_car",
    .telemetryIndex              = 0,
    .latitude                    = 0.0,
    .longitude                   = 0.0,
    .transmission_gear_position  = "neutral",
    .isoTime                     = "1970-01-01 00:00:00.000000000",
    .timeSelection               = TIME_SELECTION_NONE,
    .lastUpdateTicks             = 0,
    .updateCount                 = 0,
    .highSpeedDurationMs         = 0,
    .idleSpeedDurationMs         = 0,
    .idleSpeedDurationIntervalMs = 0,
    .obdDevice                   = NULL,
    .buzzDevice                  = NULL
};

static const char OBD_DATA_AGGREGATED_TOPIC[] = "connectedcar/trip/%s";
static const char OBD_DATA_AGGREGATED_FORMAT[] =
    "{ \r\n\
    \"vehicle_speed_mean\": %lf,     \r\n\
    \"engine_speed_mean\": %lf, \r\n\
    \"torque_at_transmission_mean\": %lf, \r\n\
    \"oil_temp_mean\": %lf, \r\n\
    \"accelerator_pedal_position_mean\": %lf, \r\n\
    \"brake_mean\": %lf, \r\n\
    \"high_speed_duration\": %lf, \r\n\
    \"high_acceleration_event\": %d, \r\n\
    \"high_braking_event\": %d, \r\n\
    \"idle_duration\": %lf, \r\n\
    \"start_time\": \"%s\", \r\n\
    \"ignition_status\": \"%s\", \r\n\
    \"brake_pedal_status\": \"%s\", \r\n\
    \"transmission_gear_position\": \"%s\", \r\n\
    \"odometer\": %lf, \r\n\
    \"fuel_level\": %lf, \r\n\
    \"fuel_consumed_since_restart\": %lf, \r\n\
    \"latitude\":%lf, \r\n\
    \"longitude\": %lf, \r\n\
    \"timestamp\": \"%s\", \r\n\
    \"trip_id\": \"%s\", \r\n\
    \"vin\": \"%s\", \r\n\
    \"end_time\": \"%s\", \r\n\
    \"name\":\"aggregated_telemetrics\" \r\n\
}";

static const char OBD_DATA_TELEMETRY_TOPIC[] = "connectedcar/telemetry/%s";
/* The value field is vary from the field. Use strcat to cat the string. */
static const char OBD_DATA_TELEMETRY_FORMAT_START[] =
    "{ \r\n\
    \"timestamp\": \"%s\", \r\n\
    \"trip_id\": \"%s\", \r\n\
    \"vin\": \"%s\", \r\n\
    \"name\": \"%s\", \r\n\
    \"value\":";
static const char OBD_DATA_TELEMETRY_FORMAT_END[] =
    " \r\n\
}";

static const char OBD_DATA_LOCATION_TOPIC[] = "connectedcar/telemetry/%s";
static const char OBD_DATA_LOCATION_FORMAT[] =
    "{ \r\n\
    \"timestamp\": \"%s\", \r\n\
    \"trip_id\": \"%s\", \r\n\
    \"vin\": \"%s\", \r\n\
    \"name\": \"location\", \r\n\
    \"latitude\": %lf, \r\n\
    \"longitude\": %lf \r\n\
}";

static const char OBD_DATA_DTC_TOPIC[] = "connectedcar/dtc/%s";
static const char OBD_DATA_DTC_FORMAT[] =
    "{\r\n\
    \"timestamp\": \"%s\",\r\n\
    \"trip_id\": \"%s\",\r\n\
    \"vin\": \"%s\",\r\n\
    \"name\": \"dtc\",\r\n\
    \"value\": \"%04x\"\r\n\
}";

static const char OBD_DATA_IGNITION_TOPIC[] = "connectedcar/ignition/%s";
static const char OBD_DATA_IGNITION_FORMAT[] =
    "{\r\n\
    \"timestamp\": \"%s\",\r\n\
    \"trip_id\": \"%s\",\r\n\
    \"vin\": \"%s\",\r\n\
    \"ignition\": \"%s\"\r\n\
}";

static const char * OBD_DATA_TELEMETRY_NAME_ARRAY[ OBD_TELEMETRY_TYPE_MAX ] =
{
    "steering_wheel_angle",
    "acceleration",
    "ignition_status",
    "oil_temp",
    "engine_speed",
    "vehicle_speed",
    "torque_at_transmission",
    "transmission_gear_position",
    "accelerator_pedal_position",
    "fuel_level",
    "fuel_consumed_since_restart",
    "odometer",
    "brake_pedal_status",
    "gear_lever_position",
    "brake"
};

/* Incomming message topic. */
static const char OBD_MESSAGE_TOPIC_FORMAT[] = "connectedcar/alert/%s/hello";

extern BaseType_t publishMqtt( MQTTContext_t * pxMQTTContext,
                               char * pTopic,
                               uint16_t topicLength,
                               uint8_t * pMsg,
                               uint32_t msgLength );

extern BaseType_t SubscribeMqttTopic( MQTTContext_t * pxMQTTContext,
                                      char * pTopic,
                                      uint16_t topicLength );

extern MQTTContext_t * setupMqttConnection( void );

static void checkObdDtcData( obdContext_t * pObdContext )
{
    uint32_t i = 0;
    uint16_t dtc[ MAX_DTC_CODES ];
    int retCodeRead = 0;


    retCodeRead = OBDLib_ReadDTC( pObdContext->obdDevice, dtc, MAX_DTC_CODES );

    for( i = 0; i < retCodeRead; i++ )
    {
        snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_DTC_TOPIC, pObdContext->vin );
        snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_DTC_FORMAT,
                  pObdContext->isoTime,
                  pObdContext->tripId,
                  pObdContext->vin,
                  dtc[ i ]
                  );
        publishMqtt( pObdContext->pMqttContext,
                     pObdContext->topicBuf,
                     strlen( pObdContext->topicBuf ),
                     pObdContext->messageBuf,
                     strlen( pObdContext->messageBuf )
                     );
        OBDLib_ClearDTC( pObdContext->obdDevice );
    }
}

static void genTripId( obdContext_t * pObdContext )
{
    ObdGpsData_t gpsData = { 0 };
    uint32_t randomBuf = 0;
    bool retGetData = false;

    /* retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_GPS_READ, &gpsData ); */
    retGetData = GPSLib_GetData( gObdContext.obdDevice, &gpsData );

    if( retGetData == true )
    {
        char * p = pObdContext->tripId + sprintf( pObdContext->tripId, "%04u%02u%02u%02u%02u%02u",
                                                  ( unsigned int ) ( gpsData.date % 100 ) + 2000, ( unsigned int ) ( gpsData.date / 100 ) % 100, ( unsigned int ) ( gpsData.date / 10000 ),
                                                  ( unsigned int ) ( gpsData.time / 1000000 ), ( unsigned int ) ( gpsData.time % 1000000 ) / 10000, ( unsigned int ) ( gpsData.time % 10000 ) / 100 );
        *p = '\0';
    }
    else
    {
        /* Random generated trip ID. */
        getrandom( &randomBuf, sizeof( randomBuf ), 0 /* flags ignored. */ );
        snprintf( pObdContext->tripId, OBD_TRIP_ID_MAX, "%08u%d\0",
                  randomBuf, xTaskGetTickCount() );
    }
}

static void covertTicksToTimeFormat( uint64_t ticks,
                                     char * pTime,
                                     uint32_t bufferLength )
{
    /* Use gettickcount as time stamp from 1970-01-01 00:00:00.0000. */
    uint64_t upTimeMs = 0;
    uint64_t days = 1;
    uint64_t hours = 0;
    uint64_t minutes = 0;
    uint64_t seconds = 0;

    upTimeMs = ( ticks ) * ( ( uint64_t ) ( ( 1000 ) / configTICK_RATE_HZ ) );

    days = upTimeMs / ( uint64_t ) ( 24 * 60 * 60 * 1000 );
    upTimeMs = upTimeMs - days * ( uint64_t ) ( 24 * 60 * 60 * 1000 );
    hours = upTimeMs / ( uint64_t ) ( 60 * 60 * 1000 );
    upTimeMs = upTimeMs - hours * ( uint64_t ) ( 60 * 60 * 1000 );
    minutes = upTimeMs / ( uint64_t ) ( 60 * 1000 );
    upTimeMs = upTimeMs - minutes * ( uint64_t ) ( 60 * 1000 );
    seconds = upTimeMs / ( uint64_t ) ( 1000 );
    upTimeMs = upTimeMs - seconds * ( uint64_t ) ( 1000 );
    snprintf( pTime, bufferLength, "%04u-%02u-%02llu %0ll2u:%02llu:%02llu.%04llu",
              1970,
              1,
              days + 1,
              hours,
              minutes,
              seconds,
              upTimeMs
              );
}

static void updateTimestamp( obdContext_t * pObdContext,
                             ObdGpsData_t * pGpsData )
{
    /* Time source selection. */
    if( pObdContext->timeSelection == TIME_SELECTION_NONE )
    {
        if( pGpsData != NULL )
        {
            configPRINTF( ( "Timestamp source GPS\r\n" ) );
            pObdContext->timeSelection = TIME_SELECTION_GPS;
        }
        else
        {
            configPRINTF( ( "Timestamp source UPTIME\r\n" ) );
            pObdContext->timeSelection = TIME_SELECTION_UPTIME;
        }
    }

    /* If we can't get GPS data, we use the uptime. */
    if( pObdContext->timeSelection == TIME_SELECTION_GPS )
    {
        if( pGpsData != NULL )
        {
            float kph = ( float ) ( ( int ) ( pGpsData->speed * 1.852f * 10 ) ) / 10;

            pObdContext->timeSelection = TIME_SELECTION_GPS;
            char * p = pObdContext->isoTime + snprintf( pObdContext->isoTime, OBD_ISO_TIME_MAX, "%04u-%02u-%02u %02u:%02u:%02u",
                                                        ( unsigned int ) ( pGpsData->date % 100 ) + 2000,
                                                        ( unsigned int ) ( pGpsData->date / 100 ) % 100,
                                                        ( unsigned int ) ( pGpsData->date / 10000 ),
                                                        ( unsigned int ) ( pGpsData->time / 1000000 ),
                                                        ( unsigned int ) ( pGpsData->time % 1000000 ) / 10000,
                                                        ( unsigned int ) ( pGpsData->time % 10000 ) / 100 );
            unsigned char tenth = ( pGpsData->time % 100 ) / 10;

            if( tenth )
            {
                p += sprintf( p, ".%c00", '0' + tenth );
            }

            *p = '0';

            configPRINTF( ( "[GPS] %lf %lf %lf km/h SATS %d Course: %d %s\r\n",
                            pGpsData->lat, pGpsData->lng, kph, pGpsData->sat, pGpsData->heading, pObdContext->isoTime ) );
        }
    }
    else if( pObdContext->timeSelection == TIME_SELECTION_UPTIME )
    {
        pObdContext->timeSelection = TIME_SELECTION_UPTIME;
        covertTicksToTimeFormat( ( uint64_t ) xTaskGetTickCount(), pObdContext->isoTime, OBD_ISO_TIME_MAX );
        /* configPRINTF(("pObdContext->isoTime = %s\r\n", pObdContext->isoTime )); */
    }
    else
    {
        configPRINTF( ( "Unable to update time with source %u\r\n", pObdContext->timeSelection ) );
    }
}

static void genSimulateGearPosition( obdContext_t * pObdContext )
{
    if( pObdContext->obdTelemetryData.vehicle_speed == 0 )
    {
        strncpy( pObdContext->transmission_gear_position, "neutral", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else if( pObdContext->obdTelemetryData.vehicle_speed < 30 )
    {
        strncpy( pObdContext->transmission_gear_position, "first", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else if( pObdContext->obdTelemetryData.vehicle_speed < 50 )
    {
        strncpy( pObdContext->transmission_gear_position, "second", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else if( pObdContext->obdTelemetryData.vehicle_speed < 70 )
    {
        strncpy( pObdContext->transmission_gear_position, "third", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else if( pObdContext->obdTelemetryData.vehicle_speed < 90 )
    {
        strncpy( pObdContext->transmission_gear_position, "fourth", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else if( pObdContext->obdTelemetryData.vehicle_speed < 110 )
    {
        strncpy( pObdContext->transmission_gear_position, "fifth", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
    else
    {
        strncpy( pObdContext->transmission_gear_position, "sixth", OBD_TRANSMISSION_GEAR_POSITION_MAX );
    }
}

static void genSimulatePadelPosition( obdContext_t * pObdContext )
{
    if( pObdContext->obdTelemetryData.engine_speed <= 0 )
    {
        pObdContext->obdTelemetryData.accelerator_pedal_position = 0;
    }
    else if( ( pObdContext->obdTelemetryData.engine_speed > 0 ) &&
             ( pObdContext->obdTelemetryData.engine_speed <= CAR_ACCELARATOR_PADEL_RPM_THRESHOLD ) )
    {
        pObdContext->obdTelemetryData.accelerator_pedal_position =
            ( pObdContext->obdTelemetryData.engine_speed * 100.0 ) / CAR_ACCELARATOR_PADEL_RPM_THRESHOLD;
    }
    else
    {
        pObdContext->obdTelemetryData.accelerator_pedal_position = 100;
    }

    pObdContext->obdAggregatedData.accelerator_pedal_position_mean =
        ( pObdContext->obdAggregatedData.accelerator_pedal_position_mean * ( pObdContext->updateCount ) ) +
        pObdContext->obdTelemetryData.accelerator_pedal_position;
    pObdContext->obdAggregatedData.accelerator_pedal_position_mean =
        pObdContext->obdAggregatedData.accelerator_pedal_position_mean / ( pObdContext->updateCount + 1 );
}

static void resetTelemetryData( obdContext_t * pObdContext )
{
    memset( &pObdContext->obdAggregatedData, 0, sizeof( ObdAggregatedData_t ) );
    memset( &pObdContext->obdTelemetryData, 0, sizeof( ObdTelemetryData_t ) );
    pObdContext->telemetryIndex = 0;
    pObdContext->latitude = 0;
    pObdContext->longitude = 0;
    pObdContext->timeSelection = TIME_SELECTION_NONE;
    pObdContext->lastUpdateTicks = 0;
    pObdContext->updateCount = 0;
    pObdContext->highSpeedDurationMs = 0;
    pObdContext->idleSpeedDurationMs = 0;
    pObdContext->idleSpeedDurationIntervalMs = 0;
}

static double obdReadVehicleSpeed( obdContext_t * pObdContext )
{
    BaseType_t retIoctl = pdPASS;
    int32_t pidValue = 0;
    double vehicleSpeed = 0;

    retIoctl = OBDLib_ReadPID( pObdContext->obdDevice, PID_SPEED, &pidValue );

    if( retIoctl == pdPASS )
    {
        vehicleSpeed = ( double ) pidValue;
    }

    return vehicleSpeed;
}

static double updateGPSData( obdContext_t * pObdContext )
{
    ObdGpsData_t gpsData = { 0 };
    bool retGetData = false;
    double kph = 0.0;

    /* Update GPS data. */
    retGetData = GPSLib_GetData( pObdContext->obdDevice, &gpsData );

    if( retGetData == true )
    {
        pObdContext->latitude = gpsData.lat;
        pObdContext->longitude = gpsData.lng;

        kph = ( double ) ( ( int ) ( gpsData.speed * 1.852f * 10 ) ) / 10;
    }

    /* Update timestamp. */
    if( retGetData == true )
    {
        updateTimestamp( pObdContext, &gpsData );
    }

    return kph;
}

static void updateTelemetryData( obdContext_t * pObdContext )
{
    int32_t pidValue = 0;
    ObdTelemetryDataType_t pidIndex = OBD_TELEMETRY_TYPE_STEERING_WHEEL_ANGLE;
    uint64_t currentTicks = ( uint64_t ) xTaskGetTickCount();
    uint64_t timeDiffMs = ( currentTicks - pObdContext->lastUpdateTicks ) * ( ( uint64_t ) ( ( 1000 ) / configTICK_RATE_HZ ) );

    bool retReadPID = true;

    updateTimestamp( pObdContext, NULL );

    /* Update telemetry and aggregated data. */
    for( pidIndex = OBD_TELEMETRY_TYPE_STEERING_WHEEL_ANGLE; pidIndex < OBD_TELEMETRY_TYPE_MAX; pidIndex++ )
    {
        switch( pidIndex )
        {
            case OBD_TELEMETRY_TYPE_OIL_TEMP:
                retReadPID = OBDLib_ReadPID( pObdContext->obdDevice, PID_ENGINE_OIL_TEMP, &pidValue );

                if( retReadPID == true )
                {
                    pObdContext->obdTelemetryData.oil_temp = ( double ) pidValue;

                    if( pObdContext->obdAggregatedData.oil_temp_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.oil_temp_mean = pObdContext->obdTelemetryData.oil_temp;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.oil_temp_mean =
                            ( pObdContext->obdAggregatedData.oil_temp_mean * ( pObdContext->updateCount ) ) +
                            pObdContext->obdTelemetryData.oil_temp;
                        pObdContext->obdAggregatedData.oil_temp_mean =
                            pObdContext->obdAggregatedData.oil_temp_mean / ( pObdContext->updateCount + 1 );
                    }
                }

                break;

            case OBD_TELEMETRY_TYPE_ENGINE_SPEED:
                retReadPID = OBDLib_ReadPID( pObdContext->obdDevice, PID_RPM, &pidValue );

                if( retReadPID == true )
                {
                    pObdContext->obdTelemetryData.engine_speed = ( double ) pidValue;

                    if( pObdContext->obdAggregatedData.engine_speed_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.engine_speed_mean = pObdContext->obdTelemetryData.engine_speed;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.engine_speed_mean =
                            ( pObdContext->obdAggregatedData.engine_speed_mean * ( pObdContext->updateCount ) ) +
                            pObdContext->obdTelemetryData.engine_speed;
                        pObdContext->obdAggregatedData.engine_speed_mean =
                            pObdContext->obdAggregatedData.engine_speed_mean / ( pObdContext->updateCount + 1 );
                    }

                    /* Update the simulated padel position. */
                    genSimulatePadelPosition( pObdContext );
                }

                break;

            case OBD_TELEMETRY_TYPE_VEHICLE_SPEED:
                retReadPID = OBDLib_ReadPID( pObdContext->obdDevice, PID_SPEED, &pidValue );

                if( retReadPID == true )
                {
                    /* Update the simulated Acceleration. */
                    double previous_speed = pObdContext->obdTelemetryData.vehicle_speed;
                    pObdContext->obdTelemetryData.vehicle_speed = ( double ) pidValue;

                    if( timeDiffMs != 0 )
                    {
                        pObdContext->obdTelemetryData.acceleration =
                            ( pObdContext->obdTelemetryData.vehicle_speed - previous_speed ) * ( 1000 ) / timeDiffMs;
                    }

                    /* configPRINTF( ( "Acceleration %lf\r\n", pObdContext->obdTelemetryData.acceleration ) ); */

                    /* Update the simulated high speed duration. */
                    if( pObdContext->obdTelemetryData.vehicle_speed > ( ( double ) CAR_HIGH_SPEED_THRESHOLD ) )
                    {
                        pObdContext->highSpeedDurationMs = pObdContext->highSpeedDurationMs + timeDiffMs;
                    }

                    /* Update the idle time. */
                    if( pObdContext->obdTelemetryData.vehicle_speed <= ( ( double ) CAR_IDLE_SPEED_THRESHOLD ) )
                    {
                        pObdContext->idleSpeedDurationMs = pObdContext->idleSpeedDurationMs + timeDiffMs;
                        pObdContext->idleSpeedDurationIntervalMs = pObdContext->idleSpeedDurationIntervalMs + timeDiffMs;
                    }
                    else
                    {
                        pObdContext->idleSpeedDurationIntervalMs = 0;
                    }

                    /* Update the simulated gear position. */
                    genSimulateGearPosition( pObdContext );

                    /* Update the aggregated vehicle speed mean. */
                    if( pObdContext->obdAggregatedData.vehicle_speed_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.vehicle_speed_mean = pObdContext->obdTelemetryData.vehicle_speed;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.vehicle_speed_mean =
                            ( pObdContext->obdAggregatedData.vehicle_speed_mean * ( pObdContext->updateCount ) ) +
                            pObdContext->obdTelemetryData.vehicle_speed;
                        pObdContext->obdAggregatedData.vehicle_speed_mean =
                            pObdContext->obdAggregatedData.vehicle_speed_mean / ( pObdContext->updateCount + 1 );
                    }
                }

                break;

            /*
             * case OBD_TELEMETRY_TYPE_TORQUE_AT_TRANSMISSION:
             *  retIoctl = OBDLib_ReadPID( pObdContext->obdDevice, PID_ENGINE_TORQUE_PERCENTAGE, &pidValue );
             *  if( retIoctl == pdPASS )
             *  {
             *      pObdContext->obdTelemetryData.torque_at_transmission = ( double )pidValue;
             *  }
             *  break;
             */
            case OBD_TELEMETRY_TYPE_FUEL_LEVEL:
                retReadPID = OBDLib_ReadPID( pObdContext->obdDevice, PID_FUEL_LEVEL, &pidValue );

                if( retReadPID == true )
                {
                    pObdContext->fuel_level = ( double ) pidValue;
                    /* Update the simulated fuel_consumed_since_restart. */
                    pObdContext->fuel_consumed_since_restart =
                        ( pObdContext->start_fuel_level - pObdContext->fuel_level ) * CAR_GAS_TANK_SIZE;
                }

                break;

            case OBD_TELEMETRY_TYPE_ODOMETER:
                /* Update simulated data. */
                pObdContext->odometer = pObdContext->obdAggregatedData.vehicle_speed_mean *
                                        ( currentTicks - pObdContext->startTicks ) / ( 60 * 60 * 1000 );
                break;

            default:
                break;
        }
    }

    /* Update ticks. */
    pObdContext->lastUpdateTicks = ( uint64_t ) xTaskGetTickCount();
    pObdContext->updateCount = pObdContext->updateCount + 1;
}

static BaseType_t sendObdLocationData( obdContext_t * pObdContext )
{
    BaseType_t retMqtt = pdPASS;

    if( ( pObdContext->latitude != 0.0 ) && ( pObdContext->longitude != 0.0 ) )
    {
        snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_LOCATION_TOPIC, pObdContext->vin );
        snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_LOCATION_FORMAT,
                  pObdContext->isoTime,
                  pObdContext->tripId,
                  pObdContext->vin,
                  pObdContext->latitude,
                  pObdContext->longitude
                  );
        retMqtt = publishMqtt( pObdContext->pMqttContext,
                               pObdContext->topicBuf,
                               strlen( pObdContext->topicBuf ),
                               pObdContext->messageBuf,
                               strlen( pObdContext->messageBuf )
                               );
    }

    return retMqtt;
}

static BaseType_t sendObdIgnitionData( obdContext_t * pObdContext )
{
    BaseType_t retMqtt = pdPASS;

    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_IGNITION_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_IGNITION_FORMAT,
              pObdContext->isoTime,
              pObdContext->tripId,
              pObdContext->vin,
              pObdContext->ignition_status
              );
    retMqtt = publishMqtt( pObdContext->pMqttContext,
                           pObdContext->topicBuf,
                           strlen( pObdContext->topicBuf ),
                           pObdContext->messageBuf,
                           strlen( pObdContext->messageBuf )
                           );
    return retMqtt;
}

static BaseType_t sendObdTelemetryData( obdContext_t * pObdContext )
{
    bool valueSupported = false;
    BaseType_t retMqtt = pdPASS;

    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_TELEMETRY_TOPIC, pObdContext->vin );

    while( valueSupported == false )
    {
        snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_TELEMETRY_FORMAT_START,
                  pObdContext->isoTime,
                  pObdContext->tripId,
                  pObdContext->vin,
                  OBD_DATA_TELEMETRY_NAME_ARRAY[ pObdContext->telemetryIndex ]
                  );

        switch( pObdContext->telemetryIndex )
        {
            case OBD_TELEMETRY_TYPE_ACCELERATION:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->obdTelemetryData.acceleration
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_IGNITION_STATUS:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%s",
                          pObdContext->messageBuf,
                          pObdContext->ignition_status
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_OIL_TEMP:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->obdTelemetryData.oil_temp
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_ENGINE_SPEED:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->obdTelemetryData.engine_speed
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_VEHICLE_SPEED:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->obdTelemetryData.vehicle_speed
                          );
                valueSupported = true;
                break;

            /*
             * case OBD_TELEMETRY_TYPE_TORQUE_AT_TRANSMISSION:
             *  snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
             *      pObdContext->messageBuf,
             *      pObdContext->obdTelemetryData.torque_at_transmission
             *  );
             *  valueSupported = true;
             *  break;
             */
            case OBD_TELEMETRY_TYPE_FUEL_LEVEL:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->fuel_level
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_ODOMETER:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->odometer
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_TRANSMISSION_GEAR_POSITION:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s\"%s\"",
                          pObdContext->messageBuf,
                          pObdContext->transmission_gear_position
                          );
                valueSupported = true;
                break;

            case OBD_TELEMETRY_TYPE_ACCELERATOR_PEDAL_POSITION:
                snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                          pObdContext->messageBuf,
                          pObdContext->obdTelemetryData.accelerator_pedal_position
                          );
                valueSupported = true;
                break;

            default:
                break;
        }

        pObdContext->telemetryIndex = ( pObdContext->telemetryIndex + 1 ) % OBD_TELEMETRY_TYPE_MAX;
    }

    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%s",
              pObdContext->messageBuf,
              OBD_DATA_TELEMETRY_FORMAT_END
              );
    retMqtt = publishMqtt( pObdContext->pMqttContext,
                           pObdContext->topicBuf,
                           strlen( pObdContext->topicBuf ),
                           pObdContext->messageBuf,
                           strlen( pObdContext->messageBuf )
                           );
    return retMqtt;
}

static BaseType_t sendObdAggregatedData( obdContext_t * pObdContext )
{
    BaseType_t retMqtt = pdPASS;

    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_AGGREGATED_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_AGGREGATED_FORMAT,
              pObdContext->obdAggregatedData.vehicle_speed_mean,
              pObdContext->obdAggregatedData.engine_speed_mean,
              pObdContext->obdAggregatedData.torque_at_transmission_mean,
              pObdContext->obdAggregatedData.oil_temp_mean,
              pObdContext->obdAggregatedData.accelerator_pedal_position_mean,
              pObdContext->obdAggregatedData.brake_mean,
              ( pObdContext->highSpeedDurationMs / ( 1000.0 ) ),
              pObdContext->obdAggregatedData.high_acceleration_event,
              pObdContext->obdAggregatedData.high_braking_event,
              ( pObdContext->idleSpeedDurationMs / ( 1000.0 ) ),
              pObdContext->obdAggregatedData.start_time,
              pObdContext->ignition_status,
              pObdContext->brake_pedal_status ? "true" : "false",
              pObdContext->transmission_gear_position,
              pObdContext->odometer,
              pObdContext->fuel_level,
              pObdContext->fuel_consumed_since_restart,
              pObdContext->latitude,
              pObdContext->longitude,
              pObdContext->isoTime,
              pObdContext->tripId,
              pObdContext->vin,
              pObdContext->isoTime  /* Use last timestamp to simulate the endtime. */
              );
    retMqtt = publishMqtt( pObdContext->pMqttContext,
                           pObdContext->topicBuf,
                           strlen( pObdContext->topicBuf ),
                           pObdContext->messageBuf,
                           strlen( pObdContext->messageBuf )
                           );
    return retMqtt;
}

int RunOBDDemo( bool awsIotMqttMode,
                const char * pIdentifier,
                void * pNetworkServerInfo,
                void * pNetworkCredentialInfo,
                const IotNetworkInterface_t * pNetworkInterface )
{
    uint64_t loopSteps = 0;
    BaseType_t retIoctl = pdPASS;
    BaseType_t retSendMsg = pdPASS;
    TickType_t startTicks = 0, elapsedTicks = 0;
    bool ignitionStatus = false;
    int i = 0;
    double vehicleSpeed = 0;
    double gpsSpeed = 0;

    gObdContext.buzzDevice = FreeRTOS_open( "/dev/buzz", 0 );

    if( gObdContext.buzzDevice != NULL )
    {
        /* Network connection success indication. */
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_ON, NULL );
        vTaskDelay( pdMS_TO_TICKS( 250 ) );
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_OFF, NULL );
    }

    /* Setup the mqtt connection. */
    for( i = 0; i < 3; i++ )
    {
        gObdContext.pMqttContext = setupMqttConnection();

        if( gObdContext.pMqttContext == NULL )
        {
            configPRINTF( ( "MQTT connection failed\r\n" ) );
        }
        else
        {
            break;
        }
    }

    /* Open the OBD devices. */
    configPRINTF( ( "Start obd device init\r\n" ) );
    gObdContext.obdDevice = FreeRTOS_open( "/dev/obd", 0 );

    if( gObdContext.obdDevice == NULL )
    {
        configPRINTF( ( "OBD device open failed\r\n" ) );
    }
    else
    {
        while( OBDLib_Init( gObdContext.obdDevice ) != 0 )
        {
        }
    }

    if( gObdContext.buzzDevice != NULL )
    {
        /* OBD connection indication. */
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_ON, NULL );
        vTaskDelay( pdMS_TO_TICKS( 250 ) );
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_OFF, NULL );
        vTaskDelay( pdMS_TO_TICKS( 50 ) );
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_ON, NULL );
        vTaskDelay( pdMS_TO_TICKS( 250 ) );
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_OFF, NULL );
    }

    /* Read VIN. */
    #ifdef OBD_DEFAULT_VIN
        strncpy( gObdContext.vin, OBD_DEFAULT_VIN, OBD_VIN_MAX );
        retIoctl = pdPASS;
    #else
        if( ODBLib_GetVIN( gObdContext.obdDevice, gObdContext.vin, OBD_VIN_MAX ) == false )
        {
            configPRINTF( ( "OBD device read VIN fail\r\n" ) );
            retIoctl = pdFAIL;
        }
        else
        {
            configPRINTF( ( "OBD vin %s\r\n", gObdContext.vin ) );
        }
    #endif /* ifdef OBD_DEFAULT_VIN */

    /* Subscribe to MQTT. */
    if( retIoctl == pdPASS )
    {
        snprintf( gObdContext.topicBuf, OBD_TOPIC_BUF_SIZE, OBD_MESSAGE_TOPIC_FORMAT, gObdContext.vin );
        SubscribeMqttTopic( gObdContext.pMqttContext, gObdContext.topicBuf, strlen( gObdContext.topicBuf ) );
    }

    /* Enable GPS device. */
    GPSLib_Begin( gObdContext.obdDevice );

    /* the external trip loop. */
    while( true )
    {
        resetTelemetryData( &gObdContext );
        loopSteps = 0;
        ignitionStatus = false;

        /* Use time info as trip ID. */
        genTripId( &gObdContext );
        configPRINTF( ( "Start a new trip id %sr\n", gObdContext.tripId ) );

        /* Main thread runs in send data loop. */
        while( true )
        {
            startTicks = xTaskGetTickCount();

            /* Check exit events. */

            /* Check the DTC events. */
            checkObdDtcData( &gObdContext );

            /* Check the Location data events. */
            gpsSpeed = updateGPSData( &gObdContext );

            if( ( loopSteps % OBD_LOCATION_DATA_INTERVAL_STEPS ) == 0 )
            {
                retSendMsg = sendObdLocationData( &gObdContext );

                if( retSendMsg == pdFAIL )
                {
                    closeMqttConnection( gObdContext.pMqttContext );
                    gObdContext.pMqttContext = setupMqttConnection();
                }
            }

            /* Check the ignition status. */
            if( ignitionStatus == false )
            {
                /* Wait until there is speed. */
                vehicleSpeed = obdReadVehicleSpeed( &gObdContext );

                /* configPRINTF(("Vehicle speed %lf for ignition\r\n", vehicleSpeed)); */
                if( ( vehicleSpeed == 0 ) && ( gpsSpeed == 0 ) )
                {
                    vTaskDelay( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) );
                    continue;
                }
                else
                {
                    ignitionStatus = true;

                    configPRINTF( ( "Vehicle speed %lf gps speed %lf for ignition\r\n", vehicleSpeed, gpsSpeed ) );

                    /* Save the start information. */
                    gObdContext.startTicks = ( uint64_t ) xTaskGetTickCount();
                    gObdContext.lastUpdateTicks = gObdContext.startTicks;
                    covertTicksToTimeFormat( gObdContext.startTicks, gObdContext.isoTime, OBD_ISO_TIME_MAX );
                    strcpy( gObdContext.obdAggregatedData.start_time, gObdContext.isoTime );
                    updateTelemetryData( &gObdContext );
                    gObdContext.start_fuel_level = gObdContext.fuel_level;

                    /* Send the ignition event. */
                    strncpy( gObdContext.ignition_status, "run", OBD_IGNITION_MAX );
                    retSendMsg = sendObdIgnitionData( &gObdContext );

                    if( retSendMsg == pdFAIL )
                    {
                        closeMqttConnection( gObdContext.pMqttContext );
                        gObdContext.pMqttContext = setupMqttConnection();
                    }
                }
            }
            else
            {
                /* Update telemetry data. */
                updateTelemetryData( &gObdContext );
            }

            /* Check the ignition off events. */
            if( ( gObdContext.idleSpeedDurationIntervalMs >= ( ( uint64_t ) CAR_IGINITION_IDLE_OFF_MS ) ) &&
                ( gpsSpeed == 0 ) )
            {
                configPRINTF( ( "Idle speed duration interval ms %llu %llu\r\n",
                                gObdContext.idleSpeedDurationIntervalMs, ( uint64_t ) CAR_IGINITION_IDLE_OFF_MS ) );
                strncpy( gObdContext.ignition_status, "off", OBD_IGNITION_MAX );
                retSendMsg = sendObdIgnitionData( &gObdContext );

                if( retSendMsg == pdFAIL )
                {
                    closeMqttConnection( gObdContext.pMqttContext );
                    gObdContext.pMqttContext = setupMqttConnection();
                }

                break;
            }

            /* Check the telemetry data events. */
            if( ( loopSteps % OBD_TELEMETRY_DATA_INTERVAL_STEPS ) == 0 )
            {
                retSendMsg = sendObdTelemetryData( &gObdContext );

                if( retSendMsg == pdFAIL )
                {
                    closeMqttConnection( gObdContext.pMqttContext );
                    gObdContext.pMqttContext = setupMqttConnection();
                }
            }

            /* Check the aggregated data events. */
            if( ( loopSteps % OBD_AGGREGATED_DATA_INTERVAL_STEPS ) == 0 )
            {
                retSendMsg = sendObdAggregatedData( &gObdContext );

                if( retSendMsg == pdFAIL )
                {
                    closeMqttConnection( gObdContext.pMqttContext );
                    gObdContext.pMqttContext = setupMqttConnection();
                }
            }

            /* Calculate remain time. */
            elapsedTicks = xTaskGetTickCount() - startTicks;

            if( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) > elapsedTicks )
            {
                vTaskDelay( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) - elapsedTicks );
            }

            loopSteps = loopSteps + 1;
        }

        /* Send the aggregated data. */
        strcpy( gObdContext.transmission_gear_position, "neutral" );
        retSendMsg = sendObdAggregatedData( &gObdContext );

        if( retSendMsg == pdFAIL )
        {
            closeMqttConnection( gObdContext.pMqttContext );
            gObdContext.pMqttContext = setupMqttConnection();
        }
    }

    return 0;
}

void demoNetworkFailureHook( void )
{
    if( gObdContext.buzzDevice == NULL )
    {
        gObdContext.buzzDevice = FreeRTOS_open( "/dev/buzz", 0 );
    }

    if( gObdContext.buzzDevice != NULL )
    {
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_ON, NULL );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
        FreeRTOS_ioctl( gObdContext.buzzDevice, ioctlBUZZ_OFF, NULL );
    }
}
