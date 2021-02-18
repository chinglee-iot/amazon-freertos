#include <string.h>

/**
 * @file aws_ble_gatt_server_demo.c
 * @brief Sample demo for a BLE GATT server
 */
#include "FreeRTOSConfig.h"
#include "iot_demo_logging.h"
#include "task.h"
#include "semphr.h"
#include "platform/iot_network.h"

#include "core_mqtt.h"

#include "obd_data.h"
#include "FreeRTOS_IO.h"
#include "obd_device.h"

/* The OBD data collect interval time. */
#define OBD_DATA_COLLECT_INTERVAL_MS        ( 2000 )

#define OBD_AGGREGATED_DATA_INTERVAL_MS     ( 20000 )
#define OBD_TELEMETRY_DATA_INTERVAL_MS      ( 2000 )
#define OBD_LOCATION_DATA_INTERVAL_MS       ( 2000 )

#define OBD_AGGREGATED_DATA_MEAN            ( 20 )

#define OBD_MESSAGE_BUF_SIZE                ( 1024 )
#define OBD_TOPIC_BUF_SIZE                  ( 64 )

#define OBD_AGGREGATED_DATA_INTERVAL_STEPS      ( OBD_AGGREGATED_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_TELEMETRY_DATA_INTERVAL_STEPS       ( OBD_TELEMETRY_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_LOCATION_DATA_INTERVAL_STEPS        ( OBD_LOCATION_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )

#define OBD_ISO_TIME_MAX                    ( 32 )

typedef struct obdContext
{
    MQTTContext_t *pMqttContext;
    ObdAggregatedData_t obdAggregatedData;
    ObdTelemetryData_t obdTelemetryData;
    char tripId[16];
    char vin[32];
    double latitude;
    double longitude;
    ObdTelemetryDataType_t telemetryIndex;
    char topicBuf[OBD_TOPIC_BUF_SIZE];
    char messageBuf[OBD_MESSAGE_BUF_SIZE];
    Peripheral_Descriptor_t obdDevice;
    char isoTime[OBD_ISO_TIME_MAX];
}obdContext_t;

static obdContext_t gObdContext =
{
    .pMqttContext = NULL,
    .obdAggregatedData = { 0 },
    .obdTelemetryData = { 0 },
    .tripId = "123",
    .vin = "WASM_test_car",
    .telemetryIndex = 0,
    .isoTime = "1970-02-03 02:59:37.401000000"
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

static const char *OBD_DATA_TELEMETRY_NAME_ARRAY[OBD_TELEMETRY_TYPE_MAX] =
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

extern BaseType_t publishMqtt( MQTTContext_t * pxMQTTContext,
                        char *pTopic,
                        uint16_t topicLength,
                        uint8_t *pMsg,
                        uint32_t msgLength);

extern MQTTContext_t *setupMqttConnection( void );

static void checkObdDtcData( obdContext_t *pObdContext )
{
    ObdDtcData_t obdData = { 0 };
    uint32_t i = 0;
    BaseType_t retIoctl = pdPASS;

    retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_DTC, &obdData );
    if( retIoctl == pdPASS )
    {
        for( i = 0; i < obdData.dtcCount; i++ )
        {
            snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_DTC_TOPIC, pObdContext->vin );
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_DTC_FORMAT,
                pObdContext->isoTime,
                pObdContext->tripId,
                pObdContext->vin,
                obdData.dtc[i]
            );
            publishMqtt( pObdContext->pMqttContext,
                pObdContext->topicBuf,
                0,
                pObdContext->messageBuf,
                strlen( pObdContext->messageBuf )
            );
        }
    }
}

static void genTripId( obdContext_t *pObdContext )
{
    ObdGpsData_t gpsData = { 0 };
    BaseType_t retIoctl = pdPASS;

    retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_GPS_READ, &gpsData );
    if( retIoctl == pdPASS )
    {
        char *p = pObdContext->tripId + sprintf(pObdContext->tripId, "%04u%02u%02u%02u%02u%02u",
            (unsigned int)(gpsData.date % 100) + 2000, (unsigned int)(gpsData.date / 100) % 100, (unsigned int)(gpsData.date / 10000),
            (unsigned int)(gpsData.time / 1000000), (unsigned int)(gpsData.time % 1000000) / 10000, (unsigned int)(gpsData.time % 10000) / 100);
        *p = '\0';
    }
    else
    {
        /* Random generated trip ID. */
        sprintf(pObdContext->tripId, "%d", xTaskGetTickCount() );
    }
}

static void updateTelemetryData( obdContext_t *pObdContext )
{
    int32_t pidValue = 0;
    ObdTelemetryDataType_t pidIndex = OBD_TELEMETRY_TYPE_STEERING_WHEEL_ANGLE;
    
    ObdGpsData_t gpsData = { 0 };
    BaseType_t retIoctl = pdPASS;

    /* Update GPS data. */
    retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_GPS_READ, &gpsData );
    if( retIoctl == pdPASS )
    {
        float kph = (float)((int)(gpsData.speed * 1.852f * 10)) / 10;

        char *p = pObdContext->isoTime + snprintf(pObdContext->isoTime, OBD_ISO_TIME_MAX, "%04u-%02u-%02u %02u:%02u:%02u",
            (unsigned int)(gpsData.date % 100) + 2000, (unsigned int)(gpsData.date / 100) % 100, (unsigned int)(gpsData.date / 10000),
            (unsigned int)(gpsData.time / 1000000), (unsigned int)(gpsData.time % 1000000) / 10000, (unsigned int)(gpsData.time % 10000) / 100);
        unsigned char tenth = (gpsData.time % 100) / 10;
        if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
        *p = '0';
      
        configPRINTF(("[GPS] %lf %lf %lf km/h SATS %d Course: %d %s\r\n",
            gpsData.lat, gpsData.lng, kph, gpsData.sat, gpsData.heading, pObdContext->isoTime));

        pObdContext->latitude = gpsData.lat;
        pObdContext->longitude = gpsData.lng;
    }
    else
    {
        /* Use gettickcount as time stamp from 1970-01-01 00:00:00.0000. */
        uint64_t upTimeMs = 0;
        uint64_t days = 0;
        uint64_t hours = 0;
        uint64_t minutes = 0;
        uint64_t seconds = 0;
        upTimeMs = ( ( uint64_t ) xTaskGetTickCount() ) * ( ( uint64_t ) ( ( 1000 ) / configTICK_RATE_HZ ) );

        days = upTimeMs / ( uint64_t )( 24*60*60*1000 );
        upTimeMs = upTimeMs - days * ( uint64_t )( 24*60*60*1000 );
        hours = upTimeMs / ( uint64_t )( 60*60*1000 );
        upTimeMs = upTimeMs - hours * ( uint64_t )( 60*60*1000 );
        minutes = upTimeMs / ( uint64_t )( 60*1000 );
        upTimeMs = upTimeMs - minutes * ( uint64_t )( 60*1000 );
        seconds = upTimeMs / ( uint64_t )( 1000 );
        upTimeMs = upTimeMs - seconds * ( uint64_t )( 1000 );
        snprintf( pObdContext->isoTime, OBD_ISO_TIME_MAX, "%04u-%02u-%02llu %0ll2u:%02llu:%02llu.%04llu",
            1970,
            1,
            days,
            hours,
            minutes,
            seconds,
            upTimeMs
            );
        configPRINTF(("pObdContext->isoTime = %s\r\n", pObdContext->isoTime ));
    }

    /* Update telemetry data. */
    for( pidIndex = OBD_TELEMETRY_TYPE_STEERING_WHEEL_ANGLE; pidIndex < OBD_TELEMETRY_TYPE_MAX; pidIndex++ )
    {
        switch( pidIndex )
        {
            case OBD_TELEMETRY_TYPE_OIL_TEMP:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_ENGINE_OIL_TEMP, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.oil_temp = ( double )pidValue;
                    if( pObdContext->obdAggregatedData.oil_temp_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.oil_temp_mean = pObdContext->obdTelemetryData.oil_temp;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.oil_temp_mean =
                            ( pObdContext->obdAggregatedData.oil_temp_mean * ( OBD_AGGREGATED_DATA_MEAN - 1 ) ) +
                            pObdContext->obdTelemetryData.oil_temp;
                        pObdContext->obdAggregatedData.oil_temp_mean = pObdContext->obdAggregatedData.oil_temp_mean / OBD_AGGREGATED_DATA_MEAN;
                    }
                }
                break;
            case OBD_TELEMETRY_TYPE_ENGINE_SPEED:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_RPM, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.engine_speed = ( double )pidValue;
                    if( pObdContext->obdAggregatedData.engine_speed_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.engine_speed_mean = pObdContext->obdTelemetryData.engine_speed;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.engine_speed_mean =
                            ( pObdContext->obdAggregatedData.engine_speed_mean * ( OBD_AGGREGATED_DATA_MEAN - 1 ) ) +
                            pObdContext->obdTelemetryData.engine_speed;
                        pObdContext->obdAggregatedData.engine_speed_mean = pObdContext->obdAggregatedData.engine_speed_mean / OBD_AGGREGATED_DATA_MEAN;
                    }
                }
                break;
            case OBD_TELEMETRY_TYPE_VEHICLE_SPEED:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_SPEED, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.vehicle_speed = ( double )pidValue;
                    if( pObdContext->obdAggregatedData.vehicle_speed_mean == 0 )
                    {
                        pObdContext->obdAggregatedData.vehicle_speed_mean = pObdContext->obdTelemetryData.vehicle_speed;
                    }
                    else
                    {
                        pObdContext->obdAggregatedData.vehicle_speed_mean =
                            ( pObdContext->obdAggregatedData.vehicle_speed_mean * ( OBD_AGGREGATED_DATA_MEAN - 1 ) ) +
                            pObdContext->obdTelemetryData.vehicle_speed;
                        pObdContext->obdAggregatedData.vehicle_speed_mean = pObdContext->obdAggregatedData.vehicle_speed_mean / OBD_AGGREGATED_DATA_MEAN;
                    }
                }
                break;
            case OBD_TELEMETRY_TYPE_TORQUE_AT_TRANSMISSION:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_ENGINE_TORQUE_PERCENTAGE, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.torque_at_transmission = ( double )pidValue;
                }
                break;
            case OBD_TELEMETRY_TYPE_FUEL_LEVEL:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_FUEL_LEVEL, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.fuel_level = ( double )pidValue;
                    pObdContext->obdAggregatedData.fuel_level = pObdContext->obdTelemetryData.fuel_level;
                }
                break;
            case OBD_TELEMETRY_TYPE_ODOMETER:
                retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_PID_ODOMETER, &pidValue );
                if( retIoctl == pdPASS )
                {
                    pObdContext->obdTelemetryData.odometer = ( double )pidValue;
                    pObdContext->obdAggregatedData.odometer = pObdContext->obdTelemetryData.odometer;
                }
                break;
            default:
                break;
        }
    }
}

static void sendObdLocationData( obdContext_t *pObdContext )
{
    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_LOCATION_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_LOCATION_FORMAT,
        pObdContext->isoTime,
        pObdContext->tripId,
        pObdContext->vin,
        "location",
        pObdContext->latitude,
        pObdContext->longitude
    );
    publishMqtt( pObdContext->pMqttContext, 
        pObdContext->topicBuf,
        0,
        pObdContext->messageBuf,
        strlen( pObdContext->messageBuf ) 
    );
}

static void sendObdTelemetryData( obdContext_t *pObdContext )
{
    int32_t pidValue = 0;
    bool valueSupported = true;

    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_TELEMETRY_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_TELEMETRY_FORMAT_START,
        pObdContext->isoTime,
        pObdContext->tripId,
        pObdContext->vin,
        OBD_DATA_TELEMETRY_NAME_ARRAY[pObdContext->telemetryIndex]
    );

    switch( pObdContext->telemetryIndex )
    {
        case OBD_TELEMETRY_TYPE_OIL_TEMP:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.oil_temp
            );
            break;
        case OBD_TELEMETRY_TYPE_ENGINE_SPEED:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.engine_speed
            );
            break;
        case OBD_TELEMETRY_TYPE_VEHICLE_SPEED:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.vehicle_speed
            );
            break;
        case OBD_TELEMETRY_TYPE_TORQUE_AT_TRANSMISSION:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.torque_at_transmission
            );
            break;
        case OBD_TELEMETRY_TYPE_FUEL_LEVEL:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.fuel_level
            );
            break;
        case OBD_TELEMETRY_TYPE_ODOMETER:
            snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%lf",
                pObdContext->messageBuf,
                pObdContext->obdTelemetryData.odometer
            );
            break;
        default:
            valueSupported = false;
            break;
    }
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%d",
        pObdContext->messageBuf,
        pidValue
    );

    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%s",
        pObdContext->messageBuf,
        OBD_DATA_TELEMETRY_FORMAT_END
    );

    if( valueSupported == true )
    {
        publishMqtt( pObdContext->pMqttContext,
            pObdContext->topicBuf,
            0,
            pObdContext->messageBuf,
            strlen( pObdContext->messageBuf )
        );
    }
    pObdContext->telemetryIndex = ( pObdContext->telemetryIndex + 1 ) % OBD_TELEMETRY_TYPE_MAX;
}

static void sendObdAggregatedData( obdContext_t *pObdContext )
{
    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_AGGREGATED_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_AGGREGATED_FORMAT,
        pObdContext->obdAggregatedData.vehicle_speed_mean,
        pObdContext->obdAggregatedData.engine_speed_mean,
        pObdContext->obdAggregatedData.torque_at_transmission_mean,
        pObdContext->obdAggregatedData.oil_temp_mean,
        pObdContext->obdAggregatedData.accelerator_pedal_position_mean,
        pObdContext->obdAggregatedData.brake_mean,
        pObdContext->obdAggregatedData.high_speed_duration,
        pObdContext->obdAggregatedData.high_acceleration_event,
        pObdContext->obdAggregatedData.high_braking_event,
        pObdContext->obdAggregatedData.idle_duration,
        pObdContext->obdAggregatedData.start_time,
        pObdContext->obdAggregatedData.ignition_status,
        pObdContext->obdAggregatedData.brake_pedal_status ? "true":"false",
        pObdContext->obdAggregatedData.transmission_gear_position,
        pObdContext->obdAggregatedData.odometer,
        pObdContext->obdAggregatedData.fuel_level,
        pObdContext->obdAggregatedData.fuel_consumed_since_restart,
        pObdContext->latitude,
        pObdContext->longitude,
        pObdContext->isoTime,
        pObdContext->tripId,
        pObdContext->vin
    );
    publishMqtt( pObdContext->pMqttContext,
        pObdContext->topicBuf,
        0,
        pObdContext->messageBuf,
        strlen( pObdContext->messageBuf )
    );
}

int RunOBDDemo( bool awsIotMqttMode,
                const char * pIdentifier,
                void * pNetworkServerInfo,
                void * pNetworkCredentialInfo,
                const IotNetworkInterface_t * pNetworkInterface )
{
    uint64_t loopSteps = 1;
    ObdVinBuffer_t vinBuffer = { 0 };
    BaseType_t retIoctl = pdPASS;
    TickType_t startTicks = 0, elapsedTicks = 0;

    /* Setup the mqtt connection. */
    gObdContext.pMqttContext = setupMqttConnection();

    /* Open the OBD devices. */
    configPRINTF(("Start obd device init\r\n"));
    while( 1 )
    {
        gObdContext.obdDevice = FreeRTOS_open( "/dev/obd", 0 );
        if( gObdContext.obdDevice != NULL )
        {
            configPRINTF(("OBD device init done\r\n"));
            break;
        }
    }

    /* Open the GPS devices. */
    retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_GPS_ENABLE, NULL );
    
    /* Use GPS time as trip ID. */
    genTripId( &gObdContext );
    configPRINTF(("trip id %sr\n", gObdContext.tripId ));
    
    /* Read VIN. */
    retIoctl = FreeRTOS_ioctl( gObdContext.obdDevice, ioctlOBD_READ_VIN, &vinBuffer );
    configPRINTF(("VIN %sr\n", vinBuffer.vinBuffer ));
    strncpy( gObdContext.vin, vinBuffer.vinBuffer, 16 );

    /* Main thread runs in send data loop. */
    while(1)
    {
        startTicks = xTaskGetTickCount();
    
        /* Check the DTC events. */
        checkObdDtcData( &gObdContext );
        
        /* Update telemetry data. */
        updateTelemetryData( &gObdContext );

        /* Check the Location data events. */
        if( ( loopSteps % OBD_LOCATION_DATA_INTERVAL_STEPS ) == 0 )
        {
            sendObdLocationData( &gObdContext );
        }

        /* Check the telemetry data events. */
        if( ( loopSteps % OBD_TELEMETRY_DATA_INTERVAL_STEPS ) == 0 )
        {
            sendObdTelemetryData( &gObdContext );
        }

        /* Check the aggregated data events. */
        if( ( loopSteps % OBD_AGGREGATED_DATA_INTERVAL_STEPS ) == 0 )
        {
            sendObdAggregatedData( &gObdContext );
        }
        
        /* Check loop exit events. */
        
        /* TODO : Calculate remain time. */
        elapsedTicks = xTaskGetTickCount() - startTicks;
        if( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) > elapsedTicks )
        {
            configPRINTF(("sleep %d ticks\r\n", pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) > elapsedTicks));
            vTaskDelay( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) - elapsedTicks );
        }
        else
        {
            configPRINTF(("loop too slow %d %d\r\n", elapsedTicks, pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS )));
        }
        loopSteps = loopSteps + 1;
    }

    return 0;
}
