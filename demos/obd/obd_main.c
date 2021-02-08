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

/* The OBD data collect interval time. */
#define OBD_DATA_COLLECT_INTERVAL_MS        ( 500 )

#define OBD_AGGREGATED_DATA_INTERVAL_MS     ( 90000 )
#define OBD_TELEMETRY_DATA_INTERVAL_MS      ( 2000 )
#define OBD_LOCATION_DATA_INTERVAL_MS       ( 2000 )

#define OBD_MESSAGE_BUF_SIZE                ( 256 )
#define OBD_TOPIC_BUF_SIZE                  ( 64 )

#define OBD_AGGREGATED_DATA_INTERVAL_STEPS      ( OBD_AGGREGATED_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_TELEMETRY_DATA_INTERVAL_STEPS       ( OBD_TELEMETRY_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )
#define OBD_LOCATION_DATA_INTERVAL_STEPS        ( OBD_LOCATION_DATA_INTERVAL_MS / OBD_DATA_COLLECT_INTERVAL_MS )

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
}obdContext_t;

static obdContext_t gObdContext =
{
    .pMqttContext = NULL,
    .tripId = "123",
    .vin = "WASM_test_car",
    .telemetryIndex = 0
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
    \"name\":\"%s\" \r\n\
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
    \"latitude\": %lf, \r\n\
    \"longitude\": %lf \r\n\
}";

static const char OBD_DATA_DTC_TOPIC[] = "connectedcar/dtc/%s";
static const char OBD_DATA_DTC_FORMAT[] =
"{\r\n\
    \"timestamp\": \"%s\",\r\n\
    \"trip_id\": \"%s\",\r\n\
    \"vin\": \"%s\",\r\n\
    \"name\": \"%s\",\r\n\
    \"value\": \"%s\"\r\n\
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
    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_DTC_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_DTC_FORMAT,
        "1970-02-03 02:59:37.401000000",
        pObdContext->tripId,
        pObdContext->vin,
        "dtc",
        "Error code"
    );
    publishMqtt( pObdContext->pMqttContext, 
        pObdContext->topicBuf,
        0,
        pObdContext->messageBuf,
        strlen( pObdContext->messageBuf ) 
    );
}

static void sendObdLocationData( obdContext_t *pObdContext )
{
    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_LOCATION_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_LOCATION_FORMAT,
        "1970-02-03 02:59:37.401000000",
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
    snprintf( pObdContext->topicBuf, OBD_TOPIC_BUF_SIZE, OBD_DATA_TELEMETRY_TOPIC, pObdContext->vin );
    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, OBD_DATA_TELEMETRY_FORMAT_START,
        "1970-02-03 02:59:37.401000000",
        pObdContext->tripId,
        pObdContext->vin,
        OBD_DATA_TELEMETRY_NAME_ARRAY[pObdContext->telemetryIndex]
    );

    snprintf( pObdContext->messageBuf, OBD_MESSAGE_BUF_SIZE, "%s%d%s",
        pObdContext->messageBuf,
        12,
        OBD_DATA_TELEMETRY_FORMAT_END
    );

    publishMqtt( pObdContext->pMqttContext, 
        pObdContext->topicBuf,
        0,
        pObdContext->messageBuf,
        strlen( pObdContext->messageBuf ) 
    );
    pObdContext->telemetryIndex = ( pObdContext->telemetryIndex + 1 ) % OBD_TELEMETRY_TYPE_MAX;
}

static void sendObdAggregatedData( obdContext_t *pObdContext )
{
}

int RunOBDDemo( bool awsIotMqttMode,
                const char * pIdentifier,
                void * pNetworkServerInfo,
                void * pNetworkCredentialInfo,
                const IotNetworkInterface_t * pNetworkInterface )
{
    uint64_t loopSteps = 1;

    /* Setup the mqtt connection. */
    gObdContext.pMqttContext = setupMqttConnection();

    /* Main thread runs in send data loop. */
    while(1)
    {
        /* Check the DTC events. */
        checkObdDtcData( &gObdContext );
        
        /* Cehck the Location data events. */
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
        
        /* Calculate remain time. */
        vTaskDelay( pdMS_TO_TICKS( OBD_DATA_COLLECT_INTERVAL_MS ) );
        loopSteps = loopSteps + 1;
    }

    return 0;
}
