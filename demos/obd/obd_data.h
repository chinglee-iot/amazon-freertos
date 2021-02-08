
#ifndef  __OBD_DATA_H__
#define __OBD_DATA_H__

typedef enum ObdTelemetryDataType
{
    OBD_TELEMETRY_TYPE_STEERING_WHEEL_ANGLE = 0,
    OBD_TELEMETRY_TYPE_ACCELERATION,
    OBD_TELEMETRY_TYPE_IGNITION_STATUS,
    OBD_TELEMETRY_TYPE_OIL_TEMP,
    OBD_TELEMETRY_TYPE_ENGINE_SPEED,
    OBD_TELEMETRY_TYPE_VEHICLE_SPEED,
    OBD_TELEMETRY_TYPE_TORQUE_AT_TRANSMISSION,
    OBD_TELEMETRY_TYPE_TRANSMISSION_GEAR_POSITION,
    OBD_TELEMETRY_TYPE_ACCELERATOR_PEDAL_POSITION,
    OBD_TELEMETRY_TYPE_FUEL_LEVEL,
    OBD_TELEMETRY_TYPE_FUEL_CONSUMED_SINCE_RESTART,
    OBD_TELEMETRY_TYPE_ODOMETER,
    OBD_TELEMETRY_TYPE_BRAKE_PEDAL_STATUS,
    OBD_TELEMETRY_TYPE_GEAR_LEVER_POSITION,
    OBD_TELEMETRY_TYPE_BRAKE,
    OBD_TELEMETRY_TYPE_MAX
} ObdTelemetryDataType_t;

typedef struct ObdAggregatedData
{
    double vehicle_speed_mean;
    double engine_speed_mean;
    double torque_at_transmission_mean;
    double oil_temp_mean;
    double accelerator_pedal_position_mean;
    double brake_mean;
    double high_speed_duration;
    uint32_t high_acceleration_event;
    uint32_t high_braking_event;
    double idle_duration;
    char start_time[32];                        // e.g., 1970-02-03 02 =59 =37.401000000
    char ignition_status[4];                    // "run", "off"
    bool brake_pedal_status;
    char transmission_gear_position[10];        // "neutral", "first", "second", "third", "fourth", "fifth", "sixth", etc
    double odometer;
    double fuel_level;                          // 0 to 100
    double fuel_consumed_since_restart;
    double latitude;
    double longitude;
    char timestamp[32];                         // e.g., 1970-02-03 02 =59 =37.401000000
    char trip_id[16];
    char vin[16];
    char name[32];                              // must be aggregated_telemetrics
} ObdAggregatedData_t;

typedef struct ObdTelemetryData
{
    double steering_wheel_angle;
    double acceleration;
    char ignition_status[16];
    double oil_temp;
    double engine_speed;
    double vehicle_speed;
    double torque_at_transmission;
    char transmission_gear_position[10];
    char accelerator_pedal_position[10];
    double fuel_level;
    double fuel_consumed_since_restart;
    double odometer;
    bool brake_pedal_status;
    char gear_lever_position[16];
    double brake;
    bool parking_break_status;
} ObdTelemetryData_t;

#endif
