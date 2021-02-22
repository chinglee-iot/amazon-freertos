#ifndef __OBD_DEVICE_H__
#define __OBD_DEVICE_H__

#include "FreeRTOS_DriverInterface.h"

// Mode 1 PIDs
#define ioctlOBD_READ_PID_ENGINE_LOAD 0x04
#define ioctlOBD_READ_PID_COOLANT_TEMP 0x05
#define ioctlOBD_READ_PID_SHORT_TERM_FUEL_TRIM_1 0x06
#define ioctlOBD_READ_PID_LONG_TERM_FUEL_TRIM_1 0x07
#define ioctlOBD_READ_PID_SHORT_TERM_FUEL_TRIM_2 0x08
#define ioctlOBD_READ_PID_LONG_TERM_FUEL_TRIM_2 0x09
#define ioctlOBD_READ_PID_FUEL_PRESSURE 0x0A
#define ioctlOBD_READ_PID_INTAKE_MAP 0x0B
#define ioctlOBD_READ_PID_RPM 0x0C
#define ioctlOBD_READ_PID_SPEED 0x0D
#define ioctlOBD_READ_PID_TIMING_ADVANCE 0x0E
#define ioctlOBD_READ_PID_INTAKE_TEMP 0x0F
#define ioctlOBD_READ_PID_MAF_FLOW 0x10
#define ioctlOBD_READ_PID_THROTTLE 0x11
#define ioctlOBD_READ_PID_AUX_INPUT 0x1E
#define ioctlOBD_READ_PID_RUNTIME 0x1F
#define ioctlOBD_READ_PID_DISTANCE_WITH_MIL 0x21
#define ioctlOBD_READ_PID_COMMANDED_EGR 0x2C
#define ioctlOBD_READ_PID_EGR_ERROR 0x2D
#define ioctlOBD_READ_PID_COMMANDED_EVAPORATIVE_PURGE 0x2E
#define ioctlOBD_READ_PID_FUEL_LEVEL 0x2F
#define ioctlOBD_READ_PID_WARMS_UPS 0x30
#define ioctlOBD_READ_PID_DISTANCE 0x31
#define ioctlOBD_READ_PID_EVAP_SYS_VAPOR_PRESSURE 0x32
#define ioctlOBD_READ_PID_BAROMETRIC 0x33
#define ioctlOBD_READ_PID_CATALYST_TEMP_B1S1 0x3C
#define ioctlOBD_READ_PID_CATALYST_TEMP_B2S1 0x3D
#define ioctlOBD_READ_PID_CATALYST_TEMP_B1S2 0x3E
#define ioctlOBD_READ_PID_CATALYST_TEMP_B2S2 0x3F
#define ioctlOBD_READ_PID_CONTROL_MODULE_VOLTAGE 0x42
#define ioctlOBD_READ_PID_ABSOLUTE_ENGINE_LOAD 0x43
#define ioctlOBD_READ_PID_AIR_FUEL_EQUIV_RATIO 0x44
#define ioctlOBD_READ_PID_RELATIVE_THROTTLE_POS 0x45
#define ioctlOBD_READ_PID_AMBIENT_TEMP 0x46
#define ioctlOBD_READ_PID_ABSOLUTE_THROTTLE_POS_B 0x47
#define ioctlOBD_READ_PID_ABSOLUTE_THROTTLE_POS_C 0x48
#define ioctlOBD_READ_PID_ACC_PEDAL_POS_D 0x49
#define ioctlOBD_READ_PID_ACC_PEDAL_POS_E 0x4A
#define ioctlOBD_READ_PID_ACC_PEDAL_POS_F 0x4B
#define ioctlOBD_READ_PID_COMMANDED_THROTTLE_ACTUATOR 0x4C
#define ioctlOBD_READ_PID_TIME_WITH_MIL 0x4D
#define ioctlOBD_READ_PID_TIME_SINCE_CODES_CLEARED 0x4E
#define ioctlOBD_READ_PID_ETHANOL_FUEL 0x52
#define ioctlOBD_READ_PID_FUEL_RAIL_PRESSURE 0x59
#define ioctlOBD_READ_PID_HYBRID_BATTERY_PERCENTAGE 0x5B
#define ioctlOBD_READ_PID_ENGINE_OIL_TEMP 0x5C
#define ioctlOBD_READ_PID_FUEL_INJECTION_TIMING 0x5D
#define ioctlOBD_READ_PID_ENGINE_FUEL_RATE 0x5E
#define ioctlOBD_READ_PID_ENGINE_TORQUE_DEMANDED 0x61
#define ioctlOBD_READ_PID_ENGINE_TORQUE_PERCENTAGE 0x62
#define ioctlOBD_READ_PID_ENGINE_REF_TORQUE 0x63

#define ioctlOBD_READ_PID_TRANSMISSION_ACTUAL_GEAR  0xA4    /* TODO: Add normalizeData case. */

#define ioctlOBD_READ_PID_ODOMETER                  0xA6    /* TODO: Add normalizeData case. */

#define ioctlOBD_READ_VIN       0x08000000
#define OBD_VIN_BUFFER_LENGTH       16
typedef struct ObdVinBuffer
{
    char vinBuffer[ OBD_VIN_BUFFER_LENGTH ];
} ObdVinBuffer_t;

/* DTC code related ioctl starts from 0x10000000. */
#define ioctlOBD_READ_DTC       0x10000000
#define ioctlOBD_CLEAR_DTC      0x10000001
#define MAX_DTC_CODES           ( 6U )

/* Device read/write related ioctl starts from 0x20000000. */
#define ioctlOBD_READ_TIMEOUT   0x20000000
#define DEFAULT_READ_TIMEOUT_MS ( 1000 )

/* GPS related ioctl. */
#define ioctlOBD_GPS_ENABLE     0x30000000
#define ioctlOBD_GPS_DISABLE    0x30000001
#define ioctlOBD_GPS_READ       0x30000002

typedef struct ObdDtcData
{
    uint16_t dtc[MAX_DTC_CODES];
    uint8_t dtcCount;
} ObdDtcData_t;

/* This should be the same GPS_Data. */
typedef struct ObdGpsData{
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

#endif
