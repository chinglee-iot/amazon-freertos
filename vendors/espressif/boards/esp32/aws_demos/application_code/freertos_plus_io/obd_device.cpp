#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_DriverInterface.h"

#include "Arduino.h"
#include "FreematicsPlus.h"

#include "obd_device.h"

static FreematicsESP32 sys;
static COBD obd;
static bool connected = false;
static unsigned long count = 0;

Peripheral_Descriptor_t Obd_Open( const int8_t * pcPath,
                                       const uint32_t ulFlags );

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes );

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes );

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const xPeripheral,
                           uint32_t ulRequest,
                           void * pvValue );

Peripheral_device_t gObdDevice = 
{
    "/dev/obd",
    Obd_Open,
    Obd_Write,
    Obd_Read,
    Obd_Ioctl,
    NULL
};

Peripheral_Descriptor_t Obd_Open( const int8_t * pcPath,
                                       const uint32_t ulFlags )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    
    /* Setup the OBD device. */
    // put your setup code here, to run once:
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED, LOW);
    Serial.begin(115200);

    // initializations
    while (!sys.begin());
    obd.begin(sys.link);
    obd.init();

    configPRINTF(("%s:%d done\r\n", __FUNCTION__, __LINE__));


    return &gObdDevice;
}

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                      void * const pvBuffer,
                      const size_t xBytes )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return 0;
}

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes )
{
    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    return 0;
}

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const xPeripheral,
                           uint32_t ulRequest,
                           void * pvValue )
{
    int32_t *pValue = NULL;
    ObdDtcData_t *pObdDtcData = NULL;
    
    // configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    
    switch( ulRequest ) 
    {
        case ioctlOBD_READ_PID_ENGINE_LOAD:
        case ioctlOBD_READ_PID_COOLANT_TEMP:
        case ioctlOBD_READ_PID_SHORT_TERM_FUEL_TRIM_1:
        case ioctlOBD_READ_PID_LONG_TERM_FUEL_TRIM_1:
        case ioctlOBD_READ_PID_SHORT_TERM_FUEL_TRIM_2:
        case ioctlOBD_READ_PID_LONG_TERM_FUEL_TRIM_2:
        case ioctlOBD_READ_PID_FUEL_PRESSURE:
        case ioctlOBD_READ_PID_INTAKE_MAP:
        case ioctlOBD_READ_PID_RPM:
        case ioctlOBD_READ_PID_SPEED:
        case ioctlOBD_READ_PID_TIMING_ADVANCE:
        case ioctlOBD_READ_PID_INTAKE_TEMP:
        case ioctlOBD_READ_PID_MAF_FLOW:
        case ioctlOBD_READ_PID_THROTTLE:
        case ioctlOBD_READ_PID_AUX_INPUT:
        case ioctlOBD_READ_PID_RUNTIME:
        case ioctlOBD_READ_PID_DISTANCE_WITH_MIL:
        case ioctlOBD_READ_PID_COMMANDED_EGR:
        case ioctlOBD_READ_PID_EGR_ERROR:
        case ioctlOBD_READ_PID_COMMANDED_EVAPORATIVE_PURGE:
        case ioctlOBD_READ_PID_FUEL_LEVEL:
        case ioctlOBD_READ_PID_WARMS_UPS:
        case ioctlOBD_READ_PID_DISTANCE:
        case ioctlOBD_READ_PID_EVAP_SYS_VAPOR_PRESSURE:
        case ioctlOBD_READ_PID_BAROMETRIC:
        case ioctlOBD_READ_PID_CATALYST_TEMP_B1S1:
        case ioctlOBD_READ_PID_CATALYST_TEMP_B2S1:
        case ioctlOBD_READ_PID_CATALYST_TEMP_B1S2:
        case ioctlOBD_READ_PID_CATALYST_TEMP_B2S2:
        case ioctlOBD_READ_PID_CONTROL_MODULE_VOLTAGE:
        case ioctlOBD_READ_PID_ABSOLUTE_ENGINE_LOAD:
        case ioctlOBD_READ_PID_AIR_FUEL_EQUIV_RATIO:
        case ioctlOBD_READ_PID_RELATIVE_THROTTLE_POS:
        case ioctlOBD_READ_PID_AMBIENT_TEMP:
        case ioctlOBD_READ_PID_ABSOLUTE_THROTTLE_POS_B:
        case ioctlOBD_READ_PID_ABSOLUTE_THROTTLE_POS_C:
        case ioctlOBD_READ_PID_ACC_PEDAL_POS_D:
        case ioctlOBD_READ_PID_ACC_PEDAL_POS_E:
        case ioctlOBD_READ_PID_ACC_PEDAL_POS_F:
        case ioctlOBD_READ_PID_COMMANDED_THROTTLE_ACTUATOR:
        case ioctlOBD_READ_PID_TIME_WITH_MIL:
        case ioctlOBD_READ_PID_TIME_SINCE_CODES_CLEARED:
        case ioctlOBD_READ_PID_ETHANOL_FUEL:
        case ioctlOBD_READ_PID_FUEL_RAIL_PRESSURE:
        case ioctlOBD_READ_PID_HYBRID_BATTERY_PERCENTAGE:
        case ioctlOBD_READ_PID_ENGINE_OIL_TEMP:
        case ioctlOBD_READ_PID_FUEL_INJECTION_TIMING:
        case ioctlOBD_READ_PID_ENGINE_FUEL_RATE:
        case ioctlOBD_READ_PID_ENGINE_TORQUE_DEMANDED:
        case ioctlOBD_READ_PID_ENGINE_TORQUE_PERCENTAGE:
        case ioctlOBD_READ_PID_ENGINE_REF_TORQUE:
            pValue = ( int32_t* )pvValue;
            obd.readPID( ulRequest, *pValue );
            break;
        case ioctlOBD_READ_DTC:
            pObdDtcData = ( ObdDtcData_t * )pvValue;
            pObdDtcData->dtcCount = obd.readDTC( pObdDtcData->dtc, MAX_DTC_CODES );
        case ioctlOBD_CLEAR_DTC:
            obd.clearDTC();
            break;
        default:
            break;
    }
    return 0;
}
