#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_DriverInterface.h"

#include "Arduino.h"
#include "FreematicsPlus.h"

#include "obd_device.h"

typedef struct ObdDeviceContext 
{
    FreematicsESP32 *pSys;
    COBD *pObd;
    uint32_t readTimeoutMs;
} ObdDeviceContext_t;

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

static FreematicsESP32 sys;
static COBD obd;
static ObdDeviceContext_t obdDeviceContext =
{
    &sys,
    &obd,
    DEFAULT_READ_TIMEOUT_MS
};

Peripheral_device_t gObdDevice = 
{
    "/dev/obd",
    Obd_Open,
    Obd_Write,
    Obd_Read,
    Obd_Ioctl,
    &obdDeviceContext
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

    return &gObdDevice;
}

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                 void * const pvBuffer,
                 const size_t xBytes )
{
    Peripheral_device_t *pDevice = ( Peripheral_device_t* )pxPeripheral;
    ObdDeviceContext_t *pObdContext = NULL;
    size_t retSize = 0;
    char * const pBuf = ( char * const ) pvBuffer;

    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    
    if( pDevice == NULL )
    {
        configPRINTF(("Obd_Read bad param pxPeripheral\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF(("Obd_Read bad param pDeviceData\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else
    {
        pObdContext = ( ObdDeviceContext_t* ) pDevice->pDeviceData;
        retSize = pObdContext->pObd->link->receive( pBuf, xBytes, pObdContext->readTimeoutMs );
    }
    
    return retSize;
}

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                       const void * pvBuffer,
                       const size_t xBytes )
{
    Peripheral_device_t *pDevice = ( Peripheral_device_t* )pxPeripheral;
    ObdDeviceContext_t *pObdContext = NULL;
    size_t retSize = 0;
    char * const pBuf = ( char * const ) pvBuffer;

    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    
    if( pDevice == NULL )
    {
        configPRINTF(("Obd_Read bad param pxPeripheral\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF(("Obd_Read bad param pDeviceData\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pBuf[xBytes-1] != '\0' )
    {
        configPRINTF(("Obd_Read bad param pvBuffer\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else
    {
        pObdContext = ( ObdDeviceContext_t* ) pDevice->pDeviceData;
        retSize = pObdContext->pObd->link->send( pBuf );
    }
    
    return retSize;
}

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const pxPeripheral,
                           uint32_t ulRequest,
                           void * pvValue )
{
    int32_t *pValue = NULL;
    ObdDtcData_t *pObdDtcData = NULL;
    Peripheral_device_t *pDevice = ( Peripheral_device_t* )pxPeripheral;
    ObdDeviceContext_t *pObdContext = NULL;
    BaseType_t retValue = FREERTOS_IO_OKAY;
    GPS_DATA *pGpsData = NULL;
    ObdVinBuffer_t *pVinBuffer = NULL;
    
    // configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    if( pDevice == NULL )
    {
        configPRINTF(("Obd_Read bad param pxPeripheral\r\n"));
        retValue = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF(("Obd_Read bad param pDeviceData\r\n"));
        retValue = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else
    {
        pObdContext = ( ObdDeviceContext_t* ) pDevice->pDeviceData;
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
                pObdContext->pObd->readPID( ulRequest, *pValue );
                break;
            case ioctlOBD_READ_DTC:
                pObdDtcData = ( ObdDtcData_t * )pvValue;
                pObdDtcData->dtcCount = pObdContext->pObd->readDTC( pObdDtcData->dtc, MAX_DTC_CODES );
            case ioctlOBD_CLEAR_DTC:
                pObdContext->pObd->clearDTC();
                break;
            case ioctlOBD_READ_TIMEOUT:
                if( pvValue != NULL )
                {
                    pObdContext->readTimeoutMs = *(( uint32_t* )pvValue);
                }
                break;
            case ioctlOBD_GPS_ENABLE:
                pObdContext->pSys->gpsBegin();
                break;
            case ioctlOBD_GPS_DISABLE:
                pObdContext->pSys->gpsEnd();
                break;
            case ioctlOBD_GPS_READ:
                if( pvValue != NULL )
                {
                    pObdContext->pSys->gpsGetData( &pGpsData );
                    memcpy( pvValue, pGpsData, sizeof(GPS_DATA) );
                }
                break;
            case ioctlOBD_READ_VIN:
                if( pvValue != NULL )
                {
                    char buffer[128] = { 0 };
                    pVinBuffer = ( ObdVinBuffer_t*) pvValue;
                    pObdContext->pObd->getVIN( buffer, sizeof(buffer) );
                    strncpy(pVinBuffer->vinBuffer, buffer, OBD_VIN_BUFFER_LENGTH - 1);
                }
                break;
            default:
                break;
        }
    }
    return retValue;
}
