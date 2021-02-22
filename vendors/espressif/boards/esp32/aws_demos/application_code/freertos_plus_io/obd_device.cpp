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

extern FreematicsESP32 sys;
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
    bool retInit = false;
    Peripheral_Descriptor_t retDesc = NULL;

    configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));

    /* Initialize OBD device. */
    obd.begin( sys.link );  /* Setup the link only. This function return void. */
    retInit = obd.init();
    if( retInit == false )
    {
        configPRINTF(("OBD init failed\r\n"));
        obd.uninit();
    }
    else
    {
        /* Setup the system indication and logging. */
        pinMode(PIN_LED, OUTPUT);
        digitalWrite(PIN_LED, HIGH);
        delay(1000);
        digitalWrite(PIN_LED, LOW);
        retDesc = ( Peripheral_Descriptor_t )&gObdDevice;
    }

    return retDesc;
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
        configPRINTF(("Obd_Write bad param pxPeripheral\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF(("Obd_Write bad param pDeviceData\r\n"));
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pBuf[xBytes-1] != '\0' )
    {
        configPRINTF(("Obd_Write bad param pvBuffer\r\n"));
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
    ObdDtcData_t *pObdDtcData = NULL;
    Peripheral_device_t *pDevice = ( Peripheral_device_t* )pxPeripheral;
    ObdDeviceContext_t *pObdContext = NULL;
    BaseType_t retValue = pdPASS;
    GPS_DATA *pGpsData = NULL;
    ObdVinBuffer_t *pVinBuffer = NULL;
    int pidValue = 0;
    
    // configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__));
    if( pDevice == NULL )
    {
        configPRINTF(("Obd_Ioctl bad param pxPeripheral\r\n"));
        retValue = pdFAIL;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF(("Obd_Ioctl bad param pDeviceData\r\n"));
        retValue = pdFAIL;
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
                if( pvValue == NULL )
                {
                    configPRINTF(("ioctlOBD_READ_PID bad param pvValue\r\n"));
                    retValue = pdFAIL;
                }
                else
                {
                    if( pObdContext->pObd->readPID( ulRequest, pidValue ) == false )
                    {
                        configPRINTF(("ioctlOBD_READ_PID fail 0x%08x\r\n", ulRequest));
                        retValue = pdFAIL;
                    }
                    else
                    {
                        *( (int*) pvValue ) = pidValue;
                    }
                }
                break;
            case ioctlOBD_READ_DTC:
                if( pvValue == NULL )
                {
                    configPRINTF(("ioctlOBD_READ_DTC bad param pvValue\r\n"));
                    retValue = pdFAIL;
                }
                else
                {
                    pObdDtcData = ( ObdDtcData_t * )pvValue;
                    pObdDtcData->dtcCount = pObdContext->pObd->readDTC( pObdDtcData->dtc, MAX_DTC_CODES );
                }
                break;
            case ioctlOBD_CLEAR_DTC:
                pObdContext->pObd->clearDTC();
                break;
            case ioctlOBD_READ_TIMEOUT:
                if( pvValue == NULL )
                {
                    configPRINTF(("ioctlOBD_READ_TIMEOUT bad param pvValue\r\n"));
                    retValue = pdFAIL;
                }
                else
                {
                    pObdContext->readTimeoutMs = *(( uint32_t* )pvValue);
                }
                break;
            case ioctlOBD_GPS_ENABLE:
                if( pObdContext->pSys->gpsBegin() == false )
                {
                    configPRINTF(("ioctlOBD_GPS_ENABLE fail\r\n"));
                    retValue = pdFAIL;
                }
                break;
            case ioctlOBD_GPS_DISABLE:
                pObdContext->pSys->gpsEnd();    /* Return value is void. */
                break;
            case ioctlOBD_GPS_READ:
                if( pvValue == NULL )
                {
                    configPRINTF(("ioctlOBD_GPS_READ bad param pvValue\r\n"));
                    retValue = pdFAIL;
                }
                else
                {
                    if( pObdContext->pSys->gpsGetData( &pGpsData ) == false )
                    {
                        configPRINTF(("ioctlOBD_GPS_READ fail\r\n"));
                        retValue = pdFAIL;
                    }
                    else
                    {
                        memcpy( pvValue, pGpsData, sizeof(GPS_DATA) );
                    }
                }
                break;
            case ioctlOBD_READ_VIN:
                if( pvValue == NULL )
                {
                    configPRINTF(("ioctlOBD_READ_VIN bad param pvValue\r\n"));
                    retValue = pdFAIL;
                }
                else
                {
                    char buffer[128] = { 0 };
                    pVinBuffer = ( ObdVinBuffer_t*) pvValue;
                    if( pObdContext->pObd->getVIN( buffer, sizeof(buffer) ) == false )
                    {
                        configPRINTF(("ioctlOBD_READ_VIN fail\r\n"));
                        retValue = pdFAIL;
                    }
                    else
                    {
                        strncpy(pVinBuffer->vinBuffer, buffer, OBD_VIN_BUFFER_LENGTH - 1);
                    }
                }
                break;
            default:
                configPRINTF(("unsupported ioctl request fail 0x%08x\r\n", ulRequest));
                retValue = pdFAIL;
                break;
        }
    }
    return retValue;
}
