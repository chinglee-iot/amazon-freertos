#include <string.h>

#include "FreeRTOS_DriverInterface.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "obd_device.h"

#define LINK_UART_BAUDRATE         ( 115200 )
#define LINK_UART_NUM              ( UART_NUM_2 )
#define LINK_UART_BUF_SIZE         ( 256 )
#define PIN_LINK_UART_RX           ( 13 )
#define PIN_LINK_UART_TX           ( 14 )
#define PIN_LINK_RESET             ( 15 )

#define OBD_TIMEOUT_LONG           ( 10000 )  /* ms */
#define DEFAULT_READ_TIMEOUT_MS    ( 1000 )

typedef struct ObdDeviceContext
{
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

static ObdDeviceContext_t obdDeviceContext =
{
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

static unsigned long IRAM_ATTR esp_millis()
{
    return ( unsigned long ) ( esp_timer_get_time() / 1000ULL );
}

static int uart_receive( char * buffer,
                         int bufsize,
                         unsigned int timeout )
{
    unsigned char n = 0;
    unsigned long startTime = esp_millis();
    unsigned long elapsed;

    for( ; ; )
    {
        elapsed = esp_millis() - startTime;

        if( elapsed > timeout )
        {
            break;
        }

        if( n >= bufsize - 1 )
        {
            break;
        }

        int len = uart_read_bytes( LINK_UART_NUM, ( uint8_t * ) buffer + n, bufsize - n - 1, 1 );

        if( len < 0 )
        {
            break;
        }

        if( len == 0 )
        {
            continue;
        }

        buffer[ n + len ] = 0;

        if( strstr( buffer + n, "\r>" ) )
        {
            n += len;
            break;
        }

        n += len;

        if( strstr( buffer, "..." ) )
        {
            buffer[ 0 ] = 0;
            n = 0;
            timeout += OBD_TIMEOUT_LONG;
        }
    }

    #if VERBOSE_LINK
        configPRINTF( ( "[UART RECV]" ) );
        configPRINTF( ( buffer ) );
    #endif
    return n;
}

static void Obd_Reset( void )
{
    gpio_set_direction( PIN_LINK_RESET, GPIO_MODE_OUTPUT );
    gpio_set_level( PIN_LINK_RESET, 0 );
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
    gpio_set_level( PIN_LINK_RESET, 1 );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
}

Peripheral_Descriptor_t Obd_Open( const int8_t * pcPath,
                                  const uint32_t ulFlags )
{
    Peripheral_Descriptor_t retDesc = NULL;

    /* Reset the OBD link. */
    Obd_Reset();

    /* Open the UART device. */
    uart_config_t uart_config =
    {
        .baud_rate           = LINK_UART_BAUDRATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    /* Configure UART parameters. */
    uart_param_config( LINK_UART_NUM, &uart_config );

    /* Set UART pins. */
    uart_set_pin( LINK_UART_NUM, PIN_LINK_UART_TX, PIN_LINK_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );

    /* Install UART driver. */
    if( uart_driver_install( LINK_UART_NUM, LINK_UART_BUF_SIZE, 0, 0, NULL, 0 ) == ESP_OK )
    {
        retDesc = ( Peripheral_Descriptor_t ) &gObdDevice;
    }
    else
    {
        configPRINTF( ( "Open OBD device fail\r\n" ) );
    }

    return retDesc;
}

size_t Obd_Write( Peripheral_Descriptor_t const pxPeripheral,
                  const void * pvBuffer,
                  const size_t xBytes )
{
    Peripheral_device_t * pDevice = ( Peripheral_device_t * ) pxPeripheral;
    size_t retSize = 0;

    if( pDevice == NULL )
    {
        configPRINTF( ( "Obd_Write bad param pxPeripheral\r\n" ) );
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF( ( "Obd_Write bad param pDeviceData\r\n" ) );
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else
    {
        retSize = uart_write_bytes( LINK_UART_NUM, pvBuffer, xBytes );
    }

    return retSize;
}

size_t Obd_Read( Peripheral_Descriptor_t const pxPeripheral,
                 void * const pvBuffer,
                 const size_t xBytes )
{
    Peripheral_device_t * pDevice = ( Peripheral_device_t * ) pxPeripheral;
    ObdDeviceContext_t * pObdContext = NULL;
    size_t retSize = 0;
    uint8_t * const buffer = ( uint8_t * const ) pvBuffer;
    uint32_t bufsize = xBytes;

    if( pDevice == NULL )
    {
        configPRINTF( ( "Obd_Read bad param pxPeripheral\r\n" ) );
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF( ( "Obd_Read bad param pDeviceData\r\n" ) );
        retSize = FREERTOS_IO_ERROR_BAD_PARAM;
    }
    else
    {
        pObdContext = ( ObdDeviceContext_t * ) pDevice->pDeviceData;
        retSize = uart_receive( buffer, bufsize, pObdContext->readTimeoutMs );
    }

    return retSize;
}

BaseType_t Obd_Ioctl( Peripheral_Descriptor_t const pxPeripheral,
                      uint32_t ulRequest,
                      void * pvValue )
{
    Peripheral_device_t * pDevice = ( Peripheral_device_t * ) pxPeripheral;
    ObdDeviceContext_t * pObdContext = NULL;
    BaseType_t retValue = pdPASS;

    /* configPRINTF(("%s:%d\r\n", __FUNCTION__, __LINE__)); */
    if( pDevice == NULL )
    {
        configPRINTF( ( "Obd_Ioctl bad param pxPeripheral\r\n" ) );
        retValue = pdFAIL;
    }
    else if( pDevice->pDeviceData == NULL )
    {
        configPRINTF( ( "Obd_Ioctl bad param pDeviceData\r\n" ) );
        retValue = pdFAIL;
    }
    else
    {
        pObdContext = ( ObdDeviceContext_t * ) pDevice->pDeviceData;

        switch( ulRequest )
        {
            case ioctlOBD_READ_TIMEOUT:

                if( pvValue == NULL )
                {
                    configPRINTF( ( "ioctlOBD_READ_TIMEOUT bad param pvValue\r\n" ) );
                    retValue = pdFAIL;
                }
                else
                {
                    pObdContext->readTimeoutMs = *( ( uint32_t * ) pvValue );
                }

                break;

            case ioctlOBD_RESET:
                Obd_Reset();
                break;

            default:
                configPRINTF( ( "unsupported ioctl request fail 0x%08x\r\n", ulRequest ) );
                retValue = pdFAIL;
                break;
        }
    }

    return retValue;
}
