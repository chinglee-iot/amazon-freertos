/*
 * Amazon FreeRTOS CELLULAR Preview Release
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#include "iot_config.h"
#include "aws_cellular_config.h"
#include "cellular_config_defaults.h"

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

#include "cellular_types.h"
#include "cellular_common.h"
#include "cellular_common_api.h"
#include "cellular_common_portable.h"
#include "limits.h"

/* Cellular module includes. */
#include "cellular_r412m.h"

/*-----------------------------------------------------------*/

static void _cellular_UrcProcessUusoco( CellularContext_t * pContext,
                                        char * pInputLine );
static void _cellular_UrcProcessUusord( CellularContext_t * pContext,
                                        char * pInputLine );
static void _cellular_UrcProcessUusocl( CellularContext_t * pContext,
                                        char * pInputLine );

/*-----------------------------------------------------------*/

/* Try to Keep this map in Alphabetical order. */
/* Cellular HAL common porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
CellularAtParseTokenMap_t CellularUrcHandlerTable[] =
{
    { "CEREG",  Cellular_CommonUrcProcessCereg },
    { "CGREG",  Cellular_CommonUrcProcessCgreg  },
    { "CREG",   Cellular_CommonUrcProcessCreg  },
    { "UUSOCL", _cellular_UrcProcessUusocl     },               /* Socket close URC. */
    { "UUSOCO", _cellular_UrcProcessUusoco     },               /* Socket connect URC. */
    { "UUSORD", _cellular_UrcProcessUusord     }                /* Socket receive URC. */
};

/* Cellular HAL common porting interface. */
/* coverity[misra_c_2012_rule_8_7_violation] */
uint32_t CellularUrcHandlerTableSize = sizeof( CellularUrcHandlerTable ) / sizeof( CellularAtParseTokenMap_t );

/*-----------------------------------------------------------*/

static void _cellular_UrcProcessUusoco( CellularContext_t * pContext,
                                        char * pInputLine )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pLocalInputLine = pInputLine;
    char * pToken = NULL;
    CellularSocketContext_t * pSocketData = NULL;
    uint8_t sessionId = 0;
    uint8_t socketError = 0;
    uint32_t socketIndex = 0;
    int32_t tempValue = 0;

    if( ( pContext != NULL ) && ( pInputLine != NULL ) )
    {
        /* The inputline is in this format +UUSOCO: <socket>,<socket_error>
         * socket_error = 0 : no error, others : error. */
        atCoreStatus = Cellular_ATGetNextTok( &pLocalInputLine, &pToken );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= MIN_TCP_SESSION_ID ) && ( tempValue <= MAX_TCP_SESSION_ID ) )
                {
                    sessionId = ( uint8_t ) tempValue;
                    socketIndex = _Cellular_GetSocketId( pContext, sessionId );
                }
                else
                {
                    IotLogError( "parsing _cellular_UrcProcessKtcpInd session ID failed" );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATGetNextTok( &pLocalInputLine, &pToken );
        }

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= 0 ) && ( tempValue <= UINT8_MAX ) )
                {
                    socketError = ( uint8_t ) tempValue;
                }
                else
                {
                    IotLogError( "parsing _cellular_UrcProcessUusoco socket error failed" );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        /* Call the callback function of this session. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSocketData = _Cellular_GetSocketData( pContext, socketIndex );

            if( pSocketData == NULL )
            {
                IotLogError( "_cellular_UrcProcessUusoco : invalid socket index %d", socketIndex );
            }
            else if( pSocketData->pModemData != ( void * ) sessionId )
            {
                IotLogError( "_cellular_UrcProcessUusoco : session not match %d socket index %d",
                             ( uint32_t ) pSocketData->pModemData, socketIndex );
            }
            else
            {
                if( socketError == 0 )
                {
                    pSocketData->socketState = SOCKETSTATE_CONNECTED;
                    IotLogDebug( "Notify session %d with socket opened\r\n", sessionId );

                    if( pSocketData->openCallback != NULL )
                    {
                        pSocketData->openCallback( CELLULAR_URC_SOCKET_OPENED,
                                                   pSocketData, pSocketData->pOpenCallbackContext );
                    }
                }
                else
                {
                    if( pSocketData->openCallback != NULL )
                    {
                        pSocketData->openCallback( CELLULAR_URC_SOCKET_OPEN_FAILED,
                                                   pSocketData, pSocketData->pOpenCallbackContext );
                    }
                }
            }
        }
    }
}

/*-----------------------------------------------------------*/

static void _cellular_UrcProcessUusord( CellularContext_t * pContext,
                                        char * pInputLine )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pLocalInputLine = pInputLine;
    char * pToken = NULL;
    CellularSocketContext_t * pSocketData = NULL;
    uint8_t sessionId = 0;
    uint32_t dataLength = 0;
    uint32_t socketIndex = 0;
    int32_t tempValue = 0;

    if( ( pContext != NULL ) && ( pInputLine != NULL ) )
    {
        /* The inputline is in this format +UUSOCO: <socket>,<data_length> */
        atCoreStatus = Cellular_ATGetNextTok( &pLocalInputLine, &pToken );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= MIN_TCP_SESSION_ID ) && ( tempValue <= MAX_TCP_SESSION_ID ) )
                {
                    sessionId = ( uint8_t ) tempValue;
                    socketIndex = _Cellular_GetSocketId( pContext, sessionId );
                }
                else
                {
                    IotLogError( "parsing _cellular_UrcProcessUusord session ID failed" );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        /* Skip data length. */

        /* Call the callback function of this session. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSocketData = _Cellular_GetSocketData( pContext, socketIndex );

            if( pSocketData == NULL )
            {
                IotLogError( "_cellular_UrcProcessUusord : invalid socket index %d", socketIndex );
            }
            else if( pSocketData->pModemData != ( void * ) sessionId )
            {
                IotLogError( "_cellular_UrcProcessUusord : session not match %d socket index %d",
                             ( uint32_t ) pSocketData->pModemData, socketIndex );
            }
            else
            {
                /* Indicate the upper layer about the data reception. */
                if( pSocketData->dataReadyCallback != NULL )
                {
                    pSocketData->dataReadyCallback( pSocketData, pSocketData->pDataReadyCallbackContext );
                }
                else
                {
                    IotLogDebug( "_cellular_UrcProcessUusord: Data ready callback not set!!" );
                }
            }
        }
    }
}

/*-----------------------------------------------------------*/

static void _cellular_UrcProcessUusocl( CellularContext_t * pContext,
                                        char * pInputLine )
{
    CellularPktStatus_t pktStatus = CELLULAR_PKT_STATUS_OK;
    CellularATError_t atCoreStatus = CELLULAR_AT_SUCCESS;
    char * pLocalInputLine = pInputLine;
    char * pToken = NULL;
    CellularSocketContext_t * pSocketData = NULL;
    uint8_t sessionId = 0;
    uint32_t dataLength = 0;
    uint32_t socketIndex = 0;
    int32_t tempValue = 0;

    if( ( pContext != NULL ) && ( pInputLine != NULL ) )
    {
        /* The inputline is in this format +UUSOCL: <socket> */
        atCoreStatus = Cellular_ATGetNextTok( &pLocalInputLine, &pToken );

        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            atCoreStatus = Cellular_ATStrtoi( pToken, 10, &tempValue );

            if( atCoreStatus == CELLULAR_AT_SUCCESS )
            {
                if( ( tempValue >= MIN_TCP_SESSION_ID ) && ( tempValue <= MAX_TCP_SESSION_ID ) )
                {
                    sessionId = ( uint8_t ) tempValue;
                    socketIndex = _Cellular_GetSocketId( pContext, sessionId );
                }
                else
                {
                    IotLogError( "parsing _cellular_UrcProcessUusocl session ID failed" );
                    atCoreStatus = CELLULAR_AT_ERROR;
                }
            }
        }

        /* Call the callback function of this session. */
        if( atCoreStatus == CELLULAR_AT_SUCCESS )
        {
            pSocketData = _Cellular_GetSocketData( pContext, socketIndex );

            if( pSocketData == NULL )
            {
                IotLogError( "_cellular_UrcProcessUusocl : invalid socket index %d", socketIndex );
            }
            else if( pSocketData->pModemData != ( void * ) sessionId )
            {
                IotLogError( "_cellular_UrcProcessUusocl : session not match %d socket index %d",
                             ( uint32_t ) pSocketData->pModemData, socketIndex );
            }
            else
            {
                /* Indicate the upper layer about the data reception. */
                if( pSocketData->closedCallback != NULL )
                {
                    pSocketData->closedCallback( pSocketData, pSocketData->pClosedCallbackContext );
                }
                else
                {
                    IotLogDebug( "_cellular_UrcProcessUusord: Data ready callback not set!!" );
                }
            }
        }
    }
}

/*-----------------------------------------------------------*/
