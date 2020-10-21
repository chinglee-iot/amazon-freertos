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

#ifndef __CELLULAR_R412M_H__
#define __CELLULAR_R412M_H__

/* Configure logs for R412M functions. */
#ifdef IOT_LOG_LEVEL_CELLULAR
    #define LIBRARY_LOG_LEVEL        IOT_LOG_LEVEL_CELLULAR
#else
    #ifdef IOT_LOG_LEVEL_GLOBAL
        #define LIBRARY_LOG_LEVEL    IOT_LOG_LEVEL_GLOBAL
    #else
        #define LIBRARY_LOG_LEVEL    IOT_LOG_INFO
    #endif
#endif

#define LIBRARY_LOG_NAME    ( "CELLULAR_R412M" )
#include "iot_logging_setup.h"

/*-----------------------------------------------------------*/

#define MIN_TCP_SESSION_ID          ( 0U )
#define MAX_TCP_SESSION_ID          ( 6U )
#define TCP_SESSION_TABLE_LEGNTH    ( MAX_TCP_SESSION_ID + 1 )

#define INVALID_SOCKET_INDEX        ( UINT32_MAX )

/*-----------------------------------------------------------*/

typedef struct cellularModuleContext
{
    uint32_t pSessionMap[ TCP_SESSION_TABLE_LEGNTH ];
} cellularModuleContext_t;

/*-----------------------------------------------------------*/

uint32_t _Cellular_GetSocketId( CellularContext_t * pContext,
                                uint8_t sessionId );

/*-----------------------------------------------------------*/

/**
 * @brief Cellular MNO profiles.
 */
typedef enum MNOProfileType
{
    MNO_PROFILE_SW_DEFAULT = 0,
    MNO_PROFILE_SIM_ICCID_IMSI_SELECT,
    MNO_PROFILE_ATT,
    MNO_PROFILE_VERIZON,
    MNO_PROFILE_TELSTRA,
    MNO_PROFILE_TMOBILE,
    MNO_PROFILE_CHINA_TELECOM,
    MNO_PROFILE_SPRINT = 8,
    MNO_PROFILE_VODAFONE = 19,
    MNO_PROFILE_GLOBAL = 90,
    MNO_PROFILE_STANDARD_EUROPE = 100,
    MNO_PROFILE_NOT_SET = 999
} MNOProfileType_t;

/*-----------------------------------------------------------*/

/* Select network MNO profile. Default value is MNO_PROFILE_SW_DEFAULT */
#define CELLULAR_CONFIG_SET_MNO_PROFILE     ( MNO_PROFILE_ATT )

/*-----------------------------------------------------------*/

/* MAX valid PDP contexts */
#define MAX_PDP_CONTEXTS                            ( 4U )

#define CELULAR_PDN_CONTEXT_TYPE_MAX_SIZE           ( 7U ) /* The length of IP type e.g. IPV4V6. */

/*-----------------------------------------------------------*/

/* +CGDCONT PDN context definition tokens */
#define CELLULAR_PDN_STATUS_POS_CONTEXT_ID       ( 0U )
#define CELLULAR_PDN_STATUS_POS_CONTEXT_TYPE     ( 1U )
#define CELLULAR_PDN_STATUS_POS_APN_NAME         ( 2U )
#define CELLULAR_PDN_STATUS_POS_IP_ADDRESS       ( 3U )

/**
 * @brief Context info from +CGDCONT (Context IP type, APN name, IP Address)
 */
typedef struct CellularPdnContextInfo
{
    BOOL contextsPresent[MAX_PDP_CONTEXTS];                           /**< Context present in +CGDCONT response or not. */
    char ipType[MAX_PDP_CONTEXTS][CELULAR_PDN_CONTEXT_TYPE_MAX_SIZE]; /**< PDN Context type. */
    char apnName[MAX_PDP_CONTEXTS][CELLULAR_APN_MAX_SIZE];            /**< APN name. */
    char ipAddress[MAX_PDP_CONTEXTS][CELLULAR_IP_ADDRESS_MAX_SIZE];   /**< IP address. */
} CellularPdnContextInfo_t;

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/


#endif /* ifndef __CELLULAR_R412M_H__ */
