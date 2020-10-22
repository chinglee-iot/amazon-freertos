# FreeRTOS AWS Reference Integrations

## Introduction

The branch is a development branch of r412m porting.

## Run mqtt demo application
1. Setup demo config
  - demos/include/aws_cellular_demo.h
  - demos/include/aws_clientcredential.h
  - demos/include/aws_clientcredential_keys.h

2. Choose cellular module
  - vendors/pc/boards/windows/CMakeLists.txt
```
-include("${AFR_VENDORS_DIR}/quectel/modules/bg96/CMakeLists.txt")
+include("${AFR_VENDORS_DIR}/ublox/modules/r412m/CMakeLists.txt")
```

3. Setup board demo config
  - vendors/pc/boards/windows/aws_demos/config_files/FreeRTOSConfig.h :
```
Setup configNETWORK_INTERFACE_TO_USE
```
  - vendors/pc/boards/windows/aws_demos/config_files/aws_cellular_config.h :
```
+#define cellularconfigCOMM_INTERFACE_PORT                   "COM42" /* Setup the com port according to your environment. */
+#define CELLULAR_SUPPORT_GETHOSTBYNAME      ( 0 )
+#define CELLULAR_IP_ADDRESS_MAX_SIZE        ( 64U )  /* IP Address is used to store the domain name. */
+#define CELLULAR_MAX_RECV_DATA_LEN          ( 1024U )
+#define CELLULAR_MAX_SEND_DATA_LEN          ( 1024U )
```
  - vendors/pc/boards/windows/aws_demos/config_files/aws_demo_config.h :
```
-#define democonfigNETWORK_TYPES                        ( AWSIOT_NETWORK_TYPE_ETH )
+#define democonfigNETWORK_TYPES                        ( AWSIOT_NETWORK_TYPE_CELLULAR )
```
  - vendors/pc/boards/windows/aws_demos/config_files/aws_iot_network_config.h : 
```
-#define configSUPPORTED_NETWORKS    ( AWSIOT_NETWORK_TYPE_ETH )
+#define configSUPPORTED_NETWORKS    ( AWSIOT_NETWORK_TYPE_ETH | AWSIOT_NETWORK_TYPE_CELLULAR )

-#define configENABLED_NETWORKS      ( AWSIOT_NETWORK_TYPE_ETH )
+#define configENABLED_NETWORKS      ( AWSIOT_NETWORK_TYPE_ETH | AWSIOT_NETWORK_TYPE_CELLULAR )

```

4. cmake option
Add extra cmake compile option
  - -DBOARD_HAS_CELLULAR=1
  - -DSECURE_SOCKETS_CELLULAR=1

## Run cellular test application
1. Setup test runner test case selection
  - vendors/pc/boards/windows/aws_tests/config_files/aws_test_runner_config.h
```
-#define testrunnerFULL_CELLULAR_ENABLED               0
-#define testrunnerFULL_TCP_ENABLED                    1
+#define testrunnerFULL_CELLULAR_ENABLED               1
+#define testrunnerFULL_TCP_ENABLED                    0
```

2. Setup cellular test config and tcp echo server config
  - tests/include/aws_test_cellular.h ( cellular test config )
 ```
  #define testCELLULAR_PDN_CONTEXT_ID                            ( CELLULAR_PDN_CONTEXT_ID_MIN )

 /* DNS server address. */
-#define testCELLULAR_DNS_SERVER_ADDRESS                        ""
+#define testCELLULAR_DNS_SERVER_ADDRESS                        "<DNS_SERVER>"

 /* Host name to resolve. The host name should only has one IP address. */
-#define testCELLULAR_HOST_NAME                                 ""
+#define testCELLULAR_HOST_NAME                                 "<HOST_NAME_TO_RESOLVE>"

 /* Host name resolved address. The resolved address should be the IP address of
  * testCELLULAR_HOST_NAME. */
-#define testCELLULAR_HOST_NAME_ADDRESS                         ""
+#define testCELLULAR_HOST_NAME_ADDRESS                         "<HOST_NAME_IP_ADDRESS>"

 /* Repeat echo server address for EDRX echo times test. */
-#define testCELLULAR_EDRX_ECHO_SERVER_ADDRESS                  ""
+#define testCELLULAR_EDRX_ECHO_SERVER_ADDRESS                  "<REPEAT_ECHO_SERVER_ADDRESS>"

 /* Repeat echo server port for EDRX echo times test. */
-#define testCELLULAR_EDRX_ECHO_SERVER_PORT                     ( )
+#define testCELLULAR_EDRX_ECHO_SERVER_PORT                     ( <REPEAT_ECHO_SERVER_PORT> )

 /* Repeat echo server send interfal for EDRX echo times test. */
-#define testCELLULAR_EDRX_ECHO_SERVER_DATA_SEND_INTERVAL_MS    ( )
+#define testCELLULAR_EDRX_ECHO_SERVER_DATA_SEND_INTERVAL_MS    ( 30000 )

 /*
  * 2 GSM
@@ -60,7 +60,7 @@
  * 4 LTE Cat M1
  * 5 LTE Cat NB1
  */
-#define testCELLULAR_EDRX_RAT                                  ( )
+#define testCELLULAR_EDRX_RAT                                  ( 4 )
 ```
  - tests/include/aws_test_tcp.h ( TCP echo server config )
```
-#define tcptestECHO_SERVER_ADDR0         34
-#define tcptestECHO_SERVER_ADDR1         218
-#define tcptestECHO_SERVER_ADDR2         25
-#define tcptestECHO_SERVER_ADDR3         197
+#define tcptestECHO_SERVER_ADDR0         <ERCHO_SERVER_IP_0>
+#define tcptestECHO_SERVER_ADDR1         <ERCHO_SERVER_IP_1>
+#define tcptestECHO_SERVER_ADDR2         <ERCHO_SERVER_IP_2>
+#define tcptestECHO_SERVER_ADDR3         <ERCHO_SERVER_IP_3>
 #define tcptestECHO_PORT                 ( <ECHO_SERVER_PORT> )
```

3. setup board test config
  - vendors/pc/boards/windows/aws_tests/config_files/aws_cellular_config.h
```
-#define cellularconfigCOMM_INTERFACE_PORT                   "COM36"
+#define cellularconfigCOMM_INTERFACE_PORT                   "<YOUR_COM_PORT>"

 /* When enable CELLULAR_CONFIG_STATIC_ALLOCATION_CONTEXT,
  * below the contexts have statically allocated,
@@ -89,4 +89,10 @@
     #endif
 #endif /* if ( CELLULAR_CONFIG_STATIC_ALLOCATION_SOCKET_CONTEXT == 1U ) */

+#define CELLULAR_MAX_RECV_DATA_LEN    ( 1024U )
+#define CELLULAR_MAX_SEND_DATA_LEN    ( 1024U )
```
  - vendors/pc/boards/windows/aws_tests/config_files/FreeRTOSConfig.h
```
-#define configNETWORK_INTERFACE_TO_USE       ( 0L )
+#define configNETWORK_INTERFACE_TO_USE       ( <YOUR_ETHERNET_INTERFACE> )
```

4. ECHO server setup
 - https://docs.aws.amazon.com/freertos/latest/portingguide/afr-echo-server.html

## Cloning
This repo uses [Git Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to bring in dependent components.

Note: If you download the ZIP file provided by GitHub UI, you will not get the contents of the submodules. (The ZIP file is also not a valid git repository)

To clone using HTTPS:
```
git clone https://github.com/chinglee-iot/amazon-freertos.git --recurse-submodules -b feature/cellular_r412m
```
Using SSH:
```
git clone git@github.com:chinglee-iot/amazon-freertos.git --recurse-submodules -b feature/cellular_r412m
```

If you have downloaded the repo without using the `--recurse-submodules` argument, you need to run:
```
git submodule update --init --recursive
```


## Mbed TLS License
This repository uses Mbed TLS under Apache 2.0
