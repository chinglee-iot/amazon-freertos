# FreeRTOS AWS Reference Integrations

## Introduction

The branch is a development branch of HL7802 porting.

## Run mqtt demo application
1. Setup demo config
  - demos/include/aws_cellular_demo.h
  - demos/include/aws_clientcredential.h
  - demos/include/aws_clientcredential_keys.h

2. Choose cellular module
  - vendors/pc/boards/windows/CMakeLists.txt
```
-include("${AFR_VENDORS_DIR}/quectel/modules/bg96/CMakeLists.txt")
+include("${AFR_VENDORS_DIR}/sierra/modules/hl7802/CMakeLists.txt")
```

3. Setup board demo config
  - vendors/pc/boards/windows/aws_demos/config_files/FreeRTOSConfig.h :
```
Setup configNETWORK_INTERFACE_TO_USE
```
  - vendors/pc/boards/windows/aws_demos/config_files/aws_cellular_config.h :
```
setup cellularconfigCOMM_INTERFACE_PORT
+#define CELLULAR_SUPPORT_GETHOSTBYNAME      ( 0 )
+#define CELLULAR_IP_ADDRESS_MAX_SIZE        ( 64U )  /* IP Address is used to store the domain name. */
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
  - vendors/pc/boards/windows/aws_demos/config_files/aws_secure_sockets_config.h : 
```
-#define socketsconfigBYTE_ORDER                   pdLITTLE_ENDIAN
+#define socketsconfigBYTE_ORDER                   pdBIG_ENDIAN
```

4. cmake option
Add extra cmake compile option
  - -DBOARD_HAS_CELLULAR=1
  - -DSECURE_SOCKETS_CELLULAR=1

## Run cellular test cases
1. Setup test config
  - tests/include/aws_test_cellular.h
  - tests/include/aws_test_tcp.h : setup tcptestECHO_SERVER_ADDR and port

2. Setup Choose cellular module
  - vendors/pc/boards/windows/CMakeLists.txt
```
-include("${AFR_VENDORS_DIR}/quectel/modules/bg96/CMakeLists.txt")
+include("${AFR_VENDORS_DIR}/sierra/modules/hl7802/CMakeLists.txt")
```

3. Setup board test config
  - vendors/pc/boards/windows/aws_tests/config_files/FreeRTOSConfig.h : Setup configNETWORK_INTERFACE_TO_USE

  - vendors/pc/boards/windows/aws_tests/config_files/aws_cellular_config.h : setup cellularconfigCOMM_INTERFACE_PORT

  - vendors/pc/boards/windows/aws_tests/config_files/aws_test_runner_config.h
```
-#define testrunnerFULL_CELLULAR_ENABLED               0
-#define testrunnerFULL_TCP_ENABLED                    1
+#define testrunnerFULL_CELLULAR_ENABLED               1
+#define testrunnerFULL_TCP_ENABLED                    0

```
4. Setup repeat echo server for EDRX echo times test
  - tools/echo_server/config.json : 
```
{
    ...
    "repeat-mode" : true,
    "repeat-interval-seconds": 30,
    ...
}

```
  - tests/include/aws_tests_cellular.h
```
/* Repeat echo server send interfal for EDRX echo times test. */
#define testCELLULAR_EDRX_ECHO_SERVER_DATA_SEND_INTERVAL_MS    ( 30000 )
```

5. cmake option
Add extra cmake compile option
  - -DBOARD_HAS_CELLULAR=1
  - -DSECURE_SOCKETS_CELLULAR=1
  - -DAFR_ENABLE_TESTS=1

## Cloning
This repo uses [Git Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to bring in dependent components.

Note: If you download the ZIP file provided by GitHub UI, you will not get the contents of the submodules. (The ZIP file is also not a valid git repository)

To clone using HTTPS:
```
git clone https://github.com/chinglee-iot/amazon-freertos.git --recurse-submodules -b feature/cellular_hl7802
```
Using SSH:
```
git clone git@github.com:chinglee-iot/amazon-freertos.git --recurse-submodules -b feature/cellular_hl7802
```

If you have downloaded the repo without using the `--recurse-submodules` argument, you need to run:
```
git submodule update --init --recursive
```


## Mbed TLS License
This repository uses Mbed TLS under Apache 2.0
