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
