#FreeRTOS coreMQTT demos
afr_demo_module( obd_demo )

#Set the default demo that is enabled in the downloaded package.
afr_set_demo_metadata( ID "OBD_DEMO" )
afr_set_demo_metadata( DESCRIPTION "Examples OBD" )
afr_set_demo_metadata( DISPLAY_NAME "OBD Demos" )

#Add the CMakeLists.txt file of module to metadata list.
afr_module_cmake_files( 
    ${AFR_CURRENT_MODULE}
    ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt
)

afr_module_sources (
    ${AFR_CURRENT_MODULE}
    INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/obd_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/obd_library.c"
    "${CMAKE_CURRENT_LIST_DIR}/gps_library.c"
    "${CMAKE_CURRENT_LIST_DIR}/buzz_library.c"
)

afr_module_dependencies (
    ${AFR_CURRENT_MODULE}
    INTERFACE
        AFR::core_mqtt
        AFR::backoff_algorithm
        AFR::transport_interface_secure_sockets
        AFR::secure_sockets
        AFR::pkcs11_helpers

        #Add dependency on the core_mqtt_demo_dependencies metadata module
        #so that FreeRTOS console shows this demo ONLY when the core MQTT library
        #( or another library depending on coreMQTT) is selected on the console.
        AFR::core_mqtt_demo_dependencies
)
