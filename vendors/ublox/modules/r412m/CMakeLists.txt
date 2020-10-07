#This port uses cellular HAL common
include( "${AFR_VENDORS_DIR}/aws/modules/3gpp/CMakeLists.txt" )

#Cellular common
afr_module( NAME cellular_module_port PUBLIC )

afr_module_sources(
    cellular_module_port
    PRIVATE
    "${AFR_VENDORS_DIR}/ublox/modules/r412m/ports/cellular/cellular_r412m.c"
    "${AFR_VENDORS_DIR}/ublox/modules/r412m/ports/cellular/cellular_r412m_api.c"
    "${AFR_VENDORS_DIR}/ublox/modules/r412m/ports/cellular/cellular_r412m_urc_handler.c"
    "${AFR_VENDORS_DIR}/ublox/modules/r412m/ports/cellular/cellular_r412m_wrapper.c"
    )

afr_module_dependencies(
    cellular_module_port
    PRIVATE
    AFR::cellular
    AFR::cellular_common
    )
