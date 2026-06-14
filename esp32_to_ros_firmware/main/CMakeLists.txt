idf_component_register(SRCS "esp32_to_ros_firmware.cpp" "robot_control.cpp" "robot_encoder.cpp" "robot_imu.cpp" "esp32_serial_transport.c"
                    INCLUDE_DIRS ".")

target_compile_definitions(${COMPONENT_LIB} PUBLIC RMW_UXRCE_TRANSPORT_CUSTOM=1)

