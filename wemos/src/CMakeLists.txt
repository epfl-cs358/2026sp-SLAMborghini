# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS
        "distress_detection.c"
        "ekf.c"
        "hcsr04_driver.c"
        "imu_encoder_driver.c"
        "motor_pid.c"
        "pure_pursuit_controller.c"
        "stanley_controller.c"
        "uart_bridge.c"
    INCLUDE_DIRS "."
)
