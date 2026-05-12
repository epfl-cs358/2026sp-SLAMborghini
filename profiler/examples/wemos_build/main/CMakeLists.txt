set(TESTS_DIR  "${CMAKE_CURRENT_LIST_DIR}/../../wemos")
set(SRC_DIR    "${CMAKE_CURRENT_LIST_DIR}/../../../../wemos/src")
set(WEMOS_DIR  "${CMAKE_CURRENT_LIST_DIR}/../../../../wemos")
set(PROF_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../")
set(REPO_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../../")

idf_component_register(
    SRCS
        "${PROF_DIR}/profiler.c"
        "${TESTS_DIR}/main.c"
        "${TESTS_DIR}/test_odometry.c"
        "${TESTS_DIR}/test_uart_echo.c"
        "${SRC_DIR}/encoder_ackermann_odometry.c"
        "${SRC_DIR}/imu_encoder_driver.c"
        "${SRC_DIR}/uart_bridge.c"
        "${WEMOS_DIR}/imu_gyro.c"
    INCLUDE_DIRS
        "${TESTS_DIR}"
        "${SRC_DIR}"
        "${WEMOS_DIR}"
        "${PROF_DIR}"
        "${REPO_DIR}"
    REQUIRES
        freertos
        esp_common
        esp_timer
        esp_driver_i2c
        heap
        driver
)
