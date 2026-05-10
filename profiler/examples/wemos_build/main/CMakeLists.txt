set(TESTS_DIR  "${CMAKE_CURRENT_LIST_DIR}/../../wemos")
set(SRC_DIR    "${CMAKE_CURRENT_LIST_DIR}/../../../../wemos/src")
set(PROF_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../")
set(REPO_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../../")

idf_component_register(
    SRCS
        "${PROF_DIR}/profiler.c"
        "${TESTS_DIR}/main.c"
        "${TESTS_DIR}/test_odometry.c"
        "${SRC_DIR}/encoder_ackermann_odometry.c"
    INCLUDE_DIRS
        "${TESTS_DIR}"
        "${SRC_DIR}"
        "${PROF_DIR}"
        "${REPO_DIR}"
    REQUIRES
        freertos
        esp_common
        esp_timer
        heap
)
