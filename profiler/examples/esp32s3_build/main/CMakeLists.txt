set(TESTS_DIR  "${CMAKE_CURRENT_LIST_DIR}/../../esp32s3")
set(SRC_DIR    "${CMAKE_CURRENT_LIST_DIR}/../../../../esp32s3/src")
set(PROF_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../")
set(REPO_DIR   "${CMAKE_CURRENT_LIST_DIR}/../../../../")

idf_component_register(
    SRCS
        # profiler library
        "${PROF_DIR}/profiler.c"
        # test harness entry point + unit tests
        "${TESTS_DIR}/main.c"
        "${TESTS_DIR}/test_lidar_driver.c"
        "${TESTS_DIR}/test_quadtree.c"
        "${TESTS_DIR}/test_lidar_to_map.c"
        "${TESTS_DIR}/test_frontier.c"
        "${TESTS_DIR}/test_hybrid_astar.c"
        "${TESTS_DIR}/test_uart_bridge.c"
        "${TESTS_DIR}/test_wifi_dashboard.c"
        # pipeline integration tests
        "${TESTS_DIR}/test_pipeline_scan_to_map.c"
        "${TESTS_DIR}/test_pipeline_frontier.c"
        "${TESTS_DIR}/test_pipeline_full_slam.c"
        "${TESTS_DIR}/test_pipeline_lidar_wifi.c"
        # real module implementations
        "${SRC_DIR}/quadtree_map.c"
        "${SRC_DIR}/quadtree_map_compat.c"
        "${SRC_DIR}/lidar_to_map.c"
        "${SRC_DIR}/frontier_detector.c"
        "${SRC_DIR}/hybrid_astar.c"
        "${SRC_DIR}/lidar_driver.c"
        "${SRC_DIR}/uart_bridge.c"
        "${SRC_DIR}/wifi_dashboard.c"
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
        driver
        nvs_flash
        esp_wifi
        esp_http_server
        esp_netif
        perfmon
)
