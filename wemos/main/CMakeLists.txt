# wemos/main/CMakeLists.txt — main ESP-IDF component
#
# Lists exactly the .c files to compile.  This replaces build_src_filter,
# which is not supported by the ESP-IDF framework in PlatformIO.
#
# Paths are relative to this file (wemos/main/).

set(REPO_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}/../..")   # 2026sp-SLAMborghini/
set(WEMOS_ROOT   "${CMAKE_CURRENT_SOURCE_DIR}/..")      # wemos/
set(S3_SRC       "${REPO_ROOT}/esp32s3/src")            # esp32s3/src/

idf_component_register(
    SRCS
        "${WEMOS_ROOT}/main.c"
        "${WEMOS_ROOT}/src/uart_bridge.c"
        "${WEMOS_ROOT}/imu_gyro.c"
        "${S3_SRC}/quadtree_map.c"
        "${S3_SRC}/quadtree_map_compat.c"
        "${S3_SRC}/frontier_detector.c"
        "${S3_SRC}/command_gen.c"
        "${S3_SRC}/wifi_dashboard.c"
        "${S3_SRC}/lidar_driver.c"
        "${S3_SRC}/lidar_to_map.c"
    INCLUDE_DIRS
        "${WEMOS_ROOT}/src"
        "${WEMOS_ROOT}"
        "${S3_SRC}"
        "${REPO_ROOT}"
)
