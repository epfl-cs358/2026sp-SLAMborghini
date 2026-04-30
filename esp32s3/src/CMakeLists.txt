# esp32s3/src/CMakeLists.txt

set(app_sources
    ../main.c
    command_gen.c
    frontier_detector.c
    hybrid_astar.c
    lidar_driver.c
    lidar_to_map.c
    map_consistency.c
    map_updater.c
    obstacle_classifier.c
    path_refiner.c
    polar_to_cart.c
    quadtree_map.c
    quadtree_map_compat.c
    rbpf.c
    uart_bridge.c
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        "."
        ".."
        "../.."
)