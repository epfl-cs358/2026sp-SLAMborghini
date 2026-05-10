idf_component_register(
    SRCS
        "main.c"
        "test_odometry.c"
        # Real module implementation from the project
        "../../../wemos/src/encoder_ackermann_odometry.c"
    INCLUDE_DIRS
        "."
        "../../../"
        "../../../wemos/src"
    REQUIRES
        profiler
        freertos
        esp_common
        esp_timer
        heap
)
