idf_component_register(
    SRCS        "profiler.c"
    INCLUDE_DIRS "."
    REQUIRES    esp_common freertos esp_hw_support heap perfmon
)
