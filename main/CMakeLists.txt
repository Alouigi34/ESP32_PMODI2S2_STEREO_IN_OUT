idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2s
)

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
    -std=gnu++17
)