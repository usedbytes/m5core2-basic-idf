idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "")

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLV_LVGL_H_INCLUDE_SIMPLE")
