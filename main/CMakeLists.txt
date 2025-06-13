idf_component_register(SRCS "master.c"
                       INCLUDE_DIRS "."
                       REQUIRES espressif__esp-modbus mb_example_common)