set(srcs "es8311.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES "MYIIC")
