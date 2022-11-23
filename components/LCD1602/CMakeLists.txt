idf_component_register(SRCS "LCD1602.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "esp32-smbus" "sys_resource" "esp32-i2c-lcd1602" )
