# Add executable. Default name is the project name, version 0.1

add_library(lcd_library STATIC lcd.c)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(Smart-Toaster Smart-Toaster.c)

pico_set_program_name(Smart-Toaster "Smart-Toaster")
pico_set_program_version(Smart-Toaster "0.1")

# no_flash means the target is to run from RAM
# pico_set_binary_type(Smart-Toaster no_flash)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Smart-Toaster 0)
pico_enable_stdio_usb(Smart-Toaster 1)

# Add the standard library to the build
target_link_libraries(Smart-Toaster
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Smart-Toaster PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Smart-Toaster 
        hardware_spi
        hardware_i2c
        lcd_library
        )

pico_add_extra_outputs(Smart-Toaster)

