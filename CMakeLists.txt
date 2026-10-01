cmake_minimum_required(VERSION 3.16)
project(aeolus_vcu C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# -Wconversion is the important one: it's what catches fixed-point values
# silently truncating when they get assigned to a narrower type.
set(VCU_WARNINGS
    -Wall -Wextra -Werror
    -Wconversion -Wsign-conversion
    -Wshadow -Wpointer-arith -Wcast-qual
    -Wstrict-prototypes -Wmissing-prototypes
)
add_compile_options(${VCU_WARNINGS})

option(VCU_SANITIZE "build with ASan/UBSan" ON)
if(VCU_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address,undefined)
endif()

# core: the controller. No hardware, no RTOS, no vendor headers, so the
# same sources build here and for the S32K344.
set(CORE_SOURCES
    core/vcu.c
    core/config.c
    core/datalog.c
    core/sense/pedals.c
    core/sense/signals.c
    core/decide/faults.c
    core/decide/state.c
    core/decide/torque.c
)
add_library(vcu STATIC ${CORE_SOURCES})
target_include_directories(vcu PUBLIC .)   # includes are written from the repo root

# sim: plant model and scenario runner, host only.
add_library(sim STATIC sim/plant.c sim/scenario.c)
target_link_libraries(sim PUBLIC vcu)

add_executable(run_scenarios tests/run_scenarios.c)
target_link_libraries(run_scenarios PRIVATE sim)

add_executable(unit_tests tests/unit_tests.c)
target_link_libraries(unit_tests PRIVATE vcu)

add_executable(vcu_demo board/host/main.c)
target_link_libraries(vcu_demo PRIVATE sim)

# ctest runs everything: each scenario, the unit tests, and (if the ARM
# compiler is installed) a target build of every core file.
enable_testing()

add_test(NAME unit COMMAND unit_tests)

file(GLOB SCENARIOS ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenarios/*.scn)
foreach(s ${SCENARIOS})
    get_filename_component(name ${s} NAME_WE)
    add_test(NAME scenario.${name} COMMAND run_scenarios ${s})
endforeach()

find_program(ARM_GCC arm-none-eabi-gcc)
if(ARM_GCC)
    foreach(src ${CORE_SOURCES})
        get_filename_component(name ${src} NAME_WE)
        add_test(NAME arm.${name}
                 COMMAND ${ARM_GCC} -std=c11 -mcpu=cortex-m7 -mthumb -Os
                         -I${CMAKE_CURRENT_SOURCE_DIR} ${VCU_WARNINGS}
                         -c ${CMAKE_CURRENT_SOURCE_DIR}/${src} -o /dev/null)
    endforeach()
else()
    message(STATUS "arm-none-eabi-gcc not found, skipping the target build check")
endif()
