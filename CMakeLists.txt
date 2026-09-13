cmake_minimum_required(VERSION 3.16)
project(aeolus_vcu C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Warnings are errors from day one. It is very cheap now and very expensive
# to retrofit once there are 4,000 lines. -Wconversion in particular catches
# the fixed-point truncation bugs that are otherwise found in a car park.
add_compile_options(
    -Wall -Wextra -Werror
    -Wconversion -Wsign-conversion
    -Wshadow -Wpointer-arith -Wcast-qual
    -Wstrict-prototypes -Wmissing-prototypes
)

option(VCU_SANITIZE "build tests with ASan/UBSan" ON)
if(VCU_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address,undefined)
endif()

# ---------------------------------------------------------------- the VCU
# Portable core. No HAL, no RTOS, no vendor headers. This library is what
# gets cross-compiled for the S32K344 later, unchanged.
add_library(vcu STATIC
    src/vcu.c
    src/app/state.c
    src/app/torque.c
    src/app/fault.c
    src/svc/apps.c
)
target_include_directories(vcu PUBLIC include)

# ------------------------------------------------------------ simulation
add_library(sim STATIC
    sim/plant.c
    sim/scenario.c
)
target_include_directories(sim PUBLIC . include)
target_link_libraries(sim PUBLIC vcu)

# ----------------------------------------------------------------- tests
add_executable(run_scenarios tests/run_scenarios.c)
target_link_libraries(run_scenarios PRIVATE sim)

enable_testing()
file(GLOB SCENARIOS ${CMAKE_CURRENT_SOURCE_DIR}/tests/scenarios/*.scn)
foreach(s ${SCENARIOS})
    get_filename_component(name ${s} NAME_WE)
    add_test(NAME scenario.${name} COMMAND run_scenarios ${s})
endforeach()

# --------------------------------------------------------- demo (POSIX)
add_executable(vcu_demo port/posix/main.c)
target_link_libraries(vcu_demo PRIVATE sim)
