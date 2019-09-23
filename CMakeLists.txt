project(codal-nrf52)

include("${CODAL_UTILS_LOCATION}")

# find sources and headers
RECURSIVE_FIND_DIR(INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/inc" "*.h")
RECURSIVE_FIND_FILE(SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/source" "*.c??")

list(APPEND SOURCE_FILES "source/reloc_vector_table.c")
list(APPEND SOURCE_FILES "source/nrf_delay.c")
list(APPEND SOURCE_FILES "source/system_nrf52.c")
list(APPEND SOURCE_FILES "asm/CortexContextSwitch.s")
list(APPEND SOURCE_FILES "asm/startup_NRF52832.s")

# add them
include_directories(${INCLUDE_DIRS})

# create our target
add_library(codal-nrf52 ${SOURCE_FILES})

target_link_libraries(
    codal-nrf52
    codal-core
)

# expose it to parent cmake.
target_include_directories(codal-nrf52 PUBLIC ${INCLUDE_DIRS})
