cmake_minimum_required(VERSION 3.1.0)
project(pulse-cntr
	VERSION "1.0.0"
	DESCRIPTION "T-platforms MRBT1 BMC pulse counter tools"
	LANGUAGES "C")

# Common build cleanups
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(DEFAULT_TTY "/dev/ttyS1" CACHE STRING "Default tty device")

# List of pulse-cntr utility source files
configure_file("config.h.in" "config.h")
file(GLOB PULSECNTR_SOURCES "src/*.c")
file(GLOB PULSECNTR_PRIVATE_HEADERS "src/*.h")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Build pulse-cntr binary
add_executable(pulse-cntr ${PULSECNTR_SOURCES} ${PULSECNTR_PRIVATE_HEADERS})

# pulse-cntr installation rules
install(TARGETS pulse-cntr DESTINATION bin)
