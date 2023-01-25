project(ULConductor)

cmake_minimum_required(VERSION 3.24)

if (WIN32)
	set(ULTRALEAP_PATH_ROOT "$ENV{ProgramFiles}/Ultraleap")
elseif (UNIX)
	set(ULTRALEAP_PATH_ROOT "/usr/share/doc/ultraleap-hand-tracking-service/lib/cmake/LeapSDK")
endif()

# WinMM
# if(RTMIDI_API_WINMM)
#   list(APPEND API_DEFS "-D__WINDOWS_MM__")
#   list(APPEND API_LIST "winmm")
#   list(APPEND LINKLIBS winmm)
# endif()

find_package(LeapSDK
	5
	REQUIRED
	PATHS
		"${ULTRALEAP_PATH_ROOT}")

set(SRCS "src/ULConductor.cpp")
set(RTMIDI "external/rtmidi")
add_subdirectory(${RTMIDI})
# add_library(rtmidi STATIC
# ${RTMIDI})
set_target_properties(rtmidi PROPERTIES LINKER_LANGUAGE CXX)


add_executable(ULConductor
			   ${SRCS})
target_include_directories(ULConductor
	PRIVATE
	${RTMIDI})

target_link_libraries(ULConductor
	PRIVATE
	rtmidi
	LeapSDK::LeapC)

# add_custom_command(TARGET ULConductor POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
# 	"${PROJECT_SOURCE_DIR}/external/rtmidi/build/Debug/rtmidi.dll}"
#  	$<TARGET_FILE_DIR:ULConductor>)