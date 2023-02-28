cmake_minimum_required(VERSION 3.20)


############### Names ###############
set(PROJECT_NAME mdp-transformer)


############### Preprocessor Macros ###############
add_compile_definitions(GIT_REPOSITORY_URL="https://github.com/Necktschnagge/mdp-tranformer")


############### Project Decklaration ###############
project(${PROJECT_NAME} VERSION 1.0)


############### Check Platform ###############
message("CMake is checking for CMAKE_SYSTEM_PROCESSOR containing arm to detect if running on Raspberry Pi:")
message("CMAKE_SYSTEM_PROCESSOR is: " ${CMAKE_SYSTEM_PROCESSOR})
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
	message("Running on Raspberry Pi.")
	set(on_target_platform true)
else(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
	message("Running on Development or Test Platform.")
	set(on_target_platform false)
endif()
if(UNIX AND NOT APPLE)
	set(LINUX true)
else()
	set(LINUX false)
endif()
if (WIN32)
	message("Running on WIN32.")
endif()
if (LINUX)
	message("Running on LINUX.")
endif()
if (APPLE)
	message("Running on APPLE.")
endif()
if (UNIX)
	message("Running on UNIX.")
endif()

if(WIN32) # Install dlls in the same directory as the executable on Windows # at least some of them
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
endif()

############### Language and Flags ###############
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS False)
if (WIN32)
	set(CMAKE_CXX_FLAGS_RELEASE "/MT /O2 /Ob2 /DNDEBUG")
	set(CMAKE_CXX_FLAGS_DEBUG "/MT /Zi /Ob0 /Od /RTC1")
endif()


############### Libraries ###############
include_directories(SYSTEM extern/spdlog/include)
include_directories(SYSTEM extern/json/include)
add_subdirectory(extern/googletest)

#Boost
if (MSVC)
    set(Boost_USE_STATIC_LIBS ON)
else()
	set(Boost_USE_STATIC_LIBS OFF)
endif (MSVC)
FIND_PACKAGE(Boost 1.54.0 COMPONENTS regex REQUIRED )
INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIR} )


############### Sources ###############
add_subdirectory(src)


############### Testing ###############
add_subdirectory(test)
enable_testing()

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -Werror)
endif()


############### Visual Studio ###############
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})#Set Visualo Studio start-up project, so that one can directly run the debugger.
