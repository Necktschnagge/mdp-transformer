file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)


############### Declaration of all targets: ###############

add_executable(${PROJECT_NAME} ${SOURCES}) # the executable binary

add_library(${PROJECT_NAME}_lib STATIC ${SOURCES}) # compiled as static library (e.g. for using in tests)


############### Libraries for target ${PROJECT_NAME} ###############

if(LINUX)
	target_link_libraries(${PROJECT_NAME} PUBLIC
		pthread
	)
endif()

#message("Custom Message:   " ${Boost_LIBRARIES})
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES} tbb)
	message("ON GCC")
else()
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
	message("NOT ON GCC")
endif()
#TARGET_LINK_LIBRARIES(${PROJECT_NAME}_lib LINK_PUBLIC ${Boost_LIBRARIES} )


############### Libraries for target ${PROJECT_NAME}_lib ###############

