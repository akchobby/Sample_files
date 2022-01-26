cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(my_sample LANGUAGES CXX)

# including headers directory
include_directories(${PROJECT_SOURCE_DIR}/include)

# adding library

## the library can be added using the object keyword, this is so that we can 
## generate the static and shared libs from the object files in one go
add_library(${PROJECT_NAME}_message_objs OBJECT message.cpp)

## very important line to ensure the  shared lib is built
set_target_properties(${PROJECT_NAME}_message_objs
			PROPERTIES
			POSITION_INDEPENDENT_CODE 1)

## static
## the target_objects is a generator expression, it retrives a list of
##all object files built for the my_sample_message_objs library.
add_library(${PROJECT_NAME}_message_static
       		STATIC
		$<TARGET_OBJECTS:${PROJECT_NAME}_message_objs>)

## shared
add_library(${PROJECT_NAME}_message_shared 
		SHARED
	       	$<TARGET_OBJECTS:${PROJECT_NAME}_message_objs>)



add_executable(${PROJECT_NAME}_test test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_message_static)
target_include_directories(${PROJECT_NAME}_test 
			PUBLIC
       		    	${PROJECT_SOURCE_DIR}/include)
