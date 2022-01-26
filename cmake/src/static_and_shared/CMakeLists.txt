cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(my_sample LANGUAGES CXX)


include_directories(${PROJECT_SOURCE_DIR}/include)

# adding library

add_library(${PROJECT_NAME}_message STATIC message.cpp)




add_executable(${PROJECT_NAME}_test test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_message)
target_include_directories(${PROJECT_NAME}_test 
			PUBLIC
       		    	${PROJECT_SOURCE_DIR}/include)
