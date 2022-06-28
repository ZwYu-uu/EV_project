cmake_minimum_required(VERSION 3.22)
project(EV_project)

set(CMAKE_CXX_STANDARD 14)
set(CLP_CBC_INC_DIR /Users/ZwYu/dist/include/coin-or)
set(CLP_CBC_LINK_DIR /Users/ZwYu/dist/lib)
set(SOURCE_FILES main.cpp parameters.h SPFA.h SPFA.cpp Bus.cpp Bus.h)

include_directories(${CLP_CBC_INC_DIR})
link_directories(${CLP_CBC_LINK_DIR})

add_executable(EV_project ${SOURCE_FILES})

target_link_libraries(EV_project -lClp -lCoinUtils -lOsi -lOsiClp -lOsiCommonTest -lCbc -lCgl -lOsiCbc)

