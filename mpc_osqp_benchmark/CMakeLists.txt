#cmake最低版本需求，不加入此行会受到警告信息
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(hello) #项目名称

#add_compile_options(-std=c++11)

#
#catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES src
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

##  build
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS} -lm")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS} -lm")
#set(CMAKE_C_FLAGS "-ldl") 
### Include real time library in linux
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lrt -ldl")
#SET(CMAKE_C_FLAGS_RELEASE "$ENV{CFLAGS} -O2 -Wall -std=c99 -lm")


##glog
#set(GLOG_LIB glog)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIB})

##gflags
#set(GFLAGS_LIB gflags)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${GFLAGS_LIB})

SET(INC_DIR /usr/local/include)
SET(LINK_DIR /usr/local/lib)

include_directories(${INC_DIR})
link_directories(${LINK_DIR})
# link_libraries(glog)
#link_libraries(gflags)
link_libraries(pthread)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # include libraries for osqp
    osqp/include 
    osqp/include/qdldl
    osqp/include/qdldl/include 
    osqp/include/pardiso
    #include libraries qp_solver
    mpc_osqp
)

set (OSQP_SRC
  osqp/src/auxil.c
  osqp/src/cs.c
  osqp/src/ctrlc.c
  osqp/src/kkt.c
  osqp/src/lib_handler.c
  osqp/src/lin_alg.c
  osqp/src/lin_sys.c
  osqp/src/osqp.c
  osqp/src/polish.c
  osqp/src/proj.c
  osqp/src/scaling.c
  osqp/src/util.c
  # qdldl 
  osqp/src/qdldl/qdldl_interface.c
  osqp/src/qdldl/qdldl.c
  # qdldl amd 
  osqp/src/qdldl/amd/src/amd_1.c
  osqp/src/qdldl/amd/src/amd_2.c
  osqp/src/qdldl/amd/src/amd_aat.c
  osqp/src/qdldl/amd/src/amd_control.c
  osqp/src/qdldl/amd/src/amd_defaults.c
  osqp/src/qdldl/amd/src/amd_info.c
  osqp/src/qdldl/amd/src/amd_order.c
  osqp/src/qdldl/amd/src/amd_postorder.c
  osqp/src/qdldl/amd/src/amd_post_tree.c
  osqp/src/qdldl/amd/src/amd_preprocess.c
  osqp/src/qdldl/amd/src/amd_valid.c
  osqp/src/qdldl/amd/src/SuiteSparse_config.c
  # pardiso
  osqp/src/pardiso/pardiso_interface.c
  osqp/src/pardiso/pardiso_loader.c
  )
add_library(osqp ${OSQP_SRC})
target_link_libraries(osqp dl)

set(mpc_osqp_SRC
  mpc_osqp/mpc_osqp.cpp
)
add_library(mpcosqp ${mpc_osqp_SRC})
target_link_libraries(mpcosqp PRIVATE osqp)



find_package(Eigen3)
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIR})



add_executable(hello main.cpp)

#target_link_libraries(hello dl)
#TARGET_LINK_LIBRARIES(dl)
#TARGET_LINK_LIBRARIES(m)

# target_link_libraries(hello glog)
# target_link_libraries(hello gflags)
target_link_libraries(hello pthread)


target_link_libraries(hello ${catkin_LIBRARIES} osqp mpcosqp)
add_dependencies(hello ${catkin_EXPORTED_TARGETS} osqp mpcosqp)


#include_directories (qp_solver)

#aux_source_directory (qp_solver SRC_LIST)
##aux_source_directory (qpOASES SRC_LIST1)
#aux_source_directory (qpOASEShpp SRC_LIST2)

#add_executable (hello main.cpp ${SRC_LIST})

#target_link_libraries(hello ${catkin_INCLUDE_DIRS} qpoases)
#add_dependencies(hello ${catkin_EXPORTED_TARGETS} qpoases)


##target_link_libraries(hello libqpOASES.so)
#
#set(GFLAGS_LIB gflags)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${GFLAGS_LIB})
#
#set(GLOG_LIB glog)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIB})


#把当前目录(.)下所有源代码文件和头文件加入变量SRC_LIST
#AUX_SOURCE_DIRECTORY(. SRC_LIST)
#生成应用程序 hello (在windows下会自动生成hello.exe)
#ADD_EXECUTABLE(hello ${SRC_LIST})
