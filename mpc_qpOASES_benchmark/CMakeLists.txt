#cmake最低版本需求，不加入此行会受到警告信息
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(hello) #项目名称

add_compile_options(-std=c++11)

#
#catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES src
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

##  build

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS} -lm")


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
link_libraries(glog)
link_libraries(gflags)
link_libraries(pthread)



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # include libraries for qpoases 
    qpOASES/include
    qpOASES/include/qpOASES
    qpOASES/include/qpOASES/extras
    #include libraries qp_solver
    qp_solver
)
#qpOASES
set(QPOASES_SRC
  qpOASES/src/BLASReplacement.cpp
  qpOASES/src/Bounds.cpp
  qpOASES/src/Constraints.cpp
  qpOASES/src/Flipper.cpp
  qpOASES/src/Indexlist.cpp
  qpOASES/src/LAPACKReplacement.cpp
  qpOASES/src/Matrices.cpp
  qpOASES/src/MessageHandling.cpp
  qpOASES/src/Options.cpp
  qpOASES/src/OQPinterface.cpp
  qpOASES/src/QProblem.cpp
  qpOASES/src/QProblemB.cpp
  qpOASES/src/SolutionAnalysis.cpp
  qpOASES/src/SparseSolver.cpp
  qpOASES/src/SQProblem.cpp
  qpOASES/src/SQProblemSchur.cpp
  qpOASES/src/SubjectTo.cpp
  qpOASES/src/Utils.cpp
  qpOASES/src/SolutionAnalysis.cpp    
)

add_library(qpoases ${QPOASES_SRC})

set(QPsolver_SRC
  qp_solver/active_set_qp_solver_test.cc
  qp_solver/active_set_qp_solver.cc
  qp_solver/qp_solver_gflags.cc
  qp_solver/qp_solver.cc
)
add_library(qpsolver ${QPsolver_SRC})
target_link_libraries(qpsolver PRIVATE qpoases)



find_package(Eigen3)
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIR})



add_executable(hello main.cpp)

target_link_libraries(hello glog)
target_link_libraries(hello gflags)
target_link_libraries(hello pthread)


target_link_libraries(hello ${catkin_LIBRARIES} qpoases qpsolver)
add_dependencies(hello ${catkin_EXPORTED_TARGETS} qpoases qpsolver)


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
