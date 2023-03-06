# 参考博客 https://blog.csdn.net/weixin_45590473/article/details/122608048?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166173420516781647577315%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=166173420516781647577315&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~pc_rank_34-1-122608048-null-null.142^v42^pc_rank_34_1,185^v2^control&utm_term=ROS%E5%B7%A5%E4%BD%9C%E7%A9%BA%E9%97%B4CMakeList.txt&spm=1018.2226.3001.4187

cmake_minimum_required(VERSION 2.8.3)
project(auto_landing)


add_compile_options(-std=c++11)

################################################################################
## 1. Find catkin macros and libraries

# 指明构建这个pkg所需功能包pkg 比如 catkin 
# 产生CMake变量：
#   <pkg_name>_FOUND
#   <pkg_name>_INCLUDE_DIRS 该功能包的头文件目录
#   <pkg_name>_LIBRARIES 该文件包的库文件目录
# ps. 这种写法把所有pkg(catkin,message_generation,...,tf的内容都加载到<catkin>_xxx里面)
#     之后的find_package同理
# catkin_XXX 作为CMAKE变量为几乎每个程序编译所需要（构建库文件所需功能包），
# 而之后的比如 Boost_XXX, OpenCV_XXX 则是在编译特定文件时用的到（依赖项）
# 故分成多个find_pkg

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
    actionlib
    tf2_eigen
    tf
    prometheus_msgs
)

# 指明系统依赖项,使用boost之类的库
find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)
# 使用OpenCV库
find_package(OpenCV REQUIRED)

################################################################################

## 2. 自定义消息文件声明与处理

# 三个宏：add_message_files， add_service_files，add_action_files来相应的控制.msg .srv .action
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


# 总之当我们调用如上add_action_files、add_service_files、add_message_files之后
# FILES标识符就代表了我们自定义的所有消息文件（包括：.srv、.msg、.action）
# add_message_files， add_service_files，add_action_files来相应的控制.msg .srv .action。
# 这些宏后面必须跟着一个调用generate_messages() 用来处理使用三个宏指明的消息文件使之生成符合C++标准（或者其他编程语言标准）的头文件和源文件。

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


################################################################################
## 3. 指明编译构建C++源文件所需的功能包

catkin_package(  
  INCLUDE_DIRS include  
#   LIBRARIES xxx
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#   DEPENDS Boost  
)  

# INCLUDE_DIRS：表明我们这个功能包的.h文件
# LIBRARIES：指明需要依赖该功能包的其他功能包
# CATKIN_DEPENDS：在catkin编译系统中，编译本功能包所需的catkin官方的依赖功能包；
# DEPENDS：在编译这些源文件时，所需要的非catkin官方的依赖功能包；


################################################################################
## 4. 头文件声明函数
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  src/lib #相对路径
  include
  ${catkin_INCLUDE_DIRS} # catkin_INCLUDE_DIRS目录下存储着着由catkin功能包编译而成的.h文件，因此以“catkin功能包名称+_INCLUDE_DIRS”组成。
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

################################################################################
## 5. Nodes
## 5-1. 声明C++库 add_library
# 自己写的库文件(自定义数据类型的声明 + 定义 + 实例化/使用)
# 当库的定义和调用分开时，我们才使用add_library{...}去告知catkin编译器（ROS使用的编译器）：有一个自定义数据类型库文件是由XXX.h和XXX.cpp文件构成的，
# add_library(xxx 
#     include/xxx.src/yyy.h
#     src/yyy.cpp
# )
## 5-2 指明需要编译的可执行文件名
add_executable(create_marker 
    src/create_marker.cpp
)
## 5-3 一般指定链接规则之前，一定要用add_dependancies设置依赖项   
# 下面这句话的意思是： // exec_name 所表示的源文件链接的前提是catkin, msg/srv/act已经编译完毕
# add_dependencies(exec_name ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})    ps. project name 就是cpp文件名

    # ${${PROJECT_NAME}_gencpp / _generate_messages_cpp}：如果在编译包或者执行文件时，需要用到msg和srv，
    # 就要显示调用由 message_generation 自动生成的用于编译消息文件的依赖项  
    # ${catkin_EXPORTED_TARGETS}：用于为add_excutable()中映射的可执行文件提供catkin官方依赖包。

## 5-4 指定要链接库或可执行目标的库
target_link_libraries(create_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


add_executable(test_for_marker src/test_for_marker.cpp)
target_link_libraries(test_for_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(test_a src/test.cpp)
add_dependencies(test_a ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_a ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(landpad_det src/landpad_det.cpp)
add_dependencies(landpad_det ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(landpad_det  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

# add_executable(mission src/mission.cpp)
# add_dependencies(mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mission ${catkin_LIBRARIES})

add_executable(my_mission src/my_mission.cpp)
add_dependencies(my_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_mission ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(my_ocean_mission src/my_ocean_mission.cpp)
add_dependencies(my_ocean_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_ocean_mission ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)

##px4_pos_estimator.cpp
add_executable(px4_pos_estimator src/px4/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator prometheus_control_gencpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)
