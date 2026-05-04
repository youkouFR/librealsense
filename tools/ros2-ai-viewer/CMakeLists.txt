# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2024 RealSense, Inc. All Rights Reserved.
cmake_minimum_required(VERSION 3.10)

project(ros2-ai-viewer)

if(WIN32)
    add_executable(${PROJECT_NAME} WIN32 "")
    target_link_libraries(${PROJECT_NAME} OpenGL::GL)
else()
    add_executable(${PROJECT_NAME} "")
    target_link_libraries(${PROJECT_NAME} OpenGL::GL dl)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

using_easyloggingpp(${PROJECT_NAME} SHARED)

include(../../common/CMakeLists.txt)

if(BUILD_GRAPHICAL_EXAMPLES)
    set(ROS2_AI_VIEWER_CPP
        ${COMMON_SRC}
        ros2-ai-viewer.cpp
        ${IMGUI_SOURCES}
        ../../common/realsense-ui-advanced-mode.h
        ../../common/rendering.h
        ../../common/model-views.h
        ../../common/model-views.cpp
        ../../common/ux-window.h
        ../../common/ux-window.cpp
        ../../common/ux-alignment.cpp
        ../../common/ux-alignment.h
        ../../common/processing-block-model.h
        ../../common/post-processing-block-model.h
        ../../common/post-processing-filter.h
        ../../common/post-processing-worker-filter.h
        ../../common/post-processing-filters-list.h
        ../../common/objects-in-frame.h
        ../../third-party/glad/glad.c
        ../../third-party/tinyfiledialogs/tinyfiledialogs.c
        ../../third-party/tinyfiledialogs/tinyfiledialogs.h
        ../../common/opengl3.cpp
        ../../common/opengl3.h
        ../../common/rs-config.h
        ../../common/rs-config.cpp
        ../../common/os.h
        ../../common/os.cpp
        ../../common/graph-model.h
        ../../common/graph-model.cpp
        ../../third-party/rsutils/include/rsutils/type/eth-config.h
        ../../third-party/implot/implot.cpp
        ../../third-party/implot/implot.h
        ../../third-party/implot/implot_internal.h
        ../../third-party/implot/implot_items.cpp
        ../../common/ros2-bridge.h
        ../../common/ros2-bridge.cpp
        ../../common/ai-inference.h
        ../../common/ai-inference.cpp
        ../../common/detection-overlay.h
        ../../common/detection-overlay.cpp
    )

    if(WIN32)
        target_sources(${PROJECT_NAME}
            PRIVATE
                ${ROS2_AI_VIEWER_CPP}
            )
    else()
        target_sources(${PROJECT_NAME}
            PRIVATE
                ${ROS2_AI_VIEWER_CPP}
            )
    endif()

    include_directories(
        ../../common
        ../../third-party
        ../../third-party/tinyfiledialogs
        ${GLFW_SOURCE_DIR}/include
        ../../third-party/imgui
        ../../third-party/implot
        ../../third-party/glad
    )

    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

    set(ROS2_AI_VIEWER_LIBS ${GTK3_LIBRARIES} Threads::Threads realsense2-gl)

    # ROS2 dependencies
    set(ROS2_FOUND FALSE)

    if(DEFINED ROS2_ROOT AND IS_DIRECTORY ${ROS2_ROOT})
        message(STATUS "Using ROS2 from ROS2_ROOT: ${ROS2_ROOT}")
        set(ROS2_INSTALL_PREFIX ${ROS2_ROOT})
        set(ROS2_FOUND TRUE)
    elseif(DEFINED ENV{ROS_DISTRO})
        set(STANDARD_ROS2_PREFIX "/opt/ros/$ENV{ROS_DISTRO}")
        if(IS_DIRECTORY ${STANDARD_ROS2_PREFIX})
            message(STATUS "Found standard ROS2 at: ${STANDARD_ROS2_PREFIX}")
            set(ROS2_INSTALL_PREFIX ${STANDARD_ROS2_PREFIX})
            set(ROS2_FOUND TRUE)
        endif()
    endif()

    if(NOT ROS2_FOUND AND IS_DIRECTORY /opt/ros/humble)
        message(STATUS "Found ROS2 Humble at: /opt/ros/humble")
        set(ROS2_INSTALL_PREFIX "/opt/ros/humble")
        set(ROS2_FOUND TRUE)
    endif()

    if(ROS2_FOUND AND IS_DIRECTORY ${ROS2_INSTALL_PREFIX})
        if(IS_DIRECTORY ${ROS2_INSTALL_PREFIX}/include/rclcpp)
            message(STATUS "ROS2 rclcpp headers found")
            
            file(GLOB ROS2_INCLUDE_DIRS ${ROS2_INSTALL_PREFIX}/include/*)
            set(ROS2_INCLUDE_PATHS "")
            foreach(dir ${ROS2_INCLUDE_DIRS})
                if(IS_DIRECTORY ${dir})
                    list(APPEND ROS2_INCLUDE_PATHS ${dir})
                endif()
            endforeach()
            
            include_directories(${ROS2_INCLUDE_PATHS})

            # Link core ROS2 libraries (extracted from ldd of librclcpp.so)
            target_link_directories(${PROJECT_NAME} PRIVATE ${ROS2_INSTALL_PREFIX}/lib)
            
            set(ROS2_CORE_LIBS
                rclcpp
                rcl
                rcl_logging_spdlog
                rcl_logging_interface
                rcl_yaml_param_parser
                rcutils
                rcpputils
                rmw
                rmw_implementation
                rmw_fastrtps_cpp
                rosidl_runtime_c
                rosidl_typesupport_c
                rosidl_typesupport_cpp
                rosidl_typesupport_fastrtps_c
                rosidl_typesupport_fastrtps_cpp
                rosidl_typesupport_introspection_c
                rosidl_typesupport_introspection_cpp
                libstatistics_collector
                tracetools
                ament_index_cpp
                yaml
                fastcdr
                fastrtps
                spdlog
                fmt
                # Message type support libraries
                builtin_interfaces__rosidl_typesupport_cpp
                builtin_interfaces__rosidl_typesupport_c
                builtin_interfaces__rosidl_typesupport_introspection_cpp
                builtin_interfaces__rosidl_typesupport_introspection_c
                builtin_interfaces__rosidl_generator_c
                statistics_msgs__rosidl_typesupport_cpp
                statistics_msgs__rosidl_typesupport_c
                statistics_msgs__rosidl_typesupport_introspection_cpp
                statistics_msgs__rosidl_typesupport_introspection_c
                std_msgs__rosidl_typesupport_cpp
                std_msgs__rosidl_typesupport_c
                std_msgs__rosidl_typesupport_introspection_cpp
                std_msgs__rosidl_typesupport_introspection_c
                std_msgs__rosidl_generator_c
                sensor_msgs__rosidl_typesupport_cpp
                sensor_msgs__rosidl_typesupport_c
                sensor_msgs__rosidl_typesupport_introspection_cpp
                sensor_msgs__rosidl_typesupport_introspection_c
                sensor_msgs__rosidl_generator_c
                rcl_interfaces__rosidl_typesupport_cpp
                rcl_interfaces__rosidl_typesupport_c
                rcl_interfaces__rosidl_generator_c
                rosgraph_msgs__rosidl_typesupport_cpp
                message_filters
            )

            find_library(CV_BRIDGE_LIB cv_bridge PATHS ${ROS2_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
            if(CV_BRIDGE_LIB)
                list(APPEND ROS2_CORE_LIBS cv_bridge image_transport)
            endif()
            
            target_link_libraries(${PROJECT_NAME} ${ROS2_CORE_LIBS})
            target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_ROS2)
            message(STATUS "ROS2 support enabled")
        else()
            message(WARNING "ROS2 rclcpp headers not found. Building without ROS2 support.")
        endif()
    else()
        message(WARNING "ROS2 not found. Building without ROS2 support.")
    endif()

    # OpenCV
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

    # ONNX Runtime
    if(DEFINED ONNXRUNTIME_ROOT AND IS_DIRECTORY ${ONNXRUNTIME_ROOT})
        message(STATUS "Enabling ONNX Runtime AI inference from: ${ONNXRUNTIME_ROOT}")
        include_directories(${ONNXRUNTIME_ROOT}/include)
        link_directories(${ONNXRUNTIME_ROOT}/lib)
        target_link_libraries(${PROJECT_NAME} onnxruntime)
        target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_ONNXRUNTIME)
    else()
        find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
            PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime
            PATH_SUFFIXES onnxruntime
        )
        find_library(ONNXRUNTIME_LIBRARY onnxruntime
            PATHS /usr/lib /usr/local/lib
        )
        if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
            message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIBRARY}")
            include_directories(${ONNXRUNTIME_INCLUDE_DIR})
            target_link_libraries(${PROJECT_NAME} ${ONNXRUNTIME_LIBRARY})
            target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_ONNXRUNTIME)
        else()
            message(WARNING "ONNX Runtime not found. AI inference will be disabled.")
        endif()
    endif()

    if(CHECK_FOR_UPDATES)
        message(STATUS "Check for updates capability added to ros2-ai-viewer")
        add_dependencies(${PROJECT_NAME} libcurl)
        set(ROS2_AI_VIEWER_LIBS ${ROS2_AI_VIEWER_LIBS} curl)
    endif()

    target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES} ${ROS2_AI_VIEWER_LIBS} tclap)

    set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Tools)
    target_include_directories(${PROJECT_NAME} PRIVATE ../../src)

    install(
        TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
