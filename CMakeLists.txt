cmake_minimum_required(VERSION 3.16)
project(hailo_detector)

# ── C++ standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 / ament dependencies ────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(vision_msgs   REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(cv_bridge     REQUIRED)
find_package(OpenCV        REQUIRED)

# ── System / hardware dependencies ────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# ── detector_node (C++) ───────────────────────────────────────────────────────
add_executable(detector_node src/detector_node.cpp)

target_compile_options(detector_node PRIVATE
    -O2 -Wall -Wextra
    ${LIBCAMERA_CFLAGS}
)

target_include_directories(detector_node PRIVATE
    include
    /usr/local/include/rpicam-apps
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(detector_node PRIVATE
    /usr/local/lib/aarch64-linux-gnu
)

target_link_libraries(detector_node
    rpicam_app
    ${LIBCAMERA_LIBRARIES}
    boost_program_options
    ${OpenCV_LIBS}
)

ament_target_dependencies(detector_node
    rclcpp
    std_msgs
    vision_msgs
    sensor_msgs
    cv_bridge
)

# ── Install C++ executable ────────────────────────────────────────────────────
install(TARGETS detector_node
    DESTINATION lib/${PROJECT_NAME}
)

# ── Install Python GUI node ───────────────────────────────────────────────────
install(PROGRAMS
    scripts/detection_gui_node
    DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch files and config ──────────────────────────────────────────
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
