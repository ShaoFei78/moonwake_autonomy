cmake_minimum_required(VERSION 3.8)
project(moonwake_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# ⚠️ 2. 查找消息生成器
find_package(rosidl_default_generators REQUIRED)

# 3. 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 4. 添加你的 C++ 节点
#    buoy_detector_node.cpp  节点主流程（同步/检查/发布）
#    failure_recorder.cpp    异常样本保存模块
#    detector_factory.cpp    后端工厂（hsv / yolo）
#    hsv_detector.cpp        原来的 OpenCV HSV 方案
#    yolo_detector.cpp       ONNX Runtime 的 YOLO 方案
add_executable(buoy_detector_node
  src/buoy_detector_node.cpp
  src/failure_recorder.cpp
  src/detector_factory.cpp
  src/hsv_detector.cpp
  src/yolo_detector.cpp
)

# 4.1 让 #include "moonwake_camera/xxx.hpp" 能找到；用到 std::filesystem，需要 C++17
target_include_directories(buoy_detector_node PRIVATE include)
target_compile_features(buoy_detector_node PRIVATE cxx_std_17)

# 4.2 可选依赖：ONNX Runtime（YOLO 后端用）
#     找不到也不报错 —— HSV 后端照样能用，YOLO 会在运行时返回 STATUS_MODEL_ERROR 并说明原因
set(ONNXRUNTIME_DIR "" CACHE PATH "ONNX Runtime 根目录（含 include/ 和 lib/）")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS
    "${ONNXRUNTIME_DIR}/include"
    "$ENV{ONNXRUNTIME_DIR}/include"
    "$ENV{HOME}/third_party/onnxruntime/include"
    /opt/onnxruntime/include
)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS
    "${ONNXRUNTIME_DIR}/lib"
    "$ENV{ONNXRUNTIME_DIR}/lib"
    "$ENV{HOME}/third_party/onnxruntime/lib"
    /opt/onnxruntime/lib
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  message(STATUS "找到 ONNX Runtime: ${ONNXRUNTIME_LIBRARY}  → 启用 YOLO 后端")
  target_compile_definitions(buoy_detector_node PRIVATE HAVE_ONNXRUNTIME)
  target_include_directories(buoy_detector_node SYSTEM PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(buoy_detector_node ${ONNXRUNTIME_LIBRARY})
  # 运行时也得能找到 libonnxruntime.so（不然启动时报 cannot open shared object file）
  get_filename_component(_ort_lib_dir "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
  set_target_properties(buoy_detector_node PROPERTIES
    BUILD_RPATH "${_ort_lib_dir}"
    INSTALL_RPATH "${_ort_lib_dir}"
  )
else()
  message(STATUS "未找到 ONNX Runtime → YOLO 后端不可用（HSV 不受影响）")
  message(STATUS "  安装后在构建时指定：colcon build --cmake-args -DONNXRUNTIME_DIR=/path/to/onnxruntime")
endif()

# 5. 链接常规依赖
ament_target_dependencies(buoy_detector_node rclcpp std_msgs cv_bridge sensor_msgs message_filters)

# 5.1 YOLO 后端用到 OpenCV 的 dnn 模块（blobFromImage / NMSBoxes），
#     cv_bridge 只带 core/imgproc，dnn 要显式链接
find_package(OpenCV QUIET COMPONENTS dnn)
if(OpenCV_FOUND)
  target_include_directories(buoy_detector_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(buoy_detector_node ${OpenCV_LIBS})
  message(STATUS "OpenCV dnn 已链接: ${OpenCV_LIBS}")
else()
  message(WARNING "没找到 OpenCV dnn 模块，YOLO 后端会链接失败")
endif()

# ⚠️ 6. 【关键步骤】将节点链接到本包生成的消息类型支持上
# 如果不加这一行，你的 C++ 代码将无法 #include 生成的消息头文件，或者链接时报错！
rosidl_target_interfaces(buoy_detector_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 7. 安装节点
install(TARGETS
  buoy_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 7.1 安装 Python 工具脚本，这样可以直接 ros2 run moonwake_camera evaluate_detector.py
install(PROGRAMS
  tools/evaluate_detector.py
  tools/tune_params.py
  tools/video_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 7.2 安装参数文件，方便用 --params-file 加载
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 7.3 安装 launch 文件，这样 ros2 launch moonwake_camera xxx.launch.py 才能找到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()