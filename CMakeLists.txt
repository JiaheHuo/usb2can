cmake_minimum_required(VERSION 3.16)
project(robstride_usb2can_ctrl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Torch hint (zip release extracted to /opt/libtorch) ----
if(NOT DEFINED Torch_DIR AND EXISTS "/opt/libtorch/share/cmake/Torch")
  set(Torch_DIR "/opt/libtorch/share/cmake/Torch" CACHE PATH "Path to TorchConfig.cmake")
endif()

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Torch REQUIRED)

option(RS_DO_MOTOR_INIT "Run motor init sequence" ON)
option(RS_ENABLE_TELEMETRY "Enable telemetry printing" ON)

set(EIGEN3_INCLUDE_DIR
    "/home/jhuo/model_based_control/cyborg-test-static-arm/src/src/azureloong_control/third_party/eigen3"
    CACHE PATH "Eigen3 include dir"
)

add_compile_definitions(
  RS_DO_MOTOR_INIT=$<IF:$<BOOL:${RS_DO_MOTOR_INIT}>,1,0>
  RS_ENABLE_TELEMETRY=$<IF:$<BOOL:${RS_ENABLE_TELEMETRY}>,1,0>
)

add_executable(motor_test
  src/main.cpp
  src/config.cpp
  src/router.cpp
  src/motor.cpp
  src/stats.cpp
  src/ankle_kinematics.cpp
  src/witmotion_imu.cpp
  src/control_modules.cpp
  src/control_utility.cpp
)

target_include_directories(motor_test PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can
  ${EIGEN3_INCLUDE_DIR}
  /opt/libtorch/include
  /opt/libtorch/include/torch/csrc/api/include
)

# Torch flags (TORCH_CXX_FLAGS is a string; split to list)
separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
target_compile_options(motor_test PRIVATE ${TORCH_CXX_FLAGS_LIST})

target_link_libraries(motor_test PRIVATE
  yaml-cpp
  Threads::Threads
  ${TORCH_LIBRARIES}
)

# ---- Tangair USB2CAN SDK: source or prebuilt lib ----
if (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/usb_can.c")
  target_sources(motor_test PRIVATE third_party/tangair_usb2can/usb_can.c)
elseif (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/usb_can.cpp")
  target_sources(motor_test PRIVATE third_party/tangair_usb2can/usb_can.cpp)
endif()

if (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.a")
  target_link_libraries(motor_test PRIVATE ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.a)
elseif (EXISTS "${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.so")
  target_link_libraries(motor_test PRIVATE ${PROJECT_SOURCE_DIR}/third_party/tangair_usb2can/libusb_can.so)
endif()

target_compile_options(motor_test PRIVATE -O2 -Wall -Wextra)
