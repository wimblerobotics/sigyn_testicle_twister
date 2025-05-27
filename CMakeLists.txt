# CMakeLists.txt for ament_python
cmake_minimum_required(VERSION 3.8)
project(sigyn_testicle_twister)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED) # Ensures python support is loaded

# Set the build type to Python
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
