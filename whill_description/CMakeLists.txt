cmake_minimum_required(VERSION 3.8)
project(whill_description)

find_package(ament_cmake REQUIRED)

# install urdf & launch so FindPackageShare can find them
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
