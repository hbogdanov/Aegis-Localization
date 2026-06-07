cmake_minimum_required(VERSION 3.16)
project(aegis_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aegis_core
  src/motion_model.cpp
  src/ekf.cpp
)

target_include_directories(aegis_core PUBLIC
  include
)

target_link_libraries(aegis_core
  Eigen3::Eigen
)

ament_target_dependencies(aegis_core Eigen3)

install(TARGETS aegis_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  ament_add_gtest(test_aegis_core
    test/test_ekf.cpp
  )
  if(TARGET test_aegis_core)
    target_link_libraries(test_aegis_core aegis_core)
    ament_target_dependencies(test_aegis_core Eigen3)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(aegis_core)
ament_package()
