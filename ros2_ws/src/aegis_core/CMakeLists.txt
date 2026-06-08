cmake_minimum_required(VERSION 3.16)
project(aegis_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(aegis_core
  src/motion_model.cpp
  src/ekf.cpp
  src/ukf.cpp
  src/particle_filter.cpp
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
  ament_add_gtest(test_ekf
    test/test_ekf.cpp
  )
  if(TARGET test_ekf)
    target_link_libraries(test_ekf aegis_core)
    ament_target_dependencies(test_ekf Eigen3)
  endif()

  ament_add_gtest(test_ukf
    test/test_ukf.cpp
  )
  if(TARGET test_ukf)
    target_link_libraries(test_ukf aegis_core)
    ament_target_dependencies(test_ukf Eigen3)
  endif()

  ament_add_gtest(test_particle_filter
    test/test_particle_filter.cpp
  )
  if(TARGET test_particle_filter)
    target_link_libraries(test_particle_filter aegis_core)
    ament_target_dependencies(test_particle_filter Eigen3)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(aegis_core)
ament_package()
