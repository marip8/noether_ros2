# Integration tests
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# Create a function for adding isolated launch tests
function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test(
    "${path}"
    RUNNER
    "${RUNNER}"
    ${ARGN}
  )
endfunction()

# Add a test for the TPP server
add_ros_isolated_launch_test(test_server.py)
install(FILES dome.ply DESTINATION share/${PROJECT_NAME}/test)
