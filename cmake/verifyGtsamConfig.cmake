#####################################################
# Verify if gtsam was built with the right options

#! verify_gtsam_config: Warn the user if GTSAM has undesired settings
#
# Use try_compile and two code snippets to detect whether or not
# GTSAM_ROT3_EXPMAP and GTSAM_TANGENT_PREINTEGRATION were set when
# building GTSAM. If they were, warn the user that their results
# using Kimera may not match our published results.
#
function(verify_gtsam_config)
    try_compile(GTSAM_ROTATION_SETTING_OKAY
        ${CMAKE_BINARY_DIR}
        "${CMAKE_SOURCE_DIR}/cmake/tests/gtsam_uses_expmap.cpp"
        CMAKE_FLAGS
            "-DINCLUDE_DIRECTORIES=${GTSAM_INCLUDE_DIR}")

    try_compile(GTSAM_INTEGRATION_SETTING_OKAY
        ${CMAKE_BINARY_DIR}
        "${CMAKE_SOURCE_DIR}/cmake/tests/gtsam_uses_manifold_preintegration.cpp"
        CMAKE_FLAGS
            "-DINCLUDE_DIRECTORIES=${GTSAM_INCLUDE_DIR}")

    if(NOT ${GTSAM_ROTATION_SETTING_OKAY})
        # we don't provide specific commands here because the default option is correct in gtsam
        message(WARNING "Detected that the GTSAM version to be used is not configured to use Rot3::Expmap for retractions. You should build GTSAM with GTSAM_ROT3_EXPMAP=ON and GTSAM_USE_QUATERNIONS=OFF.")
    endif()

    if(NOT ${GTSAM_INTEGRATION_SETTING_OKAY})
        # the default option is not what we want in gtsam, so we provide instructions
        message(WARNING "Detected that the GTSAM version to be used is not configured to use on-manifold IMU preintegration. You should build GTSAM with GTSAM_TANGENT_PREINTEGRATION=OFF. You can do this when using catkin by cleaning gtsam and running 'catkin config -a --cmake-args -DGTSAM_TANGENT_PREINTEGRATION=OFF' and then rebuilding")
    endif()

    if(NOT (${GTSAM_ROTATION_SETTING_OKAY} AND ${GTSAM_INTEGRATION_SETTING_OKAY}))
        message(WARNING "You can disable warnings related to GTSAM by setting KIMERA_VERIFY_GTSAM_CONFIG=OFF in the future.")
    endif()
endfunction()
