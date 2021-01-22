#####################################################
# Verify if gtsam was built with the right options

#! _check_gtsam_symbol: identify if a preprocessor define exists in gtsam
#
# Configure a try_compile check and set RESULT to the value of try_compile
# \arg:result result of try_compile
# \arg:symbol symbol to check
function(_check_gtsam_symbol result symbol)
    set(GTSAM_CONFIG_FILE "gtsam/config.h")
    set(GTSAM_CONFIG_SYMBOL "${symbol}")
    set(TEST_FILE_LOCATION "${CMAKE_BINARY_DIR}/gtsam_uses_feature_${symbol}.cpp")

    configure_file("${CMAKE_SOURCE_DIR}/cmake/tests/gtsam_uses_feature.cpp.in"
        ${TEST_FILE_LOCATION}
        @ONLY)

    try_compile(TEST_RESULT
        ${CMAKE_BINARY_DIR}
        ${TEST_FILE_LOCATION}
        CMAKE_FLAGS
            "-DINCLUDE_DIRECTORIES=${GTSAM_INCLUDE_DIR}")

    set(${result} ${TEST_RESULT} PARENT_SCOPE)
endfunction()

#! verify_gtsam_config: Warn the user if GTSAM has undesired settings
#
# Use try_compile and two code snippets to detect whether or not
# GTSAM_ROT3_EXPMAP and GTSAM_TANGENT_PREINTEGRATION were set when
# building GTSAM. If they were, warn the user that their results
# using Kimera may not match our published results.
#
function(verify_gtsam_config)
    _check_gtsam_symbol(GTSAM_ROT3_EXPMAP_FOUND "GTSAM_ROT3_EXPMAP")
    _check_gtsam_symbol(GTSAM_TANGENT_PREINTEGRATION_FOUND "GTSAM_TANGENT_PREINTEGRATION")

    set(GTSAM_SETTINGS_OKAY TRUE)
    if(NOT ${GTSAM_ROT3_EXPMAP_FOUND})
        set(GTSAM_SETTINGS_OKAY FALSE)
        # we don't provide specific commands here because the default option is correct in gtsam
        message(WARNING "Detected that the GTSAM version to be used is not configured to use Rot3::Expmap for retractions. You should build GTSAM with GTSAM_ROT3_EXPMAP=ON and GTSAM_USE_QUATERNIONS=OFF.")
    endif()

    if(${GTSAM_TANGENT_PREINTEGRATION_FOUND})
        set(GTSAM_SETTINGS_OKAY FALSE)
        # the default option is not what we want in gtsam, so we provide instructions
        message(WARNING "Detected that the GTSAM version to be used is not configured to use on-manifold IMU preintegration. You should build GTSAM with GTSAM_TANGENT_PREINTEGRATION=OFF. You can do this when using catkin by cleaning gtsam and running 'catkin config -a --cmake-args -DGTSAM_TANGENT_PREINTEGRATION=OFF' and then rebuilding")
    endif()

    if(NOT ${GTSAM_SETTINGS_OKAY})
        message(WARNING "You can disable warnings related to GTSAM by setting KIMERA_VERIFY_GTSAM_CONFIG=OFF in the future.")
    endif()
endfunction()
