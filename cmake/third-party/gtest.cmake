if (${BUILD_GTEST})
include(ExternalProject)
ExternalProject_Add(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG 1b6f64d659944658a4c685b7bd9f04c1c3b8a39b
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/thirdparty -DCMAKE_BUILD_TYPE=Release -DBUILD_GMOCK=OFF
    PREFIX ${THIRD_PARTY_DIR}/gtest
    SOURCE_DIR "${THIRD_PARTY_DIR}/gtest/source"
    BINARY_DIR "${THIRD_PARTY_DIR}/gtest/build"
    INSTALL_DIR ${THIRD_PARTY_DIR}/thirdparty
)
endif()
