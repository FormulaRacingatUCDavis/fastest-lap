# Google tests 
find_package(GTest PATHS ${CMAKE_BINARY_DIR})

if ( NOT ${GTest_FOUND})
    set(BUILD_GTEST YES)
endif()

# Lion
set(BUILD_LION YES)



#######################################################################




##### BUILD ALL REQUIRED THIRD PARTY LIBRARIES ##########
message(STATUS "Compilation of the required third party libraries")
configure_file(cmake/third-party/CMakeLists.txt ${CMAKE_BINARY_DIR}/thirdparty/CMakeLists.txt)

execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
    RESULT_VARIABLE THIRD_PARTY_CONFIGURE_RESULT
)
if (NOT THIRD_PARTY_CONFIGURE_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Third-party configuration failed with exit code ${THIRD_PARTY_CONFIGURE_RESULT}")
endif()

execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
    RESULT_VARIABLE THIRD_PARTY_BUILD_RESULT
)
if (NOT THIRD_PARTY_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Third-party build failed with exit code ${THIRD_PARTY_BUILD_RESULT}")
endif()
message("")
message(STATUS "Configuration of fastest-lap")
#######################################################################


find_package(GTest PATHS ${CMAKE_BINARY_DIR}/thirdparty REQUIRED)
unset(lion_DIR CACHE)
find_package(lion CONFIG REQUIRED
    PATHS ${CMAKE_BINARY_DIR}/thirdparty/lib/cmake/lion
    NO_DEFAULT_PATH)
