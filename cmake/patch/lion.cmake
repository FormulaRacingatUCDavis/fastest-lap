cmake_minimum_required(VERSION 3.10)

if (NOT DEFINED LION_SOURCE_DIR)
    message(FATAL_ERROR "LION_SOURCE_DIR is required")
endif()

function(patch_lion_file relative_path original replacement)
    set(path "${LION_SOURCE_DIR}/${relative_path}")
    if (NOT EXISTS "${path}")
        message(FATAL_ERROR "Cannot patch missing Lion file: ${path}")
    endif()

    file(READ "${path}" contents)
    string(FIND "${contents}" "${replacement}" replacement_position)
    if (NOT replacement_position EQUAL -1)
        return()
    endif()

    string(FIND "${contents}" "${original}" original_position)
    if (original_position EQUAL -1)
        message(FATAL_ERROR
            "Lion patch context was not found in ${relative_path}; "
            "the pinned Lion dependency may have changed")
    endif()

    string(REPLACE "${original}" "${replacement}" contents "${contents}")
    file(WRITE "${path}" "${contents}")
endfunction()

patch_lion_file(
    "cmake/third-party/loggercpp.cmake"
    [=[GIT_TAG main]=]
    [=[GIT_TAG e1b9708a630097e649a03f331fa56894ffca16d4]=])

patch_lion_file(
    "cmake/third-party/tinyxml.cmake"
    [=[GIT_TAG master]=]
    [=[GIT_TAG 8224e427b655b83dae5e2298f1e6919523a78737]=])

patch_lion_file(
    "cmake/third-party/lapack.cmake"
    [=[GIT_TAG master]=]
    [=[GIT_TAG fad42f8ecef33fde714d57a4dc3e38dff8c8f033]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[GIT_TAG releases/3.0.2]=]
    [=[GIT_TAG 1063386179a14b34bf762cdefca051606bd09fc4]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[GIT_TAG stable/3.14]=]
    [=[GIT_TAG 72a29c9aab198afa0dbb940339022a22c415a4eb]=])

patch_lion_file(
    "cmake/third-party/cppad.cmake"
    [=[CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=]=]
    [=[CMAKE_ARGS -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=]=])

foreach(dependency IN ITEMS tinyxml lapack)
    patch_lion_file(
        "cmake/third-party/${dependency}.cmake"
        [=[CONFIGURE_COMMAND ${CMAKE_COMMAND}
                -DCMAKE_C_COMPILER]=]
        [=[CONFIGURE_COMMAND ${CMAKE_COMMAND}
                -G "${CMAKE_GENERATOR}"
                -DCMAKE_C_COMPILER]=])
endforeach()

patch_lion_file(
    "cmake/third-party/lapack.cmake"
    [=[-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
                -DCMAKE_INSTALL_PREFIX]=]
    [=[-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
                -DCMAKE_Fortran_COMPILER:FILEPATH=${CMAKE_Fortran_COMPILER}
                -DCMAKE_BUILD_TYPE:STRING=Release
                -DCMAKE_INSTALL_PREFIX]=])

patch_lion_file(
    "cmake/third-party/CMakeLists.txt"
    [=[set(CMAKE_FORTRAN_COMPILER ${CMAKE_FORTRAN_COMPILER})]=]
    [=[set(CMAKE_Fortran_COMPILER ${CMAKE_Fortran_COMPILER})]=])

patch_lion_file(
    "cmake/get-third-party.cmake"
    [=[execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
)
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/thirdparty"
)]=]
    [=[execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
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
endif()]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[    include (ExternalProject)
    ExternalProject_Add(mumps]=]
    [=[    include (ExternalProject)

    find_program(LION_SH_EXECUTABLE NAMES sh)
    if (NOT LION_SH_EXECUTABLE)
        message(FATAL_ERROR "A POSIX shell is required to configure MUMPS and Ipopt")
    endif()

    ExternalProject_Add(mumps]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[CONFIGURE_COMMAND cd ${THIRD_PARTY_DIR}/mumps/source && ./get.Mumps && ./configure]=]
    [=[CONFIGURE_COMMAND cd ${THIRD_PARTY_DIR}/mumps/source && ${LION_SH_EXECUTABLE} ./get.Mumps && ${LION_SH_EXECUTABLE} ./configure]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[../source/configure CXX=]=]
    [=[${LION_SH_EXECUTABLE} ../source/configure CXX=]=])

patch_lion_file(
    "cmake/third-party/ipopt.cmake"
    [=[F77=${CMAKE_FORTRAN_COMPILER}]=]
    [=[F77=${CMAKE_Fortran_COMPILER}]=])
