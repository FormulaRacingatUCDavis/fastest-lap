if (WIN32)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    get_filename_component(FASTESTLAP_COMPILER_RUNTIME_DIR
                           "${CMAKE_CXX_COMPILER}" DIRECTORY)
endif()

macro(new_test)
    get_filename_component(binary_name ${CMAKE_CURRENT_LIST_DIR} NAME)
    string(REPLACE " " "_" binary_name ${binary_name})
    
    set(BINARY ${binary_name}_test)
    
    file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp ../main.cpp)
    
    set(SOURCES ${TEST_SOURCES})

    add_executable(${BINARY} ${TEST_SOURCES})
    
    add_test(NAME ${BINARY} COMMAND ${BINARY})
    
    # Link libraries
    target_link_libraries(${BINARY} LINK_PRIVATE GTest::gtest lion::lion)

    if (NOT MSYS)
        target_link_libraries(${BINARY} LINK_PRIVATE fastestlapc)
	target_compile_definitions(${BINARY} PRIVATE TEST_LIBFASTESTLAPC)
    endif()
    
    # Windows runners may not have symlink privileges. Use real directories there.
    if (WIN32)
        add_custom_target(${BINARY}_link_data ALL
                          COMMAND ${CMAKE_COMMAND} -E copy_directory
                          ${CMAKE_CURRENT_SOURCE_DIR}/data
                          ${CMAKE_BINARY_DIR}/src/test/${binary_name}/data)

        add_custom_target(${BINARY}_link_database ALL
                          COMMAND ${CMAKE_COMMAND} -E copy_directory
                          ${CMAKE_SOURCE_DIR}/database
                          ${CMAKE_BINARY_DIR}/src/test/${binary_name}/database)
    else()
        add_custom_target(${BINARY}_link_data ALL
                          COMMAND ${CMAKE_COMMAND} -E create_symlink
                          ${CMAKE_CURRENT_SOURCE_DIR}/data
                          ${CMAKE_BINARY_DIR}/src/test/${binary_name}/data)

        add_custom_target(${BINARY}_link_database ALL
                          COMMAND ${CMAKE_COMMAND} -E create_symlink
                          ${CMAKE_SOURCE_DIR}/database
                          ${CMAKE_BINARY_DIR}/src/test/${binary_name}/database)
    endif()

    if (UNIX AND NOT APPLE)
        target_link_options(${BINARY} PUBLIC -Wl,--no-as-needed -ldl)
    endif()

    # Keep Windows test executables independently launchable. In particular,
    # CTest and Explorer do not inherit the MINGW64/third-party search paths.
    if (WIN32 AND Python3_Interpreter_FOUND AND CMAKE_OBJDUMP)
        add_custom_command(TARGET ${BINARY} POST_BUILD
            COMMAND "${Python3_EXECUTABLE}"
                    "${CMAKE_SOURCE_DIR}/cmake/collect-windows-runtime.py"
                    --objdump "${CMAKE_OBJDUMP}"
                    --destination "$<TARGET_FILE_DIR:${BINARY}>"
                    --binary "$<TARGET_FILE:${BINARY}>"
                    --search-dir "$<TARGET_FILE_DIR:fastestlapc>"
                    --search-dir "${CMAKE_BINARY_DIR}/thirdparty/bin"
                    --search-dir "${FASTESTLAP_COMPILER_RUNTIME_DIR}"
            VERBATIM)
    endif()

    if (MSYS)
      ADD_CUSTOM_TARGET(${BINARY}_link_tinyxml_windows ALL
                        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/thirdparty/bin/msys-tinyxml2-9.dll ${CMAKE_BINARY_DIR}/src/test/${binary_name}/msys-tinyxml2-9.dll)
      ADD_CUSTOM_TARGET(${BINARY}_link_lapack_windows ALL
                        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/thirdparty/bin/msys-lapack-3.dll ${CMAKE_BINARY_DIR}/src/test/${binary_name}/msys-lapack-3.dll)
      ADD_CUSTOM_TARGET(${BINARY}_link_blas_windows ALL
                        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/thirdparty/bin/msys-blas-3.dll ${CMAKE_BINARY_DIR}/src/test/${binary_name}/msys-blas-3.dll)
    endif()
    
endmacro()

add_subdirectory(./actuators)
add_subdirectory(./applications)
add_subdirectory(./chassis)
add_subdirectory(./tire)
add_subdirectory(./vehicles)

# Create a test if code coverage
if (CODE_COVERAGE)
    include(code-coverage)

    add_code_coverage_all_targets(EXCLUDE test/* googletest-src/*)

    set(BINARY coverage_test)

    file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false **/*.cpp main.cpp)

    set(SOURCES ${TEST_SOURCES})

    add_executable(${BINARY} ${TEST_SOURCES})

    target_link_libraries(${BINARY} LINK_PRIVATE GTest::gtest lion::lion fastestlapc)

    target_compile_definitions(${BINARY} PRIVATE TEST_LIBFASTESTLAPC)

    set_target_properties(${BINARY} PROPERTIES 
        EXCLUDE_FROM_ALL 1 
        EXCLUDE_FROM_DEFAULT_BUILD 1
    )

    target_code_coverage(${BINARY} AUTO ALL EXCLUDE test/* build/googletest-src/*) 

    # Copy required files
    add_custom_target(${BINARY}_link_database ALL
                      COMMAND ${CMAKE_COMMAND} -E create_symlink 
                      ${CMAKE_SOURCE_DIR}/database 
                      ${CMAKE_BINARY_DIR}/src/test/database
                     )
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/src/test/data)

    add_custom_command(TARGET ${BINARY} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/test/applications/data/ ${CMAKE_BINARY_DIR}/src/test/data)

    add_custom_command(TARGET ${BINARY} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/test/actuators/data/ ${CMAKE_BINARY_DIR}/src/test/data)

    add_custom_command(TARGET ${BINARY} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/test/chassis/data/ ${CMAKE_BINARY_DIR}/src/test/data)

    add_custom_command(TARGET ${BINARY} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/test/tire/data/ ${CMAKE_BINARY_DIR}/src/test/data)

    add_custom_command(TARGET ${BINARY} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/src/test/vehicles/data/ ${CMAKE_BINARY_DIR}/src/test/data)

endif()
