if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if ( MSYS AND ${CMAKE_BUILD_TYPE} STREQUAL "Debug" )
    set(CONSTRUCT_LIBFASTESTLAPC OFF)
else()
    set(CONSTRUCT_LIBFASTESTLAPC ON)
endif()


if ( ${CONSTRUCT_LIBFASTESTLAPC} )
    add_library(fastestlapc SHARED ./fastestlapc.cpp)
    
    set_target_properties(fastestlapc PROPERTIES 
	CXX_VISIBILITY_PRESET hidden
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
    
    set_target_properties(fastestlapc PROPERTIES
        PUBLIC_HEADER ./fastestlapc.h
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    
    if (NOT MSYS)
    	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            # Only link statically against libstdc++ if the static archive is available.
            # GCC 15+ on Fedora/Nobara ships libstdc++.so only; attempting -static-libstdc++
            # there causes a link failure.
            execute_process(
                COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.a
                OUTPUT_VARIABLE LIBSTDCXX_STATIC_PATH
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            if(EXISTS "${LIBSTDCXX_STATIC_PATH}")
    	        set(LFASTESTLAPC_ADDITIONAL_FLAGS -static-libstdc++ -static-libgcc)
            else()
                message(STATUS "libstdc++.a not found (${LIBSTDCXX_STATIC_PATH}); linking dynamically against libstdc++")
            endif()
    	endif()
    endif()

    
    target_link_libraries(fastestlapc LINK_PUBLIC lion::lion ${LFASTESTLAPC_ADDITIONAL_FLAGS})
    
    if ( NOT APPLE)
        target_link_options(fastestlapc PUBLIC -Wl,--no-as-needed -ldl)
    endif()
    
    configure_file(fastestlapc.h ${CMAKE_BINARY_DIR}/include/fastestlapc.h COPYONLY)
    
    install(TARGETS fastestlapc)
endif()
