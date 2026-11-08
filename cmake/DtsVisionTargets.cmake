function(dts_enable_pch target_name)
    if(BUILD_WITH_PCH AND COMMAND target_precompile_headers)
        target_precompile_headers(
            ${target_name}
            PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${DTSVISION_PCH_HEADER}>"
        )
    endif()
endfunction()

function(dts_add_library target_name)
    set(options EXCLUDE_FROM_ALL)
    set(one_value_args TYPE)
    set(multi_value_args
        SOURCES
        GLOB
        INCLUDES
        DEFINITIONS
        LINKS
        PUBLIC_LINKS
        PRIVATE_LINKS
    )
    cmake_parse_arguments(DTS_LIB "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT DTS_LIB_TYPE)
        set(DTS_LIB_TYPE SHARED)
    endif()

    set(target_sources ${DTS_LIB_SOURCES})
    if(DTS_LIB_GLOB)
        file(GLOB_RECURSE glob_sources CONFIGURE_DEPENDS ${DTS_LIB_GLOB})
        list(APPEND target_sources ${glob_sources})
    endif()

    if(DTS_LIB_EXCLUDE_FROM_ALL)
        add_library(${target_name} ${DTS_LIB_TYPE} EXCLUDE_FROM_ALL ${target_sources})
    else()
        add_library(${target_name} ${DTS_LIB_TYPE} ${target_sources})
    endif()

    dts_enable_pch(${target_name})

    if(DTS_LIB_INCLUDES)
        target_include_directories(${target_name} PUBLIC ${DTS_LIB_INCLUDES})
    endif()

    if(DTS_LIB_DEFINITIONS)
        target_compile_definitions(${target_name} PUBLIC ${DTS_LIB_DEFINITIONS})
    endif()

    if(DTS_LIB_PUBLIC_LINKS OR DTS_LIB_PRIVATE_LINKS)
        if(DTS_LIB_LINKS OR DTS_LIB_PUBLIC_LINKS)
            target_link_libraries(${target_name} PUBLIC ${DTS_LIB_LINKS} ${DTS_LIB_PUBLIC_LINKS})
        endif()
        if(DTS_LIB_PRIVATE_LINKS)
            target_link_libraries(${target_name} PRIVATE ${DTS_LIB_PRIVATE_LINKS})
        endif()
    elseif(DTS_LIB_LINKS)
        target_link_libraries(${target_name} ${DTS_LIB_LINKS})
    endif()
endfunction()

function(dts_add_executable exe_name)
    set(options TOOL EXCLUDE_FROM_ALL)
    set(multi_value_args SOURCES LINKS PRIVATE_LINKS)
    cmake_parse_arguments(DTS_EXE "${options}" "" "${multi_value_args}" ${ARGN})

    set(exclude_from_all FALSE)
    if(DTS_EXE_EXCLUDE_FROM_ALL OR DTS_EXE_TOOL)
        set(exclude_from_all TRUE)
    endif()

    if(exclude_from_all)
        add_executable(${exe_name} EXCLUDE_FROM_ALL ${DTS_EXE_SOURCES})
    else()
        add_executable(${exe_name} ${DTS_EXE_SOURCES})
    endif()

    dts_enable_pch(${exe_name})

    set(default_links ${PROJECT_NAME}_tools relink)
    if(RERUN_FOUND)
        list(APPEND default_links ${PROJECT_NAME}_rerun)
    endif()

    if(DTS_EXE_PRIVATE_LINKS)
        target_link_libraries(${exe_name} ${default_links} ${DTS_EXE_LINKS})
        target_link_libraries(${exe_name} PRIVATE ${DTS_EXE_PRIVATE_LINKS})
    else()
        target_link_libraries(${exe_name} ${default_links} ${DTS_EXE_LINKS})
    endif()
endfunction()

function(add_common_executable exe_name src_file)
    dts_add_executable(${exe_name} SOURCES ${src_file} LINKS ${ARGN})
endfunction()

function(add_tool_executable exe_name src_file)
    dts_add_executable(${exe_name} TOOL SOURCES ${src_file} LINKS ${ARGN})
endfunction()
