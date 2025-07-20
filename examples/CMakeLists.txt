file(GLOB SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
foreach (SRC ${SRCS})
    string(REGEX REPLACE "\\.[^.]*$" "" SRC_WITHOUT_EXT ${SRC})
    add_executable(example_${SRC_WITHOUT_EXT}
            ${SRC})
    target_link_libraries(example_${SRC_WITHOUT_EXT} PUBLIC
            rbpodo)
endforeach ()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(example_${SRC_WITHOUT_EXT}
            ${SRC} PRIVATE nlohmann_json::nlohmann_json)

if (BUILD_EIGEN_EXAMPLES)
    add_subdirectory(eigen)
endif ()
