file(GLOB IMAGES ${CMAKE_CURRENT_LIST_DIR}/*.png)

add_library(lvgl_images)

find_package(Python3 COMPONENTS Interpreter)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/images_sources)

message(-- ${Python3_EXECUTABLE})

execute_process(
    COMMAND ${Python3_EXECUTABLE} Third_Party/lvgl/scripts/LVGLImage.py --ofmt C --cf ARGB8888 --align 4 images -o ${CMAKE_BINARY_DIR}/images_sources
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/generate_header.py images ${CMAKE_BINARY_DIR}/images_sources/lvgl_images.h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE stdout
    ERROR_VARIABLE stderr
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
)

foreach(IMAGE IN LISTS IMAGES)
    get_filename_component(IMAGE_BASENAME ${IMAGE} NAME)
    string(REGEX REPLACE "[.]png$" ".c" OUTPUT_SOURCE ${IMAGE_BASENAME})
    target_sources(lvgl_images PUBLIC ${CMAKE_BINARY_DIR}/images_sources/${OUTPUT_SOURCE})
endforeach()

get_target_property(LVGL_INCLUDE lvgl INCLUDE_DIRECTORIES)
get_target_property(LVGL_DEFINES lvgl COMPILE_DEFINITIONS)
target_include_directories(lvgl_images 
    PUBLIC ${CMAKE_BINARY_DIR}/images_sources
    PRIVATE ${LVGL_INCLUDE}
)
target_compile_definitions(lvgl_images PRIVATE ${LVGL_DEFINES})
