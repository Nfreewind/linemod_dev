# AUX_SOURCE_DIRECTORY(. App_LIST)

# foreach( current_app ${App_LIST} )
#     # I used a simple string replace, to cut off .cpp.
#     get_filename_component(app_name  ${current_app}  NAME_WE)
#     add_executable( ${app_name}  ${current_app})
#     message("Now name is : " ${app_name})
#     # Make sure YourLib is linked to each app
#     target_link_libraries(${app_name} ${OpenCV_LIBS} rgbd)

# endforeach( current_app ${App_LIST} )
add_executable(opencv_test opencv_test.cpp)
target_link_libraries(opencv_test ${OpenCV_LIBS})

add_executable(read_depth_and_show read_depth_and_show.cpp)
target_link_libraries(read_depth_and_show ${OpenCV_LIBS})
