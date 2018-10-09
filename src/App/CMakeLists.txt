# AUX_SOURCE_DIRECTORY(. App_LIST)

# foreach( current_app ${App_LIST} )
#     # I used a simple string replace, to cut off .cpp.
#     get_filename_component(app_name  ${current_app}  NAME_WE)
#     add_executable( ${app_name}  ${current_app})
#     message("Now name is : " ${app_name})
#     # Make sure YourLib is linked to each app
#     target_link_libraries(${app_name} ${OpenCV_LIBS} rgbd)

# endforeach( current_app ${App_LIST} )

add_executable(lmd_test linemod_test.cpp)
target_link_libraries(lmd_test ${OpenCV_LIBS} lmd_dev)
