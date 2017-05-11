
set (VISUALIZER_SOURCE
	uiforce/UIForceWidget.cpp
	visualizer_project_main.cpp)

add_executable(visualizer_project
	${CS225A_COMMON_SOURCE}
	${VISUALIZER_SOURCE})

target_link_libraries(visualizer_project
	${CS225A_COMMON_LIBRARIES})
