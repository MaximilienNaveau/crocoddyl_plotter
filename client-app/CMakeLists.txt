find_package(Qt6 REQUIRED COMPONENTS Core Widgets PrintSupport)
# qt6_standard_project_setup()

set(HEADERS
    mainwindow.h
    grpc-client.h
    QCustomPlot/qcustomplot.h
)
set(SOURCES
    main.cpp
    grpc-client.cpp
    mainwindow.cpp
    QCustomPlot/qcustomplot.cpp
)
qt_wrap_cpp(SOURCES TARGET crocoddyl_plotter_client_gui ${HEADERS})

qt_add_executable(crocoddyl_plotter_client_gui
    ${SOURCES} ${HEADERS}
)

target_link_libraries(crocoddyl_plotter_client_gui PRIVATE crocoddyl_plotter_grpc_proto Qt6::Core Qt6::Widgets Qt6::PrintSupport)
install(TARGETS crocoddyl_plotter_client_gui DESTINATION bin)