rock_library(pipeline_inspection
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(pipeline_inspection_bin Main.cpp
    DEPS pipeline_inspection)

