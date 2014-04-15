rock_library(pipeline_inspection
    SOURCES Dummy.cpp Detector.cpp PatternMatching.cpp GeomHelper.cpp
    HEADERS Dummy.hpp Detector.hpp DetectorTypes.hpp PatternMatching.hpp GeomHelper.hpp
    DEPS_PKGCONFIG base-types 
    DEPS_PKGCONFIG eigen3
    DEPS_PKGCONFIG nlopt
    DEPS_CMAKE Boost)

rock_executable(pipeline_inspection_bin Main.cpp
    DEPS pipeline_inspection)

