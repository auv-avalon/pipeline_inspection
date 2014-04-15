#ifndef PIPELINEINSPECTION_DETECTORTYPES_HPP
#define PIPELINEINSPECTION_DETECTORTYPES_HPP

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <boost/circular_buffer.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <vector>
#include <utility>
#include <nlopt.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace pipeline_inspection {
    
  typedef boost::circular_buffer< std::pair<std::vector<base::Vector3d>, base::samples::RigidBodyState> > laserInformation;
  typedef nlopt::algorithm min_algorithm;
  
  enum PipelineState{
    NORMAL = 0,
    UNDERFLOODING = 1,
    OVERFLOODING = 2,
    NO_PIPE = 3
  };
  
  struct InspectionStatus{
    
    base::Time time;
    PipelineState state;
    double pipe_width;
    double pipe_height;
    double pipe_center;
    double pipe_radius;
    double laser_width;
    double laser_height;
    double matching_error; // SSE between projected points and calculated pattern
    double projection_error; //SSE between worldpoints and laserframe
    
  };
  
  struct DetectorCalib{
    base::Vector3d cameraPos;
    base::Vector3d laserPos;
    base::Vector3d laserNorm;
    base::Quaterniond cameraOrientation;
    
    int buffer_size;
    nlopt::algorithm min_algo;
    double matcher_parameter_tolerance;
    double matcher_value_tolerance;
    int matcher_iterations;
    bool matcher_pipe_up;
    
    //To be calculated by Detector-object
    base::Vector3d laserPos2Cam;
    base::Vector3d laserNorm2Cam;
    base::Vector3d verticalPos2Laser;
    base::Vector3d verticalNorm2Laser;
  };

  
}

#endif