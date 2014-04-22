#ifndef _PIPELINEINSPECTION_PATTERNMATCHING_HPP_
#define _PIPELINEINSPECTION_PATTERNMATCHING_HPP_

#include <iostream>
#include <vector>
#include "DetectorTypes.hpp"
#include "GeomHelper.hpp"
#include <base/samples/LaserScan.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Eigen.hpp>
#include <nlopt.hpp>
#include <boost/circular_buffer.hpp>

namespace pipeline_inspection{       
  
	struct Pattern{
	  double line_height;
	  double pipe_center;
	  double pipe_radius_v;
          double pipe_radius_h;
          double line_gradient;
	};
        
        struct Boundary{
          double minY;
          double maxY;
          double minX;
          double maxX;
          double maxRad;
        };	
  
	class PatternMatching
	{
	public:
	  void init( const DetectorCalib &calib);
	  Pattern match(laserInformation &pipeBuffer , Pattern p, Boundary b);
	  
	  double SSE(Pattern p) const;  
	  
	  std::vector<base::Vector3d> points;	
	  laserInformation point_buffer;
	  DetectorCalib calib;
	  
//	private:
          
          
          //static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
          //    return (*reinterpret_cast<PatternMatching*>(data))(x, grad); }          
          	  
	};
        
          double SSE(Pattern p, std::vector<base::Vector3d> &points, DetectorCalib &calib);
          double error_func(const std::vector<double> &x, std::vector<double> &grad, void *data);        


	
} // end namespace pipeline_inspection

#endif
