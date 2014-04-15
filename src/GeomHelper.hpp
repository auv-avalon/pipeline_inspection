#ifndef _PIPELINEINSPECTION_GEOMHELPER_HPP_
#define _PIPELINEINSPECTION_GEOMHELPER_HPP_

#include <iostream>
#include <vector>
#include <base/samples/LaserScan.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Eigen.hpp>
#include <math.h>
#include <limits>

namespace pipeline_inspection{       
  
	
	struct Line{
	  base::Vector3d p; //Startpoint of the line
	  base::Vector3d direction; //Direction of the line -> this should be a unit vector
	};
  
	struct Circle{
	  base::Vector3d p; //Center of the cirlce
	  double radius; //Radius of the circle
	};
        
        struct Ellipse{
          base::Vector3d p; //Center of elipse
          double radius_v;
          double radius_h;
        };

        
          double dist_line2point(const Line &l, const base::Vector3d &point);
          double dist_circle2point(const Circle &c, const base::Vector3d &point);
          double dist_ellipse2point(const Ellipse &e, const base::Vector3d &point);
          double dist_ellipse2point(const Ellipse &e, const base::Vector3d &point, bool up);


	
} // end namespace pipeline_inspection

#endif
