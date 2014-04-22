#include "GeomHelper.hpp"

namespace pipeline_inspection
{
  
  /*double dist_line2point(const Line &line, const base::Vector3d &point){
    
    return std::fabs( ( (line.p - point) - ( (line.p - point).dot(line.direction) * line.direction ) ).norm() );    
  }*/
  
  /*
  double dist_line2point(const Line &line, const base::Vector3d &point){
    return std::fabs( line.p.z() - point.z() );
  }*/
  
  double dist_line2point(const Line &line, const base::Vector3d &point){
    
    return std::fabs( (line.p.z() + (line.p.y() * line.gradient)) - point.z() );
  }
  
  
  double dist_circle2point(const Circle &c, const base::Vector3d &point){
    
    return std::fabs( (point - c.p).norm() - c.radius);    
  }
  
  double dist_ellipse2point(const Ellipse &e, const base::Vector3d &point){
    double dist_up = dist_ellipse2point(e, point, true);
    double dist_down = dist_ellipse2point(e, point, false);
    
    if(dist_up  < dist_down)
      return dist_up;
    else
      return dist_down;
  }
  
  double dist_ellipse2point(const Ellipse &e, const base::Vector3d &point, bool up){
   
    double x = std::fabs( point.y() - e.p.y() );
    
    if( x > e.radius_h)
      return std::numeric_limits<double>::infinity();
    
    double y = std::sqrt( (1 - ( (x * x) / (e.radius_h * e.radius_h) ) ) * (e.radius_v * e.radius_v) ); //Ellipse formular
    
    if(up)
      return std::fabs( (e.p.z() + y) - point.z() );
    else    
      return std::fabs( (e.p.z() - y) - point.z() );
  
  }
  

}//end namespace
