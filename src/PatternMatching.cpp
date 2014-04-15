#include "PatternMatching.hpp"

namespace pipeline_inspection
{

  void PatternMatching::init(const DetectorCalib &calib){
    this->calib = calib;
  }
  
  Pattern PatternMatching::match(laserInformation &pipeBuffer , Pattern p, Boundary b){
    std::cout << "Matching function" << std::endl;
    if(pipeBuffer.empty() || pipeBuffer.begin()->first.size() == 0){
      std::cout << "buffer empty" << std::endl;
      return p;
    }
    std::cout << "Buffer test" << std::endl;
    point_buffer = pipeBuffer;
    
    
    nlopt::opt opt(calib.min_algo, 4);
    
    std::vector<double> lower_bounds(4);
    lower_bounds[0] = b.minY ;
    lower_bounds[1] = b.minX ;
    lower_bounds[2] = 0.0;
    lower_bounds[3] = 0.0;
    opt.set_lower_bounds(lower_bounds);
    
    std::vector<double> upper_bounds(4);
    upper_bounds[0] = b.maxY;
    upper_bounds[1] = b.maxX;
    
    if( b.maxX - b.minX > b.maxY - b.minY){
      upper_bounds[2] = b.maxX - b.minX;
      upper_bounds[3] = upper_bounds[2];
    }else{
      upper_bounds[2] = b.maxY - b.minY;
      upper_bounds[3] = upper_bounds[2];
    }  

    opt.set_upper_bounds(upper_bounds);
    
    opt.set_min_objective(error_func, (void*) this);
    
    opt.set_xtol_rel(calib.matcher_parameter_tolerance);
    opt.set_ftol_rel(calib.matcher_value_tolerance);
    opt.set_maxeval(calib.matcher_iterations);
   
    std::vector<double> x(4);
    x[0] = p.line_height;
    x[1] = p.pipe_center;
    x[2] = p.pipe_radius_h;
    x[3] = p.pipe_radius_v;
    double minf;
    
    std::cout << "Minimizer start" << std::endl;
    nlopt::result result = opt.optimize(x, minf);
    std::cout << "Minimizer stop" << std::endl;
    std::cout << "Boundary: " << std::endl;
    std::cout << "X: " << b.minX << " " << b.maxX << std::endl;
    std::cout << "Y: " << b.minY << " " << b.maxY << std::endl;
    std::cout << "Line height: " << x[0] << std::endl;
    std::cout << "Pipe radius_h: " << x[2] << std::endl;
    std::cout << "Pipe radius_v: " << x[3] << std::endl;
    
    p.line_height = x[0];
    p.pipe_center = x[1];
    p.pipe_radius_h = x[2];
    p.pipe_radius_v = x[3];
    
    return p;
  } 
  
  double error_func(const std::vector<double> &x, std::vector<double> &grad, void *data)
  {
      PatternMatching *pm = reinterpret_cast<PatternMatching*>(data);
      laserInformation *point_buffer = &(pm->point_buffer);  
      
      if (!grad.empty()) {
            grad[0] = 0.0;
            grad[1] = 0.0;
            grad[2] = 0.0;
            grad[3] = 0.0;
      }
      
      Pattern p;
      p.line_height = x[0];
      p.pipe_center = x[1];
      p.pipe_radius_h = x[2];
      p.pipe_radius_v = x[3];
      
      return SSE(p, point_buffer->begin()->first, pm->calib);
  }  
  
  
  double SSE(Pattern p, std::vector<base::Vector3d> &points, DetectorCalib &calib){

   Line line;
   line.p = base::Vector3d(0.0, 0.0, p.line_height);
   line.direction = base::Vector3d(0.0, 1.0, 0.0);
   
//    Circle circle;
//    circle.p = base::Vector3d(0.0, p.pipe_center, p.line_height);
//    circle.radius = p.pipe_radius;
   
   Ellipse ellipse;
   ellipse.p = base::Vector3d(0.0, p.pipe_center, p.line_height);
   ellipse.radius_v = p.pipe_radius_v;
   ellipse.radius_h = p.pipe_radius_h;
   
   double sum = 0.0;

   for(std::vector<base::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++){
      
     double distance;
     base::Vector3d point = *it;  
     point.x() = 0.0;
     
     if( point.y() > ellipse.p.y() - ellipse.radius_h && point.y() < ellipse.p.y() + ellipse.radius_h){
       distance = dist_ellipse2point(ellipse, point, calib.matcher_pipe_up);
     }
     else{
	distance = dist_line2point(line, point);  
     }     
      
     sum += std::pow(distance, 2.0);    
   }

   return sum;    
  }
  

}//end namespace
