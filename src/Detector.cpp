#include "Detector.hpp"

namespace pipeline_inspection
{

  void Detector::init(DetectorCalib c){
    calib = c;
    
    calib.laserPos2Cam = ( c.cameraOrientation * c.laserPos) + c.cameraPos;  //TODO inverse???
    calib.laserNorm2Cam = c.cameraOrientation * c.laserNorm; //TODO inverse???
    
    pipeBuffer.resize(calib.buffer_size);
    matcher.init(calib);
  }
  
  
  InspectionStatus Detector::inspect(base::samples::LaserScan scan, controlData::Pipeline pipe, base::samples::RigidBodyState pos){
     std::vector<base::Vector3d> points;
     points.clear();
     
     // convert scan to points
     double angle = scan.start_angle;
     uint32_t max_range = 0;
     for(std::vector<uint32_t>::iterator it = scan.ranges.begin(); it != scan.ranges.end(); it++){
	
       if(*it > max_range)
         max_range = *it;
       
       if(*it > 0 && *it < 1000000){ //Filter out min and maximum ranges
	  base::Vector3d p;
	  p.x() = (double)*it * std::cos(angle);
	  p.y() = (double)*it * std::sin(angle);
	  p.z() = 0.0;
	  
	  p *= 0.001;
	  
	  p = calib.cameraOrientation * p;
	  p += calib.cameraPos;
	
	  points.push_back(p);
       }
       
	angle += scan.angular_resolution;
       
    }
     std::cout << "Max range: " << max_range << std::endl;
     return inspect(points, pipe, pos);    
  }
  
  InspectionStatus Detector::inspect(std::vector<base::Vector3d> points, controlData::Pipeline pipe, base::samples::RigidBodyState pos){
    
    //Init structures
    InspectionStatus status;
    status.state = NO_PIPE;
    
    std::vector<base::Vector3d> laserPoints;
    laserPoints.clear();
    std::vector<base::Vector3d> worldPoints;
    worldPoints.clear();
    pipePoints.clear();
    
    //Convert points from camera frame to laser plant
    projectionSSE = 0.0;
    
    for(std::vector<base::Vector3d>::iterator it = points.begin(); it != points.end(); it++){
      
      base::Vector3d pL;
      double error = camera2LaserFrame(*it, pL);
      projectionSSE += error * error;
      
      laserPoints.push_back(pL);
    }
    
    //Project points from laserplant to vertical plant
    for(std::vector<base::Vector3d>::iterator it = laserPoints.begin(); it != laserPoints.end(); it++){
      base::Vector3d pW;
      laser2VerticalFrame(*it, pW);
      
      pipePoints.push_back(pW);
    }
    
    
    //Search for minimum and maximum in points
    double min,max;
    
    if(pipePoints.size() > 0){
      
      min = pipePoints.begin()->z();
      max = min;
      
      double laser_min = pipePoints.begin()->y();
      double laser_max = laser_min;
      
      for(std::vector<base::Vector3d>::iterator it = pipePoints.begin(); it != pipePoints.end(); it++){
	
	if(it->z() < min)
	  min = it->z();
	
	if(it->z() > max)
	  max = it->z();
	
	if(it->y() < laser_min)
	  laser_min = it->y();
	
	if(it->y() > laser_max)
	  laser_max = it->y();
      }
      
      
      double height_diff = max - min;
      double laser_diff = laser_max - laser_min;
      
      double pipe_begin = laser_max;
      double pipe_end = laser_min;
      
      //Search for the begin and end of pipe;
      for(std::vector<base::Vector3d>::iterator it = pipePoints.begin(); it != pipePoints.end(); it++){
	
        if(calib.matcher_pipe_up){ // If pipe is a maximum, search for begin and end of maximum
          if(it->z() > min + (0.5 * height_diff) && it->y() < pipe_begin)
            pipe_begin = it->y();
          
          if(it->z() > min + (0.5 * height_diff) && it->y() > pipe_end)
            pipe_end = it->y();
        }
        else{      //If pipe is minimum, search for begin and end of minimum
          if(it->z() > max - (0.5 * height_diff) && it->y() < pipe_begin)
            pipe_begin = it->y();
          
          if(it->z() > max - (0.5 * height_diff) && it->y() > pipe_end)
            pipe_end = it->y();         
          
        }
      }     
      
      status.pipe_width = pipe_end - pipe_begin;
      status.pipe_height = height_diff;
      status.laser_width = laser_diff;     
      status.pipe_radius = ((pipe_end - pipe_begin) + height_diff) / 2.0; //Radius is the avergae between height and width
      
      Pattern p;
      p.line_height = calib.matcher_pipe_up ? min : max;
      p.pipe_center = pipe_begin + ( 0.5 * (pipe_end - pipe_begin));
      //p.pipe_radius = ((pipe_end - pipe_begin) + height_diff) / 2.0; //Average between pipe height and pipe width
      p.pipe_radius_h = (pipe_end - pipe_begin);
      p.pipe_radius_v = height_diff;
      
      if(pipePoints.size() > 0){
        pipeBuffer.push_back( std::pair<std::vector<base::Vector3d>, base::samples::RigidBodyState>(pipePoints, pos) );
        Boundary b;
        b.minX = laser_min;
        b.maxX = laser_max;
        b.minY = min;
        b.maxY = max;
        Pattern pattern = matcher.match(pipeBuffer, p, b);      
          
        status.pipe_width = pattern.pipe_radius_h;
        status.pipe_height = pattern.pipe_radius_v;
        status.pipe_radius = (pattern.pipe_radius_h + pattern.pipe_radius_v) / 2.0 ;
        status.pipe_center = pattern.pipe_center;
        status.laser_height = pattern.line_height;
      }
      
    }
    status.projection_error = projectionSSE;
    return status;
  }
  
  double Detector::camera2LaserFrame(const base::Vector3d &pointC, base::Vector3d &pointL){
    
    //convert point from cameraframe to worldframe
    base::Vector3d pointW = (calib.cameraOrientation * pointC) + calib.cameraPos;    
    
    //Projekt camerapoint to laserframe
    //http://de.wikipedia.org/wiki/Orthogonalprojektion#Projektion_auf_eine_Ebene
    pointL = pointW;
    pointL -= ( ( (pointL - calib.laserPos).dot(calib.laserNorm) ) / calib.laserNorm.dot( calib.laserNorm) ) * calib.laserNorm;
   
    
    //Return the distance between the point in worldframe the the projection
    return (pointL - pointW).norm();
  }
  
  void Detector::laser2VerticalFrame(const base::Vector3d &pointL, base::Vector3d &pointV){
    
    //Define the vertical plane
    base::Vector3d verticalNorm(1.0, 0.0, 0.0);
    base::Vector3d verticalPos(0.0, 0.0, 0.0);
    
    pointV = pointL;
    pointV -= ( ( (pointL - verticalPos).dot(verticalPos) ) / verticalNorm.dot(verticalNorm) ) * verticalNorm;
    
  }
  
  std::vector<base::Vector3d> Detector::getPipePoints(){
    return pipePoints;
  }
  
  double Detector::getProjectionSSE(){
    return projectionSSE;
  }

}
