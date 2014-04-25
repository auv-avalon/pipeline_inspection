#include "Detector.hpp"

namespace pipeline_inspection
{

  void Detector::init(DetectorCalib c){
    calib = c;
    
    calib.laserPos2Cam = ( c.cameraOrientation * c.laserPos) + c.cameraPos;  //TODO inverse???
    calib.laserNorm2Cam = c.cameraOrientation * c.laserNorm; //TODO inverse???
    
    pipeBuffer.resize(calib.buffer_size);
    filteredPipeBuffer.resize(calib.buffer_size);
    statusBuffer.resize(calib.buffer_size);
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
    
    filterPoints(pipePoints, filteredPipePoints, pipe);
    
    pipeBuffer.push_back( std::pair<std::vector<base::Vector3d>, base::samples::RigidBodyState>(pipePoints, pos) );
    filteredPipeBuffer.push_back( std::pair<std::vector<base::Vector3d>, base::samples::RigidBodyState>(filteredPipePoints, pos) );    
    
    if(filteredPipePoints.size() > 0){
                  
      min = filteredPipePoints.begin()->z();
      max = min;
      
      double laser_min = filteredPipePoints.begin()->y();
      double laser_max = laser_min;
      
      for(std::vector<base::Vector3d>::iterator it = filteredPipePoints.begin(); it != filteredPipePoints.end(); it++){
	
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
      for(std::vector<base::Vector3d>::iterator it = filteredPipePoints.begin(); it != filteredPipePoints.end(); it++){
	
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
        
        Boundary b;
        b.minX = laser_min;
        b.maxX = laser_max;
        b.minY = min;
        b.maxY = max;
        b.maxRad = calib.pipe_radius_h * (1.0 + (calib.pipe_tolerance_h * 2.0) );
        
        double error;
        Pattern pattern = matcher.match(filteredPipePoints, p, b, error);      
          
        status.pipe_width = pattern.pipe_radius_h;
        status.pipe_height = pattern.pipe_radius_v;
        status.pipe_radius = (pattern.pipe_radius_h + pattern.pipe_radius_v) / 2.0 ;
        status.pipe_center = pattern.pipe_center;
        status.laser_height = pattern.line_height;
        status.laser_gradient = pattern.line_gradient;
        status.matching_error = error;
                
        if(filteredPipePoints.size() > 0){        
          status.matching_variance = error/filteredPipePoints.size();
        }
        else{
          status.matching_variance = 0.0;
        }
          
        pattern.var = status.matching_variance;          
          
        status.state = ratePipe(pattern);
      }
      
    }
    status.projection_error = projectionSSE;
    statusBuffer.push_back(status);
    
    
    return status;
  }
  
  double Detector::camera2LaserFrame(const base::Vector3d &pointC, base::Vector3d &pointL){
    
    //convert point from cameraframe to worldframe
    base::Vector3d pointW = (calib.cameraOrientation * pointC) + calib.cameraPos;
        
    if(calib.invert_z)
      pointW.z() = -pointW.z();
    
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
    pointV.x() = 0.0;
    pointV.y() *= -1.0; //HACK invert y
    
  }
  
  std::vector<base::Vector3d> Detector::getPipePoints(){
    return pipePoints;
  }
  
  double Detector::getProjectionSSE(){
    return projectionSSE;
  }
  
  
  base::samples::Pointcloud Detector::getPointcloud(bool relative_map){
    
    base::samples::Pointcloud cloud;
       
    if(!pipeBuffer.empty()){
     
      base::Vector3d first_pos;
      
      if(relative_map){
        //first_pos = pipeBuffer.begin()->second.position; //Align map to first sample
          
          
        //Align map to middle sample 
        first_pos = pipeBuffer[(int) (pipeBuffer.size() / 2.0)].second.position;        
        
      }
      else{
        first_pos = base::Vector3d(0.0, 0.0, 0.0);
      }
      
      boost::circular_buffer<InspectionStatus>::iterator it_s = statusBuffer.begin();
      
      for(laserInformation::iterator it = pipeBuffer.begin(); it != pipeBuffer.end() && it_s != statusBuffer.end(); it++, it_s++){
        std::vector<base::Vector3d>& ps = it->first;
        base::samples::RigidBodyState& rbs = it->second;
        
        double pipe_begin = it_s->pipe_center - it_s->pipe_width;
        double pipe_end = it_s->pipe_center + it_s->pipe_width;

        for(std::vector<base::Vector3d>::iterator jt = ps.begin(); jt != ps.end(); jt++){

          base::Vector3d pos = rbs.position - first_pos;
          pos.x() *= calib.movement_factor;
          pos.y() *= calib.movement_factor;
          pos.z() = calib.z_offset;
          base::Vector3d point = *jt;          
          
          base::Vector3d p = (rbs.orientation * point) + pos;
          cloud.points.push_back(p);
          
          if( jt->y() < pipe_begin || jt-> y() > pipe_end)
            cloud.colors.push_back(calib.ground_color);          
          else if(it_s->state == UNDERFLOODING)
            cloud.colors.push_back(calib.underflooding_color);
          else if(it_s->state == OVERFLOODING)
            cloud.colors.push_back(calib.overflooding_color);
          else if(it_s->state == NORMAL)
            cloud.colors.push_back(calib.pipe_color);
          else if(it_s->state == NO_PIPE)
            cloud.colors.push_back(calib.ground_color);          
          
        }       
      }
     
     cloud.time = pipeBuffer.back().second.time;
     
    }
     
    return cloud;
  }
  
  PipelineState Detector::ratePipe(Pattern p){
    
    if(p.var > calib.matcher_variance_threshold)
      return NO_PIPE;
    
    double min_radius_h = calib.pipe_radius_h * (1.0 - calib.pipe_tolerance_h);
    double max_radius_h = calib.pipe_radius_h * (1.0 + calib.pipe_tolerance_h);
    double min_radius_v = calib.pipe_radius_v * (1.0 - calib.pipe_tolerance_v);
    double max_radius_v = calib.pipe_radius_v * (1.0 + calib.pipe_tolerance_v);
    
    if(p.pipe_radius_h > max_radius_h || p.pipe_radius_v < calib.pipe_min_radius) //Pipe width to big or pipe to low -> detect nothing
      return NO_PIPE;
         
    if(p.pipe_radius_v > max_radius_v) //Pipe is to high
      return UNDERFLOODING;
    
    if(p.pipe_radius_v < min_radius_v) //Pipe is to low
      return OVERFLOODING;
    
    //if(p.pipe_radius_h > max_radius && p.pipe_radius_v > min_radius) //Pipe width is to big, heigth is normal
    //  return OVERFLOODING;
    
    if(p.pipe_radius_h > min_radius_h && p.pipe_radius_h < max_radius_h) //Pipe is okay
      return NORMAL;
    
    return NO_PIPE;    
    
  }
  
  void Detector::filterPoints(const std::vector<base::Vector3d> &src, std::vector<base::Vector3d> &dst, controlData::Pipeline pipe){
    dst.clear();    
    
    double left_border = 0.0;
    double right_border = 0.0;
    
    if(pipe.angle > calib.max_pipe_angle || pipe.angle < - calib.max_pipe_angle
        || pipe.confidence < calib.min_pipe_confidence)
    {
      
      if(src.size() > 0){
        
        double min = src.begin()->y();
        double max = min;
        
        for(std::vector<base::Vector3d>::const_iterator it = src.begin(); it != src.end(); it++){
          
          if(it->y() < min)
            min = it->y();
          
          if(it->y() > max)
            max = it->y();
          
        }
      
        double span = max - min;
        left_border = min + (span * calib.left_laser_boundary);
        right_border = max - (span * calib.right_laser_boundary);  
        
        return; //TODO do nothing, if no pipe avaialble?
        
      }      
      
    }
    else{
      left_border = pipe.y_m - (pipe.width_m * 2.0);
      right_border = pipe.y_m + (pipe.width_m * 2.0);      
    }
    
    
    for(std::vector<base::Vector3d>::const_iterator it = src.begin(); it != src.end(); it++){
      
      if(it->y() > left_border && it->y() < right_border){       
        dst.push_back(*it); 
      }  
           
      
    }
    
  }
  
  
  
}//end namespace

