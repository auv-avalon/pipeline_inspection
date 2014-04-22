#ifndef _PIPELINEINSPECTION_DETECTOR_HPP_
#define _PIPELINEINSPECTION_DETECTOR_HPP_

#include <iostream>
#include <vector>
#include <math.h>
#include "DetectorTypes.hpp"
#include "PatternMatching.hpp"
#include <base/samples/LaserScan.hpp>
#include <base/Eigen.hpp>
#include <base/samples/Pointcloud.hpp>
#include <offshore_pipeline_detector/pipeline.h>
#include <boost/circular_buffer.hpp>

namespace pipeline_inspection
{
	class Detector
	{
	public:
	  void init(DetectorCalib c);
	  InspectionStatus inspect(base::samples::LaserScan scan, controlData::Pipeline pipe, base::samples::RigidBodyState pos);
	  InspectionStatus inspect(std::vector<base::Vector3d> point, controlData::Pipeline pipe, base::samples::RigidBodyState pos);
	  
	  /**
	   * converts the point in the cameraframe to the world frame and projects it to the laserplane
	   * @return: the projection error
	   */
	  double camera2LaserFrame(const base::Vector3d &pointC, base::Vector3d &pointL);
	  
	  void laser2VerticalFrame(const base::Vector3d &pointL, base::Vector3d &pointW);
	  
	  std::vector<base::Vector3d> getPipePoints();
	  double getProjectionSSE();
          
          /**
           * Converts the laser points to a pointlcoad map
           * @param relative_map: if true, the positions will be projeted to global origin
           */
          base::samples::Pointcloud getPointcloud(bool relative_map = true);
          
          PipelineState ratePipe(Pattern p);
          
          void filterPoints(const std::vector<base::Vector3d> &src, std::vector<base::Vector3d> &dst, controlData::Pipeline pipe);
	  
	private:
	  DetectorCalib calib;
	  PatternMatching matcher;
	  
	  std::vector<base::Vector3d> pipePoints;
          std::vector<base::Vector3d> filteredPipePoints;
          laserInformation pipeBuffer;
          laserInformation filteredPipeBuffer;
          boost::circular_buffer<InspectionStatus> statusBuffer;
          	  
	  double projectionSSE; //sum of squared projection errors
	};

} // end namespace pipeline_inspection

#endif
