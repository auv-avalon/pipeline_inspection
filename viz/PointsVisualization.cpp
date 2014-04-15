#include <iostream>
#include "PointsVisualization.hpp"

using namespace vizkit3d;

PointsVisualization::PointsVisualization()
{
  newPoints = false;
}

PointsVisualization::~PointsVisualization()
{
}

osg::ref_ptr<osg::Node> PointsVisualization::createMainNode()
{
  std::cout << "CreateMainNode start" << std::endl;
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    //pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    //pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    std::cout << "CreateMainNode end" << std::endl;
    return mainNode;
}

void PointsVisualization::updateMainNode ( osg::Node* node )
{
  
  std::cout << "updateMainNode" << std::endl;
    if(newPoints){

      // Update the main node using the data in p->data
      
      for(std::vector<base::Vector3d>::iterator it = data.begin(); it != data.end(); it++){
	pointsOSG->push_back(osg::Vec3d(it->x()+ 0.05 , it->y() , it->z() ));
	pointsOSG->push_back(osg::Vec3d(it->x()- 0.05 , it->y() , it->z() ));
	
	pointsOSG->push_back(osg::Vec3d(it->x(), it->y() + 0.05, it->z() ));
	pointsOSG->push_back(osg::Vec3d(it->x(), it->y() - 0.05, it->z() ));
	
	color->push_back(osg::Vec4f(255, 0, 0, 255));
      }
      
      drawArrays->setCount(pointsOSG->size());
      pointGeom->setVertexArray(pointsOSG);
      pointGeom->setColorArray(color);
      
      newPoints = false;
    }
   std::cout << "updateMainNode end" << std::endl; 
}

void PointsVisualization::updateDataIntern(std::vector<base::Vector3d> const& value)
{
    std::cout << "updateDataIntern" << std::endl;
    data = value;
    std::cout << "got new sample data" << std::endl;
    newPoints = true;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(PointsVisualization)

