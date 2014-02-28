#include <iostream>
#include "PointsVisualization.hpp"

using namespace vizkit3d;

struct PointsVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::vector<base::Vector3d> data;
};


PointsVisualization::PointsVisualization()
    : p(new Data)
{
}

PointsVisualization::~PointsVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> PointsVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void PointsVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void PointsVisualization::updateDataIntern(std::vector<base::Vector3d> const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(PointsVisualization)

