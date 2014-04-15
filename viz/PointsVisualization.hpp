#ifndef pipeline_inspection_PointsVisualization_H
#define pipeline_inspection_PointsVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Node>
#include <osg/Geometry>
#include <base/Eigen.hpp>

namespace vizkit3d
{
    class PointsVisualization
        : public vizkit3d::Vizkit3DPlugin<std::vector<base::Vector3d> >
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        PointsVisualization();
        ~PointsVisualization();

    Q_INVOKABLE void updateData(std::vector<base::Vector3d> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<base::Vector3d> >::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<base::Vector3d> const& plan);
        
    private:
	  std::vector<base::Vector3d> data;
	  std::vector< base::Vector3d > channelInfos;
	  osg::ref_ptr<osg::Vec3Array> pointsOSG;
	  osg::ref_ptr<osg::DrawArrays> drawArrays;
	  osg::ref_ptr<osg::Geometry> pointGeom;
	  osg::ref_ptr<osg::Vec4Array> color;
	  
	  bool newPoints;
    };
}
#endif
