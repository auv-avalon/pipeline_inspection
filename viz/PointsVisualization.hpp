#ifndef pipeline_inspection_PointsVisualization_H
#define pipeline_inspection_PointsVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/Eigen.hpp>

namespace vizkit3d
{
    class PointsVisualization
        : public vizkit3d::Vizkit3DPlugin<std::vector<base::Vector3d>>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        PointsVisualization();
        ~PointsVisualization();

    Q_INVOKABLE void updateData(std::vector<base::Vector3d> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<base::Vector3d>>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<base::Vector3d> const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
