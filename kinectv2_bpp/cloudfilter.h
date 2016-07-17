#ifndef CLOUDFILTER_H
#define CLOUDFILTER_H

#include "AllHeader.h"

class CloudFilter
{
public:
    CloudFilter();
    void passThrough( PointCloudXYZRGB::Ptr cloud , std::string axis, float min, float max);
    void voxelGrid( PointCloudXYZRGB::Ptr cloud , float leaf);
    void extractIndices( PointCloudXYZRGB::Ptr cloud, pcl::PointIndices::Ptr inliers , bool remove_plane);
    void outlierRemoval(PointCloudXYZRGB::Ptr cloud, int meank, double stddev);
};

#endif // CLOUDFILTER_H
