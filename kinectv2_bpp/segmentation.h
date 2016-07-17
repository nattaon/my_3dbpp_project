#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "AllHeader.h"

class Segmentation
{
public:
    Segmentation();
    pcl::PointIndices::Ptr planeSegmentation(
            PointCloudXYZRGB::Ptr& cloud ,
            double threshold, int iscolored);

    pcl::ModelCoefficients::Ptr extractPlaneCoeff(
            PointCloudXYZRGB::Ptr& cloud ,
            double threshold);
	
	void ClusterExtractOneItem(
		PointCloudXYZRGB::Ptr cloud,
		double cluster_tolerance,
		int cluster_min_size,
		int cluster_max_size);


	std::vector<PointCloudXYZRGB::Ptr> ClusterExtractItems(
		PointCloudXYZRGB::Ptr cloud,
		double cluster_tolerance,
		int cluster_min_size,
		int cluster_max_size);

	void cluster_boundingbox(PointCloudXYZRGB::Ptr cloud);


};

#endif // SEGMENTATION_H
