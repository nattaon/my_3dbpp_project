#include "cloudfilter.h"

CloudFilter::CloudFilter()
{

}

void CloudFilter::passThrough( PointCloudXYZRGB::Ptr cloud , std::string axis, float min, float max)
{
	if (max = 99 || min==99) return;

    PointCloudXYZRGB::Ptr cloud_filtered( new PointCloudXYZRGB );

    pcl::PassThrough<PointTypeXYZRGB> pass;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (min, max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
	//pcl::console::print_info("cloud_filtered sized : %d\n", cloud_filtered->size());

	// cloud_filtered->size()==0 will cause error "vector subscript out of range"
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}


}
void CloudFilter::voxelGrid( PointCloudXYZRGB::Ptr cloud , float leaf)
{
    //show number of pointcloud before apply filter
    //pcl::console::print_info( "before point clouds : %d\n", cloud->size() );

    //do filter
    //since kinectfusion use kinect camera coordinate
    //in case of  0.01m will be filter as 1 cm


    pcl::VoxelGrid<PointTypeXYZRGB> grid;

    //set scope for filter
    grid.setLeafSize( leaf, leaf, leaf );

    //set cloud to filter
    grid.setInputCloud( cloud );

    //do filter(save output in new pointcloud)
    PointCloudXYZRGB::Ptr cloud_filtered( new PointCloudXYZRGB );
    grid.filter( *cloud_filtered );

    //return pointcloud
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}

    //show number of pointclound after apply filter
    //pcl::console::print_info( "after point clouds : %d\n", cloud->size() );
}

void CloudFilter::extractIndices( PointCloudXYZRGB::Ptr cloud, pcl::PointIndices::Ptr inliers , bool remove_plane)
{
    PointCloudXYZRGB::Ptr tmp( new PointCloudXYZRGB );
    pcl::copyPointCloud( *cloud, *tmp );

    //filtering
    pcl::ExtractIndices<PointTypeXYZRGB> extract;
    extract.setInputCloud( tmp );
    extract.setIndices( inliers );

    //true:remove plane, flase:remove environment
    extract.setNegative( remove_plane );
    extract.filter( *cloud );

    //pcl::console::print_info( "after point clouds : %d\n", cloud->size() );

}

void CloudFilter::outlierRemoval(PointCloudXYZRGB::Ptr cloud, int meank, double stddev)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointTypeXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meank);//The number of neighbors to analyze for each point
    sor.setStddevMulThresh (stddev);//the standard deviation multiplier

    PointCloudXYZRGB::Ptr cloud_filtered( new PointCloudXYZRGB );
    sor.filter (*cloud_filtered);
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}
    //pcl::console::print_info( "after point clouds : %d\n", cloud->size() );
}
