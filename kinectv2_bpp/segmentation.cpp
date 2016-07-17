#include "segmentation.h"
#include "miscellaneous.h"
using namespace std;
Segmentation::Segmentation()
{

}

pcl::PointIndices::Ptr Segmentation::planeSegmentation( PointCloudXYZRGB::Ptr& cloud ,double threshold, int iscolored)
{
	//cout << "thres=" << threshold << endl;

    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );

    // Create the segmentation object
    pcl::SACSegmentation<PointTypeXYZRGB> seg;
    seg.setOptimizeCoefficients( true );
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setMethodType( pcl::SAC_RANSAC );
    seg.setDistanceThreshold( threshold );

    seg.setInputCloud( cloud );
    seg.segment( *inliers, *coefficients);


	//cout << "coefficients : " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << endl;

    if(iscolored==1)
    {
       // color plane to red
        for( size_t i=0; i < inliers->indices.size(); i++){
            cloud->points[inliers->indices[i]].r=255;
            cloud->points[inliers->indices[i]].g=0;
            cloud->points[inliers->indices[i]].b=0;
        }
    }


    return inliers;
}


pcl::ModelCoefficients::Ptr Segmentation::extractPlaneCoeff( PointCloudXYZRGB::Ptr& cloud ,double threshold)
{
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );

    // Create the segmentation object
    pcl::SACSegmentation<PointTypeXYZRGB> seg;
    seg.setOptimizeCoefficients( true );
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setMethodType( pcl::SAC_RANSAC );
    seg.setDistanceThreshold( threshold );

    seg.setInputCloud( cloud );
    seg.segment( *inliers, *coefficients);


    return coefficients;
}
void Segmentation::ClusterExtractOneItem(
	PointCloudXYZRGB::Ptr cloud,
	double cluster_tolerance,
	int cluster_min_size,
	int cluster_max_size)
{
	//test if can call parent or not

	//write cloud array in parent


	// Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointTypeXYZRGB>::Ptr tree (new pcl::search::KdTree<PointTypeXYZRGB>);
    tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> vector_cluster_indices;

    vector_cluster_indices.clear();
    pcl::EuclideanClusterExtraction<PointTypeXYZRGB> ec;
	ec.setClusterTolerance(cluster_tolerance); //0.02=2cm   //0.005
	ec.setMinClusterSize(cluster_min_size);//tmp->size()*0.2
	ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (vector_cluster_indices);

	cout << "cluster total  " << vector_cluster_indices.size() <<endl;

	for (std::vector<pcl::PointIndices>::const_iterator it = vector_cluster_indices.begin(); it != vector_cluster_indices.end(); ++it)
	{
		PointCloudXYZRGB::Ptr cloud_cluster(new PointCloudXYZRGB);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}
		pcl::copyPointCloud(*cloud_cluster, *cloud);
		break;

	}
}

std::vector<PointCloudXYZRGB::Ptr> Segmentation::ClusterExtractItems(
	PointCloudXYZRGB::Ptr cloud,
	double cluster_tolerance,
	int cluster_min_size,
	int cluster_max_size)
{
	Miscellaneous *mis = new Miscellaneous();
	int color;
	PointCloudXYZRGB::Ptr cloud_no_color(new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud, *cloud_no_color);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointTypeXYZRGB>::Ptr tree(new pcl::search::KdTree<PointTypeXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> vector_cluster_indices;

	vector_cluster_indices.clear();
	pcl::EuclideanClusterExtraction<PointTypeXYZRGB> ec;
	ec.setClusterTolerance(cluster_tolerance); //0.02=2cm   //0.005
	ec.setMinClusterSize(cluster_min_size);//tmp->size()*0.2
	ec.setMaxClusterSize(cluster_max_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(vector_cluster_indices);

	cout << "cluster total  " << vector_cluster_indices.size() << endl;
	
	std::vector<PointCloudXYZRGB::Ptr> array_cluster_cloud;
	cout << "array_cluster_cloud size" << array_cluster_cloud.size() << endl;

    for (std::vector<pcl::PointIndices>::const_iterator it = vector_cluster_indices.begin (); it != vector_cluster_indices.end (); ++it)
    {

        PointCloudXYZRGB::Ptr cloud_cluster (new PointCloudXYZRGB);
		color=mis->randomcolor();
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			//cout << ' ' << *pit << endl;
			//if (*pit % 4 == 0)
			
			cloud->points[*pit].rgb = color;
			cloud_cluster->points.push_back(cloud_no_color->points[*pit]);
			
		}
		//pcl::copyPointCloud(*cloud_cluster, *cloud);
		array_cluster_cloud.push_back(cloud_cluster);
        
    }
	cout << "array_cluster_cloud size" << array_cluster_cloud.size() << endl;

	return array_cluster_cloud;
}

void Segmentation::cluster_boundingbox( PointCloudXYZRGB::Ptr cloud )
{
	/*
    pcl::MomentOfInertiaEstimation <PointTypeXYZRGB> feature_extractor;


  PointTypeXYZRGB min_point_OBB;
  PointTypeXYZRGB max_point_OBB;
  PointTypeXYZRGB position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

    std::string cubename="";

    for (int i=0; i <sourceClouds.size() ; i++)
    {



        feature_extractor.setInputCloud (sourceClouds[i]);

        feature_extractor.compute();
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        //feature_extractor.getMassCenter (mass_center);


        cubename="cube_"+std::to_string(i+1);

        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);
        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, cubename);

        //bpp
        resetPointCloudPosition(sourceClouds[i],min_point_OBB);//reset orientation??
        //assign value of width-height-depth to binpacking
        w[i]=float_scale_to_int(max_point_OBB.x-min_point_OBB.x);
        h[i]=float_scale_to_int(max_point_OBB.y-min_point_OBB.y);
        d[i]=float_scale_to_int(max_point_OBB.z-min_point_OBB.z);
    }
	*/

}

void cluster_binpacking( PointCloudXYZRGB::Ptr cloud )
{
	/*

    //binpacking
    int packingtype=1;
    int nodelimit=0;
    int iterlimit=0;
    int timelimit=0;

    int nodeused, iterused, timeused;
    int ub, lb;//, solved, gap, sumnode, sumiter;
    //double time, sumtime, deviation, sumub, sumlb, sumdev;
    printboxes_array(sourceClouds.size(), w, h, d, bno, x, y, z);

    binpack3d(sourceClouds.size(), BINDIM, BINDIM, BINDIM, w, h, d, x, y, z, bno, &lb, &ub,
                nodelimit, iterlimit, timelimit,
                &nodeused, &iterused, &timeused,
                packingtype);

    printboxes_array(sourceClouds.size(), w, h, d, bno, x, y, z);




    viewer2->removeAllPointClouds();
    viewer2->addCube (0, BINDIM*0.001, 0, BINDIM*0.001, 0, BINDIM*0.001, 1.0, 0.0, 0.0, "bin");
*/




/*	for (int i = 0; i < sourceClouds.size() ; i++)
    {
        sourceClouds[i].
        movePointCloudPosition_xyz(sourceClouds[i],x[i],y[i],z[i]);

        //assign name for keep track draw pointcloud/boundingbox
        std::string cloud_name="cloud"+std::to_string(i);
        std::string cube_name="AABB"+std::to_string(i);

        cout << "\naddcloud : " <<  cloud_name << endl;
        cout << "addcube : " <<  cube_name << endl;
        cout << sourceClouds[i] << endl;

        //draw bounding box
        //viewer2->addCube (min_AABB.x, max_AABB.x, min_AABB.y, max_AABB.y, min_AABB.z, max_AABB.z, 1.0, 1.0, 0.0, cube_name);

        //draw color point cloud
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sourceClouds[i]);
        viewer2->addPointCloud<pcl::PointXYZRGB> (sourceClouds[i], rgb, cloud_name);
        viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);

    }
*/

}

