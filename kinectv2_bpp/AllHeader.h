#ifndef ALLHEADER_H
#define ALLHEADER_H

/* call me
#include "AllHeader.h"
*/



#include <Windows.h>
#include <Kinect.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <stdint.h>
#include <string> 

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Point Cloud Library
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h> 

#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>

#include <boost/thread/thread.hpp>




// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Qt
#include <QApplication>
#include <QMainWindow>
#include <QTime>
#include <QLineEdit>
#include <QTreeWidgetItem>
#include <QMessageBox>
#include <QFileDialog>



typedef pcl::PointXYZRGB PointTypeXYZRGB;
typedef pcl::PointCloud<PointTypeXYZRGB> PointCloudXYZRGB;
using namespace std;

#endif