#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/StdVector>
#include <pcl/features/moment_of_inertia_estimation.h>

#define MAXBOXES 20 
#define BINDIM 500 
using namespace std;


typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointXYZ PointXYZ;

typedef pcl::PointXYZRGB MyPointType;

//Declaration
void binpack3d(int n, int W, int H, int D,
               int *w, int *h, int *d,
               int *x, int *y, int *z, int *bno,
               int *lb, int *ub,
               int nodelimit, int iterlimit, int timelimit,
               int *nodeused, int *iterused, int *timeused,
               int packingtype);

int float_scale_to_int(float number);
float int_scale_to_float(int number);
void printboxes_array(int size);

void resetPointCloudPosition( pcl::PointCloud<MyPointType>::Ptr cloud , MyPointType min_point);
void movePointCloudPosition( pcl::PointCloud<MyPointType>::Ptr cloud , float additive_point);
void movePointCloudPosition_xyz( pcl::PointCloud<MyPointType>::Ptr cloud , int x, int y, int z);
void print_cloud_info( pcl::PointCloud<PointXYZRGB>::Ptr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
int readfile_setorigin_savepcd();
int readfile_with_origin_size();

int w[MAXBOXES],h[MAXBOXES],d[MAXBOXES];
int x[MAXBOXES],y[MAXBOXES],z[MAXBOXES];
int bno[MAXBOXES];

	// array of each point cloud
	vector < pcl::PointCloud<MyPointType>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <MyPointType>::Ptr > > sourceClouds;
	

//Definition
int main (int argc, char** argv)
{

	
	int errorcode;
	//errorcode=readfile_setorigin_savepcd();
	errorcode=readfile_with_origin_size();
	if(errorcode!=0) return errorcode;
	
	
	string cloud_name="";
	string cube_name="";
	


	//viewer_bpp = output window screen for showing graphic
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_bpp (new pcl::visualization::PCLVisualizer ("3D viewer_bpp"));
	viewer_bpp->setBackgroundColor (0, 0, 0);
	viewer_bpp->addCoordinateSystem (1.0);
	viewer_bpp->initCameraParameters ();



	//for calculate bounding box
	pcl::MomentOfInertiaEstimation <MyPointType> feature_extractor;
	MyPointType min_point_AABB;
	MyPointType max_point_AABB;


	//for each point cloud in array
	//-get boundingbox size and keep it in array for computing 3dbpp
	//-reposition point cloud to center
	//-reposition it to be arranged
 /*	for (int i = 0; i < sourceClouds.size() ; i++)
   {

		feature_extractor.setInputCloud (sourceClouds[i]);
		feature_extractor.compute();
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);


		cloud_name="cloud"+to_string(i);
		cube_name="AABB"+to_string(i);

		//draw bounding box
		viewer_bpp->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, cube_name);	


		//draw color point cloud
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sourceClouds[i]);
		viewer_bpp->addPointCloud<PointXYZRGB> (sourceClouds[i], rgb, cloud_name);
		viewer_bpp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);

	}
*/
	int packingtype=1;
	int nodelimit=0;
	int iterlimit=0;
	int timelimit=0;

	int nodeused, iterused, timeused;
	int ub, lb;//, solved, gap, sumnode, sumiter;
	//double time, sumtime, deviation, sumub, sumlb, sumdev;

   binpack3d(sourceClouds.size(), BINDIM, BINDIM, BINDIM, w, h, d, x, y, z, bno, &lb, &ub, 
              nodelimit, iterlimit, timelimit, 
              &nodeused, &iterused, &timeused, 
              packingtype);
			  

	printboxes_array(sourceClouds.size());

   
	// to write continue
	// each pointcloud is already reset to be at original position
	// do translate point cloud to calculated position in box
	// boundingbox position
	for (int i = 0; i < sourceClouds.size() ; i++)
   {

	   // translate point cloud to calculated position
		movePointCloudPosition_xyz(sourceClouds[i],x[i],y[i],z[i]);

		//get bounding box position again for draw bounding box
		feature_extractor.setInputCloud (sourceClouds[i]);
		feature_extractor.compute();
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);

		//assign name for keep track draw pointcloud/boundingbox
		cloud_name="cloud"+to_string(i);
		cube_name="AABB"+to_string(i);



		//current_x_width= max_point_AABB.x-min_point_AABB.x;
		cout << "\n pointcloud " << i << endl;
		cout << "PointCloudPosition " << endl;
		cout << "x=" << min_point_AABB.x <<" width  : " << max_point_AABB.x-min_point_AABB.x << endl;
		cout << "y=" << min_point_AABB.y <<" height : " << max_point_AABB.y-min_point_AABB.y << endl;
		cout << "z=" << min_point_AABB.z <<" depth  : " << max_point_AABB.z-min_point_AABB.z << endl;
		
		cout << "addcloud : " <<  cloud_name << endl;
		cout << "addcube : " <<  cube_name << endl;

		//draw bounding box
		viewer_bpp->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, cube_name);	


		//draw color point cloud
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(sourceClouds[i]);
		viewer_bpp->addPointCloud<PointXYZRGB> (sourceClouds[i], rgb, cloud_name);
		viewer_bpp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);


   }




	viewer_bpp->setCameraClipDistances(0.003, 3.0);
	
	//setCameraPosition(position, focalpoint, viewup)
	viewer_bpp->setCameraPosition( 1.0, 1.0, 0.7,
								0.3, 0.3, 0.3,
								0.0, 0.0, 1.0);
	//draw container
	viewer_bpp->addCube (0, int_scale_to_float(BINDIM), 0, int_scale_to_float(BINDIM), 0, int_scale_to_float(BINDIM), 1.0, 0.0, 0.0, "container");	
		
	//viewer_bpp->updateCamera();

	while (!viewer_bpp->wasStopped ()) // necessary for drawing things in viewer_bpp
	{
		viewer_bpp->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
	cin.get();

  return (0);
}
int float_scale_to_int(float number)
{
	return (int)(number*1000);
}
float int_scale_to_float(int number)
{
	float value=number*0.001;
	return value;
}

void printboxes_array(int size)
{
  printf("\nprint box array\n");
	for (int i = 0; i < size; i++) {
    printf("%2d (%2d %2d %2d) : Bin %2d (%2d, %2d, %2d)\n",
           i,w[i],h[i],d[i],bno[i],x[i],y[i],z[i]);
  }

}


void resetPointCloudPosition( pcl::PointCloud<MyPointType>::Ptr cloud , MyPointType min_point)
{

   for( int i=0; i < cloud->points.size(); i++){
        cloud->points[i].x-=min_point.x;
        cloud->points[i].y-=min_point.y;
        cloud->points[i].z-=min_point.z;
    }
	

	//return pointcloud
	//pcl::copyPointCloud( *cloud_filtered, *cloud );

}
void movePointCloudPosition( pcl::PointCloud<MyPointType>::Ptr cloud , float additive_point)
{
	if(additive_point==0) return;

   for( int i=0; i < cloud->points.size(); i++){
        cloud->points[i].x+=additive_point;
        //cloud->points[i].y-=min_point.y;
        //cloud->points[i].z-=min_point.z;
    }

}
void movePointCloudPosition_xyz( pcl::PointCloud<MyPointType>::Ptr cloud , int x, int y, int z)
{

	//if(x==y==z==0) return;

   for( int i=0; i < cloud->points.size(); i++){
        cloud->points[i].x+=int_scale_to_float(x);
        cloud->points[i].y+=int_scale_to_float(y);
        cloud->points[i].z+=int_scale_to_float(z);
    }
}

void print_cloud_info( pcl::PointCloud<PointXYZRGB>::Ptr cloud)
{
  for (size_t i = 0; i < cloud->points.size (); ++i){
    uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;
    cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z
              << " "    << (int)r
              << " "    << (int)g
              << " "    << (int)b
             // << " "    << cloud->points[i].imX
             // << " "    << cloud->points[i].imY 
			  << std::endl;
  }
  
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_bpp (new pcl::visualization::PCLVisualizer ("3D viewer_bpp"));
	viewer_bpp->setBackgroundColor (0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer_bpp->addPointCloud<PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer_bpp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer_bpp->addCoordinateSystem (1.0);
	viewer_bpp->initCameraParameters ();
	return (viewer_bpp);
}


int readfile_setorigin_savepcd()
{
	//read file
	//load point cloud from filename in each line
	//load point cloud into array of point cloud


	// open file for read input point cloud name
	ifstream infile;
	infile.open("input.txt");

	ofstream outfile;
	outfile.open ("output.txt");

	string sLine = ""; //for read each line in file
	string filename = "";

	//for calculate bounding box
	pcl::MomentOfInertiaEstimation <MyPointType> feature_extractor;
	MyPointType min_point_AABB;
	MyPointType max_point_AABB;

	int index=0;
	while (!infile.eof())
	{
		 

		getline(infile, sLine); //cout << sLine << endl;
		
        pcl::PointCloud<MyPointType>::Ptr sourceCloud(new pcl::PointCloud<MyPointType>);
        if (pcl::io::loadPCDFile<MyPointType>(sLine, *sourceCloud) != 0)
        {
            return -1;
        }
		int point=sLine.find_last_of("\\");
		filename="origin_"+sLine.substr(point+1);

        //cout << "Loaded file " << sLine << " (" << sourceCloud->size() << " points)" << endl;
        
		//push loaded point cloud into array
		sourceClouds.push_back(sourceCloud);
		// sourceCloud->clear();


		//get bounding box position for translate it to origin
		feature_extractor.setInputCloud (sourceClouds[index]);
		feature_extractor.compute();
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);

		//use current point cloud position to reset it to origin
		resetPointCloudPosition(sourceClouds[index],min_point_AABB);

		//keep value of w*h*d in to array for compute 3dbpp
		w[index]=float_scale_to_int(max_point_AABB.x-min_point_AABB.x);
		h[index]=float_scale_to_int(max_point_AABB.y-min_point_AABB.y);
		d[index]=float_scale_to_int(max_point_AABB.z-min_point_AABB.z);

		outfile << filename << " "
				<< max_point_AABB.x-min_point_AABB.x << " " 
				<< max_point_AABB.y-min_point_AABB.y << " " 
				<< max_point_AABB.z-min_point_AABB.z << endl;

		pcl::io::savePCDFile(filename,*sourceClouds[index]);

		index++;
	


	}
	infile.close();
	outfile.close();


	return 0;

}

int readfile_with_origin_size()
{
	//read file
	//load point cloud from filename in each line
	//load point cloud into array of point cloud


	// open file for read input point cloud name
	ifstream infile;
	infile.open("input.txt");


	string sLine =""; //for read each line in file
	string filename="";
	string width,height,depth;

	int index=0;
	int end_name_pos,end_w_pos,end_h_pos,end_d_pos;
	while (!infile.eof())
	{
		 

		getline(infile, sLine); //cout << sLine << endl;
		
        
		

		end_name_pos=sLine.find(" ");
		filename=sLine.substr(0,end_name_pos);
		sLine=sLine.substr(end_name_pos+1);

		end_w_pos=sLine.find(" ");
		width=sLine.substr(0,end_w_pos);
		sLine=sLine.substr(end_w_pos+1);

		end_h_pos=sLine.find(" ");
		height=sLine.substr(0,end_h_pos);
		depth=sLine.substr(end_h_pos+1);

/*		cout << filename << "*" << endl
			 << width << "*" << endl
			 << height << "*" << endl
			 << depth << "*" << endl;
*/


		

		pcl::PointCloud<MyPointType>::Ptr sourceCloud(new pcl::PointCloud<MyPointType>);
        if (pcl::io::loadPCDFile<MyPointType>(filename, *sourceCloud) != 0)
        {
            return -1;
        }
		


       // cout << "Loaded file " << sLine << " (" << sourceCloud->size() << " points)" << endl;
        
		//push loaded point cloud into array
		sourceClouds.push_back(sourceCloud);
		// sourceCloud->clear();

		//keep value of w*h*d in to array for compute 3dbpp
		w[index]=float_scale_to_int(stod(width));
		h[index]=float_scale_to_int(stod(height));
		d[index]=float_scale_to_int(stod(depth));

		index++;
	


	}
	infile.close();



	return 0;
}