#ifndef ReadWriteFile_H
#define ReadWriteFile_H

#include "AllHeader.h"


class ReadWriteFile
{
public:
    ReadWriteFile();

	//pass by reference, to return change of value
	void ReadPassthroughFilterValueFile(string filename,
		double &passthrough_xmin, double &passthrough_xmax,
		double &passthrough_ymin, double &passthrough_ymax,
		double &passthrough_zmin, double &passthrough_zmax);

	void WritePassthroughFilterValueFile(string filename,
		double passthrough_xmin, double passthrough_xmax,
		double passthrough_ymin, double passthrough_ymax,
		double passthrough_zmin, double passthrough_zmax);

	//pass by reference, to return change of value
	void ReadPlaneParameterFile(string filename,
		Eigen::Vector3f &plane_center_translate,
		double &plane_first_x_axis_rotate,
		double &plane_second_y_axis_rotate,
		PointTypeXYZRGB &min_point_plane_AABB,
		PointTypeXYZRGB &max_point_plane_AABB);

	void WritePlaneParameterFile(string filename, 
		Eigen::Vector3f plane_center_translate,
		double plane_first_x_axis_rotate,
		double plane_second_y_axis_rotate,
		PointTypeXYZRGB min_point_plane_AABB,
		PointTypeXYZRGB max_point_plane_AABB);

	//pass by address(pointer)
	void ReadPcdFile(string filename, PointCloudXYZRGB::Ptr pointcloud);
	void WritePcdFile(string filename, PointCloudXYZRGB::Ptr pointcloud);

	//pass by reference(&)  and pass by address(*)
	void ReadListofPcdfromTextFile(
		string filename, int &total,
		int &bin_width, int &bin_height, int &bin_depth,
		vector<string> &array_pcd_filename, 
		int *item_w, int *item_h, int *item_d);
	void WriteListofPcdtoTextFile(
		string filename, int total,
		int bin_width, int bin_height, int bin_depth,
		vector<string> array_pcd_filename, 
		int *item_w, int *item_h, int *item_d);



};

#endif // ReadWriteFile_H
