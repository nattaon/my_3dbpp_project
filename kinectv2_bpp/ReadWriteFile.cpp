#include "ReadWriteFile.h"

ReadWriteFile::ReadWriteFile()
{

}

void ReadWriteFile::ReadPassthroughFilterValueFile(string filename,
	double &passthrough_xmin, double &passthrough_xmax,
	double &passthrough_ymin, double &passthrough_ymax,
	double &passthrough_zmin, double &passthrough_zmax)
{//pass by reference, to return change of value
	
	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return;
	}

	string sLine;
	int index1 = 0;
	int index2 = 0;

	//read : max min
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	passthrough_xmax = stod(sLine.substr(0, index1));
	passthrough_xmin = stod(sLine.substr(index1));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	passthrough_ymax = stod(sLine.substr(0, index1));
	passthrough_ymin = stod(sLine.substr(index1));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	passthrough_zmax = stod(sLine.substr(0, index1));
	passthrough_zmin = stod(sLine.substr(index1));
	/*
	cout
		<< passthrough_xmax << " "
		<< passthrough_xmin << endl

		<< passthrough_ymax << " "
		<< passthrough_ymin << endl

		<< passthrough_zmax << " "
		<< passthrough_zmin << endl;
*/


	infile.close();
}
void ReadWriteFile::WritePassthroughFilterValueFile(string filename,
	double passthrough_xmin, double passthrough_xmax,
	double passthrough_ymin, double passthrough_ymax,
	double passthrough_zmin, double passthrough_zmax)
{//pass by reference, to return change of value
	//cout << "WritePassthroughFilterValueFile" << endl;

	ofstream outfile;
	outfile.open(filename);
	
	//print : max min
	outfile
		<< passthrough_xmax << " "
		<< passthrough_xmin << endl

		<< passthrough_ymax << " "
		<< passthrough_ymin << endl

		<< passthrough_zmax << " "
		<< passthrough_zmin << endl;

	outfile.close();
}

void ReadWriteFile::ReadPlaneParameterFile(string filename,
	Eigen::Vector3f &plane_center_translate,
	double &plane_first_x_axis_rotate,
	double &plane_second_y_axis_rotate,
	PointTypeXYZRGB &min_point_plane_AABB,
	PointTypeXYZRGB &max_point_plane_AABB)
{

	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return;
	}

	string sLine;
	int index1 = 0;
	int index2 = 0;

	//1.get plane_center_translate
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");

	plane_center_translate[0] = stod(sLine.substr(0, index1));
	plane_center_translate[1] = stod(sLine.substr(index1, index2));
	plane_center_translate[2] = stod(sLine.substr(index2));

	//2.get plane_first_x_axis_rotate
	getline(infile, sLine);
	plane_first_x_axis_rotate = stod(sLine);

	//3.get plane_second_y_axis_rotate
	getline(infile, sLine);
	plane_second_y_axis_rotate = stod(sLine);

	//4.get min_point_plane_AABB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");

	min_point_plane_AABB.x = stod(sLine.substr(0, index1));
	min_point_plane_AABB.y = stod(sLine.substr(index1, index2));
	min_point_plane_AABB.z = stod(sLine.substr(index2));

	//5.get max_point_plane_AABB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");

	max_point_plane_AABB.x = stod(sLine.substr(0, index1));
	max_point_plane_AABB.y = stod(sLine.substr(index1, index2));
	max_point_plane_AABB.z = stod(sLine.substr(index2));

	infile.close();


}
void ReadWriteFile::WritePlaneParameterFile(string filename,
	Eigen::Vector3f plane_center_translate,
	double plane_first_x_axis_rotate,
	double plane_second_y_axis_rotate,
	PointTypeXYZRGB min_point_plane_AABB,
	PointTypeXYZRGB max_point_plane_AABB)
{
	//cout << "WritePlaneParameterFile" << endl;

	ofstream outfile;
	outfile.open(filename);

	outfile
		<< plane_center_translate[0] << " "
		<< plane_center_translate[1] << " "
		<< plane_center_translate[2] << endl

		<< plane_first_x_axis_rotate << " " << endl

		<< plane_second_y_axis_rotate << " " << endl

		<< min_point_plane_AABB.x << " "
		<< min_point_plane_AABB.y << " "
		<< min_point_plane_AABB.z << endl

		<< max_point_plane_AABB.x << " "
		<< max_point_plane_AABB.y << " "
		<< max_point_plane_AABB.z << endl;

	outfile.close();
}



void ReadWriteFile::ReadPcdFile(string filename, PointCloudXYZRGB::Ptr pointcloud)
{
	//cout << "ReadPcdFile filename=" << filename << endl;	

	if (pcl::io::loadPCDFile<PointTypeXYZRGB>(filename, *pointcloud) == -1) 
	{	//read .pcd and write value in to *pointcloud,
		//(*) is dereference operator it get value from the address)
		PCL_ERROR("Couldn't read file pcd \n");
		//return (-1);
	}


	//cout << "pointcloud=" << pointcloud->points.size() << endl;
	/*
	for (int i = 0; i < pointcloud->points.size(); i+=100)
	cout
	<< " " << pointcloud->points[i].x
	<< " " << pointcloud->points[i].y
	<< " " << pointcloud->points[i].z << endl;
	*/

}
void ReadWriteFile::WritePcdFile(string filename, PointCloudXYZRGB::Ptr pointcloud)
{
	//pcl::io::WritePcdFileFileBinary 

	//cout << "filename=" << filename << endl;
	//cout << "pointcloud=" << pointcloud->points.size() << endl;

/*	for (int i = 0; i < pointcloud->points.size(); i+=100)
	cout
	<< " " << pointcloud->points[i].x
	<< " " << pointcloud->points[i].y
	<< " " << pointcloud->points[i].z << endl;
*/
	pointcloud->width = 1;
	pointcloud->height = pointcloud->points.size();

	pcl::io::savePCDFile(filename, *pointcloud);
	//write value in to *pointcloud,
	//(*) is dereference operator it get value from the address)
}


//pass by reference(&) 
void ReadWriteFile::ReadListofPcdfromTextFile(string filename, int &total,
	int &bin_width, int &bin_height, int &bin_depth,
	vector<string> &array_pcd_filename,
	int *item_w, int *item_h, int *item_d)
{
	cout << "ReadListofPcdfromTextFile" << endl;

	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return;
	}

	string sLine;

	//1.get bin size
	getline(infile, sLine);
	int index1 = 0;
	int index2 = 0;

	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");

	bin_width = stod(sLine.substr(0, index1));
	bin_height = stod(sLine.substr(index1, index2));
	bin_depth = stod(sLine.substr(index2));

	//2.get items total
	getline(infile, sLine);
	total = stod(sLine);


	//item_w = new int[total];
	//item_h = new int[total];
	//item_d = new int[total];


	//3.get array of pcd filename and item dimension
	for (int i = 0; i < total; i++)
	{
		getline(infile, sLine);
		array_pcd_filename.push_back(string(sLine));//pcd file name

		getline(infile, sLine);
		index1 = sLine.find(" ");
		index2 = sLine.find_last_of(" ");

		Eigen::Vector3f dimension;

		item_w[i] = stod(sLine.substr(0, index1));
		item_h[i] = stod(sLine.substr(index1, index2));
		item_d[i] = stod(sLine.substr(index2));


	}

	//cout << "item_w " << item_w << endl;
	//cout << "item_h " << item_h << endl;
	//cout << "item_d " << item_d << endl;



}

void ReadWriteFile::WriteListofPcdtoTextFile(string filename, int total,
	int bin_width, int bin_height, int bin_depth,
	vector<string> array_pcd_filename, 
	int *item_w, int *item_h, int *item_d)
{

	cout << "WriteListofPcdtoTextFile" << endl;
	
	ofstream outfile;
	outfile.open(filename);

	outfile
		<< bin_width << " " << bin_height << " " << bin_depth << endl
		<< total << endl;

	for (int i = 0; i < total; i++)
	{
		outfile << array_pcd_filename[i] << endl;
		outfile << item_w[i] << " "
				<< item_h[i] << " "
				<< item_d[i] << endl;
	}
	
	outfile.close();

}

