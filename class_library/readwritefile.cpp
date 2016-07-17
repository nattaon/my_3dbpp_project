#include "readwritefile.h"
#include <fstream>
#include <iostream>

ReadWriteFile::ReadWriteFile()
{

}
void ReadWriteFile::OpenTxtFileBoxes(string filename)
{
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

	bin_w = stod(sLine.substr(0, index1));
	bin_h = stod(sLine.substr(index1, index2));
	bin_d = stod(sLine.substr(index2));

	//2.get boxes total
	getline(infile, sLine);
	total_boxes=stod(sLine);


	//3.get array of boxes
	int *boxes_width = new int[total_boxes];
	int *boxes_height = new int[total_boxes];
	int *boxes_depth = new int[total_boxes];



	int c_width, c_height, c_depth;

	for (int i = 0; i < total_boxes; i++)
	{
		getline(infile, sLine);

		//cout << "sLine=" << sLine << "*" << endl;

		index1 = sLine.find(" ");
		index2 = sLine.find_last_of(" ");

		c_width = stod(sLine.substr(0, index1));
		c_height = stod(sLine.substr(index1, index2));
		c_depth = stod(sLine.substr(index2));

		//cout << i << ":" << c_width << "," << c_height << "," << c_depth << endl;

		boxes_width[i] = c_width;
		boxes_height[i] = c_height;
		boxes_depth[i] = c_depth;

	}
	boxes_w = boxes_width; //save value in this class parameter
	boxes_h = boxes_height;
	boxes_d = boxes_depth;


}
void ReadWriteFile::SaveTxtFileBoxes(string filename, int total,
	int bin_width, int bin_height, int bin_depth,
	int *width, int *height, int *depth)
{
	cout << "SaveTxtFileBoxes" << endl;
	
	ofstream outfile;
	outfile.open(filename);

	outfile
		<< bin_width << " " << bin_height << " " << bin_depth << endl
		<< total << endl;

	for (int i = 0; i < total; i++)
	{
		outfile << width[i] << " " << height[i] << " " << depth[i] << endl;
	}


	outfile.close();
}

int ReadWriteFile::GetTotalBox()
{
	return total_boxes;
}
int ReadWriteFile::GetBinWidth()
{
	return bin_w;
}
int ReadWriteFile::GetBinHeight()
{
	return bin_h;
}
int ReadWriteFile::GetBinDepth()
{
	return bin_d;
}

int *ReadWriteFile::GetBoxesWidth()
{
	return boxes_w;
}
int *ReadWriteFile::GetBoxesHeight()
{
	return boxes_h;
}
int *ReadWriteFile::GetBoxesDepth()
{
	return boxes_d;
}
