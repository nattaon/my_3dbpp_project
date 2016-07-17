#ifndef READWRITEFILE_H
#define READWRITEFILE_H
#include<string>
using namespace std;
class ReadWriteFile
{
public:
    ReadWriteFile();
	void OpenTxtFileBoxes(string filename);
	void SaveTxtFileBoxes(string filename, int total,
		int bin_width, int bin_height, int bin_depth,
		int *width, int *height, int *depth);
	int GetTotalBox();
	int GetBinWidth();
	int GetBinHeight();
	int GetBinDepth();
	int *GetBoxesWidth();
	int *GetBoxesHeight();
	int *GetBoxesDepth();


private:
	int total_boxes;
	int bin_w, bin_h, bin_d;
	int *boxes_w, *boxes_h, *boxes_d;



};

#endif // READWRITEFILE_H