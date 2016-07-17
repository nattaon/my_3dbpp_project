#ifndef CALCULATEBPP_H
#define CALCULATEBPP_H

//#include "../"

class CalculateBPP
{
public:
    CalculateBPP();
	void SetPackingType(int type);
	void SetBinSize(int bin_width, int bin_height, int bin_depth);
	void SetBoxesSize(int total, int *width, int *height, int *depth);
	void SetLimit(int node_limit, int iter_limit, int time_limit);
	void CalculateBinpack(int *x_pos, int *y_pos, int *z_pos, int *bin_num);
	void SortBoxesOrder();
	bool CheckAllCanFit();

private:
	int total_boxes;
	int b_w, b_h, b_d;
	int *w, *h, *d;
	int *x, *y, *z;
	int *bno;
	int ub, lb, solved, gap, sumnode, sumiter;
	double time, sumtime, deviation, sumub, sumlb, sumdev;
	int nodelimit, iterlimit, timelimit;
	int nodeused, iterused, timeused;
	int packingtype;

	void quickSort(int *arr, int left, int right);
	void swap_xyzwhd(int i, int j);
	void swap_pare(int *a, int *b);
	void printboxes_array();


};

#endif // CALCULATEBPP_H