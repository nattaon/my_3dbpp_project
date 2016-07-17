#ifndef BinpackingCalculate_H
#define BinpackingCalculate_H

#include "AllHeader.h"

class BinpackingCalculate
{
public:
    BinpackingCalculate();
	void SetPackingType(int type);
	void SetBinSize(int bin_width, int bin_height, int bin_depth);
	void SetBoxesSize(int total, int *width, int *height, int *depth);
	void SetLimit(int node_limit, int iter_limit, int time_limit);
	void CalculateBinpack(int *new_order, int *x_pos, int *y_pos, int *z_pos, int *bin_num);
	bool CheckAllCanFit();
	void PrintArrayValue();
	void SortMaxtoMin(int *arr, int left, int right);
	void SortMintoMax(int *arr, int left, int right);
	void SwapArrayIndex(int i, int j);
	void SwapEachValue(int *a, int *b);


private:
	int total_boxes;
	int *item_order;
	int b_w, b_h, b_d;
	int *w, *h, *d;
	int *x, *y, *z;
	int *bno;
	int ub, lb, solved, gap, sumnode, sumiter;
	double time, sumtime, deviation, sumub, sumlb, sumdev;
	int nodelimit, iterlimit, timelimit;
	int nodeused, iterused, timeused;
	int packingtype;



};

#endif // BinpackingCalculate_H