#include "calculatebpp.h"
#include <iostream>
void binpack3d(int n, int W, int H, int D,
	int *w, int *h, int *d,
	int *x, int *y, int *z, int *bno,
	int *lb, int *ub,
	int nodelimit, int iterlimit, int timelimit,
	int *nodeused, int *iterused, int *timeused,
	int packingtype);

CalculateBPP::CalculateBPP()
{

}
void CalculateBPP::SetPackingType(int type)
{
	packingtype = type;
}
void CalculateBPP::SetBinSize(int bin_width, int bin_height, int bin_depth)
{
	b_w = bin_width;
	b_h = bin_height;
	b_d = bin_depth;

}
void CalculateBPP::SetBoxesSize(int total, int *width, int *height, int *depth)
{
	total_boxes = total;
	w = width;
	h = height;
	d = depth;

}
void CalculateBPP::SetLimit(int node_limit, int iter_limit, int time_limit)
{
	nodelimit = node_limit;
	iterlimit = iter_limit;
	timelimit = time_limit;
}
void CalculateBPP::CalculateBinpack(int *x_pos, int *y_pos, int *z_pos, int *bin_num)
{
	x = x_pos;
	y = y_pos;
	z = z_pos;
	bno = bin_num;

	binpack3d(total_boxes, 
		b_w, b_h, b_d, 
		w, h, d, 
		x, y, z, bno, &lb, &ub,
		nodelimit, iterlimit, timelimit,
		&nodeused, &iterused, &timeused,
		packingtype);

	std::cout << "nodeused = " << nodeused << std::endl;
	std::cout << "iterused = " << iterused << std::endl;
	std::cout << "timeused = " << timeused << std::endl;

	//printboxes_array();
}

void CalculateBPP::SortBoxesOrder()
{
	//first, sort by z position
	quickSort(z, 0, total_boxes - 1);

	//find each same z range
	int z_start_index = 0;
	int x_start_index = 0;
	for (int i = z_start_index; i<total_boxes; i++)
	{
		if (z[i] != z[i + 1])
		{
			quickSort(x, z_start_index, i);
			//printboxes_array();
			x_start_index = z_start_index;
			for (int j = x_start_index; j<i + 1; j++)
			{
				if (x[j] != x[j + 1])
				{
					quickSort(y, x_start_index, j);
					//printboxes_array();

					x_start_index = j + 1;
				}
			}
			z_start_index = i + 1;
		}
	}
}

/*
void sort_box_order(int n, int *w, int *h, int *d, int *x, int *y, int *z, int *bno)
{

	//next sort by x

	//last sort by y

	//quickSort(y,0,n-1);
	//quickSort(x,0,n-1);

}
*/


void CalculateBPP::quickSort(int *arr, int left, int right) //int *arr = int arr[]
{

	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] < pivot)
			i++;
		while (arr[j] > pivot)
			j--;

		if (i <= j) {

			swap_xyzwhd(i, j);
			//tmp = arr[i];
			//arr[i] = arr[j];
			//arr[j] = tmp;
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		quickSort(arr, left, j);
	if (i < right)
		quickSort(arr, i, right);

}

void CalculateBPP::swap_xyzwhd(int i, int j)
{
	swap_pare(&x[i], &x[j]);
	swap_pare(&y[i], &y[j]);
	swap_pare(&z[i], &z[j]);
	swap_pare(&w[i], &w[j]);
	swap_pare(&h[i], &h[j]);
	swap_pare(&d[i], &d[j]);

}
void CalculateBPP::swap_pare(int *a, int *b)
{
	int tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}



void CalculateBPP::printboxes_array()
{
	std::cout << "print box array" << std::endl;
	for (int i = 0; i < total_boxes; i++) {
		std::cout
			<< i << ":"
			<< w[i] << " " << h[i] << " " << d[i] << " "
			<< "bin_num:" << bno[i] << " "
			<< "pos:"
			<< x[i] << " " << y[i] << " " << z[i] << " "
			<< std::endl;
	}

}

bool CalculateBPP::CheckAllCanFit()
{
	int boxes_volumn = 0;
	int bin_volumn = b_w*b_h*b_d;
	for (int i = 0; i < total_boxes; i++)
	{
		boxes_volumn += (w[i]*h[i]*d[i]);
	}

	std::cout << "boxes:" << boxes_volumn << std::endl;
	std::cout << "bin:" << bin_volumn << std::endl;

	if (boxes_volumn > bin_volumn) return false;
	else return true;
}
