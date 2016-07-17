#include "BinpackingCalculate.h"

void binpack3d(int n, int W, int H, int D,
	int *w, int *h, int *d,
	int *x, int *y, int *z, int *bno,
	int *lb, int *ub,
	int nodelimit, int iterlimit, int timelimit,
	int *nodeused, int *iterused, int *timeused,
	int packingtype);

BinpackingCalculate::BinpackingCalculate()
{

}
void BinpackingCalculate::SetPackingType(int type)
{
	packingtype = type;
}
void BinpackingCalculate::SetBinSize(int bin_width, int bin_height, int bin_depth)
{
	b_w = bin_width;
	b_h = bin_height;
	b_d = bin_depth;

}
void BinpackingCalculate::SetBoxesSize(int total, int *width, int *height, int *depth)
{
	total_boxes = total;
	w = width;  //copy address
	h = height;
	d = depth;

}
void BinpackingCalculate::SetLimit(int node_limit, int iter_limit, int time_limit)
{
	nodelimit = node_limit;
	iterlimit = iter_limit;
	timelimit = time_limit;
}
void BinpackingCalculate::CalculateBinpack(int *new_order, int *x_pos, int *y_pos, int *z_pos, int *bin_num)
{
	//copy address
	x = x_pos;
	y = y_pos;
	z = z_pos;
	bno = bin_num;
	item_order = new_order;

	binpack3d(total_boxes, 
		b_w, b_h, b_d, 
		w, h, d, 
		x, y, z, bno, &lb, &ub,
		nodelimit, iterlimit, timelimit,
		&nodeused, &iterused, &timeused,
		packingtype);

	cout << "nodeused = " << nodeused << endl;
	cout << "iterused = " << iterused << endl;
	cout << "timeused = " << timeused << endl;


	//sort priority 
	//same z-max,
	//same y-min
	//x from max to min

	cout << "\nbefore sort" << endl;
	PrintArrayValue();

	//cout << "sort z max to min" << endl;
	SortMaxtoMin(z, 0, total_boxes - 1);




	//sort x min to max
	
	
	int z_start_index = 0;
	//in each same z value range , sort y min to max
	for (int i = z_start_index; i < total_boxes; i++)
	{
		//detect that next z is last item, or new range
		if (i == total_boxes - 1 || z[i] != z[i + 1])
		{
			//each range of same z, sort x
			//cout << "\n z=" << z[i] << " range " << z_start_index <<","<<i<<endl;
			//cout << "sort y min to max" << endl;
			SortMintoMax(y, z_start_index, i);

			//cout << "\nafter sort" << endl;
			//PrintArrayValue();
			
			z_start_index = i + 1;

		}
		
	}
	

	int y_start_index = 0;
	//in each same y value range, sort x max to min
	for (int i = y_start_index; i < total_boxes; i++)
	{
		//detect that next y is last item, or new range
		if (i == total_boxes - 1 || y[i] != y[i + 1])
		{
			//each range of same y, sort x
			//cout << "\n y=" << y[i] << " range " << y_start_index << "," << i << endl;
			//cout << "sort x max to min" << endl;
			SortMaxtoMin(x, y_start_index, i);

			//cout << "\nafter sort" << endl;
			//PrintArrayValue();

			y_start_index = i + 1;
		}


	}

	cout << "\nafter sort" << endl;
	PrintArrayValue();

}


bool BinpackingCalculate::CheckAllCanFit()
{
	int boxes_volumn = 0;
	int bin_volumn = b_w*b_h*b_d;
	for (int i = 0; i < total_boxes; i++)
	{
		boxes_volumn += (w[i]*h[i]*d[i]);
	}

	cout << "boxes:" << boxes_volumn << endl;
	cout << "bin:" << bin_volumn << endl;

	if (boxes_volumn > bin_volumn) return false;
	else return true;
}


void BinpackingCalculate::PrintArrayValue()
{
	cout << "PrintArrayValue" << endl;
	for (int i = 0; i < total_boxes; i++) {
		cout
			<< i << ":from " << item_order[i] << " @ "
			<< x[i] << " " << y[i] << " " << z[i] << " "
			<< endl;
	}
}

void BinpackingCalculate::SortMaxtoMin(int *arr, int left, int right)
{
	//cout << "\nSortMaxtoMin" << endl;
	//PrintArrayValue();

	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] > pivot)
			i++;
		while (arr[j] < pivot)
			j--;

		if (i <= j)
		{
			SwapArrayIndex(i, j);
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		SortMaxtoMin(arr, left, j);
	if (i < right)
		SortMaxtoMin(arr, i, right);
}
void BinpackingCalculate::SortMintoMax(int *arr, int left, int right)
{
	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] < pivot)
			i++;
		while (arr[j] > pivot)
			j--;

		if (i <= j)
		{
			SwapArrayIndex(i, j);
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		SortMintoMax(arr, left, j);
	if (i < right)
		SortMintoMax(arr, i, right);
}
void BinpackingCalculate::SwapArrayIndex(int i, int j)
{

	SwapEachValue(&item_order[i], &item_order[j]);
	SwapEachValue(&x[i], &x[j]);
	SwapEachValue(&y[i], &y[j]);
	SwapEachValue(&z[i], &z[j]);
}
void BinpackingCalculate::SwapEachValue(int *a, int *b)
{
	int tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}
