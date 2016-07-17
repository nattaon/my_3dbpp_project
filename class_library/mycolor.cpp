#include "mycolor.h"
#include <iostream>

MyColor::MyColor()
{
	for (int i = 0; i < MAX_COLOR; i++)
	{
		SetRandomColor(i);

	}
}

void MyColor::SetRandomColor(int index)
{
	// make new class for this
	//prevent alway random by random at first time new class
	float r = (rand() % 200 + 56) / 255.0;
	float g = (rand() % 200 + 56) / 255.0;
	float b = (rand() % 200 + 56) / 255.0;

	//std::cout << "rgb" << r << "," << g << "," << b << std::endl;

	color_table[index][0] = r;
	color_table[index][1] = g;
	color_table[index][2] = b;
	//glColor3d(r, g, b);

}

float* MyColor::GetColorIndex(int index)
{
	return color_table[index];
}


